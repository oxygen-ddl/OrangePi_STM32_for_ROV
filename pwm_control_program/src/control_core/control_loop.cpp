/**
 * @file   control_loop.cpp
 * @brief  控制主循环实现（ControlIntent + Guard 仲裁版 / Path A）
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <utility>

// cpp 内部依赖：导航订阅器 + NavState
#include "io/nav/nav_state_subscriber.hpp"
#include "shared/msg/nav_state.hpp"

// 日志实现（cpp 内部使用，不透传到头文件）
#include "io/log/pwm_logger.hpp"

// 兜底：若 ControllerManager 析构仍需完整类型
#include "controllers/controller_base.hpp"

// 统一时间基
#include "platform/timebase.hpp"

namespace rovctrl::control_core {

using clock      = std::chrono::steady_clock;
using duration_d = std::chrono::duration<double>;

namespace {

// 统一：用项目 timebase（避免重复定义 now_mono_ns）
inline std::uint64_t now_mono_ns()
{
    return static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());
}

} // namespace

// ============================================================================
// NavSub (PIMPL) —— 只在 .cpp 依赖 shared/msg 与共享内存订阅实现
// ============================================================================

struct ControlLoop::NavSub {
    rovctrl::io::NavStateSubscriber sub{};
    std::string shm_name{"/rov_nav_state_v1"};

    bool init() { return sub.init(shm_name); }
    bool ok() const noexcept { return sub.ok(); }

    bool read_latest(shared::msg::NavState& out) const { return sub.read_latest(out); }
};

// ============================================================================
// PwmLog (PIMPL) —— ControlLoop::PwmLog 抽象接口的具体实现
// ============================================================================

namespace {

class PwmLogImpl final : public ControlLoop::PwmLog {
public:
    bool init(const std::string& root_dir,
              Mode               mode,
              const std::string& prefix) override
    {
        const rovctrl::io::PwmLogger::Mode m =
            (mode == Mode::AppliedOnly) ? rovctrl::io::PwmLogger::Mode::AppliedOnly
                                        : rovctrl::io::PwmLogger::Mode::CmdAndApplied;

        return logger_.init(root_dir, m, prefix);
    }

    bool is_open() const noexcept override { return logger_.is_open(); }

    void logApplied(double t_s,
                    const std::array<float, 8>& applied) override
    {
        logger_.logApplied(t_s, applied);
    }

    void logCmdAndApplied(double t_s,
                          const std::array<float, 8>& cmd,
                          const std::array<float, 8>& applied) override
    {
        logger_.logCmdAndApplied(t_s, cmd, applied);
    }

    void close() noexcept override
    {
        // 如果你的 rovctrl::io::PwmLogger 没有 close()，可以留空；
        // 其 std::ofstream 在析构时会自动关闭。
    }

private:
    rovctrl::io::PwmLogger logger_;
};

} // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

ControlLoop::ControlLoop(const Config&                 cfg,
                         rovctrl::platform::PwmClient& pwm,
                         rovctrl::io::InputProviderPtr input,
                         ControllerManager&&           ctrl_mgr,
                         std::atomic_bool*             external_stop_flag)
    : cfg_(cfg)
    , pwm_(pwm)
    , input_(std::move(input))
    , ctrl_mgr_(std::move(ctrl_mgr))
    , external_stop_(external_stop_flag)
    , guard_(cfg.guard_cfg)
    , nav_sub_(std::make_unique<NavSub>())
{
    // 这里不强制创建 pwm_logger_，在 run() 里按 enable 开关创建更清晰
}

ControlLoop::~ControlLoop() noexcept = default;

// ============================================================================
// 导航反馈更新（并返回 nav 是否有效）
// ============================================================================

bool ControlLoop::update_nav_feedback_(shared::msg::NavState& nav_out)
{
    state_.nav_valid = false;

    if (!nav_sub_ || !nav_sub_->ok()) {
        last_nav_valid_ = false;
        return false;
    }

    shared::msg::NavState nav{};
    if (!nav_sub_->read_latest(nav)) {
        last_nav_valid_ = false;
        return false;
    }

    last_nav_valid_ = true;

    state_.nav_valid = true;
    state_.nav_t_ns  = nav.t_ns;

    for (std::size_t i = 0; i < 3; ++i) {
        state_.nav_pos_ned[i] = nav.pos[i];
        state_.nav_vel_ned[i] = nav.vel[i];
        state_.nav_rpy[i]     = nav.rpy[i];
        state_.nav_omega_b[i] = nav.omega_b[i];
        state_.nav_acc_b[i]   = nav.acc_b[i];
    }

    state_.nav_depth        = nav.depth;
    state_.nav_status_flags = nav.status_flags;

    nav_out = nav;
    return true;
}

// ============================================================================
// 构造 8 路推进器归一化指令
// ============================================================================

bool ControlLoop::build_thruster_command_(ThrusterArray& thr_out)
{
    if (output_.has_thruster_command) {
        thr_out = output_.thruster_command;
        return true;
    }

    if (output_.has_body_wrench) {
        std::array<double, 6> wrench{};
        for (std::size_t i = 0; i < 6; ++i) {
            wrench[i] = output_.body_wrench[i];
        }

        std::array<float, 8> thr{};
        if (!allocator_.allocate(wrench, thr)) {
            return false;
        }

        thr_out = thr;
        return true;
    }

    return false;
}

// ============================================================================
// 执行 failsafe（Guard 只建议，ControlLoop 执行）
// ============================================================================

void ControlLoop::execute_failsafe_(FailsafeAction a)
{
    // 清空控制输出，避免残留
    output_ = ControlOutput{};

    // 尽量把控制器切到 failsafe（若内部实现支持）
    (void)ctrl_mgr_.set_mode(ControlMode::kFailsafe);

    ThrusterArray zero{};
    zero.fill(0.0f);

    // kEmergencyStop：如 PwmClient 有专门 API，可在此调用
    (void)a;

    (void)pwm_.setTargets(zero);
    (void)pwm_.step();
}

// ============================================================================
// GuardResult -> reference
// ============================================================================

void ControlLoop::build_reference_from_guard_()
{
    ref_ = ControlReference{}; // 每周期重建，避免旧字段残留

    // 这里假设 GuardResult 里有 effective_intent 字段（你前面已按此方向改）
    const auto& eff = guard_result_.effective_intent;

    // 1) explicit ref
    if (eff.has_ref) {
        ref_ = eff.ref;
    }

    // 2) teleop dof
    if (eff.has_teleop_dof) {
        ref_.dof_cmd     = eff.teleop_dof_cmd;
        ref_.use_dof_cmd = true;
    }

    // 3) ref_delta：建议由控制器层融合；若临时需要，可在此做保守叠加
    // if (eff.has_ref_delta) { ... }
}

// ============================================================================
// 主循环
// ============================================================================

int ControlLoop::run()
{
    // 1) 基本检查
    if (!input_) {
        std::cerr << "[ControlLoop] input_ is null, cannot run.\n";
        return -2;
    }

    if (!ctrl_mgr_.has_active_controller()) {
        std::cerr << "[ControlLoop] ControllerManager has no active controller.\n";
        std::cerr << "             status.ok=" << ctrl_mgr_.status().ok
                  << " mode=" << static_cast<int>(ctrl_mgr_.status().mode)
                  << " active='" << ctrl_mgr_.status().active_controller
                  << "' err='" << ctrl_mgr_.status().last_error << "'\n";
        return -1;
    }

    // 2) init input
    if (!input_->init()) {
        std::cerr << "[ControlLoop] InputProvider::init() failed.\n";
        return -3;
    }

    // 3) PWM log（按开关创建 PIMPL）
    if (cfg_.enable_pwm_log) {
        pwm_logger_ = std::make_unique<PwmLogImpl>();
        if (!pwm_logger_->init("./logs",
                               ControlLoop::PwmLog::Mode::CmdAndApplied,
                               "pwm_log")) {
            std::cerr << "[ControlLoop] PwmLogger init failed, continue without logging.\n";
            pwm_logger_.reset();
        }
    }

    // 4) nav sub
    if (nav_sub_) {
        if (!nav_sub_->init()) {
            std::cerr << "[ControlLoop] Warning: NavStateSubscriber init failed on "
                      << nav_sub_->shm_name << ". Running without navigation feedback.\n";
            last_nav_valid_ = false;
        } else {
            std::cout << "[ControlLoop] NavStateSubscriber initialized on "
                      << nav_sub_->shm_name << ".\n";
        }
    } else {
        std::cerr << "[ControlLoop] Warning: nav_sub_ is null.\n";
        last_nav_valid_ = false;
    }

    // 5) allocator
    if (!allocator_.init(cfg_.thruster_alloc)) {
        std::cerr << "[ControlLoop] ThrusterAllocator::init() failed.\n";
        return -4;
    }
    if (!allocator_.ok()) {
        std::cerr << "[ControlLoop] ThrusterAllocator is not OK.\n";
        return -5;
    }

    // 6) loop freq
    double loop_hz = cfg_.loop_hz;
    if (loop_hz <= 0.0) {
        std::cerr << "[ControlLoop] invalid loop_hz=" << loop_hz << ", fallback to 100.\n";
        loop_hz = 100.0;
    }

    const auto   loop_period = duration_d(1.0 / loop_hz);
    const double dt_nom      = 1.0 / loop_hz;

    double dt_max = cfg_.dt_clamp_max_sec;
    if (dt_max <= 0.0) dt_max = 0.2;
    if (dt_max < dt_nom) dt_max = dt_nom;

    auto last_tick = clock::now();
    auto next_tick = last_tick + loop_period;
    start_time_    = last_tick;

    int step_err_count   = 0;
    int nav_miss_counter = 0;

    std::cout << "[ControlLoop] starting, loop_hz=" << loop_hz
              << " Hz, active_controller=" << ctrl_mgr_.active_controller_name()
              << ", mode=" << static_cast<int>(ctrl_mgr_.mode()) << "\n";

    while (true) {
        // 7) external stop
        if (external_stop_ && external_stop_->load()) {
            std::cout << "[ControlLoop] external stop flag set, exiting loop.\n";
            break;
        }

        // 8) schedule
        auto now = clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
            now = clock::now();
        } else {
            const auto lag = duration_d(now - next_tick).count();
            if (lag > 5.0 * dt_nom) {
                next_tick = now;
            }
        }

        double dt = duration_d(now - last_tick).count();
        last_tick = now;
        next_tick += loop_period;

        if (dt <= 0.0) dt = dt_nom;
        if (dt > dt_max) dt = dt_max;

        const double t_s = duration_d(now - start_time_).count();
        state_.timestamp_sec = t_s;

        // 9) nav update
        shared::msg::NavState nav{};
        const bool nav_ok = update_nav_feedback_(nav);

        if (nav_sub_ && nav_sub_->ok()) {
            if (!state_.nav_valid) {
                ++nav_miss_counter;

                if (!cfg_.allow_run_without_nav) {
                    std::cerr << "[ControlLoop] NavState missing and allow_run_without_nav=false, entering failsafe.\n";
                    execute_failsafe_(FailsafeAction::kEmergencyStop);
                    return -8;
                }

                if (nav_miss_counter == 100 ||
                    (cfg_.step_error_log_interval > 0 &&
                     nav_miss_counter % cfg_.step_error_log_interval == 0)) {
                    std::cerr << "[ControlLoop] Warning: no stable NavState for "
                              << nav_miss_counter << " cycles.\n";
                }
            } else {
                nav_miss_counter = 0;
            }
        } else {
            nav_miss_counter = 0;
        }

        // 10) poll input (ControlIntent)
        ControlIntent intent{};
        if (!input_->poll(state_, intent)) {
            std::cerr << "[ControlLoop] InputProvider::poll(state,intent) failed.\n";
            execute_failsafe_(FailsafeAction::kEmergencyStop);
            return -10;
        }

        if (intent.request_exit) {
            std::cout << "[ControlLoop] Input provider requested exit.\n";
            break;
        }

        // 11) guard
        {
            const std::uint64_t now_ns = now_mono_ns();
            const shared::msg::NavState* nav_ptr = nav_ok ? &nav : nullptr;

            guard_result_ = guard_.step(now_ns, state_, nav_ptr, intent);

            if (guard_result_.effective_intent.request_exit) {
                std::cout << "[ControlLoop] Guard requested exit.\n";
                break;
            }

            if (guard_result_.failsafe != FailsafeAction::kNone) {
                execute_failsafe_(guard_result_.failsafe);
                continue;
            }

            if (guard_result_.mode_changed) {
                (void)ctrl_mgr_.set_mode(guard_result_.effective_mode);
            }
        }

        // 12) build ref
        build_reference_from_guard_();

        // 13) compute
        output_ = ControlOutput{};
        if (!ctrl_mgr_.compute(state_, ref_, output_, dt)) {
            std::cerr << "[ControlLoop] ControllerManager::compute() failed: "
                      << ctrl_mgr_.status().last_error << "\n";

            if (cfg_.enter_failsafe_on_controller_error) {
                execute_failsafe_(FailsafeAction::kZeroOutput);
                continue;
            }
            return -20;
        }

        // 14) thruster command
        ThrusterArray thr_cmd{};
        if (!build_thruster_command_(thr_cmd)) {
            thr_cmd.fill(0.0f);
        }

        // 15) setTargets
        {
            const int rc = pwm_.setTargets(thr_cmd);
            if (rc < 0) {
                std::cerr << "[ControlLoop] pwm_.setTargets() rc=" << rc
                          << " msg=" << pwm_.status().last_error_msg << "\n";
            }
        }

        // 16) step
        const int step_rc = pwm_.step();
        if (step_rc < 0) {
            ++step_err_count;

            if (step_err_count <= 3 ||
                (cfg_.step_error_log_interval > 0 &&
                 step_err_count % cfg_.step_error_log_interval == 0)) {
                std::cerr << "[ControlLoop] pwm_.step() rc=" << step_rc
                          << " msg=" << pwm_.status().last_error_msg
                          << " (error count=" << step_err_count << ")\n";
            }

            if (cfg_.max_step_errors > 0 && step_err_count > cfg_.max_step_errors) {
                std::cerr << "[ControlLoop] pwm_.step() errors exceed max_step_errors="
                          << cfg_.max_step_errors << ", entering failsafe and abort.\n";
                execute_failsafe_(FailsafeAction::kEmergencyStop);
                return -30;
            }
            continue;
        }

        step_err_count = 0;

        // 17) log
        if (pwm_logger_ && pwm_logger_->is_open()) {
            std::array<float, 8> cmd{};
            std::array<float, 8> applied{};

            for (std::size_t i = 0; i < 8; ++i) {
                cmd[i]     = thr_cmd[i];
                applied[i] = thr_cmd[i]; // TODO: 若能读到 safety-layer applied，替换
            }

            pwm_logger_->logCmdAndApplied(t_s, cmd, applied);
        }
    }

    execute_failsafe_(FailsafeAction::kEmergencyStop);

    std::cout << "[ControlLoop] loop exited normally.\n";
    return 0;
}

} // namespace rovctrl::control_core
