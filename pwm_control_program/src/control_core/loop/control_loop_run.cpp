/**
 * @file   control_loop_run.cpp
 * @brief  ControlLoop::run main loop (Path A)
 */

#include "control_core/control_loop.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "shared/msg/nav_state.hpp" // run() 里在栈上构造 NavState，必须 include 完整定义

namespace rovctrl::control_core {

using clock      = std::chrono::steady_clock;
using duration_d = std::chrono::duration<double>;

int ControlLoop::run()
{
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

    if (!input_->init()) {
        std::cerr << "[ControlLoop] InputProvider::init() failed.\n";
        return -3;
    }

    // PWM log（按开关创建）
    if (cfg_.enable_pwm_log) {
        pwm_logger_ = make_pwm_logger_();
        if (!pwm_logger_ || !pwm_logger_->init("./logs",
                                               ControlLoop::PwmLog::Mode::CmdAndApplied,
                                               "pwm_log")) {
            std::cerr << "[ControlLoop] PwmLogger init failed, continue without logging.\n";
            pwm_logger_.reset();
        }
    }

    // allocator
    if (!allocator_.init(cfg_.thruster_alloc)) {
        std::cerr << "[ControlLoop] ThrusterAllocator::init() failed.\n";
        return -4;
    }
    if (!allocator_.ok()) {
        std::cerr << "[ControlLoop] ThrusterAllocator is not OK.\n";
        return -5;
    }

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

    // ---------------- helpers (define once, not per-iteration) ----------------

    auto log_pwm_cmd_applied = [&](double t_s, const ThrusterArray& thr_cmd) {
        if (!pwm_logger_ || !pwm_logger_->is_open()) return;

        std::array<float, 8> cmd{};
        std::array<float, 8> applied{};
        for (std::size_t i = 0; i < 8; ++i) {
            cmd[i]     = thr_cmd[i];
            applied[i] = thr_cmd[i]; // TODO: 若能读到 safety-layer applied，替换
        }
        pwm_logger_->logCmdAndApplied(t_s, cmd, applied);
    };

    auto neutral_and_step = [&](double t_s) {
        ThrusterArray thr_cmd{};
        thr_cmd.fill(0.0f); // 0 -> neutral
        (void)pwm_.setTargets(thr_cmd);
        (void)pwm_.step();
        log_pwm_cmd_applied(t_s, thr_cmd);
    };

    auto has_active_command_from_intent = [&](const ControlIntent& in) -> bool {
    // 1) 任何显式“非运动”命令都视为 active，避免被归中逻辑吞掉
    if (in.request_exit) return true;
    if (in.has_estop_cmd) return true;  // estop / clear_estop
    if (in.has_arm_cmd)   return true;  // arm / disarm
    if (in.has_mode_request && in.mode_request != ControlMode::kNone) return true;

    if (in.has_ref)       return true;
    if (in.has_ref_delta) return true;

    // 2) 遥控输入：只有当 DOF 确实“非零”时，才认为 active
    //    这样才能让“无输入归中”在松手/停止按键后触发
    if (in.has_teleop_dof) {
        constexpr double eps = 0.02; // 经验阈值：抑制抖动/浮点噪声
        const auto absd = [](double x) { return (x >= 0.0) ? x : -x; };

        const auto& c = in.teleop_dof_cmd;
        if (absd(c.surge) > eps) return true;
        if (absd(c.sway)  > eps) return true;
        if (absd(c.heave) > eps) return true;
        if (absd(c.roll)  > eps) return true;
        if (absd(c.pitch) > eps) return true;
        if (absd(c.yaw)   > eps) return true;

        // DOF 全部接近 0：视为“无外界运动命令”
        return false;
    }

    // 3) 其他情况：无外界输入/命令
    return false;
};



    double no_input_sec = 0.0;
    const double no_input_neutral_sec =
        (cfg_.no_input_neutral_ms > 0) ? (cfg_.no_input_neutral_ms / 1000.0) : 0.2;

    // ---------------- main loop ----------------
    while (true) {
        // 外部退出（Ctrl+C / UI quit 等）
        if (external_stop_ && external_stop_->load()) {
            const double t_s = duration_d(clock::now() - start_time_).count();
            std::cout << "[ControlLoop] external stop flag set, exiting loop.\n";
            neutral_and_step(t_s); // 退出前归中一次
            break;
        }

        // 固定周期调度
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

        // nav update（NavSub 的懒初始化/订阅 init 在 update_nav_feedback_ 内部完成）
        shared::msg::NavState nav{};
        const bool nav_ok = update_nav_feedback_(nav);

        // 只有 Auto 模式要求导航（Manual/Teleop 不需要）
        const ControlMode mode_now = ctrl_mgr_.mode();
        const bool nav_required = (mode_now == ControlMode::kAuto);

        if (!nav_ok) {
            ++nav_miss_counter;

            if (nav_required) {
                if (!cfg_.allow_run_without_nav) {
                    std::cerr << "[ControlLoop] NavState missing in AUTO mode and allow_run_without_nav=false, entering failsafe.\n";
                    execute_failsafe_(FailsafeAction::kEmergencyStop);
                    return -8;
                }

                if (nav_miss_counter == 100 ||
                    (cfg_.step_error_log_interval > 0 &&
                     nav_miss_counter % cfg_.step_error_log_interval == 0)) {
                    std::cerr << "[ControlLoop] Warning(AUTO): no stable NavState for "
                              << nav_miss_counter << " cycles.\n";
                }
            } else {
                // Manual / Failsafe：忽略导航缺失，避免干扰遥控
                // Manual / Failsafe：忽略导航缺失，避免干扰遥控
            if (nav_miss_counter == 1) {
                std::cout << "[ControlLoop] NavState missing (ignored in mode="
                          << static_cast<int>(mode_now) << ").\n";
                }
            }
        } else {
            nav_miss_counter = 0;
        }

        // poll input
        ControlIntent intent{};
        if (!input_->poll(state_, intent)) {
            std::cerr << "[ControlLoop] InputProvider::poll(state,intent) failed.\n";
            execute_failsafe_(FailsafeAction::kEmergencyStop);
            return -10;
        }

        // input 请求退出
        if (intent.request_exit) {
            std::cout << "[ControlLoop] Input provider requested exit.\n";
            neutral_and_step(t_s);
            break;
        }

        // 无输入 watchdog：达到阈值后归中并跳过控制器计算
        if (!has_active_command_from_intent(intent)) {
            no_input_sec += dt;
        } else {
            no_input_sec = 0.0;
        }

        if (no_input_sec >= no_input_neutral_sec) {
            neutral_and_step(t_s);
            continue;
        }

        // guard
        {
            const std::uint64_t now_ns = now_mono_ns_();
            const shared::msg::NavState* nav_ptr = nav_ok ? &nav : nullptr;

            guard_result_ = guard_.step(now_ns, state_, nav_ptr, intent);

            if (guard_result_.effective_intent.request_exit) {
                std::cout << "[ControlLoop] Guard requested exit.\n";
                neutral_and_step(t_s);
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

        build_reference_from_guard_();

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

        ThrusterArray thr_cmd{};
        if (!build_thruster_command_(thr_cmd)) {
            thr_cmd.fill(0.0f);
        }

        {
            const int rc = pwm_.setTargets(thr_cmd);
            if (rc < 0) {
                std::cerr << "[ControlLoop] pwm_.setTargets() rc=" << rc
                          << " msg=" << pwm_.status().last_error_msg << "\n";
            }
        }

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

        log_pwm_cmd_applied(t_s, thr_cmd);
    }

    // 正常退出：再保险归中一次（比无条件 E-Stop 更符合“退出键结束程序”的语义）
    neutral_and_step(duration_d(clock::now() - start_time_).count());

    std::cout << "[ControlLoop] loop exited normally.\n";
    return 0;
}


} // namespace rovctrl::control_core
