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

    while (true) {
        if (external_stop_ && external_stop_->load()) {
            std::cout << "[ControlLoop] external stop flag set, exiting loop.\n";
            break;
        }

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

        if (!nav_ok) {
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

        // poll input
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

        // guard
        {
            const std::uint64_t now_ns = now_mono_ns_();
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
