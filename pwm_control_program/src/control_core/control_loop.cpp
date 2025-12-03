// src/control_core/control_loop.cpp
#include "control_core/control_loop.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace rovctrl::control_core {

using clock      = std::chrono::steady_clock;
using duration_d = std::chrono::duration<double>;

ControlLoop::ControlLoop(const Config&                cfg,
                         rovctrl::platform::PwmClient& pwm,
                         rovctrl::io::InputProviderPtr input,
                         ControllerPtr                 controller,
                         std::atomic_bool*             external_stop_flag)
    : cfg_(cfg)
    , pwm_(pwm)
    , input_(std::move(input))
    , controller_(std::move(controller))
    , external_stop_(external_stop_flag)
{}

// 主循环实现
int ControlLoop::run()
{
    // -------- 基本合法性检查 --------
    if (!controller_) {
        std::cerr << "[ControlLoop] controller_ is null, cannot run.\n";
        return -1;
    }

    if (!input_) {
        std::cerr << "[ControlLoop] input_ is null, cannot run.\n";
        return -2;
    }

    // -------- 初始化输入源（例如 TeleopInputProvider 打开终端 raw 模式） --------
    if (!input_->init()) {
        std::cerr << "[ControlLoop] InputProvider::init() failed.\n";
        return -3;
    }

    // -------- 初始化 PWM 日志（可配置开关） --------
    if (cfg_.enable_pwm_log) {
        if (!pwm_logger_.init("./logs")) {
            std::cerr << "[ControlLoop] PwmLogger init failed, continue without logging.\n";
            // 这里不强行退出，只是后续 is_open() 为 false，不写日志
        }
    }

    // -------- 循环频率配置 --------
    if (cfg_.loop_hz <= 0.0) {
        std::cerr << "[ControlLoop] invalid loop_hz = " << cfg_.loop_hz
                  << ", fallback to 100 Hz.\n";
        cfg_.loop_hz = 100.0;
    }

    const double loop_hz     = cfg_.loop_hz;
    const auto   loop_period = duration_d(1.0 / loop_hz);

    auto last_tick = clock::now();
    auto next_tick = last_tick + loop_period;
    start_time_    = last_tick;  // 日志时间零点（steady_clock）

    int step_err_count = 0;

    std::cout << "[ControlLoop] starting, loop_hz = " << loop_hz << " Hz\n";

    while (true) {
        // -------- 外部退出标志检查（例如 Ctrl+C 信号） --------
        if (external_stop_ && external_stop_->load()) {
            std::cout << "[ControlLoop] external stop flag set, exiting loop.\n";
            break;
        }

        // -------- 固定周期调度 --------
        auto now = clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
            now = clock::now();
        }

        double dt = duration_d(now - last_tick).count();
        if (dt <= 0.0) {
            dt = 1.0 / loop_hz;  // 防守性兜底，避免 0 或负数
        }
        last_tick = now;
        next_tick = now + loop_period;

        if (cfg_.log_timing) {
            // 如有需要，可以在这里打印/统计 dt
            // std::cout << "[ControlLoop] dt = " << dt << " s\n";
        }

        // -------- 输入更新：状态 + 参考量 --------
        bool request_exit = false;
        if (!input_->poll(state_, ref_, request_exit)) {
            std::cerr << "[ControlLoop] InputProvider::poll() failed, exiting.\n";
            return -10;
        }
        if (request_exit) {
            std::cout << "[ControlLoop] Input provider requested exit, exiting loop.\n";
            break;
        }

        // -------- 控制器计算 --------
        output_ = ControlOutput{};  // 清空上次输出与标志位
        if (!controller_->compute(state_, ref_, output_, dt)) {
            std::cerr << "[ControlLoop] controller_->compute() failed, exiting.\n";
            return -20;
        }

        // -------- 输出映射到 PWM 客户端 --------
        //
        // 约定策略：
        //   - 如果 has_thruster_command = true：
        //       output_.thruster_command 视为已经完成 DOF→推进器映射的归一化指令，
        //       由 controller 或 allocation 层负责物理意义；
        //       ControlLoop 仅负责下发；
        //
        if (output_.has_thruster_command) {
            int rc = pwm_.setTargets(output_.thruster_command);
            if (rc < 0) {
                std::cerr << "[ControlLoop] pwm_.setTargets() rc=" << rc
                          << " msg=" << pwm_.status().last_error_msg << "\n";
                // setTargets 失败暂时不作为致命错误处理，交由 step() 统一判断
            }
        }

        // -------- 安全层 step：限斜率 + 分组 + 实际下发 --------
        int step_rc = pwm_.step();
        if (step_rc < 0) {
            ++step_err_count;
            if (step_err_count <= 3 ||
                (cfg_.step_error_log_interval > 0 &&
                 step_err_count % cfg_.step_error_log_interval == 0)) {
                std::cerr << "[ControlLoop] pwm_.step() rc=" << step_rc
                          << " msg=" << pwm_.status().last_error_msg
                          << " (error count = " << step_err_count << ")\n";
            }

            if (cfg_.max_step_errors > 0 &&
                step_err_count > cfg_.max_step_errors) {
                std::cerr << "[ControlLoop] pwm_.step() errors exceed max_step_errors ("
                          << cfg_.max_step_errors << "), aborting.\n";
                return -30;
            }
        } else {
            // 有一次成功就清零错误计数
            step_err_count = 0;

            // 成功下发后记录一条 PWM 日志（与控制环锁步）
            if (pwm_logger_.is_open() && output_.has_thruster_command) {
                double t_s = duration_d(now - start_time_).count();
                pwm_logger_.log(t_s, output_.thruster_command);
            }
        }
    }

    std::cout << "[ControlLoop] loop exited normally.\n";
    return 0;
}

} // namespace rovctrl::control_core

/*
 * @file control_loop.cpp
 * @brief 控制主循环实现
 *
 * 负责周期性调用输入源更新、控制器计算、PWM 输出等。
 *
 * @author 王雨舒
 * @date 2025-12-02
 */
