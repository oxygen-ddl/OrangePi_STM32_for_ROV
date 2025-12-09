/**
 * @file   control_loop.cpp
 * @brief  控制主循环实现
 *
 * 负责周期性执行以下步骤：
 *   1. 处理外部停止标志；
 *   2. 固定周期调度（基于 steady_clock）；
 *   3. 从 InputProvider 读取当前控制参考与状态；
 *   4. 从导航共享内存订阅最新 NavState（如可用）；
 *   5. 调用控制器计算控制输出（body wrench / thruster command）；
 *   6. 如有需要，调用推力分配模块将 body wrench 映射为 8 通道推进器指令；
 *   7. 通过 PwmClient 下发 PWM（含安全层 step）；
 *   8. 记录 PWM 日志（可配置启用）。
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace rovctrl::control_core {

using clock      = std::chrono::steady_clock;
using duration_d = std::chrono::duration<double>;

// ============================================================================
// 构造函数
// ============================================================================

ControlLoop::ControlLoop(const Config&                 cfg,
                         rovctrl::platform::PwmClient& pwm,
                         rovctrl::io::InputProviderPtr input,
                         ControllerPtr                 controller,
                         std::atomic_bool*             external_stop_flag)
    : cfg_(cfg)
    , pwm_(pwm)
    , input_(std::move(input))
    , controller_(std::move(controller))
    , external_stop_(external_stop_flag)
{
    // 这里只做依赖注入，不做重型初始化，重型动作在 run() 中完成：
    //  - input_->init()
    //  - pwm_logger_.init()
    //  - nav_sub_.init()
    //  - allocator_.init()
}

// ============================================================================
// 主循环实现
// ============================================================================

int ControlLoop::run()
{
    // ------------------------------------------------------------------------
    // 1. 基本合法性检查
    // ------------------------------------------------------------------------
    if (!controller_) {
        std::cerr << "[ControlLoop] controller_ is null, cannot run.\n";
        return -1;
    }

    if (!input_) {
        std::cerr << "[ControlLoop] input_ is null, cannot run.\n";
        return -2;
    }

    // ------------------------------------------------------------------------
    // 2. 初始化输入源（例如 TeleopInputProvider 打开终端 raw 模式）
    // ------------------------------------------------------------------------
    if (!input_->init()) {
        std::cerr << "[ControlLoop] InputProvider::init() failed.\n";
        return -3;
    }

    // ------------------------------------------------------------------------
    // 3. 初始化 PWM 日志（可配置开关）
    // ------------------------------------------------------------------------
    if (cfg_.enable_pwm_log) {
        if (!pwm_logger_.init("./logs",
                              rovctrl::io::PwmLogger::Mode::CmdAndApplied,
                              "pwm_log")) {
            std::cerr << "[ControlLoop] PwmLogger init failed, "
                         "continue without logging.\n";
            // 不强制退出；后续通过 is_open() 判断是否写日志
        }
    }

    // ------------------------------------------------------------------------
    // 4. 初始化导航状态订阅器（从导航进程共享内存读取 NavState）
    // ------------------------------------------------------------------------
    {
        const char* shm_name = "/rov_nav_state_v1";
        if (!nav_sub_.init(shm_name)) {
            std::cerr << "[ControlLoop] Warning: NavStateSubscriber init failed on "
                      << shm_name << ". Running without navigation feedback.\n";
            last_nav_valid_ = false;
        } else {
            std::cout << "[ControlLoop] NavStateSubscriber initialized on "
                      << shm_name << ".\n";
        }
    }

    // ------------------------------------------------------------------------
    // 5. 循环频率配置
    // ------------------------------------------------------------------------
    if (cfg_.loop_hz <= 0.0) {
        std::cerr << "[ControlLoop] invalid loop_hz = " << cfg_.loop_hz
                  << ", fallback to 100 Hz.\n";
        cfg_.loop_hz = 100.0;
    }

    const double loop_hz     = cfg_.loop_hz;
    const auto   loop_period = duration_d(1.0 / loop_hz);

    auto last_tick = clock::now();
    auto next_tick = last_tick + loop_period;
    start_time_    = last_tick;  // 作为日志时间零点（steady_clock）

    int step_err_count   = 0;
    int nav_miss_counter = 0;

    // ------------------------------------------------------------------------
    // 6. 推力分配器初始化（基于 YAML 配置）
    // ------------------------------------------------------------------------
    if (!allocator_.init(cfg_.thruster_alloc)) {
        std::cerr << "[ControlLoop] ThrusterAllocator::init() failed.\n";
        return -4;
    }

    if (!allocator_.ok()) {
        std::cerr << "[ControlLoop] ThrusterAllocator is not in OK state.\n";
        return -5;
    }

    std::cout << "[ControlLoop] starting, loop_hz = " << loop_hz << " Hz\n";

    // ========================================================================
    // 主控制循环
    // ========================================================================
    while (true) {
        // --------------------------------------------------------------------
        // 7. 外部退出标志检查（例如上层 main 收到 Ctrl+C/SIGINT 后设置）
        // --------------------------------------------------------------------
        if (external_stop_ && external_stop_->load()) {
            std::cout << "[ControlLoop] external stop flag set, exiting loop.\n";
            break;
        }

        // --------------------------------------------------------------------
        // 8. 固定周期调度（sleep_until + dt 计算）
        // --------------------------------------------------------------------
        auto now = clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
            now = clock::now();
        }

        double dt = duration_d(now - last_tick).count();
        if (dt <= 0.0) {
            // 防守性兜底：避免 dt 为 0 或负数
            dt = 1.0 / loop_hz;
        }
        last_tick = now;
        next_tick = now + loop_period;

        if (cfg_.log_timing) {
            // 可在此处打印或统计 dt
            // std::cout << "[ControlLoop] dt = " << dt << " s\n";
        }

        // --------------------------------------------------------------------
        // 9. 导航状态更新（从共享内存读取最新 NavState，并注入到内部状态）
        // --------------------------------------------------------------------
        {
            state_.nav_valid = false;  // 默认置 false，每次循环重置

            if (nav_sub_.ok()) {
                shared::msg::NavState nav{};
                if (nav_sub_.read_latest(nav)) {
                    last_nav_state_  = nav;
                    last_nav_valid_  = true;
                    nav_miss_counter = 0;

                    // ==== 将 NavState 映射到内部导航状态 ====
                    state_.nav_valid = true;
                    state_.nav_t_ns = nav.t_ns;

                    // 位置 / 速度 / 姿态（NED）
                    for (std::size_t i = 0; i < 3; ++i) {
                        state_.nav_pos_ned[i] = nav.pos[i];
                        state_.nav_vel_ned[i] = nav.vel[i];
                        state_.nav_rpy[i]     = nav.rpy[i];
                    }

                    // 深度
                    state_.nav_depth = nav.depth;

                    // body 角速度 / 线加速度
                    for (std::size_t i = 0; i < 3; ++i) {
                        state_.nav_omega_b[i] = nav.omega_b[i];
                        state_.nav_acc_b[i]   = nav.acc_b[i];
                    }

                    // 状态标志（新版字段 status_flags）
                    state_.nav_status_flags = nav.status_flags;

                } else {
                    // 本周期没有读到稳定的 NavState
                    ++nav_miss_counter;
                    last_nav_valid_ = false;

                    if (nav_miss_counter == 100 ||
                        (cfg_.step_error_log_interval > 0 &&
                         nav_miss_counter % cfg_.step_error_log_interval == 0)) {
                        std::cerr << "[ControlLoop] Warning: no stable NavState for "
                                  << nav_miss_counter << " cycles.\n";
                    }
                }
            }
            // 若 nav_sub_.ok() 为 false，则保持 nav_valid = false，
            // 控制器可以据此做降级策略
        }

        // --------------------------------------------------------------------
        // 10. 输入更新：从 InputProvider 读取状态 + 参考量
        // --------------------------------------------------------------------
        bool request_exit = false;
        if (!input_->poll(state_, ref_, request_exit)) {
            std::cerr << "[ControlLoop] InputProvider::poll() failed, exiting.\n";
            return -10;
        }
        if (request_exit) {
            std::cout << "[ControlLoop] Input provider requested exit, exiting loop.\n";
            break;
        }

        // --------------------------------------------------------------------
        // 11. 控制器计算
        // --------------------------------------------------------------------
        output_ = ControlOutput{};  // 清空上次输出与标志位
        if (!controller_->compute(state_, ref_, output_, dt)) {
            std::cerr << "[ControlLoop] controller_->compute() failed, exiting.\n";
            return -20;
        }

        // --------------------------------------------------------------------
        // 12. 推力分配：如有 body_wrench，则映射为 8 通道归一化推进器指令
        // --------------------------------------------------------------------
        if (output_.has_body_wrench) {
            std::array<double, 6> wrench{};
            for (std::size_t i = 0; i < 6; ++i) {
                wrench[i] = output_.body_wrench[i];
            }

            std::array<float, 8> thruster_cmd{};
            if (!allocator_.allocate(wrench, thruster_cmd)) {
                std::cerr << "[ControlLoop] ThrusterAllocator::allocate() failed.\n";
                // 当前策略：视为致命错误退出；后续可改为“保留上一次指令 / 清零”
                return -21;
            }

            output_.thruster_command     = thruster_cmd;
            output_.has_thruster_command = true;
        }

        // --------------------------------------------------------------------
        // 13. 输出映射到 PWM 客户端（setTargets）
        // --------------------------------------------------------------------
        //
        // 约定策略：
        //   - 如果 has_thruster_command = true：
        //       output_.thruster_command 视为已经完成 DOF→推进器映射的归一化指令，
        //       由控制器（手动）或 ThrusterAllocator 负责其物理意义；
        //       ControlLoop 仅负责调用下发接口。
        //
        if (output_.has_thruster_command) {
            int rc = pwm_.setTargets(output_.thruster_command);
            if (rc < 0) {
                std::cerr << "[ControlLoop] pwm_.setTargets() rc=" << rc
                          << " msg=" << pwm_.status().last_error_msg << "\n";
                // setTargets 失败暂不视为致命错误，由 step() 的错误统计统一处理
            }
        }

        // --------------------------------------------------------------------
        // 14. 安全层 step：限斜率 + AB 分组 + 实际下发 + 心跳 ACK
        // --------------------------------------------------------------------
        //
        // 注意：pwm_.step() 内部包含 ACK 接收与错误处理；
        //       等价于 C 层的 pwm_host_poll(1)，即 ~1 ms 的阻塞等待。
        //
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
                std::cerr << "[ControlLoop] pwm_.step() errors exceed "
                          << "max_step_errors (" << cfg_.max_step_errors
                          << "), aborting.\n";
                return -30;
            }
        } else {
            // 有一次成功就清零错误计数
            step_err_count = 0;

            // ----------------------------------------------------------------
            // 15. 成功下发后记录一条 PWM 日志（与控制环锁步）
            // ----------------------------------------------------------------
            if (pwm_logger_.is_open()) {
                const double t_s = duration_d(now - start_time_).count();

                // 当前阶段：applied 先用 cmd 占位，等接入 PwmClient::getLastApplied() 后替换
                std::array<float, 8> cmd{};
                std::array<float, 8> applied{};

                if (output_.has_thruster_command) {
                    cmd     = output_.thruster_command;
                    applied = cmd;
                } else {
                    // 无推进器命令时，记录为 0；后续可根据需要改为“上一次输出”
                    cmd.fill(0.0f);
                    applied.fill(0.0f);
                }

                pwm_logger_.logCmdAndApplied(t_s, cmd, applied);
            }
        }
    } // while (true)

    std::cout << "[ControlLoop] loop exited normally.\n";
    return 0;
}

} // namespace rovctrl::control_core
