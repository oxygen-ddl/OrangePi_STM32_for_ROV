#pragma once
#ifndef ROVCTRL_CONTROL_CORE_APP_CONTEXT_HPP
#define ROVCTRL_CONTROL_CORE_APP_CONTEXT_HPP

#include <atomic>
#include <memory>
#include <ostream>
#include <string>

#include "control_core/control_loop.hpp"
#include "control_core/thruster_allocation.hpp"

#include "controllers/controller_manager.hpp"

// Input providers
#include "io/input/input_provider.hpp"

// PwmClient
#include "platform/pwm_client.hpp"

namespace rovctrl::control_core {

/**
 * @brief app_main 的构建选项（来自 CLI）
 *
 * 目标：
 *  - 把 app_main.cpp 中“组装应用”的大量细节迁移出去；
 *  - app_main.cpp 只负责：解析参数 -> build_context -> run -> shutdown。
 */
struct AppBuildOptions {
    // loop / pwm
    double loop_hz      = 100.0;
    double pwm_ctrl_hz  = 100.0;
    double max_step_pct = 0.2;

    // config paths (CLI raw)
    std::string pwm_config_cli;
    std::string control_config_cli;
    std::string traj_config_cli;
    std::string alloc_config_cli;

    // inputs
    bool enable_gcs    = true;
    bool enable_teleop = true;

    // PWM dummy backend (方式2)
    bool pwm_dummy       = false;
    bool pwm_dummy_print = false;

    // gcs listen port (保持与旧行为一致：暂时硬编码 14600，但保留成可调项)
    int gcs_bind_port = 14600;
    int gcs_ttl_ms    = 200;
};

/**
 * @brief build_app_context 返回结果
 */
struct AppBuildResult {
    bool        ok       = false;
    int         err_code = 0;
    std::string err_msg;
};

/**
 * @brief 应用运行上下文：由 build_app_context() 构建，供 app_main() 使用。
 *
 * 注意：这里把关键对象“放在一个地方”，便于：
 *  - 统一初始化/失败清理；
 *  - 将来做单元测试（mock PwmClient / InputProvider）；
 *  - 将 app_main.cpp 变薄。
 */
struct AppContext {
    // Assembled modules
    platform::PwmClient          pwm_client;
    rovctrl::io::InputProviderPtr input;

    // Controller manager (后续会 move 给 ControlLoop)
    ControllerManager ctrl_mgr{ControllerManagerOptions{}};

    // ControlLoop config (已填充 alloc 等)
    ControlLoop::Config loop_cfg{};

    // Optional: stop flag pointer (来自 app_main)
    std::atomic_bool* stop_flag = nullptr;

    /**
     * @brief 安全关闭：尽量归中位 + shutdown()
     * @param log 日志输出
     * @param estop_seconds 期望收敛时间（秒）
     */
    void shutdown(std::ostream& log, float estop_seconds = 1.0f);
};

/**
 * @brief 构建完整 AppContext（包含：PWM、alloc、input、controller、loop_cfg）
 *
 * @param opt   CLI 选项
 * @param argv0 argv[0]（用于 resolve_config_path）
 * @param stop_flag 退出标志
 * @param out_ctx 输出上下文
 * @param log   日志输出
 */
AppBuildResult build_app_context(const AppBuildOptions& opt,
                                 const char*            argv0,
                                 std::atomic_bool*      stop_flag,
                                 AppContext&            out_ctx,
                                 std::ostream&          log);

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_APP_CONTEXT_HPP
