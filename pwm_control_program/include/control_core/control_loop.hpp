// include/control_core/control_loop.hpp
#pragma once

#include <atomic>
#include <memory>

#include "control_core/control_types.hpp"
#include "platform/pwm_client.hpp"
#include "io/input_provider.hpp"
#include "controllers/controller_base.hpp"

namespace rovctrl::control_core {

/**
 * @brief 控制主循环
 *
 * 职责：
 *   - 以固定频率（loop_hz）运行控制周期；
 *   - 每个周期：
 *       1) 从 InputProvider 读取/更新 ControlState、ControlReference；
 *       2) 调用 Controller 计算 ControlOutput；
 *       3) 将输出映射到 PwmClient，并调用 pwm_client.step() 完成安全层下发；
 *   - 处理退出条件与 step 错误计数，避免日志刷屏。
 *
 * 注意：
 *   - 仅负责时间调度与模块串联，不关心具体控制算法和推力分配细节；
 *   - 推力分配可以在 Controller 内部完成，也可以在将来引入 allocation 层后在实现中调用。
 */
class ControlLoop {
public:
    struct Config {
        double loop_hz = 100.0;   ///< 主循环频率（Hz）

        int    max_step_errors          = 1000; ///< 允许的最大连续 step 错误次数，超过则强制退出
        int    step_error_log_interval  = 100;  ///< 每 N 次错误打印一次日志（防止刷屏）
        bool   log_timing               = false;///< 是否打印循环周期/耗时等调试信息（实现中可选用）
    };

    using ControllerPtr = std::shared_ptr<rovctrl::controllers::ControllerBase>;

    /**
     * @brief 构造控制循环
     *
     * @param cfg         控制循环配置（频率 / 错误处理策略）
     * @param pwm         PWM 客户端引用（由外部管理生命周期）
     * @param input       输入提供者（键盘 / 上位机 / 自动轨迹等）
     * @param controller  控制器（PID / MPC / SMC 等）
     * @param external_stop_flag 可选的外部停止标志指针：
     *                           - 若非空，将在每次循环检查 *external_stop_flag；
     *                           - 常用于响应 Ctrl+C 信号或上层 App 的退出请求。
     */
    ControlLoop(const Config& cfg,
                rovctrl::platform::PwmClient& pwm,
                rovctrl::io::InputProviderPtr input,
                ControllerPtr controller,
                std::atomic_bool* external_stop_flag = nullptr);

    /**
     * @brief 运行主循环（阻塞，直到退出）
     *
     * 退出条件：
     *   - external_stop_flag 非空且被置为 true；
     *   - Controller 或 InputProvider 返回致命错误（由实现判断）；
     *   - pwm_client.step() 连续错误次数超过 cfg.max_step_errors；
     *
     * @return 0 正常退出；非 0 表示发生错误或异常退出。
     */
    int run();

    /// 访问配置（只读）
    const Config& config() const noexcept { return cfg_; }

private:
    Config                        cfg_;
    rovctrl::platform::PwmClient& pwm_;
    rovctrl::io::InputProviderPtr input_;
    ControllerPtr                 controller_;
    std::atomic_bool*             external_stop_;  ///< 不拥有，只引用

    // 循环内部状态缓冲，避免每次分配
    ControlState      state_{};
    ControlReference  ref_{};
    ControlOutput     output_{};
};

} // namespace rovctrl::control_core
