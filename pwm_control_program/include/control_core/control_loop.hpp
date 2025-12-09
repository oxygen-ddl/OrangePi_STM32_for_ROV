#pragma once
/**
 * @file   control_loop.hpp
 * @brief  顶层控制循环（定时执行：读取输入 → 计算控制 → 推力分配 → 下发 PWM）。
 *
 * 该类负责：
 *  - 驱动主循环（固定频率，如 100 Hz）；
 *  - 组织输入来源（键盘遥控 / 程序输入等）；
 *  - 调用具体控制器（手动、PID、后续 MPC 等）；
 *  - 调用推力分配器 ThrusterAllocator，将 body wrench 映射为 8 路归一化指令；
 *  - 调用 PwmClient 下发控制指令；
 *  - 记录必要日志（PWM 等）；
 *  - 从导航进程订阅 NavState（共享内存），缓存为反馈状态。
 */

#include <atomic>
#include <chrono>
#include <memory>

#include "control_core/control_types.hpp"
#include "controllers/controller_base.hpp"
#include "control_core/thruster_allocation.hpp"

#include "platform/pwm_client.hpp"
#include "io/input_provider.hpp"
#include "io/pwm_logger.hpp"
#include "io/nav_state_subscriber.hpp"   // 导航状态订阅器

namespace rovctrl::control_core {

/**
 * @class ControlLoop
 * @brief 控制主循环：按固定频率执行控制流程。
 *
 * 典型调用方式：
 * @code
 *   rovctrl::platform::PwmClient pwm;
 *   auto input      = create_input_provider(...);
 *   auto controller = create_controller(...);
 *
 *   ControlLoop::Config cfg;
 *   cfg.loop_hz = 100.0;
 *
 *   ControlLoop loop(cfg, pwm, input, controller);
 *   return loop.run();
 * @endcode
 */
class ControlLoop {
public:
    /**
     * @brief 控制循环配置。
     */
    struct Config {
        double loop_hz = 100.0;   ///< 控制循环频率 [Hz]

        int  max_step_errors         = 1000; ///< 允许的最大“步长误差”次数
        int  step_error_log_interval = 100;  ///< 步长误差日志输出间隔
        bool log_timing              = false;///< 是否记录周期 timing 统计
        bool enable_pwm_log          = true; ///< 是否记录 PWM 日志

        /// 推力分配相关配置（由 YAML 解析填充）
        ThrusterAllocationConfig thruster_alloc{};
    };

    using ControllerPtr = std::shared_ptr<rovctrl::controllers::IController>;

    /**
     * @brief 构造控制循环。
     *
     * @param cfg                循环配置（频率、日志配置、推力分配配置等）
     * @param pwm                已初始化好的 PwmClient（底层通信）
     * @param input              输入源（键盘、程序自动输入等）
     * @param controller         具体控制器实现（manual / PID / MPC 等）
     * @param external_stop_flag 外部停止标志（可选，nullptr 则忽略）
     */
    ControlLoop(const Config& cfg,
                rovctrl::platform::PwmClient& pwm,
                rovctrl::io::InputProviderPtr input,
                ControllerPtr controller,
                std::atomic_bool* external_stop_flag = nullptr);

    /**
     * @brief 运行主控制循环（阻塞直到退出）。
     *
     * 返回值约定：
     *  - 0：正常退出；
     *  - 非 0：错误或异常退出。
     */
    int run();

    const Config& config() const noexcept { return cfg_; }

private:
    // =============== 配置与依赖对象 ===============

    Config                        cfg_;
    rovctrl::platform::PwmClient& pwm_;
    rovctrl::io::InputProviderPtr input_;
    ControllerPtr                 controller_;
    std::atomic_bool*             external_stop_{nullptr};

    // =============== 控制状态与 I/O 缓存 ===============

    ControlState     state_{};   ///< 当前控制状态（可在控制器之间共享）
    ControlReference ref_{};     ///< 当前参考输入（由 InputProvider 提供）
    ControlOutput    output_{};  ///< 控制器输出（body wrench / thruster_cmd 等）

    // =============== PWM 日志与计时 ===============

    rovctrl::io::PwmLogger                pwm_logger_;
    std::chrono::steady_clock::time_point start_time_{};

    // =============== 导航状态反馈 ===============

    /// 导航状态订阅器：从 /rov_nav_state_v1 共享内存读取 NavState。
    rovctrl::io::NavStateSubscriber nav_sub_;

    /// 最近一次成功读取的导航状态。
    shared::msg::NavState last_nav_state_{};

    /// 最近一周期是否获得了有效导航状态。
    bool                  last_nav_valid_{false};

    // =============== 推力分配器 ===============

    /// 将 body_wrench[6] → 8 路归一化指令的分配器
    ThrusterAllocator allocator_;
};

} // namespace rovctrl::control_core
