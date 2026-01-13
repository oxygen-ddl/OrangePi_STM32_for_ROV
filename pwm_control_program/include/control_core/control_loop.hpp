#pragma once
#ifndef ROVCTRL_CONTROL_CORE_CONTROL_LOOP_HPP
#define ROVCTRL_CONTROL_CORE_CONTROL_LOOP_HPP

/**
 * @file   control_loop.hpp
 * @brief  顶层控制循环（定时执行：读取输入 → Guard 仲裁 → 计算控制 → 推力分配 → 下发 PWM）。
 *
 * 设计原则：
 *  - 头文件只暴露“控制循环的公共接口与必要类型”；
 *  - 导航共享内存 / 消息类型通过 PIMPL 隐藏在 .cpp 内部，避免上层耦合实现细节；
 *  - IO / 日志 / 具体算法逻辑尽量留在 .cpp，便于后续独立演进（换控制器、换日志后端等）。
 *
 * 新人阅读提示（整体逻辑）：
 *  - ControlLoop 是「控制主循环」的统一调度入口，只做“编排”，不做具体控制算法；
 *  - 输入来自 IInputProvider（例如：Intent SHM → 键盘 / GCS / 自动控制）；
 *  - ControlGuard 负责安全与仲裁（急停、Arm/Disarm、模式切换、输入 TTL 等）；
 *  - ControllerManager 负责在当前模式下计算期望的控制输出（力 / 力矩等）；
 *  - ThrusterAllocator 负责把控制输出映射到 8 路推进器指令；
 *  - PwmClient 负责把推进器指令以安全的方式下发到 STM32。
 *
 * 换句话说：ControlLoop =「把各个模块串起来，让系统按固定频率安全地工作」。
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "control_core/control_guard.hpp"
#include "control_core/control_intent.hpp"
#include "control_core/thruster_allocation.hpp"
#include "controllers/controller_manager.hpp"
#include "io/input/input_provider.hpp"   // rovctrl::io::InputProviderPtr
#include "platform/pwm_client.hpp"
#include "io/nav/nav_state_view.hpp"     // rovctrl::io::NavStateView（仅在私有函数签名中使用）

namespace shared::msg {
struct NavState; // 仅前向声明；具体定义在 nav_core 中
}

namespace rovctrl::control_core {

/**
 * @brief 控制主循环配置
 *
 * 新人可以把它理解为“上电前的控制参数面板”：
 *  - loop_hz：主循环频率；
 *  - guard_cfg：安全守护策略（急停、超时、模式切换规则等）；
 *  - thruster_alloc：8 通道推进器几何布局、权重等；
 *  - no_input_neutral_ms：多久没收到有效输入就自动归中。
 */
class ControlLoop {
public:
    struct Config {
        double loop_hz = 100.0;   ///< 控制主循环频率（Hz）

        // --- 运行健壮性（log / watchdog） ---
        int  max_step_errors         = 1000;  ///< pwm_.step 连续错误上限（>0 生效）
        int  step_error_log_interval = 100;   ///< 每多少次 step 错误打印一次日志
        bool log_timing              = false; ///< 如需打印每周期 dt / jitter，可开启
        bool enable_pwm_log          = true;  ///< 是否记录 PWM 日志（Cmd / Applied）

        // dt 限幅（防止系统卡顿时 dt 过大导致积分爆炸）
        double dt_clamp_max_sec = 0.2;

        // --- 安全 & 仲裁（由 ControlGuardConfig 描述策略） ---
        ControlGuardConfig guard_cfg{};

        bool allow_run_without_nav              = true; ///< 手动模式可容忍无导航；自动模式可按策略关闭
        bool enter_failsafe_on_controller_error = true; ///< 控制器计算失败时是否直接进入 failsafe

        // --- 推力分配配置（推进器几何） ---
        ThrusterAllocationConfig thruster_alloc{};

        /**
         * @brief 输入源长期无更新时的“自动归中时间”（毫秒）
         *
         * 行为说明：
         *  - 若在该时间内没有有效 ControlIntent（无遥控 DOF、无模式切换等），
         *    则 ControlLoop 主动把推进器命令归为 0（neutral），并跳过控制器计算；
         *  - 0 表示“使用 run() 内部默认值”（例如 200ms）。
         */
        std::uint32_t no_input_neutral_ms = 0;
    };

    // ================= PWM 日志后端接口 =================
    /**
     * @brief 抽象的 PWM 日志输出接口
     *
     * 设计目的：
     *  - 让 ControlLoop 不直接依赖 <fstream> 等具体 IO；
     *  - 由 .cpp 提供具体实现（例如 CSV / 二进制日志），也便于日后更换实现。
     */
    struct PwmLog {
        enum class Mode : std::uint8_t {
            AppliedOnly,    ///< 只记录实际下发的 PWM
            CmdAndApplied   ///< 同时记录“控制器输出的 cmd”和“安全层之后的 applied”
        };

        virtual ~PwmLog() = default;

        virtual bool init(const std::string& root_dir,
                          Mode               mode,
                          const std::string& prefix) = 0;

        virtual bool is_open() const noexcept = 0;

        virtual void logApplied(double t_s,
                                const std::array<float, 8>& applied) = 0;

        virtual void logCmdAndApplied(double t_s,
                                      const std::array<float, 8>& cmd,
                                      const std::array<float, 8>& applied) = 0;

        virtual void close() noexcept = 0;
    };

public:
    /**
     * @brief 构造控制循环
     *
     * @param cfg                控制循环配置（频率 / Guard / 分配器 / 日志等）
     * @param pwm                底层 PWM 客户端，负责真正“写入”到 STM32 / libpwm_host
     * @param input              输入提供者（键盘 / GCS / 自动算法），统一输出 ControlIntent
     * @param ctrl_mgr           控制器管理器（Manual / Auto / MPC / RL 等）
     * @param external_stop_flag 外部停止标志（如主进程收到 Ctrl+C 后置 true）
     *
     * 整体调用关系（简略）：
     *  main()
     *    -> 创建 PwmClient / InputProvider / ControllerManager
     *    -> 构造 ControlLoop
     *    -> loop.run();
     */
    ControlLoop(const Config&                  cfg,
                rovctrl::platform::PwmClient&  pwm,
                rovctrl::io::InputProviderPtr  input,
                ControllerManager&&            ctrl_mgr,
                std::atomic_bool*              external_stop_flag = nullptr);

    // NOTE: 定义在 .cpp 中（NavSub 为不完整类型）
    ~ControlLoop() noexcept;

    ControlLoop(const ControlLoop&)            = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    // 当前保守禁用 move，后续如有需要可放开
    ControlLoop(ControlLoop&&)                 = delete;
    ControlLoop& operator=(ControlLoop&&)      = delete;
    // 建议若要启用：
    // ControlLoop(ControlLoop&&) noexcept = default;
    // ControlLoop& operator=(ControlLoop&&) noexcept = default;

    /**
     * @brief 启动控制主循环（阻塞调用）
     *
     * 主循环内部典型流程（每个周期）：
     *  1. 固定周期调度（根据 loop_hz 计算 sleep_until）；
     *  2. 从导航共享内存读取最新 NavStateView（若可用）；
     *  3. 调用 input_->poll(state_, intent_) 获取最新 ControlIntent；
     *  4. 将 intent_ 交给 ControlGuard 进行安全仲裁，得到 GuardResult；
     *  5. 根据 GuardResult 构造 ControlReference（位置 / 姿态 / 速度等参考）；
     *  6. 调用 ControllerManager 计算控制输出 ControlOutput（如 body wrench）；
     *  7. 调用 ThrusterAllocator 将控制量映射为 8 通道推进器命令；
     *  8. 根据 intent_ 中的 MotorTestCmd（如有）进行单电机测试覆盖；
     *  9. 通过 PwmClient 下发 PWM，并按需记录日志；
     * 10. 处理 failsafe / exit / 外部停止标志等退出条件。
     *
     * @return 0 正常退出；负数表示不同错误码（见 .cpp 中实现）
     */
    int run();

    const Config& config() const noexcept { return cfg_; }

private:
    // =============== 配置与依赖对象 ===============
    Config                         cfg_{};
    rovctrl::platform::PwmClient&  pwm_;         ///< 底层 PWM 客户端（引用，由外部持有）
    rovctrl::io::InputProviderPtr  input_;       ///< 输入源（键盘 / GCS / 自动控制）
    ControllerManager              ctrl_mgr_;    ///< 控制器管理（当前模式 + 控制算法）
    std::atomic_bool*              external_stop_{nullptr}; ///< 外部停止标志（可为空）

    // =============== Guard（安全 / 仲裁）==============
    ControlGuard guard_;                         ///< 统一处理 estop / arm / ttl / 模式切换等

    // =============== 控制状态与 I/O 缓存 ===============
    ControlState  state_{};        ///< 当前估计状态（时间戳、位姿、速度等）
    ControlIntent intent_{};       ///< 原始输入（由 input_ 提供）
    GuardResult   guard_result_{}; ///< Guard 仲裁后的结果（effective_intent / 模式等）

    ControlReference ref_{};       ///< 控制器使用的参考量（setpoint）
    ControlOutput    output_{};    ///< 控制器输出（如 body wrench / 期望推力）

    // =============== 计时 ===============
    std::chrono::steady_clock::time_point start_time_{};

    // =============== 导航状态反馈（PIMPL） ===============
    /**
     * @brief NavSub：导航订阅子模块
     *
     * 职责：
     *  - 订阅来自 nav_core 的 NavState / NavStateView（通常通过 SHM）；
     *  - 提供线程安全的“最新快照”给控制循环；
     *  - 屏蔽具体 SHM 或其它 IPC 细节。
     */
    struct NavSub;
    struct NavSubDeleter {
        void operator()(NavSub*) noexcept;
    };
    std::unique_ptr<NavSub, NavSubDeleter> nav_sub_;

    bool last_nav_valid_{false};   ///< 上一周期是否成功获得有效导航数据（用于告警与状态统计）

    // =============== PWM 日志（PIMPL，避免头文件引入 <fstream>） ===============
    std::unique_ptr<PwmLog> pwm_logger_;

    // =============== 推力分配器 ===============
    ThrusterAllocator allocator_;

private:
    // ================== 私有步骤函数（实现均在 .cpp） ==================

    /**
     * @brief 从 NavSub 中读取最新 NavStateView，并更新 state_
     * @param nav_view_out 输出：本周期使用的导航快照（含 pub_mono_ns 等）
     * @return true 表示本周期有可用导航数据；false 表示缺失（可能刚启动或 nav_core 未运行）
     */
    bool update_nav_feedback_(rovctrl::io::NavStateView& nav_view_out);

    /**
     * @brief 根据 GuardResult（effective_intent / 模式）构造 ControlReference
     *
     * 典型行为：
     *  - Manual/Teleop：将 teleop DOF 转换为速度 / 推力参考；
     *  - Auto：根据任务轨迹 / setpoint 生成位置/姿态参考；
     *  - Failsafe：参考量保持或渐近归零。
     */
    void build_reference_from_guard_();

    /**
     * @brief 根据当前 ControlOutput / 模式 / 分配策略，构建 8 通道推进器命令
     * @param thr_out 输出：每个推进器的归一化命令（-1..+1 或工程自定义）
     * @return true 成功；false 表示当前无法生成有效命令（例如分配器状态异常）
     */
    bool build_thruster_command_(ThrusterArray& thr_out);

    /**
     * @brief 执行 failsafe 策略（例如 ZeroOutput / EmergencyStop）
     *
     * 一般流程：
     *  - 根据 FailsafeAction 决定是否直接设定所有推进器为 0 或进入急停；
     *  - 调用 pwm_ 下发安全指令；
     *  - 按需记录日志。
     */
    void execute_failsafe_(FailsafeAction a);

    /**
     * @brief 单电机测试覆盖逻辑
     *
     * 使用场景：
     *  - 上位机 / GCS 通过 Intent 中的 MotorTestCmd 请求对单个推进器做点动测试；
     *  - 在正常控制命令计算完成之后调用本函数，可在必要时覆盖 thr_cmd 中单通道输出；
     *  - 需要在实现中显式约束测试持续时间、幅值上限，避免误操作导致危险。
     *
     * @param intent   Guard 后生效的 ControlIntent（包含 has_motor_test + motor_test）
     * @param thr_cmd  输入/输出：当前计算好的 8 通道命令，可能被单通道测试覆盖
     */
    void apply_motor_test_override(
                const ControlIntent& intent,
                ThrusterArray& thr_cmd) noexcept;

    /**
     * @brief 构造 PWM 日志实现（由 .cpp 中提供具体类型）
     *
     * 好处：
     *  - ControlLoop 不关心日志实现细节；
     *  - 不在头文件中引入 <fstream> 等重依赖；
     *  - 便于针对 PC / 嵌入式分别实现不同的日志策略。
     */
    std::unique_ptr<PwmLog> make_pwm_logger_();

    /**
     * @brief 获取当前 monotonic time（纳秒）
     *
     * 说明：
     *  - 基于平台封装（platform::timebase），与导航/日志统一时间基；
     *  - 用于 Guard ttl 判断、导航 age 估计等。
     */
    std::uint64_t now_mono_ns_() const;
};

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_CONTROL_LOOP_HPP
