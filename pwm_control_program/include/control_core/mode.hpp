#pragma once

#include <string>
#include <string_view>

namespace rovctrl {

// 前向声明其他模块的核心类型，避免在头文件里产生重型依赖
namespace platform {
class PwmClient;
}

namespace io {
class IInputProvider;
}

namespace controllers {
class IController;
}

namespace allocation {
class ThrustAllocator;
}

namespace control_core {

/**
 * @brief 模式运行上下文
 *
 * 说明：
 *   - 提供模式访问“系统资源”的入口（pwm 客户端、输入源、控制器、推力分配器等）；
 *   - 为了减少耦合，这里只持有指针，不负责资源的创建和生命周期管理；
 *   - 哪些指针必须非空，由具体项目约定（你可以在构造时做断言）。
 */
struct ModeContext {
    platform::PwmClient*      pwm         = nullptr;  ///< PWM 客户端（底层安全层封装）
    io::IInputProvider*       input       = nullptr;  ///< 当前输入源（键盘 / 上位机 / 自动脚本）
    controllers::IController* controller  = nullptr;  ///< 当前控制器（PID / MPC / SMC 等）
    allocation::ThrustAllocator* allocator = nullptr; ///< 推力分配器（DOF → 8 路推进器）

    double loop_hz = 100.0;                           ///< 主循环频率（Hz），供模式内部参考
};

/**
 * @brief 模式更新结果：用于驱动主循环的控制流
 *
 * 使用约定：
 *   - request_exit = true:
 *       表示当前模式希望整个程序退出（例如用户按 ESC 或发生致命错误）；
 *   - switch_mode = true 且 next_mode_id 非空：
 *       表示当前模式希望切换到另一个模式，由外部 ModeManager 或上层逻辑执行切换。
 */
struct ModeUpdateResult {
    bool        request_exit = false;
    bool        switch_mode  = false;
    std::string next_mode_id;    ///< 目标模式 ID（例如 "teleop" / "pid_demo" / "auto_depth"）
};

/**
 * @brief 控制模式抽象基类
 *
 * 设计目的：
 *   - 把“模式相关逻辑”（初始化、每周期控制、模式切换意图）从 main / 控制循环中解耦；
 *   - 每种具体模式（TeleopMode、PidDemoMode、AutoDepthMode...）都实现这个接口；
 *   - 控制循环只关心：当前 Mode 是谁、每次 tick 调用 update() 得到 ModeUpdateResult。
 *
 * 约定：
 *   - 一个 Mode 在生命周期内绑定一个 ModeContext；
 *   - 不可拷贝，仅以指针或智能指针形式在上层（ModeManager / ControlLoop）中管理；
 *   - 不要在 update() 里做长时间阻塞操作，由上层调度线程和 IO。
 */
class Mode {
public:
    explicit Mode(ModeContext& ctx) : ctx_(ctx) {}
    virtual ~Mode() = default;

    Mode(const Mode&)            = delete;
    Mode& operator=(const Mode&) = delete;

    /**
     * @brief 模式 ID，用于内部识别与切换
     *
     * 要求：
     *   - 短小、稳定、无空格，例如："teleop" / "pid_demo" / "auto_depth"；
     *   - 上层可以用它作为 key，在一个 map<string, unique_ptr<Mode>> 中查找模式。
     */
    virtual std::string_view id() const = 0;

    /**
     * @brief 模式显示名称，用于日志输出和人机界面
     *
     * 默认返回 id()，如需更友好的名称可以在子类中重载。
     */
    virtual std::string_view display_name() const { return id(); }

    /**
     * @brief 进入模式时调用
     *
     * 调用时机：
     *   - 当前模式被激活之前（第一次 update() 之前）；
     *   - 或从其他模式切换到本模式时。
     *
     * 典型用途：
     *   - 重置内部状态（PID 积分、滤波器状态）；
     *   - 将电机推至安全中位 / 清空 teleop 输入；
     *   - 打印模式说明 / 启动相关日志记录。
     */
    virtual void on_enter() {}

    /**
     * @brief 离开模式时调用
     *
     * 调用时机：
     *   - 模式被切换到其他模式之前；
     *   - 程序退出前，控制循环停止前。
     *
     * 典型用途：
     *   - 将 DOF 收敛到安全状态（例如 thrust → 0）；
     *   - 输出统计信息 / 停止日志记录。
     */
    virtual void on_exit() {}

    /**
     * @brief 每个控制周期被调用一次（核心控制逻辑）
     *
     * @param dt 距离上一次 update 的时间（秒），由控制循环计算并传入
     *
     * 返回值：
     *   - ModeUpdateResult，用于告诉上层当前模式是否希望切换或退出；
     *   - 如果不需要切换或退出，可以返回默认构造的 ModeUpdateResult。
     *
     * 典型流程：
     *   1. 通过 ctx().input 读取输入（teleop，将来可以是期望轨迹/上位机指令）；
     *   2. 通过 ctx().controller 计算 DOF 控制量；
     *   3. 通过 ctx().allocator 生成各推进器 thrust / duty；
     *   4. 通过 ctx().pwm 或 pwm_control 安全层完成占空比下发；
     *   5. 根据内部条件决定是否请求切换模式/退出。
     */
    virtual ModeUpdateResult update(double dt) = 0;

protected:
    /// 访问模式上下文（仅供子类使用）
    ModeContext&       ctx()       { return ctx_; }
    const ModeContext& ctx() const { return ctx_; }

private:
    ModeContext& ctx_;
};

} // namespace control_core
} // namespace rovctrl
