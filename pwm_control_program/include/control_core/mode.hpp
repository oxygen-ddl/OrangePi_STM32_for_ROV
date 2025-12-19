#pragma once

#include <string>
#include <string_view>

namespace rovctrl::platform {
class PwmClient;
}

namespace rovctrl::io {
class IInputProvider;
}

namespace rovctrl::controllers {
// 这里只做前向声明，不在 mode.hpp 里定义 IController 的细节
class IController;
}

namespace rovctrl::allocation {
class ThrustAllocator;
}

namespace rovctrl::control_core {

/**
 * @brief 模式运行上下文
 *
 * 说明：
 *   - 提供模式访问“系统资源”的入口（pwm 客户端、输入源、控制器、推力分配器等）；
 *   - 为了减少耦合，这里只持有指针，不负责资源的创建和生命周期管理；
 *   - 哪些指针必须非空，由具体项目约定（你可以在构造时做断言）。
 */
struct ModeContext {
    rovctrl::platform::PwmClient*          pwm        = nullptr;  ///< PWM 客户端（底层安全层封装）
    rovctrl::io::IInputProvider*           input      = nullptr;  ///< 当前输入源（键盘 / 上位机 / 自动脚本）
    rovctrl::controllers::IController*     controller = nullptr;  ///< 当前控制器（PID / MPC / SMC 等）
    rovctrl::allocation::ThrustAllocator*  allocator  = nullptr;  ///< 推力分配器（DOF → 8 路推进器）

    double loop_hz = 100.0;                                      ///< 主循环频率（Hz），供模式内部参考
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

    /// 模式 ID，用于内部识别与切换
    virtual std::string_view id() const = 0;

    /// 模式显示名称，用于日志输出和人机界面（默认返回 id()）
    virtual std::string_view display_name() const { return id(); }

    /// 进入模式时调用
    virtual void on_enter() {}

    /// 离开模式时调用
    virtual void on_exit() {}

    /// 每个控制周期被调用一次（核心控制逻辑）
    /// @param dt 距离上一次 update 的时间（秒）
    virtual ModeUpdateResult update(double dt) = 0;

protected:
    /// 访问模式上下文（仅供子类使用）
    ModeContext&       ctx()       { return ctx_; }
    const ModeContext& ctx() const { return ctx_; }

private:
    ModeContext& ctx_;
};

} // namespace rovctrl::control_core
