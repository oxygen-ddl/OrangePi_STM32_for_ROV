#pragma once
#ifndef ROVCTRL_IO_INPUT_PROVIDER_HPP
#define ROVCTRL_IO_INPUT_PROVIDER_HPP

/**
 * @file    input_provider.hpp
 * @brief   控制栈的“输入源”抽象接口
 *
 * 设计目标：
 *   - 给控制主循环提供一个统一的输入入口，不关心具体来源是键盘、
 *     上位机、自动轨迹还是脚本；
 *   - 对上层暴露：当前控制模式 / 参考值 / 手动输出 / 退出请求；
 *   - 对下层可选地注入导航状态（ControlState），便于做更复杂的逻辑。
 *
 * 典型实现：
 *   - TeleopInput（键盘）：从终端读键 → 维护一个手动 ControlOutput，
 *     current_mode() = MANUAL；
 *   - AutoScriptInput（自动脚本）：根据时间生成 ControlReference，
 *     current_mode() = PID_POSITION / MPC 等；
 *   - 未来：RemoteCommandInput（网络指令）等。
 */

#include <cstdint>
#include <memory>

#include "control_core/control_types.hpp"

namespace rovctrl::io {

namespace cc = rovctrl::control_core;

/**
 * @brief 输入源接口：控制循环每个周期调用 update() 拉取一次输入
 */
class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    /**
     * @brief 更新上下文：控制循环可以把“当前时间 / 当前导航状态”传进来
     *
     * 可以为空指针（nav_state == nullptr），此时实现类只基于自身状态工作。
     * 以后你要做“基于当前位置自动生成下一个参考点”时，这个上下文就会有用。
     */
    struct UpdateContext {
        std::int64_t              t_ns{0};         ///< 当前循环时间（ns），可选
        const cc::ControlState*   nav_state{nullptr}; ///< 当前导航状态（可选）
    };

    /**
     * @brief 拉取一次输入（由控制循环每个周期调用）
     *
     * @param ctx  上下文信息（当前时间 + 导航状态），可以只填时间或全空
     *
     * 说明：
     *   - Teleop 实现里，通常会在这里读键盘 / 处理缓冲区；
     *   - 自动脚本实现里，可以用 t_ns 推进参考轨迹；
     *   - 实现应内部维护当前 ControlMode / Reference / ManualOutput。
     */
    virtual void update(const UpdateContext& ctx) = 0;

    /**
     * @brief 当前控制模式（手动 / PID / MPC / SMC …）
     *
     * 控制主循环可根据该模式决定：
     *   - MANUAL：使用 manual_output() 直接生成 PWM；
     *   - PID/MPC：使用 reference() + 控制器计算输出。
     */
    virtual cc::ControlMode current_mode() const = 0;

    /**
     * @brief 当前控制参考值（位置/姿态/速度/深度）
     *
     * 在 MANUAL 模式下，部分实现可以返回“全 0 + enable_* = false”的参考，
     * 控制循环只使用 manual_output()。
     */
    virtual const cc::ControlReference& reference() const = 0;

    /**
     * @brief 当前手动输出（DOF 指令），主要在 MANUAL 模式下使用
     *
     * 典型映射：
     *   - surge_cmd / sway_cmd / heave_cmd / yaw_cmd ∈ [-1, 1]
     *   - 可直接映射到推力分配模块。
     *
     * 约定：
     *   - 若 current_mode() != MANUAL，则 valid 字段可以为 false，
     *     控制循环不使用该输出。
     */
    virtual const cc::ControlOutput& manual_output() const = 0;

    /**
     * @brief 是否请求退出（例如 Teleop ESC）
     *
     * 控制主循环可据此决定跳出 while 循环，执行停机流程。
     */
    virtual bool exit_requested() const = 0;

    /**
     * @brief 重置内部状态（例如切换模式 / 恢复安全状态时调用）
     *
     * 建议行为：
     *   - 清零手动输出 DOF；
     *   - 清空参考值 / 标志位；
     *   - 清除退出请求标志。
     */
    virtual void reset() = 0;
};

using InputProviderPtr = std::shared_ptr<IInputProvider>;

} // namespace rovctrl::io

#endif // ROVCTRL_IO_INPUT_PROVIDER_HPP
