#ifndef ROVCTRL_CONTROLLERS_CONTROLLER_BASE_HPP
#define ROVCTRL_CONTROLLERS_CONTROLLER_BASE_HPP

/**
 * @file    controller_base.hpp
 * @brief   控制器抽象基类接口（PID / MPC / SMC / 手动控制等的统一入口）
 *
 * 设计目标：
 *   - 为所有控制算法（PID, MPC, SMC, RL ...）提供统一接口；
 *   - 隔离控制算法与平台/执行细节（PWM、安全层、线程、日志）；
 *   - 使控制主循环能够无差别调度任意控制器；
 *   - 便于模式切换与日志系统。
 *
 * 约束：
 *   - 不抛异常（noexcept）；
 *   - compute() 必须可用于实时周期调用；
 *   - 控制器只负责“算输出”，不负责推力分配/下发 PWM。
 */

#include <cstdint>
#include <string>

#include "control_core/control_mode.hpp"   // rovctrl::control_core::ControlMode
#include "control_core/control_types.hpp"  // ControlState / ControlReference / ControlOutput

namespace rovctrl::controllers {

/**
 * @brief 控制器抽象基类
 *
 * 所有控制器必须实现以下特性：
 *   - 提供控制器标识名称；
 *   - 定义其所属的 ControlMode（Manual / Auto / Failsafe 等）；
 *   - 接受当前状态 + 参考输入，计算控制输出；
 *   - 支持 reset() 清空内部状态（积分项等）。
 */
class IController {
public:
    virtual ~IController() = default;

    /// @brief 控制器名称（用于日志/调试/UI）
    virtual const char* name() const noexcept = 0;

    /// @brief 控制器所处的逻辑控制模式（Manual / Auto / Failsafe）
    virtual rovctrl::control_core::ControlMode mode() const noexcept = 0;

    /// @brief 重置控制器内部状态（积分项、滤波器、优化器 warm-start 等）
    virtual void reset() noexcept = 0;

    /**
     * @brief 控制计算主入口
     *
     * @param state   当前状态（来自导航/传感器融合）
     * @param ref     当前参考输入（来自 teleop / trajectory / 上位机）
     * @param output  输出结构（6-DOF wrench 或 8 路推进器指令）
     * @param dt_sec  控制周期（秒）。约定：dt_sec > 0；上层应做 clamp，避免过大 dt。
     *
     * @return true  ：本周期计算成功
     * @return false ：输出无效，上层可选择忽略或进入保护模式
     */
    virtual bool compute(const rovctrl::control_core::ControlState&      state,
                         const rovctrl::control_core::ControlReference& ref,
                         rovctrl::control_core::ControlOutput&          output,
                         double                                         dt_sec) noexcept = 0;
};

} // namespace rovctrl::controllers

#endif // ROVCTRL_CONTROLLERS_CONTROLLER_BASE_HPP
