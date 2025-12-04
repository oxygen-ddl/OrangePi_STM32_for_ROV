#pragma once

/**
 * @file    controller_base.hpp
 * @brief   控制器抽象基类接口（PID / MPC / SMC / 手动控制等的统一入口）
 *
 * 设计目标：
 *   - 为所有控制算法（PID, MPC, SMC, RL ...）提供统一接口；
 *   - 隔离控制算法与平台/执行细节（PWM、安全层、线程、日志）；
 *   - 使控制主循环能够无差别调度任意控制器；
 *   - 便于模式切换与日志系统。
 */

#include <string>

#include "control_core/control_types.hpp"

namespace rovctrl::controllers {

using rovctrl::control_core::ControlMode;
using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlOutput;

/**
 * @brief 控制器抽象基类
 *
 * 所有控制器必须实现以下特性：
 *   - 提供控制器标识名称；
 *   - 定义其所属的 ControlMode（手动/自动/故障保护等）；
 *   - 接受当前状态 + 参考输入，计算控制输出；
 *   - 支持 reset() 清空内部状态（积分项等）。
 *
 * 注意：
 *   - 不抛异常；
 *   - compute() 执行必须快速，可用于实时控制；
 *   - 控制器不负责推力到 PWM 的映射，这由更上层处理。
 */
class IController {
public:
    virtual ~IController() = default;

    /**
     * @brief 控制器名称（用于日志/调试）
     */
    virtual const char* name() const noexcept = 0;

    /**
     * @brief 控制器所处的逻辑控制模式
     *
     * 示例：
     *   - ControlMode::MANUAL
     *   - ControlMode::AUTO
     *   - ControlMode::FAILSAFE
     */
    virtual ControlMode mode() const noexcept = 0;

    /**
     * @brief 重置控制器内部状态（积分项、滤波器等）
     *
     * 常用于：
     *   - 模式切换
     *   - 导航重置 / ESKF 收敛前
     *   - 发生跳变（如丢帧、初始化）时
     */
    virtual void reset() noexcept = 0;

    /**
     * @brief 控制计算主入口
     *
     * @param state   当前状态（来自导航）
     * @param ref     当前参考输入（来自 teleop / trajectory）
     * @param output  输出结构（6-DOF 力/力矩 或 8 路推进器指令）
     * @param dt_sec  控制周期
     *
     * @return true  ：本周期计算成功
     * @return false ：输出无效，上层可选择忽略或进入保护模式
     */
    virtual bool compute(const ControlState&     state,
                         const ControlReference& ref,
                         ControlOutput&          output,
                         double                  dt_sec) noexcept = 0;
};

} // namespace rovctrl::controllers
