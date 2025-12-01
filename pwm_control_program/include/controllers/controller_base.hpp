#pragma once

/**
 * @file    controller_base.hpp
 * @brief   控制器抽象基类接口（PID / MPC / SMC 等的统一入口）
 *
 * 设计目标：
 *   - 为后续所有控制算法（PID, MPC, SMC, RL ...）提供统一的 C++ 接口；
 *   - 隔离“控制算法”与“平台/执行细节”（PWM、安全层、线程、日志等）；
 *   - 便于在控制主循环中进行模式切换、统一调度与日志记录。
 *
 * 核心概念：
 *   - ControlState      : 系统当前状态（来自导航 / 估计器）
 *   - ControlReference  : 控制参考（来自手动输入 / 轨迹规划 / 上位机）
 *   - ControlOutput     : 控制器输出（4-DOF 指令，供推力分配 / PWM 映射使用）
 *
 * 注意：本文件只定义“接口”，不依赖具体控制算法实现。
 */

#include <cstdint>
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
 * 约定：
 *   - 所有控制器应是“无异常接口”：不抛异常，用返回值 / 输出字段表达错误；
 *   - 线程模型由上层控制循环负责（通常在单线程控制循环中调用 compute()）；
 *   - 控制器内部可以维护积分项、滤波器状态等“有记忆”的变量。
 */
class IController {
public:
    virtual ~IController() = default;

    /**
     * @brief 控制器名称（用于日志 / 调试）
     *
     * 示例：
     *   - "PID_POSITION_4DOF"
     *   - "MPC_TRACKING"
     */
    virtual const char* name() const noexcept = 0;

    /**
     * @brief 控制模式（枚举，用于状态机与日志）
     *
     * 例如：
     *   - ControlMode::MANUAL
     *   - ControlMode::PID_POSITION
     *   - ControlMode::MPC
     */
    virtual ControlMode mode() const noexcept = 0;

    /**
     * @brief 重置控制器内部状态
     *
     * 使用场景：
     *   - 模式切换（例如 MANUAL → PID）时清空积分项、历史误差；
     *   - 重大状态跳变（例如导航重置 / ESKF 重定位）后，避免旧状态污染控制。
     */
    virtual void reset() noexcept = 0;

    /**
     * @brief 控制主入口：根据当前状态与参考，计算本周期的控制输出
     *
     * @param state   当前系统状态（导航 + 传感器融合）
     * @param ref     当前期望（位置 / 速度 / 姿态 / 深度等）
     * @param dt_sec  控制时间步长（秒），由上层控制循环传入
     *
     * @return ControlOutput:
     *   - output.mode   一般返回该控制器自己的 mode()
     *   - output.t_ns   建议填 state.t_ns 或当前时间戳
     *   - output.*_cmd  为本周期的 4-DOF 指令（范围建议 [-1, 1]）
     *   - output.valid  控制器是否给出有效输出（false 时上层可保持上一帧输出）
     *
     * 要求：
     *   - 不抛异常；
     *   - 不直接访问 PWM / 通讯等底层资源，只做“算法层”的输入 → 输出映射。
     */
    virtual ControlOutput compute(
        const ControlState&     state,
        const ControlReference& ref,
        double                  dt_sec) noexcept = 0;
};

} // namespace rovctrl::controllers
