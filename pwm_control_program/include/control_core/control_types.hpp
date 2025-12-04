#pragma once

#include <array>
#include <cstdint>
#include <cstddef>  // 为 std::size_t 显式引入

namespace rovctrl::control_core {

/**
 * @brief 控制模式枚举
 *
 * 说明：
 *  - 统一采用 PascalCase 命名：Unknown / Manual / Auto / Failsafe
 *  - 目前实际使用的是 Manual，将来可以扩展到 Auto（PID/MPC 等自动控制）、
 *    Failsafe（失效保护）等模式。
 */
enum class ControlMode : std::uint8_t {
    Unknown  = 0,  ///< 未知 / 未初始化
    Manual   = 1,  ///< 手动控制模式（键盘 / 手柄等）
    Auto     = 2,  ///< 自动控制模式（PID / MPC / SMC 等）
    Failsafe = 3,  ///< 失效保护 / 紧急模式
};

/**
 * @brief 6 自由度 DOF 索引（线速度/加速度用前 3 项，角速度/角加速度用后 3 项）
 *
 * 约定:
 *   - 0: Surge  (前后 / x)
 *   - 1: Sway   (左右 / y)
 *   - 2: Heave  (上下 / z)
 *   - 3: Roll   (横滚)
 *   - 4: Pitch  (俯仰)
 *   - 5: Yaw    (偏航)
 */
enum class DofIndex : std::size_t {
    Surge = 0,
    Sway  = 1,
    Heave = 2,
    Roll  = 3,
    Pitch = 4,
    Yaw   = 5
};

inline constexpr std::size_t kNumDof       = 6;
inline constexpr std::size_t kNumThrusters = 8;   ///< 目前默认 8 推进器 ROV

using DofVector     = std::array<double, kNumDof>;
using ThrusterArray = std::array<float,  kNumThrusters>;

/**
 * @brief 位姿（位置 + 姿态）
 *
 * 坐标系约定:
 *   - 位置 (x, y, z) 建议使用 NED / ENU 之一，在文档与实现中统一；
 *   - 姿态 (roll, pitch, yaw) 单位为弧度 [rad]，采用 Z-Y-X 欧拉角顺序。
 */
struct Pose {
    double x     = 0.0;
    double y     = 0.0;
    double z     = 0.0;

    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};

/**
 * @brief 6 自由度速度（线速度 + 角速度）
 *
 * 线速度单位通常 m/s，角速度单位 rad/s。
 */
struct Twist {
    double surge = 0.0;  ///< 线速度 x
    double sway  = 0.0;  ///< 线速度 y
    double heave = 0.0;  ///< 线速度 z

    double roll_rate  = 0.0;  ///< 角速度 x
    double pitch_rate = 0.0;  ///< 角速度 y
    double yaw_rate   = 0.0;  ///< 角速度 z
};

/**
 * @brief 6 自由度加速度（线加速度 + 角加速度）
 *
 * 线加速度单位 m/s^2，角加速度单位 rad/s^2。
 */
struct Accel {
    double surge = 0.0;
    double sway  = 0.0;
    double heave = 0.0;

    double roll_acc  = 0.0;
    double pitch_acc = 0.0;
    double yaw_acc   = 0.0;
};

/**
 * @brief 归一化 6-DOF 指令（用于 Teleop / 上位机输入等）
 *
 * 典型约定:
 *   - 取值范围 [-1, 1]，表示各 DOF 的相对推力指令；
 *   - 由推力分配模块（ThrustAllocator）映射为各推进器推力/占空比。
 */
struct DofCommand {
    double surge = 0.0;
    double sway  = 0.0;
    double heave = 0.0;

    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};

/**
 * @brief 控制状态：由传感器融合模块提供的当前系统估计状态
 *
 * 用途:
 *   - 控制器（PID/MPC/SMC）使用该状态作为反馈量；
 *   - 记录时刻、有效标志用于调试与数据分析。
 */
struct ControlState {
    Pose   pose;          ///< 位置 + 姿态
    Twist  velocity;      ///< 线速度 + 角速度
    Accel  accel;         ///< 线加速度 + 角加速度（如可用）

    // 时间戳（单位秒，可以是相对时间或系统单调时间）
    double timestamp_sec = 0.0;

    // 有效性标志（可根据传感器情况设置）
    bool has_pose     = false;
    bool has_velocity = false;
    bool has_accel    = false;
};

/**
 * @brief 控制参考量：控制目标（期望状态 / 期望 DOF 指令）
 *
 * 支持两种典型用法:
 *   1. 轨迹跟踪 / 自动控制:
 *      - 使用 pose_ref / vel_ref 等字段；
 *   2. 手动/半自动控制:
 *      - 使用 dof_cmd（归一化 6-DOF 指令）。
 *
 * 控制器可以根据 use_xxx 标志决定采用哪一类参考。
 */
struct ControlReference {
    // 期望姿态与位置（轨迹跟踪）
    Pose   pose_ref;         ///< 期望位姿
    Twist  vel_ref;          ///< 期望速度（可选）
    Accel  accel_ref;        ///< 期望加速度（可选）

    bool   use_pose_ref   = false;
    bool   use_vel_ref    = false;
    bool   use_accel_ref  = false;

    // 归一化 DOF 指令（用于 Teleop 等）
    DofCommand dof_cmd;      ///< [-1,1] 范围的指令，含 6DOF

    bool   use_dof_cmd   = false;   ///< true 时，控制器优先按照 dof_cmd 执行
};

/**
 * @brief 控制输出：控制器输出到推力分配/执行层的结果
 *
 * 两种层次:
 *   1. body_wrench: 6-DOF 力/力矩（物理量），从模型预测控制等算法出来；
 *   2. thruster_command: 8 路推进器归一化输出（占空比/推力指令），可直接给 PWM 层。
 *
 * 具体使用哪一级由上层控制栈设计决定。
 */
struct ControlOutput {
    // 6-DOF 期望力/力矩（物理量），单位可约定为 N / N·m
    DofVector body_wrench{};     ///< [Fx, Fy, Fz, Mx, My, Mz]
    bool      has_body_wrench = false;

    // 8 路推进器指令（例如 [-1,1] 对应反转/正转最大功率）
    ThrusterArray thruster_command{};   ///< 推进器级的归一化指令
    bool          has_thruster_command = false;
};

} // namespace rovctrl::control_core
