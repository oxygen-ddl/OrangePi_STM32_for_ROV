#pragma once

#include <array>
#include <cstdint>
#include <cstddef>  // for std::size_t

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
inline constexpr std::size_t kNumThrusters = 8;   ///< 默认 8 推进器 ROV

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
 * @brief 控制状态：由传感器融合模块 / 导航进程提供的当前系统估计状态
 *
 * 用途:
 *   - 控制器（PID/MPC/SMC）使用该状态作为反馈量；
 *   - timestamp / has_xxx / nav_valid 等标志用于调试与数据分析。
 *
 * 注意：
 *   - 新增的 nav_* 字段是“来自导航进程的反馈”，不会改变原有 pose/velocity 的语义；
 *   - 控制器可以选择使用 pose/velocity（本地估计）或 nav_xxx（共享内存 NavState）。
 */
struct ControlState {
    // ================== 原有本地估计状态 ==================
    Pose   pose;          ///< 位置 + 姿态
    Twist  velocity;      ///< 线速度 + 角速度
    Accel  accel;         ///< 线加速度 + 角加速度（如可用）

    /// 时间戳（单位秒，可以是相对时间或系统单调时间）
    double timestamp_sec = 0.0;

    /// 有效性标志（根据传感器情况设置）
    bool has_pose     = false;
    bool has_velocity = false;
    bool has_accel    = false;

    // ================== 新增：导航反馈（共享内存 NavState 映射） ==================

    /**
     * @brief 当前循环是否存在可用导航状态
     *
     * 由控制循环根据 NavStateSubscriber 结果设置：
     *   - true  表示 nav_* 字段有效；
     *   - false 表示本周期未获得稳定导航状态，控制器可据此做降级策略。
     */
    bool nav_valid = false;

    /// 导航时间戳（纳秒），直接来源于 shared::msg::NavState::t_ns
    std::uint64_t nav_t_ns = 0;

    /// NED 坐标系下的位置 [x, y, z]
    std::array<double, 3> nav_pos_ned{0.0, 0.0, 0.0};

    /// NED 坐标系下的速度 [vx, vy, vz]
    std::array<double, 3> nav_vel_ned{0.0, 0.0, 0.0};

    /// 欧拉角 [roll, pitch, yaw]，单位 rad，来自 NavState::rpy
    std::array<double, 3> nav_rpy{0.0, 0.0, 0.0};

    /// 深度（正向向下，单位 m，来自 NavState::depth）
    double nav_depth = 0.0;

    /// 机体系角速度 [wx, wy, wz]，来自 NavState::omega_b
    std::array<double, 3> nav_omega_b{0.0, 0.0, 0.0};

    /// 机体系线加速度 [ax, ay, az]，来自 NavState::acc_b
    std::array<double, 3> nav_acc_b{0.0, 0.0, 0.0};

    /// 状态标志（bitmask，直接转抄 NavState::status_flags）
    std::uint16_t nav_status_flags = 0;

    // ……后面还有你原来的 body_wrench / thruster_command 等字段……
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
