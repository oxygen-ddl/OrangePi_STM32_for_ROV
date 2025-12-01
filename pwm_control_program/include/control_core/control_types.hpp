#pragma once

/**
 * @file    control_types.hpp
 * @brief   控制主循环中的核心数据类型定义
 *
 * 设计目标：
 *   - 作为控制栈的“公共语言”，避免各模块各自定义结构体；
 *   - 对上可以由导航状态 / 轨迹规划 / 手动输入填充；
 *   - 对下可以被 PID / MPC / SMC / 推力分配模块复用；
 *   - 从“虚拟 4-DOF 指令”一直贯通到“单个电机 PWM 指令”。
 *
 * 当前阶段（仅保留手动 + PID）：
 *   - ControlState 可以只填一部分字段（姿态 / 深度等）；
 *   - ControlReference 可以由键盘输入或简单目标值构造；
 *   - ControlOutput 至少保证 4-DOF 指令可用，其余字段由推力分配层逐步填充。
 */

#include <cstdint>
#include <array>

namespace rovctrl::control_core {

/// 统一约定最大推进器数量（便于数组尺寸）
/// 若后续换成 6/10/12 推进器，只需要改这一处并同步调整推力分配矩阵。
constexpr std::size_t MAX_THRUSTERS = 8;

/// 控制模式（后续可以扩展更多）
enum class ControlMode : std::uint8_t {
    MANUAL = 0,     ///< 纯手动（键盘 → 直接 DOF 指令）
    PID_POSITION,   ///< PID 位置/姿态控制（本阶段目标）
    PID_VELOCITY,   ///< 仅速度控制（预留）
    MPC,            ///< 模型预测控制（预留）
    SMC             ///< 滑模控制（预留）
};

/// 当前系统状态（由导航模块 / 传感器融合提供）
/// 注意：目前导航侧还没真正 IPC 接上，可以只填一部分字段。
struct ControlState {
    /// 主时间戳（ns），通常使用导航状态的 est_ns
    std::int64_t t_ns{0};

    /// 位置（m），机体在某个导航坐标系下的位置 [x, y, z]
    std::array<double, 3> pos{0.0, 0.0, 0.0};

    /// 速度（m/s），同一坐标系下的线速度 [vx, vy, vz]
    std::array<double, 3> vel{0.0, 0.0, 0.0};

    /// 姿态（rad）：欧拉角 [roll, pitch, yaw]
    std::array<double, 3> rpy{0.0, 0.0, 0.0};

    /// 角速度（rad/s）：机体系角速度 [wx, wy, wz]
    std::array<double, 3> ang_vel{0.0, 0.0, 0.0};

    /// 深度（m），向下为正；若未单独提供，可由 z 轴推导
    double depth{0.0};

    /// 导航状态是否有效（例如 ESKF valid）
    bool nav_valid{false};
};

/// 控制参考（期望值）
/// 可以由：
///   - 键盘（手动给 DOF 档位 / 速度目标）
///   - 轨迹规划（给位置/姿态随时间演化）
///   - 上位机（任务级命令）
/// 填充。
struct ControlReference {
    /// 主时间戳（ns），表示该参考值规划的生效时间（可选）
    std::int64_t t_ns{0};

    /// 期望位置（m）
    std::array<double, 3> pos_ref{0.0, 0.0, 0.0};

    /// 期望速度（m/s）
    std::array<double, 3> vel_ref{0.0, 0.0, 0.0};

    /// 期望姿态（rad）
    std::array<double, 3> rpy_ref{0.0, 0.0, 0.0};

    /// 期望深度（m）
    double depth_ref{0.0};

    /// 权重 / 优先级（可选，用于混合控制或多目标权衡）
    double weight_pos{1.0};
    double weight_vel{1.0};
    double weight_att{1.0};
    double weight_depth{1.0};

    /// 是否启用各类约束/目标（用于简化控制器逻辑）
    bool enable_pos{false};
    bool enable_vel{false};
    bool enable_att{false};
    bool enable_depth{false};
};

/// 控制输出（控制器 → 推力分配 / PWM 映射）
///
/// 我们按“层次”划分三个级别：
///   1) 虚拟 4-DOF 指令（兼容现有 teleop 语义；[-1, 1] 档位）
///   2) 机体系 6-DoF 力/力矩（N, N·m），供推力分配使用
///   3) 单个推进器级别的指令（力 / 归一化档位 / PWM 百分比）
///
/// 在实际运行中：
///   - 键盘模式 / 简单 PID：通常只写第 1 层，后两层由 allocation/pwm_mapper 填充；
///   - 高级 MPC：可以直接写第 2 层（期望广义力），再由 allocation 层下钻到第 3 层。
struct ControlOutput {
    /// 控制模式（便于日志和调试）
    ControlMode mode{ControlMode::MANUAL};

    /// 本次输出对应的时间戳（ns）
    std::int64_t t_ns{0};

    // ===== (1) 虚拟 4-DOF 指令（现有 teleop 兼容层） =====
    //
    // 建议范围：[-1, 1]
    //   - surge > 0 : 前进， < 0 : 后退
    //   - sway  > 0 : 右移， < 0 : 左移
    //   - heave > 0 : 上升， < 0 : 下潜
    //   - yaw   > 0 : 左转， < 0 : 右转（或相反，按既有约定）
    double surge_cmd{0.0};
    double sway_cmd{0.0};
    double heave_cmd{0.0};
    double yaw_cmd{0.0};

    /// 该层输出是否已填充（方便日志/调试和多控制器管线）
    bool virtual_dof_valid{false};

    // ===== (2) 机体系 6-DoF 力 / 力矩（中间层：供推力分配使用） =====
    //
    // wrench_cmd[0..2] = [Fx, Fy, Fz]  (N)
    // wrench_cmd[3..5] = [Mx, My, Mz]  (N·m)
    //
    // 由控制器根据误差 + 动力学模型等计算，
    // 推力分配器拿这个 6 维向量乘以分配矩阵 → 单个推进器力。
    std::array<double, 6> wrench_cmd{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool wrench_valid{false};

    // ===== (3) 单个推进器级别指令（底层推进器空间） =====
    //
    // 3.1 推进器力（N），由 thrust_allocator 产生
    std::array<double, MAX_THRUSTERS> thruster_force_N{};
    // 3.2 推进器归一化档位（[-1, 1]），用于抽象电机输出强度
    //     例如：-1 → 反向最大，0 → 停止/中位，1 → 正向最大
    std::array<double, MAX_THRUSTERS> thruster_norm{};
    // 3.3 推进器 PWM 百分比（5.0~10.0），最终交给 pwm_control 安全层
    //     由 pwm_mapper 负责从 thruster_norm / thruster_force_N 计算得到
    std::array<double, MAX_THRUSTERS> thruster_pwm_pct{};
    bool thruster_setpoint_valid{false};

    /// 总体输出是否有效（例如某些模式下，控制器可能选择“保持上一帧输出不变”）
    bool valid{false};
};

} // namespace rovctrl::control_core
