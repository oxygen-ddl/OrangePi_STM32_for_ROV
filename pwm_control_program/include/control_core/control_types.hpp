#pragma once

#include <array>
#include <cstdint>
#include <cstddef>   // std::size_t
#include <string>    // for std::string (stability across includes)
#include <type_traits>
#include <tuple>     // std::tuple_size


namespace rovctrl::control_core {

/**
 * @brief 6 自由度 DOF 索引（运动学：速度/加速度/姿态角速度）
 *
 * 约定:
 *   - 0: Surge  (x)
 *   - 1: Sway   (y)
 *   - 2: Heave  (z)
 *   - 3: Roll
 *   - 4: Pitch
 *   - 5: Yaw
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

/// 兼容：历史上你们可能用 DofVector 表示 6 维量（可能是速度，也可能是 wrench）
using DofVector     = std::array<double, kNumDof>;
using ThrusterArray = std::array<float,  kNumThrusters>;

/// 建议：明确“力/力矩”语义（Fx,Fy,Fz,Mx,My,Mz）
using WrenchVector  = DofVector;

/**
 * @brief 位姿（位置 + 姿态）
 * - 姿态 roll/pitch/yaw 单位 rad
 */
struct Pose {
    double x     = 0.0;
    double y     = 0.0;
    double z     = 0.0;

    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};

struct Twist {
    double surge = 0.0;
    double sway  = 0.0;
    double heave = 0.0;

    double roll_rate  = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate   = 0.0;
};

struct Accel {
    double surge = 0.0;
    double sway  = 0.0;
    double heave = 0.0;

    double roll_acc  = 0.0;
    double pitch_acc = 0.0;
    double yaw_acc   = 0.0;
};

struct DofCommand {
    double surge = 0.0;
    double sway  = 0.0;
    double heave = 0.0;

    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};

/**
 * @brief 控制输出：控制器输出到推力分配/执行层的结果
 */
struct ControlOutput {
    WrenchVector body_wrench{};     ///< [Fx, Fy, Fz, Mx, My, Mz]
    bool         has_body_wrench = false;

    ThrusterArray thruster_command{};   ///< 8 路推进器归一化指令
    bool          has_thruster_command = false;
};

/**
 * @brief 控制状态：传感器融合 / 导航进程提供的当前系统估计状态
 */
struct ControlState {
    // ================== 本地估计状态 ==================
    Pose   pose{};
    Twist  velocity{};
    Accel  accel{};

    double timestamp_sec = 0.0;

    bool has_pose     = false;
    bool has_velocity = false;
    bool has_accel    = false;

    // ================== 导航反馈（共享内存 NavState 映射） ==================
    bool nav_valid = false;

    std::uint64_t nav_t_ns = 0;

    std::array<double, 3> nav_pos_ned{0.0, 0.0, 0.0};
    std::array<double, 3> nav_vel_ned{0.0, 0.0, 0.0};
    std::array<double, 3> nav_rpy{0.0, 0.0, 0.0};

    double nav_depth = 0.0;

    std::array<double, 3> nav_omega_b{0.0, 0.0, 0.0};
    std::array<double, 3> nav_acc_b{0.0, 0.0, 0.0};

    std::uint16_t nav_status_flags = 0;

    // ================== 兼容区（减少老代码摩擦） ==================
    // 若你们旧版本某些控制器/日志器读取 state.last_output，可以先在过渡期保留。
    // 后续统一迁移后可删除。
    ControlOutput last_output{};
    bool          has_last_output = false;
};

/**
 * @brief 控制参考量：控制目标（期望状态 / 期望 DOF 指令）
 */
struct ControlReference {
    Pose   pose_ref{};
    Twist  vel_ref{};
    Accel  accel_ref{};

    bool use_pose_ref  = false;
    bool use_vel_ref   = false;
    bool use_accel_ref = false;

    DofCommand dof_cmd{};
    bool       use_dof_cmd = false;
};

// ============ 编译期一致性检查（防止未来改错） ============
static_assert(kNumDof == 6, "kNumDof must be 6");
static_assert(std::tuple_size<DofVector>::value == 6, "DofVector must be 6D");
static_assert(std::tuple_size<ThrusterArray>::value == 8, "ThrusterArray must be 8-ch");

} // namespace rovctrl::control_core
