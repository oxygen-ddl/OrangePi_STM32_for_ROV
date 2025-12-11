#pragma once

#include <vector>
#include <cstddef>
#include <string>

#include "control_core/control_types.hpp"

namespace rovctrl::control_core {

/* ===========================================
 *  轨迹元信息枚举（与 config/trajectory.yaml 对应）
 * =========================================== */

/// 轨迹坐标系类型（示例：NED / ENU）
enum class TrajectoryFrame {
    NED,
    ENU,
    Unknown
};

/// 航向/角速度的单位（示例：rad / deg）
enum class AngleUnit {
    Rad,
    Deg,
    Unknown
};

/// 轨迹类型（当前仅支持分段线性插值）
enum class TrajectoryType {
    Piecewise,
    Unknown
};

/* ===========================================
 *  轨迹数据结构（由外部模块填充）
 * =========================================== */

/**
 * @brief 轨迹中的一个离散点
 *
 * 约定：
 *  - t_s 为轨迹时间轴上的相对时间（从 0 开始，单位秒）；
 *  - pose/vel/accel 为在该时刻的期望状态；
 *  - 当前关注主要自由度为 x,y,z,yaw，其余可按需填 0。
 *
 * 典型来源：
 *  - 由 config_loader 从 trajectory.yaml 解析后填充；
 *  - 或由上位机 / 内部生成器直接构造。
 *
 * 注意：
 *  - 这里不直接处理 YAML，也不涉及文件路径。
 */
struct TrajectoryPoint {
    double t_s = 0.0;  ///< 轨迹内部时间（s）

    Pose   pose{};     ///< 期望位姿（Pose 中的 x,y,z,roll,pitch,yaw）
    Twist  vel{};      ///< 期望速度（surge,sway,heave,roll_rate,pitch_rate,yaw_rate）
    Accel  accel{};    ///< 期望加速度（可选，当前可置 0）
};

/**
 * @brief 完整轨迹配置结构
 *
 * 本结构通常由“配置加载模块”（例如 utils/config_loader）填充，
 * TrajectoryTracking 只使用其中的元信息和 points，不关心其来源。
 */
struct TrajectoryConfig {
    TrajectoryFrame frame      = TrajectoryFrame::Unknown;
    AngleUnit       angle_unit = AngleUnit::Unknown;
    TrajectoryType  type       = TrajectoryType::Unknown;

    // 原始字符串（用于调试 / 日志）
    std::string frame_raw;
    std::string angle_unit_raw;
    std::string type_raw;

    /// 轨迹点列表，要求在进入 TrajectoryTracking 前可排序为 t_s 递增
    std::vector<TrajectoryPoint> points;
};

/**
 * @brief 轨迹采样结果（插值后给控制器使用）
 *
 * 注意：t_s 为轨迹时间（相对轨迹起点），不是系统绝对时间。
 */
struct TrajectorySample {
    double t_s = 0.0;   ///< 对应轨迹时间（内部时间轴）

    Pose  pose_ref{};
    Twist vel_ref{};
    Accel accel_ref{};

    bool  valid = false;
};

/**
 * @brief 轨迹跟踪误差结构（便于调试 / 日志记录 / 控制器使用）
 *
 * 误差定义约定：
 *  - pose_err = ref.pose - state.pose
 *  - vel_err  = ref.vel  - state.velocity
 */
struct TrackingError {
    Pose   pose_err{};       ///< 位置 / 姿态误差
    Twist  vel_err{};        ///< 速度 / 角速度误差
    double yaw_err_norm = 0.0; ///< wrap 到 [-pi,pi] 的 yaw 误差
    bool   valid = false;
};

/* ===========================================
 *  TrajectoryTracking：轨迹插值与误差计算
 * =========================================== */

/**
 * @brief 轨迹跟踪器（运行时模块）
 *
 * 职责：
 *   1) 保存离散轨迹点；
 *   2) 给定当前控制时间，进行轨迹时间映射和插值；
 *   3) 输出轨迹参考 TrajectorySample；
 *   4) 根据当前状态和参考，计算 TrackingError；
 *   5) 将参考写入 ControlReference，供 PID/MPC 使用。
 *
 * 非职责：
 *   - 不负责读取 YAML / 文件；
 *   - 不负责任何路径解析或 config_loader 逻辑。
 *
 * 轨迹数据来源：
 *   - 通常由外部 config_loader 从 YAML 解析为 TrajectoryConfig，
 *     然后调用 set_trajectory(cfg) 注入；
 *   - 也可以由上位机 / 内部生成器直接构造 std::vector<TrajectoryPoint> 注入。
 */
class TrajectoryTracking {
public:
    TrajectoryTracking() = default;

    /**
     * @brief 使用一组轨迹点设置轨迹
     *
     * 要求：
     *  - points 按 t_s 从小到大排序；如未排序，本函数会自动排序。
     */
    void set_trajectory(std::vector<TrajectoryPoint> points);

    /**
     * @brief 使用高层 TrajectoryConfig 设置轨迹
     *
     * 典型流程：
     *   1) 外部模块（如 config_loader）从 YAML 解析出 TrajectoryConfig；
     *   2) 调用本函数，内部会保存 frame/angle_unit/type，并使用 cfg.points。
     *
     * 说明：
     *   - 本函数不会解析 YAML，仅使用结构体；
     *   - 坐标系/单位的真正语义解释由更上层或日志/调试使用。
     */
    void set_trajectory(const TrajectoryConfig& cfg);

    /// 清空轨迹与时间零点、元信息
    void clear();

    /// 是否已经配置了可用轨迹
    bool has_trajectory() const noexcept { return !traj_.empty(); }

    /**
     * @brief 设置控制时间的零点（通常在控制循环启动时调用）
     *
     * 示例：
     *   double loop_start = now();
     *   tracking.reset_time_origin(loop_start);
     *
     * 后续：
     *   sample(t_now) 会内部使用 t_rel = t_now - t0_s_ 映射到轨迹时间轴。
     */
    void reset_time_origin(double t0_s);

    /**
     * @brief 在给定当前控制时间下采样轨迹
     *
     * @param t_now_s 控制时间（相对值，例如从 loop_start 起算的秒数）
     * @param out     输出采样结果
     *
     * @return true   有效采样；false = 轨迹为空或其他异常
     */
    bool sample(double t_now_s, TrajectorySample& out) const;

    /**
     * @brief 将轨迹采样结果填充到 ControlReference 中
     *
     * 典型使用：
     *   TrajectorySample s;
     *   tracking.sample(t_now, s);
     *   ControlReference ref;
     *   tracking.fill_reference(s, ref);
     */
    void fill_reference(const TrajectorySample& sample,
                        ControlReference&       ref_out) const;

    /**
     * @brief 计算当前状态相对于轨迹参考的跟踪误差
     *
     * 误差定义：
     *  - pose_err: ref.pose - state.pose
     *  - vel_err : ref.vel  - state.velocity
     *  - yaw 误差使用 wrap_angle 到 [-pi, pi]
     */
    void compute_error(const ControlState&     state,
                       const TrajectorySample& sample,
                       TrackingError&          err_out) const;

    /// 访问轨迹元信息（来自最近一次 set_trajectory(cfg)）
    TrajectoryFrame frame() const noexcept { return frame_; }
    AngleUnit       angle_unit() const noexcept { return angle_unit_; }
    TrajectoryType  type() const noexcept { return type_; }

private:
    std::vector<TrajectoryPoint> traj_;  ///< 已排序轨迹点
    double t0_s_ = 0.0;                  ///< 控制时间 → 轨迹时间的偏移

    // 保存自 TrajectoryConfig 的元信息（不影响插值，仅用于调试/上层逻辑）
    TrajectoryFrame frame_      = TrajectoryFrame::Unknown;
    AngleUnit       angle_unit_ = AngleUnit::Unknown;
    TrajectoryType  type_       = TrajectoryType::Unknown;

    /// 在线性插值（假定 traj_ 非空，且 t_rel 在轨迹时间范围内）
    TrajectoryPoint interpolate(double t_rel) const;

    /// 将角度 wrap 到 [-pi, pi]
    static double wrap_angle(double ang);
};

} // namespace rovctrl::control_core
