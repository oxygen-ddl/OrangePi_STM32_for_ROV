#pragma once

#include <vector>
#include <cstddef>

#include "control_core/control_types.hpp"

namespace rovctrl::control_core {

/**
 * @brief 轨迹中的一个采样点
 *
 * 约定：
 *  - t_s 为轨迹时间轴上的相对时间（从 0 开始，单位秒）；
 *  - pose/vel/accel 为在该时刻的期望状态；
 *  - 对于当前方案，主要关心 x,y,z,yaw 四个自由度，其余可根据需要填 0。
 */
struct TrajectoryPoint {
    double t_s = 0.0;  ///< 轨迹时间（s，轨迹内部时间轴）

    Pose   pose{};     ///< 期望位姿
    Twist  vel{};      ///< 期望速度（可选）
    Accel  accel{};    ///< 期望加速度（可选）
};

/**
 * @brief 轨迹采样结果（供控制器使用的参考）
 */
struct TrajectorySample {
    double t_s = 0.0;  ///< 当前采样对应的轨迹时间

    Pose   pose_ref{};
    Twist  vel_ref{};
    Accel  accel_ref{};

    bool   valid = false;
};

/**
 * @brief 轨迹跟踪误差（便于调试 / 日志记录）
 */
struct TrackingError {
    Pose   pose_err{};   ///< 位置 / 姿态误差（ref - state）
    Twist  vel_err{};    ///< 速度 / 角速度误差（ref - state）
    double yaw_err_norm = 0.0; ///< 归一化到 [-pi,pi] 的 yaw 误差
    bool   valid = false;
};

/**
 * @brief 轨迹跟踪器：负责
 *   1) 保存离散轨迹点；
 *   2) 给定当前时间，插值出轨迹参考；
 *   3) 根据当前状态和参考，计算误差；
 *   4) 可直接填充 ControlReference，供 PID/MPC 使用。
 */
class TrajectoryTracking {
public:
    TrajectoryTracking() = default;

    /**
     * @brief 设置轨迹（外部可由 YAML 解析后调用）
     *
     * 要求：
     *  - points 按 t_s 从小到大排序；
     *  - 若未排序，本函数会自动排序；
     */
    void set_trajectory(std::vector<TrajectoryPoint> points);

    /// 清空轨迹
    void clear();

    /// 是否已经有可用轨迹
    bool has_trajectory() const noexcept { return !traj_.empty(); }

    /**
     * @brief 复位时间零点（通常在控制循环启动时调用）
     *
     * @param t0_s 控制循环启动时的“控制时间”（单位秒）。
     *             后续 sample() 中会用 t_now_s - t0_s_ 映射到轨迹时间轴。
     */
    void reset_time_origin(double t0_s);

    /**
     * @brief 在给定当前“控制时间”采样轨迹，并填充 TrajectorySample
     *
     * @param t_now_s   控制侧当前时间（相对值即可，如 loop_start 到现在的秒数）
     * @param out       采样结果
     * @return true     有效采样；false 表示轨迹为空或时间超出范围
     */
    bool sample(double t_now_s, TrajectorySample& out) const;

    /**
     * @brief 将采样结果填充到 ControlReference 中
     *
     * 用法：在控制循环中先 sample() 得到 TrajectorySample，再调用本函数。
     */
    void fill_reference(const TrajectorySample& sample,
                        ControlReference&       ref_out) const;

    /**
     * @brief 计算当前状态相对于轨迹参考的误差
     *
     * 误差定义：
     *  - pose_err: ref.pose - state.pose
     *  - vel_err : ref.vel  - state.velocity
     *  - yaw 误差使用 wrap 到 [-pi, pi]，结果写入 pose_err.yaw 和 yaw_err_norm。
     */
    void compute_error(const ControlState&    state,
                       const TrajectorySample& sample,
                       TrackingError&          err_out) const;

private:
    std::vector<TrajectoryPoint> traj_;
    double                       t0_s_ = 0.0;  ///< 控制时间零点

    /// 在线性插值轨迹（假定已保证 traj_ 非空）
    TrajectoryPoint interpolate(double t_rel) const;

    /// 将角度 wrap 到 [-pi, pi]
    static double wrap_angle(double ang);
};

} // namespace rovctrl::control_core
