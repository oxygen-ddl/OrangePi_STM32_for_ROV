// src/control_core/trajectory_tracking.cpp

#include "control_core/trajectory_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace rovctrl::control_core {

void TrajectoryTracking::set_trajectory(std::vector<TrajectoryPoint> points)
{
    // 按 t_s 排序，保证插值逻辑成立
    std::sort(points.begin(), points.end(),
              [](const TrajectoryPoint& a, const TrajectoryPoint& b) {
                  return a.t_s < b.t_s;
              });

    traj_ = std::move(points);
}

void TrajectoryTracking::set_trajectory(const TrajectoryConfig& cfg)
{
    // 保存元信息（便于上层调试 / 日志 / 坐标系处理）
    frame_      = cfg.frame;
    angle_unit_ = cfg.angle_unit;
    type_       = cfg.type;

    // 实际轨迹点交给已有接口处理（排序等逻辑共用）
    set_trajectory(cfg.points);
}

void TrajectoryTracking::clear()
{
    traj_.clear();
    t0_s_       = 0.0;
    frame_      = TrajectoryFrame::Unknown;
    angle_unit_ = AngleUnit::Unknown;
    type_       = TrajectoryType::Unknown;
}

void TrajectoryTracking::reset_time_origin(double t0_s)
{
    t0_s_ = t0_s;
}

bool TrajectoryTracking::sample(double t_now_s, TrajectorySample& out) const
{
    if (traj_.empty()) {
        out.valid = false;
        return false;
    }

    const double t_rel = t_now_s - t0_s_;  // 映射到轨迹时间轴

    // 早于轨迹起点：直接使用首点
    if (t_rel <= traj_.front().t_s) {
        const TrajectoryPoint& p = traj_.front();
        out.t_s       = p.t_s;
        out.pose_ref  = p.pose;
        out.vel_ref   = p.vel;
        out.accel_ref = p.accel;
        out.valid     = true;
        return true;
    }

    // 晚于轨迹终点：使用末尾点（也可以按需求改变策略）
    if (t_rel >= traj_.back().t_s) {
        const TrajectoryPoint& p = traj_.back();
        out.t_s       = p.t_s;
        out.pose_ref  = p.pose;
        out.vel_ref   = p.vel;
        out.accel_ref = p.accel;
        out.valid     = true;
        return true;
    }

    // 一般情况：在线性插值
    const TrajectoryPoint p = interpolate(t_rel);
    out.t_s       = p.t_s;
    out.pose_ref  = p.pose;
    out.vel_ref   = p.vel;
    out.accel_ref = p.accel;
    out.valid     = true;
    return true;
}

void TrajectoryTracking::fill_reference(const TrajectorySample& sample,
                                        ControlReference&       ref_out) const
{
    if (!sample.valid) {
        ref_out.use_pose_ref  = false;
        ref_out.use_vel_ref   = false;
        ref_out.use_accel_ref = false;
        ref_out.use_dof_cmd   = false;
        return;
    }

    ref_out.pose_ref   = sample.pose_ref;
    ref_out.vel_ref    = sample.vel_ref;
    ref_out.accel_ref  = sample.accel_ref;

    ref_out.use_pose_ref  = true;
    ref_out.use_vel_ref   = true;   // 如暂时不用速度参考，可置为 false
    ref_out.use_accel_ref = false;  // 预留给 MPC 使用
    ref_out.use_dof_cmd   = false;  // Teleop dof_cmd 由输入模块决定
}

void TrajectoryTracking::compute_error(const ControlState&     state,
                                       const TrajectorySample& sample,
                                       TrackingError&          err_out) const
{
    if (!sample.valid || !state.has_pose) {
        err_out.valid = false;
        return;
    }

    // 位置误差（假定 NavState 与轨迹在同一坐标系，frame_ 仅供上层解释）
    err_out.pose_err.x = sample.pose_ref.x - state.pose.x;
    err_out.pose_err.y = sample.pose_ref.y - state.pose.y;
    err_out.pose_err.z = sample.pose_ref.z - state.pose.z;

    // 姿态误差（roll/pitch/yaw）
    const double d_roll  = wrap_angle(sample.pose_ref.roll  - state.pose.roll);
    const double d_pitch = wrap_angle(sample.pose_ref.pitch - state.pose.pitch);
    const double d_yaw   = wrap_angle(sample.pose_ref.yaw   - state.pose.yaw);

    err_out.pose_err.roll  = d_roll;
    err_out.pose_err.pitch = d_pitch;
    err_out.pose_err.yaw   = d_yaw;
    err_out.yaw_err_norm   = d_yaw;

    // 速度误差（如果 state.has_velocity=false，则认为当前速度为 0）
    if (state.has_velocity) {
        err_out.vel_err.surge      = sample.vel_ref.surge      - state.velocity.surge;
        err_out.vel_err.sway       = sample.vel_ref.sway       - state.velocity.sway;
        err_out.vel_err.heave      = sample.vel_ref.heave      - state.velocity.heave;
        err_out.vel_err.roll_rate  = sample.vel_ref.roll_rate  - state.velocity.roll_rate;
        err_out.vel_err.pitch_rate = sample.vel_ref.pitch_rate - state.velocity.pitch_rate;
        err_out.vel_err.yaw_rate   = sample.vel_ref.yaw_rate   - state.velocity.yaw_rate;
    } else {
        err_out.vel_err = Twist{};
    }

    err_out.valid = true;
}

TrajectoryPoint TrajectoryTracking::interpolate(double t_rel) const
{
    // 此时保证 traj_ 非空，且 t_rel 在 (front.t_s, back.t_s) 范围内

    // 查找第一个 t_s >= t_rel 的点
    auto it_upper = std::upper_bound(
        traj_.begin(), traj_.end(), t_rel,
        [](double t, const TrajectoryPoint& p) { return t < p.t_s; });

    // it_upper 不会是 begin（前面已经判断 t_rel > front.t_s）
    auto it_lower = it_upper - 1;

    const TrajectoryPoint& p0 = *it_lower;
    const TrajectoryPoint& p1 = *it_upper;

    const double dt    = p1.t_s - p0.t_s;
    const double alpha = (dt > 0.0) ? (t_rel - p0.t_s) / dt : 0.0;

    auto lerp = [alpha](double a, double b) {
        return a + alpha * (b - a);
    };

    TrajectoryPoint out;
    out.t_s = t_rel;

    // 位置插值
    out.pose.x = lerp(p0.pose.x, p1.pose.x);
    out.pose.y = lerp(p0.pose.y, p1.pose.y);
    out.pose.z = lerp(p0.pose.z, p1.pose.z);

    // 姿态插值（简单线性 + wrap）
    out.pose.roll  = wrap_angle(lerp(p0.pose.roll,  p1.pose.roll));
    out.pose.pitch = wrap_angle(lerp(p0.pose.pitch, p1.pose.pitch));
    out.pose.yaw   = wrap_angle(lerp(p0.pose.yaw,   p1.pose.yaw));

    // 速度插值
    out.vel.surge      = lerp(p0.vel.surge,      p1.vel.surge);
    out.vel.sway       = lerp(p0.vel.sway,       p1.vel.sway);
    out.vel.heave      = lerp(p0.vel.heave,      p1.vel.heave);
    out.vel.roll_rate  = lerp(p0.vel.roll_rate,  p1.vel.roll_rate);
    out.vel.pitch_rate = lerp(p0.vel.pitch_rate, p1.vel.pitch_rate);
    out.vel.yaw_rate   = lerp(p0.vel.yaw_rate,   p1.vel.yaw_rate);

    // 加速度插值（如暂不使用，可忽略）
    out.accel.surge     = lerp(p0.accel.surge,     p1.accel.surge);
    out.accel.sway      = lerp(p0.accel.sway,      p1.accel.sway);
    out.accel.heave     = lerp(p0.accel.heave,     p1.accel.heave);
    out.accel.roll_acc  = lerp(p0.accel.roll_acc,  p1.accel.roll_acc);
    out.accel.pitch_acc = lerp(p0.accel.pitch_acc, p1.accel.pitch_acc);
    out.accel.yaw_acc   = lerp(p0.accel.yaw_acc,   p1.accel.yaw_acc);

    return out;
}

double TrajectoryTracking::wrap_angle(double ang)
{
    constexpr double PI     = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;

    if (std::isnan(ang) || std::isinf(ang)) {
        return 0.0;
    }

    ang = std::fmod(ang + PI, TWO_PI);
    if (ang < 0.0) {
        ang += TWO_PI;
    }
    return ang - PI;
}

} // namespace rovctrl::control_core
