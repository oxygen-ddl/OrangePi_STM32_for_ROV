// pwm_control_program/src/utils/config_loader_trajectory.cpp

#include "utils/config_loader.hpp"

#include <algorithm>
#include <exception>
#include <string>

#include <yaml-cpp/yaml.h>

#include "control_core/trajectory_tracking.hpp"
#include "utils/detail/config_log.hpp"
#include "utils/detail/trajectory_parsers.hpp"
#include "utils/detail/yaml_helpers.hpp"

namespace rovctrl::utils {

namespace fs = std::filesystem;
using detail::LogLevel;
using detail::log_line;

bool load_trajectory_config(const fs::path&                 path,
                            control_core::TrajectoryConfig& cfg,
                            std::ostream&                   log)
{
    try {
        log_line(log, "Traj", LogLevel::Info,
                 std::string("Loading trajectory.yaml: ") + path.string());

        YAML::Node root = YAML::LoadFile(path.string());

        if (!root["trajectory"]) {
            log_line(log, "Traj", LogLevel::Err,
                     "YAML missing 'trajectory' root node.");
            return false;
        }

        YAML::Node traj_node = root["trajectory"];

        if (traj_node["frame"]) {
            cfg.frame_raw = traj_node["frame"].as<std::string>();
            cfg.frame     = detail::parse_frame(cfg.frame_raw);
        } else {
            cfg.frame_raw = "";
            cfg.frame     = control_core::TrajectoryFrame::Unknown;
        }

        if (traj_node["angle_unit"]) {
            cfg.angle_unit_raw = traj_node["angle_unit"].as<std::string>();
            cfg.angle_unit     = detail::parse_angle_unit(cfg.angle_unit_raw);
        } else {
            cfg.angle_unit_raw = "";
            cfg.angle_unit     = control_core::AngleUnit::Unknown;
        }

        if (traj_node["type"]) {
            cfg.type_raw = traj_node["type"].as<std::string>();
            cfg.type     = detail::parse_traj_type(cfg.type_raw);
        } else {
            cfg.type_raw = "";
            cfg.type     = control_core::TrajectoryType::Unknown;
        }

        if (!traj_node["waypoints"] || !traj_node["waypoints"].IsSequence()) {
            log_line(log, "Traj", LogLevel::Err,
                     "'trajectory.waypoints' missing or not a sequence.");
            return false;
        }

        YAML::Node wps = traj_node["waypoints"];
        if (wps.size() == 0) {
            log_line(log, "Traj", LogLevel::Err,
                     "'trajectory.waypoints' is empty.");
            return false;
        }

        cfg.points.clear();
        cfg.points.reserve(wps.size());

        constexpr double PI = 3.14159265358979323846;
        auto yaw_to_rad = [&](double yaw_val) {
            if (cfg.angle_unit == control_core::AngleUnit::Deg) return yaw_val * PI / 180.0;
            return yaw_val;
        };

        for (std::size_t i = 0; i < wps.size(); ++i) {
            YAML::Node wp = wps[i];
            if (!wp["t"]) {
                log_line(log, "Traj", LogLevel::Err,
                         "waypoint[" + std::to_string(i) + "] missing 't'.");
                return false;
            }

            control_core::TrajectoryPoint pt;
            pt.t_s = wp["t"].as<double>();

            const double x   = detail::get_double_or(wp, "x", 0.0);
            const double y   = detail::get_double_or(wp, "y", 0.0);
            const double z   = detail::get_double_or(wp, "z", 0.0);
            const double yaw = detail::get_double_or(wp, "yaw", 0.0);

            pt.pose.x     = x;
            pt.pose.y     = y;
            pt.pose.z     = z;
            pt.pose.roll  = 0.0;
            pt.pose.pitch = 0.0;
            pt.pose.yaw   = yaw_to_rad(yaw);

            const double vx       = detail::get_double_or(wp, "vx", 0.0);
            const double vy       = detail::get_double_or(wp, "vy", 0.0);
            const double vz       = detail::get_double_or(wp, "vz", 0.0);
            const double yaw_rate = detail::get_double_or(wp, "yaw_rate", 0.0);

            pt.vel.surge      = vx;
            pt.vel.sway       = vy;
            pt.vel.heave      = vz;
            pt.vel.roll_rate  = 0.0;
            pt.vel.pitch_rate = 0.0;
            pt.vel.yaw_rate   = yaw_rate;

            pt.accel.surge     = 0.0;
            pt.accel.sway      = 0.0;
            pt.accel.heave     = 0.0;
            pt.accel.roll_acc  = 0.0;
            pt.accel.pitch_acc = 0.0;
            pt.accel.yaw_acc   = 0.0;

            cfg.points.push_back(pt);
        }

        std::sort(cfg.points.begin(), cfg.points.end(),
                  [](const control_core::TrajectoryPoint& a,
                     const control_core::TrajectoryPoint& b) { return a.t_s < b.t_s; });

        log_line(log, "Traj", LogLevel::Info,
                 "trajectory.yaml loaded OK: waypoints=" + std::to_string(cfg.points.size()) +
                 ", frame=" + cfg.frame_raw +
                 ", angle_unit=" + cfg.angle_unit_raw +
                 ", type=" + cfg.type_raw);

        return true;
    } catch (const std::exception& e) {
        log_line(log, "Traj", LogLevel::Err,
                 std::string("Failed to load trajectory.yaml: ") + e.what());
        return false;
    }
}

} // namespace rovctrl::utils
