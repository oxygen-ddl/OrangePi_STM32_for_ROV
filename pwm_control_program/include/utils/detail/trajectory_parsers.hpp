#pragma once
#ifndef ROVCTRL_UTILS_DETAIL_TRAJECTORY_PARSERS_HPP
#define ROVCTRL_UTILS_DETAIL_TRAJECTORY_PARSERS_HPP

#include <string>

#include "control_core/trajectory_tracking.hpp"

namespace rovctrl::utils::detail {

inline control_core::TrajectoryFrame parse_frame(const std::string& s)
{
    if (s == "NED" || s == "ned") return control_core::TrajectoryFrame::NED;
    if (s == "ENU" || s == "enu") return control_core::TrajectoryFrame::ENU;
    return control_core::TrajectoryFrame::Unknown;
}

inline control_core::AngleUnit parse_angle_unit(const std::string& s)
{
    if (s == "rad" || s == "RAD") return control_core::AngleUnit::Rad;
    if (s == "deg" || s == "DEG") return control_core::AngleUnit::Deg;
    return control_core::AngleUnit::Unknown;
}

inline control_core::TrajectoryType parse_traj_type(const std::string& s)
{
    if (s == "piecewise" || s == "PIECEWISE") {
        return control_core::TrajectoryType::Piecewise;
    }
    return control_core::TrajectoryType::Unknown;
}

} // namespace rovctrl::utils::detail

#endif
