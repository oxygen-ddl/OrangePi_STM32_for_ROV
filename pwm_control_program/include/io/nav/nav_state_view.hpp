#pragma once
#ifndef ROVCTRL_IO_NAV_STATE_VIEW_HPP
#define ROVCTRL_IO_NAV_STATE_VIEW_HPP

#include <cstdint>
#include <type_traits>

namespace rovctrl::io {

/**
 * @brief Navigation state view for control loop (decoupled from shared memory layout)
 */
struct NavStateView {
    std::uint64_t t_ns = 0;

    double pos[3] = {};
    double vel[3] = {};
    double rpy[3] = {};

    double depth = 0.0;

    double omega_b[3] = {};
    double acc_b[3]   = {};

    std::uint16_t status_flags = 0;
    std::uint8_t  health       = 0;

    bool valid = false;
};

static_assert(std::is_trivially_copyable_v<NavStateView>,
              "NavStateView must be trivially copyable");

} // namespace rovctrl::io

#endif
