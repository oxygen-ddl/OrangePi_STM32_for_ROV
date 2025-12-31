#pragma once
#ifndef ROVCTRL_IO_NAV_STATE_VIEW_HPP
#define ROVCTRL_IO_NAV_STATE_VIEW_HPP

#include <cstdint>
#include "shared/msg/nav_state_view.hpp"

namespace rovctrl::io {

/**
 * @brief Control-side navigation view.
 *
 * Design:
 *  - wire: the exact payload published by gateway (ABI-stable)
 *  - pub_*: publisher timestamps from SHM header (not part of wire payload)
 *  - age_ms_local: recomputed by control side based on pub_mono_ns
 *
 * Rule of thumb:
 *  - business/controls should read from this type, not from shared::msg directly.
 */
struct NavStateView final {
    // ---- wire payload (gateway->control ABI) ----
    shared::msg::NavStateView wire{};

    // ---- publisher meta (from SHM header) ----
    std::uint64_t pub_mono_ns = 0;
    std::uint64_t pub_wall_ns = 0;

    // ---- control-side computed meta ----
    std::uint32_t age_ms_local = 0;

    const shared::msg::NavStateView& payload() const noexcept { return wire; }
    shared::msg::NavStateView&       payload() noexcept { return wire; }
};

} // namespace rovctrl::io

// Optional compatibility alias for old namespace usage
namespace rovctrl::io::nav {
using NavStateView = rovctrl::io::NavStateView;
}

#endif // ROVCTRL_IO_NAV_STATE_VIEW_HPP
/**
 * @file    control_loop_nav.cpp
 * @brief   NavSub PIMPL + navigation feedback update (B2: NavStateView)
 */