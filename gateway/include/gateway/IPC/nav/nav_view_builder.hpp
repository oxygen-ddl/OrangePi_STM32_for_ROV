#pragma once
#ifndef COMM_GCS_IPC_NAV_NAV_VIEW_BUILDER_HPP
#define COMM_GCS_IPC_NAV_NAV_VIEW_BUILDER_HPP

#include <cstdint>
#include <cstring>

#include "shared/msg/nav_state.hpp"
#include "shared/msg/nav_state_view.hpp"

namespace comm_gcs::ipc::nav {

/**
 * @brief Convert shared::msg::NavState -> shared::msg::NavStateView
 *
 * Default policy (conservative):
 *   - valid = (health != UNINITIALIZED && health != INVALID)
 *   - copy kinematics fields directly
 *   - preserve status_flags/health for downstream decision-making
 *
 * Timebase:
 *   - NavState::t_ns is steady_clock ns (publisher side)
 *   - If your view includes age_ms or publish timestamps, compute outside and fill here.
 */
struct NavViewBuilder final {
    static shared::msg::NavStateView build(const shared::msg::NavState& s) noexcept;
};

} // namespace comm_gcs::ipc::nav

#endif // COMM_GCS_IPC_NAV_NAV_VIEW_BUILDER_HPP
