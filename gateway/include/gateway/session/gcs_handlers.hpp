#pragma once

#include <cstdint>
#include <functional>

#include "proto_gcs/gcs_protocol.hpp"  // EstopCmd / SetModeCmd / SetDofCmd / StatusTelemetry etc.
#include "gateway/udp/udp_endpoint.hpp"   // UdpAddress

namespace comm_gcs::session {

/**
 * @brief Session-level events (already validated by session policy).
 *
 * Rules:
 *  - Callbacks should be fast and non-throwing (or session will treat as fault).
 *  - No cross-project includes here (only proto_gcs + comm_gcs basics).
 */
struct GcsSessionEvents final {
    // Session lifecycle
    std::function<void(std::uint64_t /*session_id*/, const comm_gcs::UdpAddress& /*peer*/)>
        on_session_established;

    std::function<void()> on_session_lost;

    // Commands (validated)
    std::function<void(const rovctrl::io::gcs::EstopCmd&)>   on_estop;
    std::function<void(const rovctrl::io::gcs::SetModeCmd&)> on_set_mode;
    std::function<void(const rovctrl::io::gcs::SetDofCmd&)>  on_set_dof;

    // Optional: raw packet hook for debug (after CRC/len ok, before policy)
    // std::function<void(const comm_gcs::UdpAddress&, comm_gcs::BytesView)> on_packet_rx_ok;
};

} // namespace comm_gcs::session
