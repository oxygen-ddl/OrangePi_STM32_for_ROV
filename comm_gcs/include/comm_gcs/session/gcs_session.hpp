#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "comm_gcs/bytes.hpp"
#include "comm_gcs/udp_endpoint.hpp"          // for UdpAddress
#include "comm_gcs/codec/gcs_codec.hpp"       // ParsedPacket + build_packet/ack/parse
#include "proto_gcs/gcs_protocol.hpp"            // wire protocol

namespace comm_gcs::session {

struct GcsSessionConfig {
    // STATUS telemetry sending rate (Hz). 0 disables periodic status.
    int telem_hz{10};

    // If true: reject command packets when session not established.
    bool require_session_for_commands{true};

    // If true: for packets with bad CRC/format, send ACK(BAD_FORMAT/CRC_FAIL) if we can.
    // Note: for totally broken packets we may not be able to recover seq/session_id.
    bool ack_on_parse_error{false};

    // If true: session must match on non-handshake messages (including HEARTBEAT if session_id != 0).
    bool strict_session_check{true};
};

struct GcsSessionState {
    bool        established{false};
    bool        link_alive{false};
    bool        estop{false};

    std::uint64_t session_id{0};
    std::uint64_t gcs_nonce{0};
    std::uint64_t rov_nonce{0};

    std::uint32_t last_cmd_seq{0};  // dedup for command class packets
    bool          have_peer{false};
    UdpAddress    peer{};
};

struct GcsSessionEvents {
    // Called when a valid command is accepted.
    std::function<void(const rovctrl::io::gcs::SetDofCmd&)>  on_set_dof;
    std::function<void(const rovctrl::io::gcs::SetModeCmd&)> on_set_mode;
    std::function<void(const rovctrl::io::gcs::EstopCmd&)>   on_estop;

    // Optional: observe handshake progression.
    std::function<void(std::uint64_t session_id)> on_session_established;
    std::function<void()> on_session_lost;
};

class GcsSession {
public:
    explicit GcsSession(GcsSessionConfig cfg = {}, GcsSessionEvents ev = {});

    // Reset the entire session state (drops established session).
    void reset();

    const GcsSessionState& state() const noexcept { return st_; }
    const GcsSessionConfig& config() const noexcept { return cfg_; }

    // Process one UDP packet. Returns zero or more outgoing packets (already encoded wire bytes).
    std::vector<std::vector<comm_gcs::Byte>> on_packet(const UdpAddress& from, BytesView raw);

    // Provide current status (wire struct) and let session decide whether to emit STATUS by rate limiter.
    // Returns an encoded packet if emitted; otherwise std::nullopt.
    std::optional<std::vector<comm_gcs::Byte>> tick_status(const rovctrl::io::gcs::StatusTelemetry& st_wire);

    // Force build a STATUS packet immediately (no rate limit).
    std::vector<comm_gcs::Byte> build_status_now(const rovctrl::io::gcs::StatusTelemetry& st_wire);

    // Convenience: build an ACK packet (rarely needed externally).
    std::vector<comm_gcs::Byte> build_ack(std::uint32_t ack_seq, rovctrl::io::gcs::AckCode code);

private:
    // Internal helpers
    std::vector<std::vector<comm_gcs::Byte>> handle_parsed_(const UdpAddress& from,
                                                            const comm_gcs::codec::ParsedPacket& pp);

    bool session_id_valid_(std::uint64_t sid) const noexcept;

    std::vector<comm_gcs::Byte> make_connect_ack_(std::uint32_t seq_tx, const UdpAddress& to);
    std::vector<comm_gcs::Byte> make_ack_if_req_(const rovctrl::io::gcs::PacketHeader& in_hdr,
                                                 rovctrl::io::gcs::AckCode code);

    bool is_command_type_(std::uint8_t msg_type) const noexcept;
    bool dedup_cmd_seq_ok_(std::uint32_t seq_in, rovctrl::io::gcs::AckCode* out_code) noexcept;

    std::uint64_t rand_u64_();

private:
    GcsSessionConfig cfg_{};
    GcsSessionEvents ev_{};

    GcsSessionState st_{};

    // TX sequence for server->gcs packets (CONNECT_ACK/ACK/STATUS)
    std::uint32_t tx_seq_{1};

    // Telemetry rate limiter
    std::uint64_t next_telem_ns_{0};
};

} // namespace comm_gcs::session
