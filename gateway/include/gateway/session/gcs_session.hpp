#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "gateway/bytes.hpp"
#include "gateway/udp/udp_endpoint.hpp"      // comm_gcs::UdpAddress
#include "gateway/codec/gcs_codec.hpp"   // ParsedPacket + build/parse helpers
#include "proto_gcs/gcs_protocol.hpp"     // wire protocol structs/enums

namespace comm_gcs::session {

// ------------------------------------
// Config / State / Events
// ------------------------------------

struct GcsSessionConfig final {
    // STATUS telemetry sending rate (Hz). 0 disables periodic status.
    int telem_hz{10};

    // If true: reject command packets when session not established.
    bool require_session_for_commands{true};

    // If true: for packets with bad CRC/format, send ACK(BAD_FORMAT/CRC_FAIL) if we can.
    // Note: for totally broken packets we may not be able to recover seq/session_id.
    bool ack_on_parse_error{false};

    // If true: session must match on non-handshake messages (including HEARTBEAT if session_id != 0).
    bool strict_session_check{true};

    // Optional: session timeout (ms). 0 disables timeout-based reset.
    std::uint32_t session_timeout_ms{1500};
};

struct GcsSessionState final {
    bool established{false};
    bool link_alive{false};
    bool estop{false};

    std::uint64_t session_id{0};
    std::uint64_t gcs_nonce{0};
    std::uint64_t rov_nonce{0};

    std::uint32_t last_cmd_seq{0};   // dedup for command class packets
    bool          have_peer{false};
    comm_gcs::UdpAddress peer{};

    // observability counters (optional, but very useful)
    std::uint64_t rx_ok{0};
    std::uint64_t rx_bad{0};

    // last time we saw any valid packet from current peer (steady ns)
    std::uint64_t last_rx_ns{0};
};

struct GcsSessionEvents final {
    // Called when a valid command is accepted.
    std::function<void(const rovctrl::io::gcs::SetDofCmd&)>  on_set_dof;
    std::function<void(const rovctrl::io::gcs::SetModeCmd&)> on_set_mode;
    std::function<void(const rovctrl::io::gcs::EstopCmd&)>   on_estop;
    std::function<void(const rovctrl::io::gcs::MotorTestCmd&)> on_motor_test;
    // ★ 新增：Arm / Disarm 事件
    std::function<void(const rovctrl::io::gcs::ArmCmd&)>     on_arm;

    // Optional: observe handshake progression.
    std::function<void(std::uint64_t session_id, const comm_gcs::UdpAddress& peer)> on_session_established;
    std::function<void()> on_session_lost;
};

// ------------------------------------
// Session
// ------------------------------------

class GcsSession final {
public:
    using ByteVec   = std::vector<comm_gcs::Byte>;
    using PacketVec = std::vector<ByteVec>;

    explicit GcsSession(GcsSessionConfig cfg = {}, GcsSessionEvents ev = {});

    // Reset the entire session state (drops established session).
    void reset();

    const GcsSessionState&  state()  const noexcept { return st_; }
    const GcsSessionConfig& config() const noexcept { return cfg_; }

    // Process one UDP packet. Returns zero or more outgoing packets (already encoded wire bytes).
    PacketVec on_packet(const comm_gcs::UdpAddress& from, comm_gcs::BytesView raw);

    // Provide current status (wire struct) and let session decide whether to emit STATUS by rate limiter.
    // Returns an encoded packet if emitted; otherwise std::nullopt.
    std::optional<ByteVec> tick_status(const rovctrl::io::gcs::StatusTelemetry& st_wire);

    // Force build a STATUS packet immediately (no rate limit).
    ByteVec build_status_now(const rovctrl::io::gcs::StatusTelemetry& st_wire) const;

    // Convenience: build an ACK packet (rarely needed externally).
    ByteVec build_ack(std::uint32_t ack_seq, rovctrl::io::gcs::AckCode code);

private:
    // Internal helpers
    PacketVec handle_parsed_(const comm_gcs::UdpAddress& from,
                             const comm_gcs::codec::ParsedPacket& pp);

    bool session_id_valid_(std::uint64_t sid) const noexcept;

    ByteVec make_connect_ack_(std::uint32_t seq_tx, const comm_gcs::UdpAddress& to);
    std::optional<ByteVec> make_ack_if_req_(const rovctrl::io::gcs::PacketHeader& in_hdr,
                                            rovctrl::io::gcs::AckCode code);

    static bool is_command_type_(std::uint8_t msg_type) noexcept;

    // Dedup command sequence; returns true if accepted. If rejected, out_code will be set.
    bool dedup_cmd_seq_ok_(std::uint32_t seq_in, rovctrl::io::gcs::AckCode* out_code) noexcept;

    static std::uint64_t rand_u64_();

    void maybe_timeout_reset_(std::uint64_t now_ns);

private:
    GcsSessionConfig cfg_{};
    GcsSessionEvents ev_{};

    GcsSessionState st_{};

    // TX sequence for server->gcs packets (CONNECT_ACK/ACK/STATUS)
    mutable std::uint32_t tx_seq_{1};

    // Telemetry rate limiter (steady ns)
    std::uint64_t next_telem_ns_{0};
};

} // namespace comm_gcs::session
