#include "io/input/gcs_input_adapter.hpp"

#include <array>
#include <cstring>
#include <random>

namespace rovctrl::io {

using namespace rovctrl::io::gcs;

static inline bool flag_has(std::uint16_t flags, std::uint16_t f) noexcept {
    return (flags & f) != 0;
}

static std::uint64_t gen_nonce64() {
    static thread_local std::mt19937_64 rng{std::random_device{}()};
    return rng();
}

bool GcsInputAdapter::read_header(const std::uint8_t* data, std::size_t len, PacketHeader& out) {
    if (!data || len < sizeof(PacketHeader)) return false;
    std::memcpy(&out, data, sizeof(PacketHeader));
    return true;
}

bool GcsInputAdapter::validate_packet(const PacketHeader& h,
                                      const std::uint8_t* payload,
                                      std::size_t plen,
                                      AckCode& err) {
    if (!header_basic_valid(h)) {
        err = AckCode::BAD_FORMAT;
        return false;
    }
    if (plen != static_cast<std::size_t>(h.payload_len)) {
        err = AckCode::BAD_FORMAT;
        return false;
    }

    const std::uint32_t hc = calc_header_crc(h);
    if (hc != h.header_crc32c) {
        err = AckCode::CRC_FAIL;
        return false;
    }

    if (h.payload_len > 0) {
        const std::uint32_t pc = crc32c(payload, plen);
        if (pc != h.payload_crc32c) {
            err = AckCode::CRC_FAIL;
            return false;
        }
    }

    err = AckCode::OK;
    return true;
}

void GcsInputAdapter::maybe_send_ack(UdpPeer peer, const PacketHeader& h, AckCode code) {
    if (!cfg_.enable_ack) return;
    if (!flag_has(h.flags, FLAG_ACK_REQ)) return;
    send_ack(peer, h.seq, code);
}

void GcsInputAdapter::send_ack(UdpPeer peer, std::uint32_t ack_seq, AckCode code) {
    if (!peer.valid()) return;

    PacketHeader h{};
    h.magic   = kMagic;
    h.version = kProtoVersion;
    h.msg_type = static_cast<std::uint8_t>(MsgType::ACK);
    h.flags    = FLAG_IS_ACK;
    h.seq      = 0;
    h.session_id = 0; // 当前最小实现：不引入真实 session_id
    h.payload_len = static_cast<std::uint32_t>(sizeof(AckPayload));
    h.send_time_ms = 0;

    AckPayload p{};
    p.ack_seq  = ack_seq;
    p.ack_code = static_cast<std::uint16_t>(code);
    p.reason   = 0;

    h.payload_crc32c = crc32c(&p, sizeof(p));
    h.header_crc32c  = calc_header_crc(h);

    std::array<std::uint8_t, sizeof(PacketHeader) + sizeof(AckPayload)> buf{};
    std::memcpy(buf.data(), &h, sizeof(h));
    std::memcpy(buf.data() + sizeof(h), &p, sizeof(p));

    send_(peer.ip_be, peer.port_be, buf.data(), buf.size());
    session_.inc_tx_ok();
}

static inline rovctrl::control_core::ControlMode map_wire_mode(std::uint8_t m) noexcept {
    switch (static_cast<WireControlMode>(m)) {
    case WireControlMode::Manual:   return rovctrl::control_core::ControlMode::kManual;
    case WireControlMode::Auto:     return rovctrl::control_core::ControlMode::kAuto;
    case WireControlMode::Failsafe: return rovctrl::control_core::ControlMode::kFailsafe;
    case WireControlMode::Unknown:
    default:                        return rovctrl::control_core::ControlMode::kUnknown;
    }
}

void GcsInputAdapter::on_packet(const std::uint8_t* data,
                                std::size_t len,
                                std::uint32_t src_ip_be,
                                std::uint16_t src_port_be,
                                std::uint64_t now_ns) {
    if (!data || len < sizeof(PacketHeader)) {
        session_.inc_rx_bad_format();
        return;
    }

    UdpPeer peer{src_ip_be, src_port_be};

    PacketHeader h{};
    if (!read_header(data, len, h)) {
        session_.inc_rx_bad_format();
        return;
    }

    const std::size_t plen = len - sizeof(PacketHeader);
    const std::uint8_t* payload = data + sizeof(PacketHeader);

    AckCode err = AckCode::OK;
    if (!validate_packet(h, payload, plen, err)) {
        // ACK 仍可回（帮助上位机停止重发）
        session_.inc_rx_bad_format();
        maybe_send_ack(peer, h, err);
        return;
    }

    // peer 接受（锁定/允许切换由 session 管）
    if (!session_.on_rx_packet(peer, now_ns)) {
        session_.inc_rx_bad_session();
        maybe_send_ack(peer, h, AckCode::INVALID_SESSION);
        return;
    }

    // 去重
    if (!session_.accept_seq(h.seq)) {
        session_.inc_rx_dup_or_old();
        if (cfg_.ack_on_dup) maybe_send_ack(peer, h, AckCode::SEQ_OLD_OR_DUP);
        return;
    }

    session_.inc_rx_ok();

    const MsgType mt = static_cast<MsgType>(h.msg_type);
    switch (mt) {
    case MsgType::CONNECT_REQ:
        if (!cfg_.enable_handshake) {
            maybe_send_ack(peer, h, AckCode::NOT_SUPPORTED);
            return;
        }
        handle_connect_req(peer, h, payload, plen);
        maybe_send_ack(peer, h, AckCode::OK);
        return;

    case MsgType::CONNECT_CONFIRM:
        if (!cfg_.enable_handshake) {
            maybe_send_ack(peer, h, AckCode::NOT_SUPPORTED);
            return;
        }
        handle_connect_confirm(h, payload, plen);
        maybe_send_ack(peer, h, AckCode::OK);
        return;

    case MsgType::HEARTBEAT:
        handle_heartbeat(now_ns);
        maybe_send_ack(peer, h, AckCode::OK);
        return;

    case MsgType::SET_MODE:
        if (cfg_.require_session_for_commands && !session_.session_established()) {
            maybe_send_ack(peer, h, AckCode::INVALID_SESSION);
            return;
        }
        handle_set_mode(h, payload, plen, now_ns);
        maybe_send_ack(peer, h, AckCode::OK);
        return;

    case MsgType::SET_DOF_CMD:
        if (cfg_.require_session_for_commands && !session_.session_established()) {
            maybe_send_ack(peer, h, AckCode::INVALID_SESSION);
            return;
        }
        handle_set_dof(h, payload, plen, now_ns);
        maybe_send_ack(peer, h, AckCode::OK);
        return;

    case MsgType::ESTOP:
        if (cfg_.require_session_for_commands && !session_.session_established()) {
            maybe_send_ack(peer, h, AckCode::INVALID_SESSION);
            return;
        }
        handle_estop(h, payload, plen, now_ns);
        maybe_send_ack(peer, h, AckCode::OK);
        return;

    // 这些通常是 ROV 端不会收到的
    case MsgType::ACK:
    case MsgType::CONNECT_ACK:
    case MsgType::STATUS:
    default:
        maybe_send_ack(peer, h, AckCode::NOT_SUPPORTED);
        return;
    }
}

// ---------------------------------------------------------------------------
// handshake
// ---------------------------------------------------------------------------

void GcsInputAdapter::handle_connect_req(UdpPeer peer,
                                        const PacketHeader& /*h*/,
                                        const std::uint8_t* payload,
                                        std::size_t plen) {
    if (plen != sizeof(ConnectReq)) {
        session_.inc_rx_bad_format();
        return;
    }

    ConnectReq req{};
    std::memcpy(&req, payload, sizeof(req));
    last_gcs_nonce_ = req.gcs_nonce;

    if (rov_nonce_ == 0) rov_nonce_ = gen_nonce64();

    // reply CONNECT_ACK
    PacketHeader rh{};
    rh.magic   = kMagic;
    rh.version = kProtoVersion;
    rh.msg_type = static_cast<std::uint8_t>(MsgType::CONNECT_ACK);
    rh.flags    = 0;
    rh.seq      = 0;
    rh.session_id = 0;
    rh.payload_len = static_cast<std::uint32_t>(sizeof(ConnectAck));
    rh.send_time_ms = 0;

    ConnectAck ack{};
    ack.gcs_nonce_echo = req.gcs_nonce;
    ack.rov_nonce      = rov_nonce_;
    ack.rov_caps       = 0;
    ack.result_code    = 0;

    rh.payload_crc32c = crc32c(&ack, sizeof(ack));
    rh.header_crc32c  = calc_header_crc(rh);

    std::array<std::uint8_t, sizeof(PacketHeader) + sizeof(ConnectAck)> buf{};
    std::memcpy(buf.data(), &rh, sizeof(rh));
    std::memcpy(buf.data() + sizeof(rh), &ack, sizeof(ack));

    send_(peer.ip_be, peer.port_be, buf.data(), buf.size());
    session_.inc_tx_ok();

    // 等 confirm 再认为 established
    session_.set_session_established(false);
}

void GcsInputAdapter::handle_connect_confirm(const PacketHeader& /*h*/,
                                            const std::uint8_t* payload,
                                            std::size_t plen) {
    if (plen != sizeof(ConnectConfirm)) {
        session_.inc_rx_bad_format();
        return;
    }

    ConnectConfirm cc{};
    std::memcpy(&cc, payload, sizeof(cc));

    if (rov_nonce_ != 0 && cc.rov_nonce_echo == rov_nonce_) {
        session_.set_session_established(true);
    } else {
        session_.set_session_established(false);
        session_.inc_rx_bad_session();
    }
}

// ---------------------------------------------------------------------------
// heartbeat
// ---------------------------------------------------------------------------

void GcsInputAdapter::handle_heartbeat(std::uint64_t now_ns) {
    session_.on_rx_heartbeat(now_ns);
}

// ---------------------------------------------------------------------------
// commands
// ---------------------------------------------------------------------------

void GcsInputAdapter::handle_set_mode(const PacketHeader& h,
                                      const std::uint8_t* payload,
                                      std::size_t plen,
                                      std::uint64_t now_ns) {
    (void)h;
    if (plen != sizeof(SetModeCmd)) {
        session_.inc_rx_bad_format();
        return;
    }

    SetModeCmd cmd{};
    std::memcpy(&cmd, payload, sizeof(cmd));

    rovctrl::control_core::ControlIntent intent{};
    intent.seq = static_cast<std::uint64_t>(h.seq);
    intent.stamp_ns = now_ns;
    intent.ttl_ms = cfg_.default_ttl_ms;

    intent.has_mode_request = true;
    intent.mode_request = map_wire_mode(cmd.mode);

    // 控制器选择：目前 ControlIntent 里没有字段承载 auto_controller
    // 所以此处不写入；未来你若要支持“SelectAutoController”，建议：
    // 1) 扩展 ControlIntent 增加 desired_auto_controller 字符串；
    // 2) 或改为写入 ControlReference/Mode(业务模式)。
    //
    // 这里先只做模式切换，保持一致性与可编译。

    sink_.submit_gcs_intent(intent);
    session_.on_rx_cmd(now_ns);
}

void GcsInputAdapter::handle_set_dof(const PacketHeader& h,
                                     const std::uint8_t* payload,
                                     std::size_t plen,
                                     std::uint64_t now_ns) {
    (void)h;
    if (plen != sizeof(SetDofCmd)) {
        session_.inc_rx_bad_format();
        return;
    }

    SetDofCmd cmd{};
    std::memcpy(&cmd, payload, sizeof(cmd));

    rovctrl::control_core::ControlIntent intent{};
    intent.seq = static_cast<std::uint64_t>(h.seq);
    intent.stamp_ns = now_ns;
    intent.ttl_ms = cfg_.default_ttl_ms;

    intent.has_teleop_dof = true;
    intent.teleop_dof_cmd.surge = cmd.dof[0];
    intent.teleop_dof_cmd.sway  = cmd.dof[1];
    intent.teleop_dof_cmd.heave = cmd.dof[2];
    intent.teleop_dof_cmd.roll  = cmd.dof[3];
    intent.teleop_dof_cmd.pitch = cmd.dof[4];
    intent.teleop_dof_cmd.yaw   = cmd.dof[5];

    sink_.submit_gcs_intent(intent);
    session_.on_rx_cmd(now_ns);
}

void GcsInputAdapter::handle_estop(const PacketHeader& h,
                                   const std::uint8_t* payload,
                                   std::size_t plen,
                                   std::uint64_t now_ns) {
    (void)h;
    if (plen != sizeof(EstopCmd)) {
        session_.inc_rx_bad_format();
        return;
    }

    EstopCmd cmd{};
    std::memcpy(&cmd, payload, sizeof(cmd));

    rovctrl::control_core::ControlIntent intent{};
    intent.seq = static_cast<std::uint64_t>(h.seq);
    intent.stamp_ns = now_ns;
    intent.ttl_ms = cfg_.default_ttl_ms;

    intent.has_estop_cmd = true;
    if (cmd.enable) {
        intent.estop = true;
        intent.clear_estop = false;
    } else {
        // 约定：enable=0 表示“解除急停”
        intent.estop = false;
        intent.clear_estop = true;
    }

    sink_.submit_gcs_intent(intent);
    session_.on_rx_cmd(now_ns);
}

} // namespace rovctrl::io
