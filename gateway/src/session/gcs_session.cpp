#include "gateway/session/gcs_session.hpp"

#include <chrono>
#include <cstring>
#include <random>
#include <utility>
#include <iostream>


namespace comm_gcs::session {

using namespace rovctrl::io::gcs;
using comm_gcs::codec::ParsedPacket;

static inline std::uint64_t steady_now_ns()
{
    using namespace std::chrono;
    return static_cast<std::uint64_t>(
        duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

GcsSession::GcsSession(GcsSessionConfig cfg, GcsSessionEvents ev)
    : cfg_(cfg)
    , ev_(std::move(ev))
{
    reset();
}

void GcsSession::reset()
{
    const bool was_established = st_.established;

    st_ = GcsSessionState{};
    tx_seq_ = 1;
    next_telem_ns_ = 0;

    if (was_established && ev_.on_session_lost) {
        ev_.on_session_lost();
    }
}

void GcsSession::maybe_timeout_reset_(std::uint64_t now_ns)
{
    if (!st_.established) return;
    if (cfg_.session_timeout_ms == 0) return;
    if (st_.last_rx_ns == 0) return;

    const std::uint64_t timeout_ns =
        static_cast<std::uint64_t>(cfg_.session_timeout_ms) * 1000ull * 1000ull;

    if (now_ns > st_.last_rx_ns && (now_ns - st_.last_rx_ns) > timeout_ns) {
        reset();
    }
}

GcsSession::PacketVec GcsSession::on_packet(const comm_gcs::UdpAddress& from, comm_gcs::BytesView raw)
{
    PacketVec out;

    const std::uint64_t now_ns = steady_now_ns();
    maybe_timeout_reset_(now_ns);

    // Record peer for potential STATUS TX
    st_.peer      = from;
    st_.have_peer = true;
    st_.link_alive = true;
    st_.last_rx_ns = now_ns;

    AckCode ec{};
    std::string emsg;
    auto parsed = comm_gcs::codec::parse_and_validate(raw, &ec, &emsg);
    if (!parsed) {
        st_.rx_bad++;

        // Best-effort ACK on parse error:
        // Only attempt if caller asked for it AND raw looks like it contains a header.
        if (cfg_.ack_on_parse_error && raw.data && raw.size >= sizeof(PacketHeader)) {
            PacketHeader h{};
            std::memcpy(&h, raw.data, sizeof(PacketHeader));

            // If ACK requested, respond with parse error code.
            // ec is already set by parse_and_validate() if it got far enough.
            if (auto ack = make_ack_if_req_(h, ec == AckCode::OK ? AckCode::BAD_FORMAT : ec)) {
                out.emplace_back(std::move(*ack));
            }
        }

        (void)emsg;
        return out;
    }

    st_.rx_ok++;
    auto outs = handle_parsed_(from, *parsed);
    for (auto& p : outs) out.emplace_back(std::move(p));
    return out;
}

GcsSession::PacketVec GcsSession::handle_parsed_(const comm_gcs::UdpAddress& from,
                                                 const ParsedPacket& pp)
{
    PacketVec out;

    const PacketHeader& h = pp.hdr;
    const MsgType mt = static_cast<MsgType>(h.msg_type);

    // strict session check (handshake msgs are exempt)
    const bool is_handshake =
        (mt == MsgType::CONNECT_REQ) ||
        (mt == MsgType::CONNECT_ACK) ||
        (mt == MsgType::CONNECT_CONFIRM);

    if (!is_handshake && cfg_.strict_session_check) {
        // allow heartbeat with session_id=0 (optional heartbeat)
        if (mt == MsgType::HEARTBEAT) {
            if (h.session_id != 0 && !session_id_valid_(h.session_id)) {
                if (auto ack = make_ack_if_req_(h, AckCode::INVALID_SESSION)) {
                    out.emplace_back(std::move(*ack));
                }
                return out;
            }
        } else {
            if (!session_id_valid_(h.session_id)) {
                if (auto ack = make_ack_if_req_(h, AckCode::INVALID_SESSION)) {
                    out.emplace_back(std::move(*ack));
                }
                return out;
            }
        }
    }

    // Dedup for command packets
    if (is_command_type_(h.msg_type)) {
        AckCode dedup_code = AckCode::OK;
        if (!dedup_cmd_seq_ok_(h.seq, &dedup_code)) {
            if (auto ack = make_ack_if_req_(h, dedup_code)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }
    }

    auto ack_ok_if_req = [&](){
        if (auto ack = make_ack_if_req_(h, AckCode::OK)) {
            out.emplace_back(std::move(*ack));
        }
    };

    switch (mt) {
    case MsgType::CONNECT_REQ: {
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(ConnectReq))) {
            if (auto ack = make_ack_if_req_(h, AckCode::BAD_FORMAT)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }

        ConnectReq req{};
        std::memcpy(&req, pp.payload.data, sizeof(req));

        st_.gcs_nonce   = req.gcs_nonce;
        st_.rov_nonce   = rand_u64_();
        st_.session_id  = rand_u64_();   // 简化：随机 session_id
        st_.established = false;
        st_.last_cmd_seq = 0;

        // send CONNECT_ACK immediately
        out.emplace_back(make_connect_ack_(tx_seq_++, from));
        return out;
    }

    case MsgType::CONNECT_CONFIRM: {
        // 临时 DEBUG
        std::cerr << "[RX_CONFIRM] from=" << from.ip << ":" << from.port
                  << " seq=" << h.seq
                  << " sid=" << h.session_id
                  << " flags=0x" << std::hex << int(h.flags) << std::dec
                  << " plen=" << h.payload_len
                  << "\n";

        // 1) session id 必须匹配
        if (!session_id_valid_(h.session_id)) {
            // 握手阶段：无条件回 ACK，让对端尽快失败而不是等超时
            out.emplace_back(build_ack(h.seq, AckCode::INVALID_SESSION));
            return out;
        }

        // 2) payload 必须正确
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(ConnectConfirm))) {
            out.emplace_back(build_ack(h.seq, AckCode::BAD_FORMAT));
            return out;
        }

        ConnectConfirm cc{};
        std::memcpy(&cc, pp.payload.data, sizeof(cc));

        // 3) nonce 必须正确
        if (cc.rov_nonce_echo != st_.rov_nonce) {
            out.emplace_back(build_ack(h.seq, AckCode::BAD_FORMAT));
            return out;
        }

        // 4) 状态更新
        const bool was_established = st_.established;
        st_.established = true;

        // 5) 关键：无条件 ACK(OK)，不依赖 FLAG_ACK_REQ
        out.emplace_back(build_ack(h.seq, AckCode::OK));

        if (!was_established && ev_.on_session_established) {
            ev_.on_session_established(st_.session_id, st_.peer);
        }
        return out;
    }

    case MsgType::HEARTBEAT: {
        // Heartbeat payload may be 0 or sizeof(Heartbeat)
        // We accept both; if present, you may inspect now_ms.
        ack_ok_if_req();
        return out;
    }

    case MsgType::ESTOP: {
        if (cfg_.require_session_for_commands && !st_.established) {
            if (auto ack = make_ack_if_req_(h, AckCode::INVALID_SESSION)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(EstopCmd))) {
            if (auto ack = make_ack_if_req_(h, AckCode::BAD_FORMAT)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }

        EstopCmd cmd{};
        std::memcpy(&cmd, pp.payload.data, sizeof(cmd));

        st_.estop = (cmd.enable != 0);

        if (ev_.on_estop) ev_.on_estop(cmd);
        ack_ok_if_req();
        return out;
    }

    case MsgType::ARM: {
        if (cfg_.require_session_for_commands && !st_.established) {
            if (auto ack = make_ack_if_req_(h, AckCode::INVALID_SESSION)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }

        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(ArmCmd))) {
            if (auto ack = make_ack_if_req_(h, AckCode::BAD_FORMAT)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }

        ArmCmd cmd{};
        std::memcpy(&cmd, pp.payload.data, sizeof(cmd));

        if (ev_.on_arm) {
            ev_.on_arm(cmd);
        }

        ack_ok_if_req();
        return out;
    }

    case MsgType::SET_MODE: {
        if (cfg_.require_session_for_commands && !st_.established) {
            if (auto ack = make_ack_if_req_(h, AckCode::INVALID_SESSION)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(SetModeCmd))) {
            if (auto ack = make_ack_if_req_(h, AckCode::BAD_FORMAT)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }

        SetModeCmd cmd{};
        std::memcpy(&cmd, pp.payload.data, sizeof(cmd));

        if (ev_.on_set_mode) ev_.on_set_mode(cmd);
        ack_ok_if_req();
        return out;
    }

    case MsgType::SET_DOF_CMD: {
        if (cfg_.require_session_for_commands && !st_.established) {
            if (auto ack = make_ack_if_req_(h, AckCode::INVALID_SESSION)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }

        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(SetDofCmd))) {
            if (auto ack = make_ack_if_req_(h, AckCode::BAD_FORMAT)) {
                out.emplace_back(std::move(*ack));
            }
            return out;
        }

        SetDofCmd cmd{};
        std::memcpy(&cmd, pp.payload.data, sizeof(cmd));

        if (ev_.on_set_dof) {
            ev_.on_set_dof(cmd);
        }

        ack_ok_if_req();
        return out;
    }

    case MsgType::STATUS: {
        // Usually ROV->GCS, server side ignore
        if (auto ack = make_ack_if_req_(h, AckCode::NOT_SUPPORTED)) {
            out.emplace_back(std::move(*ack));
        }
        return out;
    }

    case MsgType::ACK: {
        // Server side ignore, or you can add RTT tracking later.
        return out;
    }

    case MsgType::CONNECT_ACK: {
        // Not expected on server side
        if (auto ack = make_ack_if_req_(h, AckCode::NOT_SUPPORTED)) {
            out.emplace_back(std::move(*ack));
        }
        return out;
    }

    default: {
        if (auto ack = make_ack_if_req_(h, AckCode::NOT_SUPPORTED)) {
            out.emplace_back(std::move(*ack));
        }
        return out;
    }
    } // end switch

    // 理论上不会到这里
    return out;
}

bool GcsSession::session_id_valid_(std::uint64_t sid) const noexcept
{
    return (st_.session_id != 0) && (sid == st_.session_id);
}

bool GcsSession::is_command_type_(std::uint8_t msg_type) noexcept
{
    const MsgType mt = static_cast<MsgType>(msg_type);
    switch (mt) {
    case MsgType::SET_MODE:
    case MsgType::SET_DOF_CMD:
    case MsgType::ESTOP:
        return true;
    default:
        return false;
    }
}

bool GcsSession::dedup_cmd_seq_ok_(std::uint32_t seq_in, AckCode* out_code) noexcept
{
    // Simple monotonic check: reject old or duplicate
    if (seq_in <= st_.last_cmd_seq) {
        if (out_code) *out_code = AckCode::SEQ_OLD_OR_DUP;
        return false;
    }
    st_.last_cmd_seq = seq_in;
    if (out_code) *out_code = AckCode::OK;
    return true;
}

GcsSession::ByteVec GcsSession::make_connect_ack_(std::uint32_t seq_tx, const comm_gcs::UdpAddress& /*to*/)
{
    ConnectAck ack{};
    ack.gcs_nonce_echo = st_.gcs_nonce;
    ack.rov_nonce      = st_.rov_nonce;
    ack.rov_caps       = 0;
    ack.result_code    = 0;

    auto payload = comm_gcs::codec::to_bytes_vec(ack);
    PacketHeader hh = comm_gcs::codec::make_header(
        static_cast<std::uint8_t>(MsgType::CONNECT_ACK),
        seq_tx,
        st_.session_id,
        0,
        static_cast<std::uint32_t>(payload.size())
    );

    auto pkt = comm_gcs::codec::build_packet(
        hh,
        comm_gcs::BytesView{payload.data(), payload.size()}
    );

    std::cerr << "[DBG][TX_CONNECT_ACK] sizeof(PacketHeader)=" << sizeof(PacketHeader)
              << " sizeof(ConnectAck)=" << sizeof(ConnectAck)
              << " payload.size=" << payload.size()
              << " header.payload_len=" << hh.payload_len
              << " pkt.size=" << pkt.size()
              << "\n";

    // dump tail 40 bytes to see the extra 32
    std::cerr << "[DBG][TX_CONNECT_ACK] tail40=";
    size_t start = (pkt.size() > 40) ? (pkt.size() - 40) : 0;
    for (size_t i = start; i < pkt.size(); ++i) {
        fprintf(stderr, "%02X", (unsigned)pkt[i]);
    }
    std::cerr << "\n";
    return pkt;
}

std::optional<GcsSession::ByteVec>

GcsSession::make_ack_if_req_(const PacketHeader& in_hdr, AckCode code) 
{
    if ((in_hdr.flags & FLAG_ACK_REQ) == 0) return std::nullopt;

    // build_ack() always returns a full encoded packet
    return comm_gcs::codec::build_ack(tx_seq_++, st_.session_id, in_hdr.seq, code);
}

GcsSession::ByteVec
GcsSession::build_ack(std::uint32_t ack_seq, AckCode code)
{
    const std::uint32_t tx_seq = tx_seq_++;  // server->gcs TX 序号

    auto pkt = comm_gcs::codec::build_ack(
        tx_seq,
        st_.session_id,
        ack_seq,
        code
    );

    // DEBUG（现在这个非常关键）
    std::cerr << "[ACK_TX] tx_seq=" << tx_seq
              << " sid=" << st_.session_id
              << " ack_seq=" << ack_seq
              << " code=" << static_cast<std::uint16_t>(code)
              << " bytes=" << pkt.size()
              << "\n";

    return pkt;
}

GcsSession::ByteVec GcsSession::build_status_now(const StatusTelemetry& st_wire) const
{
    auto payload = comm_gcs::codec::to_bytes_vec(st_wire);

    PacketHeader h = comm_gcs::codec::make_header(
        static_cast<std::uint8_t>(MsgType::STATUS),
        tx_seq_++,
        st_.session_id,
        0,
        static_cast<std::uint32_t>(payload.size())
    );

    return comm_gcs::codec::build_packet(h, comm_gcs::BytesView{payload.data(), payload.size()});
}

std::optional<GcsSession::ByteVec> GcsSession::tick_status(const StatusTelemetry& st_wire)
{
    if (cfg_.telem_hz <= 0) return std::nullopt;
    if (!st_.have_peer) return std::nullopt;

    const std::uint64_t now_ns = steady_now_ns();
    maybe_timeout_reset_(now_ns);
    if (!st_.have_peer) return std::nullopt; // reset might drop it

    if (next_telem_ns_ == 0) {
        next_telem_ns_ = now_ns;
    }
    if (now_ns < next_telem_ns_) return std::nullopt;

    const std::uint64_t hz = static_cast<std::uint64_t>(cfg_.telem_hz);
    const std::uint64_t period_ns = (hz == 0) ? 0 : (1000000000ull / hz);
    next_telem_ns_ = now_ns + period_ns;

    return build_status_now(st_wire);
}

std::uint64_t GcsSession::rand_u64_()
{
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    return gen();
}

} // namespace comm_gcs::session
