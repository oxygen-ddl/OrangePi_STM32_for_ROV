#include "comm_gcs/session/gcs_session.hpp"

#include <chrono>
#include <cstring>
#include <random>

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
    : cfg_(cfg), ev_(std::move(ev))
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

std::vector<std::vector<comm_gcs::Byte>> GcsSession::on_packet(const UdpAddress& from, BytesView raw)
{
    // Record peer for potential STATUS TX
    st_.peer = from;
    st_.have_peer = true;
    st_.link_alive = true;

    AckCode ec{};
    std::string emsg;
    auto parsed = comm_gcs::codec::parse_and_validate(raw, &ec, &emsg);
    if (!parsed) {
        // Usually cannot safely ACK because header might be garbage.
        // If you want "best effort" ACK on parse error, you'd need partial header extraction.
        (void)emsg;
        return {};
    }

    return handle_parsed_(from, *parsed);
}

std::vector<std::vector<comm_gcs::Byte>> GcsSession::handle_parsed_(const UdpAddress& from,
                                                                    const ParsedPacket& pp)
{
    std::vector<std::vector<comm_gcs::Byte>> out;

    const PacketHeader& h = pp.hdr;
    const MsgType mt = static_cast<MsgType>(h.msg_type);

    // strict session check (handshake msgs are exempt)
    const bool is_handshake =
        (mt == MsgType::CONNECT_REQ) || (mt == MsgType::CONNECT_ACK) || (mt == MsgType::CONNECT_CONFIRM);

    if (!is_handshake && cfg_.strict_session_check) {
        // allow heartbeat with session_id=0 (optional heartbeat)
        if (mt == MsgType::HEARTBEAT) {
            if (h.session_id != 0 && !session_id_valid_(h.session_id)) {
                if (h.flags & FLAG_ACK_REQ) {
                    out.emplace_back(make_ack_if_req_(h, AckCode::INVALID_SESSION));
                }
                return out;
            }
        } else {
            if (!session_id_valid_(h.session_id)) {
                if (h.flags & FLAG_ACK_REQ) {
                    out.emplace_back(make_ack_if_req_(h, AckCode::INVALID_SESSION));
                }
                return out;
            }
        }
    }

    // Dedup for command packets
    if (is_command_type_(h.msg_type)) {
        AckCode dedup_code = AckCode::OK;
        if (!dedup_cmd_seq_ok_(h.seq, &dedup_code)) {
            if (h.flags & FLAG_ACK_REQ) {
                out.emplace_back(make_ack_if_req_(h, dedup_code));
            }
            return out;
        }
    }

    auto ack_ok_if_req = [&](){
        if (h.flags & FLAG_ACK_REQ) {
            out.emplace_back(make_ack_if_req_(h, AckCode::OK));
        }
    };

    switch (mt) {
    case MsgType::CONNECT_REQ: {
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(ConnectReq))) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::BAD_FORMAT));
            return out;
        }

        ConnectReq req{};
        std::memcpy(&req, pp.payload.data, sizeof(req));

        st_.gcs_nonce = req.gcs_nonce;
        st_.rov_nonce = rand_u64_();
        st_.session_id = rand_u64_();   // 简化：随机 session_id
        st_.established = false;
        st_.last_cmd_seq = 0;

        // send CONNECT_ACK immediately
        out.emplace_back(make_connect_ack_(tx_seq_++, from));
        return out;
    }

    case MsgType::CONNECT_CONFIRM: {
        if (!session_id_valid_(h.session_id)) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::INVALID_SESSION));
            return out;
        }
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(ConnectConfirm))) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::BAD_FORMAT));
            return out;
        }
        ConnectConfirm cc{};
        std::memcpy(&cc, pp.payload.data, sizeof(cc));

        if (cc.rov_nonce_echo != st_.rov_nonce) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::BAD_FORMAT));
            return out;
        }

        const bool was_established = st_.established;
        st_.established = true;

        ack_ok_if_req();

        if (!was_established && ev_.on_session_established) {
            ev_.on_session_established(st_.session_id);
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
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::INVALID_SESSION));
            return out;
        }
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(EstopCmd))) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::BAD_FORMAT));
            return out;
        }
        EstopCmd cmd{};
        std::memcpy(&cmd, pp.payload.data, sizeof(cmd));

        st_.estop = (cmd.enable != 0);

        if (ev_.on_estop) ev_.on_estop(cmd);
        ack_ok_if_req();
        return out;
    }

    case MsgType::SET_MODE: {
        if (cfg_.require_session_for_commands && !st_.established) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::INVALID_SESSION));
            return out;
        }
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(SetModeCmd))) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::BAD_FORMAT));
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
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::INVALID_SESSION));
            return out;
        }
        if (!comm_gcs::codec::payload_size_is(pp.payload, sizeof(SetDofCmd))) {
            if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::BAD_FORMAT));
            return out;
        }
        SetDofCmd cmd{};
        std::memcpy(&cmd, pp.payload.data, sizeof(cmd));

        if (ev_.on_set_dof) ev_.on_set_dof(cmd);
        ack_ok_if_req();
        return out;
    }

    case MsgType::STATUS:
        // Usually ROV->GCS, server side ignore
        if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::NOT_SUPPORTED));
        return out;

    case MsgType::ACK:
        // Server side ignore, or you can add RTT tracking later.
        return out;

    case MsgType::CONNECT_ACK:
        // Not expected on server side
        if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::NOT_SUPPORTED));
        return out;

    default:
        if (h.flags & FLAG_ACK_REQ) out.emplace_back(make_ack_if_req_(h, AckCode::NOT_SUPPORTED));
        return out;
    }
}

bool GcsSession::session_id_valid_(std::uint64_t sid) const noexcept
{
    return (st_.session_id != 0) && (sid == st_.session_id);
}

bool GcsSession::is_command_type_(std::uint8_t msg_type) const noexcept
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

std::vector<comm_gcs::Byte> GcsSession::make_connect_ack_(std::uint32_t seq_tx, const UdpAddress& /*to*/)
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

    return comm_gcs::codec::build_packet(hh, BytesView{payload.data(), payload.size()});
}

std::vector<comm_gcs::Byte> GcsSession::make_ack_if_req_(const PacketHeader& in_hdr, AckCode code)
{
    // We always build ACK with FLAG_IS_ACK; seq is server tx_seq_
    return comm_gcs::codec::build_ack(tx_seq_++, st_.session_id, in_hdr.seq, code);
}

std::vector<comm_gcs::Byte> GcsSession::build_ack(std::uint32_t ack_seq, AckCode code)
{
    // External convenience
    return comm_gcs::codec::build_ack(tx_seq_++, st_.session_id, ack_seq, code);
}

std::vector<comm_gcs::Byte> GcsSession::build_status_now(const StatusTelemetry& st_wire)
{
    auto payload = comm_gcs::codec::to_bytes_vec(st_wire);

    PacketHeader h = comm_gcs::codec::make_header(
        static_cast<std::uint8_t>(MsgType::STATUS),
        tx_seq_++,
        st_.session_id,
        0,
        static_cast<std::uint32_t>(payload.size())
    );

    return comm_gcs::codec::build_packet(h, BytesView{payload.data(), payload.size()});
}

std::optional<std::vector<comm_gcs::Byte>> GcsSession::tick_status(const StatusTelemetry& st_wire)
{
    if (cfg_.telem_hz <= 0) return std::nullopt;
    if (!st_.have_peer) return std::nullopt;

    const std::uint64_t now_ns = steady_now_ns();
    if (next_telem_ns_ == 0) {
        next_telem_ns_ = now_ns;
    }
    if (now_ns < next_telem_ns_) return std::nullopt;

    const std::uint64_t period_ns = static_cast<std::uint64_t>(1000000000ull / static_cast<std::uint64_t>(cfg_.telem_hz));
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
