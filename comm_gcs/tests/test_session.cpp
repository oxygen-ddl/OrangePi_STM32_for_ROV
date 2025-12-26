#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "comm_gcs/bytes.hpp"
#include "comm_gcs/codec/gcs_codec.hpp"
#include "comm_gcs/session/gcs_session.hpp"
#include "proto_gcs/gcs_protocol.hpp"

namespace {

#define TEST_CHECK(cond)                                                                          \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__ << " CHECK(" #cond ") failed\n";\
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

#define TEST_EQ(a, b)                                                                             \
    do {                                                                                          \
        auto _va = (a);                                                                           \
        auto _vb = (b);                                                                           \
        if (!((_va) == (_vb))) {                                                                  \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " EQ(" #a ", " #b ") failed\n";                                          \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

using comm_gcs::Byte;
using comm_gcs::BytesView;
using comm_gcs::UdpAddress;

using rovctrl::io::gcs::AckCode;
using rovctrl::io::gcs::MsgType;
using rovctrl::io::gcs::PacketHeader;

static std::optional<comm_gcs::codec::ParsedPacket> parse_pkt(const std::vector<Byte>& pkt,
                                                              AckCode* ec = nullptr,
                                                              std::string* emsg = nullptr)
{
    AckCode tmp_ec{};
    std::string tmp_msg;
    auto pp = comm_gcs::codec::parse_and_validate(
        BytesView{pkt.data(), pkt.size()},
        ec ? ec : &tmp_ec,
        emsg ? emsg : &tmp_msg
    );
    return pp;
}

static std::vector<Byte> build_pkt(MsgType mt,
                                  std::uint32_t seq,
                                  std::uint64_t session_id,
                                  std::uint16_t flags,
                                  BytesView payload)
{
    PacketHeader h = comm_gcs::codec::make_header(
        static_cast<std::uint8_t>(mt),
        seq,
        session_id,
        flags,
        static_cast<std::uint32_t>(payload.size)
    );
    return comm_gcs::codec::build_packet(h, payload);
}

// 找到 outs 里第一个指定 msg_type 的包（用于握手 ack / ack 包）
static std::optional<std::vector<Byte>> find_first_type(const std::vector<std::vector<Byte>>& outs, MsgType mt)
{
    for (const auto& pkt : outs) {
        auto pp = parse_pkt(pkt);
        if (!pp) continue;
        if (static_cast<MsgType>(pp->hdr.msg_type) == mt) return pkt;
    }
    return std::nullopt;
}

static int test_handshake_establish()
{
    using namespace comm_gcs::session;
    using namespace rovctrl::io::gcs;

    bool cb_established = false;
    std::uint64_t cb_sid = 0;

    GcsSessionConfig cfg{};
    cfg.telem_hz = 10;
    cfg.require_session_for_commands = true;
    cfg.strict_session_check = true;

    GcsSessionEvents ev{};
    ev.on_session_established = [&](std::uint64_t sid){
        cb_established = true;
        cb_sid = sid;
    };

    GcsSession sess(cfg, ev);

    const UdpAddress from{"127.0.0.1", 50000};

    // 1) CONNECT_REQ
    ConnectReq req{};
    req.gcs_nonce = 0x1111222233334444ull;
    auto req_bytes = comm_gcs::codec::to_bytes_vec(req);

    auto pkt_req = build_pkt(MsgType::CONNECT_REQ, /*seq*/1, /*session*/0,
                             /*flags*/0, BytesView{req_bytes.data(), req_bytes.size()});

    auto outs1 = sess.on_packet(from, BytesView{pkt_req.data(), pkt_req.size()});
    // 期望：立即给 CONNECT_ACK
    auto ack_pkt_opt = find_first_type(outs1, MsgType::CONNECT_ACK);
    TEST_CHECK(ack_pkt_opt.has_value());

    auto pp_ack = parse_pkt(*ack_pkt_opt);
    TEST_CHECK(pp_ack.has_value());
    TEST_EQ(static_cast<MsgType>(pp_ack->hdr.msg_type), MsgType::CONNECT_ACK);
    TEST_CHECK(pp_ack->hdr.session_id != 0);

    // 解析 CONNECT_ACK payload
    TEST_EQ(pp_ack->hdr.payload_len, static_cast<std::uint32_t>(sizeof(ConnectAck)));
    ConnectAck ca{};
    std::memcpy(&ca, pp_ack->payload.data, sizeof(ca));
    TEST_EQ(ca.gcs_nonce_echo, req.gcs_nonce);
    TEST_CHECK(ca.rov_nonce != 0);

    const std::uint64_t sid = pp_ack->hdr.session_id;

    // 2) CONNECT_CONFIRM（带回 rov_nonce）
    ConnectConfirm cc{};
    cc.rov_nonce_echo = ca.rov_nonce;
    auto cc_bytes = comm_gcs::codec::to_bytes_vec(cc);

    // 这里 flags 带 ACK_REQ，期望 session 回 ACK(OK)
    auto pkt_cc = build_pkt(MsgType::CONNECT_CONFIRM, /*seq*/2, sid,
                            /*flags*/FLAG_ACK_REQ, BytesView{cc_bytes.data(), cc_bytes.size()});

    auto outs2 = sess.on_packet(from, BytesView{pkt_cc.data(), pkt_cc.size()});

    auto ack2_opt = find_first_type(outs2, MsgType::ACK);
    TEST_CHECK(ack2_opt.has_value());

    auto pp2 = parse_pkt(*ack2_opt);
    TEST_CHECK(pp2.has_value());
    TEST_EQ(static_cast<MsgType>(pp2->hdr.msg_type), MsgType::ACK);

    // payload: AckPayload
    TEST_EQ(pp2->hdr.payload_len, static_cast<std::uint32_t>(sizeof(AckPayload)));
    AckPayload ap{};
    std::memcpy(&ap, pp2->payload.data, sizeof(ap));
    TEST_EQ(ap.ack_seq, 2u);
    TEST_EQ(ap.ack_code, static_cast<std::uint16_t>(AckCode::OK));

    // 回调应触发
    TEST_CHECK(cb_established);
    TEST_EQ(cb_sid, sid);

    return 0;
}

static int test_command_rejected_before_established()
{
    using namespace comm_gcs::session;
    using namespace rovctrl::io::gcs;

    GcsSessionConfig cfg{};
    cfg.telem_hz = 10;
    cfg.require_session_for_commands = true;  // 未建立 session 不能执行命令
    cfg.strict_session_check = true;

    GcsSessionEvents ev{};
    GcsSession sess(cfg, ev);

    const UdpAddress from{"127.0.0.1", 50001};

    // 未握手，直接发 ESTOP（带 ACK_REQ）
    EstopCmd cmd{};
    cmd.enable = 1;
    auto cmd_bytes = comm_gcs::codec::to_bytes_vec(cmd);

    auto pkt = build_pkt(MsgType::ESTOP, /*seq*/10, /*session*/0,
                         /*flags*/FLAG_ACK_REQ, BytesView{cmd_bytes.data(), cmd_bytes.size()});

    auto outs = sess.on_packet(from, BytesView{pkt.data(), pkt.size()});
    auto ack_opt = find_first_type(outs, MsgType::ACK);
    TEST_CHECK(ack_opt.has_value());

    auto pp = parse_pkt(*ack_opt);
    TEST_CHECK(pp.has_value());

    AckPayload ap{};
    std::memcpy(&ap, pp->payload.data, sizeof(ap));
    TEST_EQ(ap.ack_seq, 10u);
    TEST_EQ(ap.ack_code, static_cast<std::uint16_t>(AckCode::INVALID_SESSION));

    return 0;
}

static int test_dedup_seq_old_or_dup()
{
    using namespace comm_gcs::session;
    using namespace rovctrl::io::gcs;

    // 先完成握手建立 session，然后用相同 seq 重复发命令 -> SEQ_OLD_OR_DUP
    bool established = false;
    std::uint64_t sid = 0;
    std::uint64_t rov_nonce = 0;

    GcsSessionConfig cfg{};
    cfg.telem_hz = 10;
    cfg.require_session_for_commands = true;
    cfg.strict_session_check = true;

    GcsSessionEvents ev{};
    ev.on_session_established = [&](std::uint64_t s){ established = true; sid = s; };

    GcsSession sess(cfg, ev);
    const UdpAddress from{"127.0.0.1", 50002};

    // CONNECT_REQ
    rovctrl::io::gcs::ConnectReq req{};
    req.gcs_nonce = 0xAABBCCDD11223344ull;
    auto req_bytes = comm_gcs::codec::to_bytes_vec(req);
    auto pkt_req = build_pkt(MsgType::CONNECT_REQ, 1, 0, 0, BytesView{req_bytes.data(), req_bytes.size()});
    auto outs1 = sess.on_packet(from, BytesView{pkt_req.data(), pkt_req.size()});
    auto ack_pkt_opt = find_first_type(outs1, MsgType::CONNECT_ACK);
    TEST_CHECK(ack_pkt_opt.has_value());

    auto pp_ack = parse_pkt(*ack_pkt_opt);
    TEST_CHECK(pp_ack.has_value());
    sid = pp_ack->hdr.session_id;

    rovctrl::io::gcs::ConnectAck ca{};
    std::memcpy(&ca, pp_ack->payload.data, sizeof(ca));
    rov_nonce = ca.rov_nonce;
    TEST_CHECK(sid != 0 && rov_nonce != 0);

    // CONNECT_CONFIRM
    rovctrl::io::gcs::ConnectConfirm cc{};
    cc.rov_nonce_echo = rov_nonce;
    auto cc_bytes = comm_gcs::codec::to_bytes_vec(cc);
    auto pkt_cc = build_pkt(MsgType::CONNECT_CONFIRM, 2, sid, FLAG_ACK_REQ, BytesView{cc_bytes.data(), cc_bytes.size()});
    (void)sess.on_packet(from, BytesView{pkt_cc.data(), pkt_cc.size()});
    TEST_CHECK(established);

    // 发一个 SET_MODE seq=100
    rovctrl::io::gcs::SetModeCmd sm{};
    sm.mode = static_cast<std::uint8_t>(WireControlMode::Manual);
    auto sm_bytes = comm_gcs::codec::to_bytes_vec(sm);

    auto pkt1 = build_pkt(MsgType::SET_MODE, 100, sid, FLAG_ACK_REQ, BytesView{sm_bytes.data(), sm_bytes.size()});
    auto outs_cmd1 = sess.on_packet(from, BytesView{pkt1.data(), pkt1.size()});
    auto ack1_opt = find_first_type(outs_cmd1, MsgType::ACK);
    TEST_CHECK(ack1_opt.has_value());

    // 重复 seq=100 再发一次 -> 应 SEQ_OLD_OR_DUP
    auto pkt2 = build_pkt(MsgType::SET_MODE, 100, sid, FLAG_ACK_REQ, BytesView{sm_bytes.data(), sm_bytes.size()});
    auto outs_cmd2 = sess.on_packet(from, BytesView{pkt2.data(), pkt2.size()});
    auto ack2_opt = find_first_type(outs_cmd2, MsgType::ACK);
    TEST_CHECK(ack2_opt.has_value());

    auto pp2 = parse_pkt(*ack2_opt);
    TEST_CHECK(pp2.has_value());
    rovctrl::io::gcs::AckPayload ap2{};
    std::memcpy(&ap2, pp2->payload.data, sizeof(ap2));
    TEST_EQ(ap2.ack_seq, 100u);
    TEST_EQ(ap2.ack_code, static_cast<std::uint16_t>(AckCode::SEQ_OLD_OR_DUP));

    return 0;
}

static int test_tick_status_rate_limit()
{
    using namespace comm_gcs::session;
    using namespace rovctrl::io::gcs;

    // 该测试不追求严格时间学意义，只验证“不是每次 tick 都产出包”
    GcsSessionConfig cfg{};
    cfg.telem_hz = 5; // 5Hz -> 200ms/包
    cfg.require_session_for_commands = false;
    cfg.strict_session_check = false;

    GcsSessionEvents ev{};
    GcsSession sess(cfg, ev);

    // 先让 session 认为“有 peer”，否则 tick_status 会直接 nullopt
    const UdpAddress from{"127.0.0.1", 50003};
    // 发送一个 CONNECT_REQ 让 have_peer=true 且 session_id 被生成（不一定要建立）
    ConnectReq req{};
    req.gcs_nonce = 1;
    auto req_bytes = comm_gcs::codec::to_bytes_vec(req);
    auto pkt_req = build_pkt(MsgType::CONNECT_REQ, 1, 0, 0, BytesView{req_bytes.data(), req_bytes.size()});
    (void)sess.on_packet(from, BytesView{pkt_req.data(), pkt_req.size()});

    StatusTelemetry st{};
    st.session_established = 0;
    st.link_alive = 1;
    st.estop = 0;
    st.mode = static_cast<std::uint8_t>(WireControlMode::Unknown);
    st.t_ns = 0;

    int produced = 0;
    int suppressed = 0;

    // 20 次 tick，每次间隔 20ms，总时长约 400ms
    for (int i = 0; i < 20; ++i) {
        auto pkt_opt = sess.tick_status(st);
        if (pkt_opt) produced++;
        else suppressed++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 5Hz 在 400ms 内理论上 <=2 次（考虑实现 next_telem_ns_ 初始为 now 可能更快产出 1 次）
    TEST_CHECK(produced >= 1);
    TEST_CHECK(suppressed >= 1);

    return 0;
}

} // namespace

int main()
{
    int rc = 0;

    rc = test_handshake_establish();
    if (rc != 0) return rc;

    rc = test_command_rejected_before_established();
    if (rc != 0) return rc;

    rc = test_dedup_seq_old_or_dup();
    if (rc != 0) return rc;

    rc = test_tick_status_rate_limit();
    if (rc != 0) return rc;

    std::cout << "[test_session] all tests passed.\n";
    return 0;
}
