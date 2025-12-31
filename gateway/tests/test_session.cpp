#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "gateway/bytes.hpp"
#include "gateway/codec/gcs_codec.hpp"
#include "gateway/session/gcs_session.hpp"
#include "proto_gcs/gcs_protocol.hpp"

namespace {

// ======================== printable helpers ========================

// 把 enum class / byte / 小整数统一转成可打印的整数
template <class T>
static inline auto to_printable(T v)
{
    if constexpr (std::is_enum_v<T>) {
        return static_cast<std::underlying_type_t<T>>(v);
    } else if constexpr (std::is_same_v<T, std::uint8_t> || std::is_same_v<T, std::int8_t>) {
        return static_cast<int>(v); // 避免按 char 打印
    } else {
        return v;
    }
}

// ======================== tiny test macros ========================

#define TEST_CHECK(cond)                                                                          \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " CHECK(" #cond ") failed\n";                                            \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

#define TEST_EQ(a, b)                                                                             \
    do {                                                                                          \
        auto _va = (a);                                                                           \
        auto _vb = (b);                                                                           \
        if (!((_va) == (_vb))) {                                                                  \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " EQ(" #a ", " #b ") failed. got=" << to_printable(_va)                  \
                      << " expect=" << to_printable(_vb) << "\n";                                 \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

// ======================== aliases ========================

using comm_gcs::Byte;
using comm_gcs::BytesView;
using comm_gcs::UdpAddress;

using comm_gcs::codec::ParsedPacket;

using rovctrl::io::gcs::AckCode;
using rovctrl::io::gcs::MsgType;
using rovctrl::io::gcs::PacketHeader;

// ======================== helpers ========================

static std::optional<ParsedPacket> parse_pkt(const std::vector<Byte>& pkt,
                                             AckCode* ec = nullptr,
                                             std::string* emsg = nullptr)
{
    AckCode tmp_ec{};
    std::string tmp_msg;
    return comm_gcs::codec::parse_and_validate(
        BytesView{pkt.data(), pkt.size()},
        ec ? ec : &tmp_ec,
        emsg ? emsg : &tmp_msg
    );
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
        static_cast<std::uint32_t>(payload.size) // BytesView uses `.size`
    );
    return comm_gcs::codec::build_packet(h, payload);
}

// 找到 outs 里第一个指定 msg_type 的包，并 parse 成 ParsedPacket
static std::optional<ParsedPacket> find_first_parsed(const std::vector<std::vector<Byte>>& outs, MsgType mt)
{
    for (const auto& pkt : outs) {
        auto pp = parse_pkt(pkt);
        if (!pp) continue;
        if (static_cast<MsgType>(pp->hdr.msg_type) == mt) return pp;
    }
    return std::nullopt;
}

// 断言一个 ACK 包（hdr.ack_seq + payload.ack_code）
static int expect_ack(const ParsedPacket& pp, std::uint32_t expect_ack_seq, AckCode expect_code)
{
    using namespace rovctrl::io::gcs;

    TEST_EQ(static_cast<MsgType>(pp.hdr.msg_type), MsgType::ACK);
    TEST_EQ(pp.hdr.ack_seq, expect_ack_seq);
    TEST_EQ(pp.hdr.payload_len, static_cast<std::uint32_t>(sizeof(AckPayload)));

    AckPayload ap{};
    std::memcpy(&ap, pp.payload.data, sizeof(ap));
    TEST_EQ(ap.ack_code, static_cast<std::uint16_t>(expect_code));
    return 0;
}

// 完成握手：CONNECT_REQ -> CONNECT_ACK -> CONNECT_CONFIRM(ACK_REQ) -> ACK(OK)
// 返回 session_id；并可选输出 rov_nonce（便于调试）
static std::uint64_t do_handshake(comm_gcs::session::GcsSession& sess,
                                 const UdpAddress& from,
                                 std::uint64_t* rov_nonce_out = nullptr)
{
    using namespace rovctrl::io::gcs;

    // 1) CONNECT_REQ
    ConnectReq req{};
    req.gcs_nonce = 0x1111222233334444ull;
    auto req_bytes = comm_gcs::codec::to_bytes_vec(req);

    auto pkt_req = build_pkt(
        MsgType::CONNECT_REQ,
        /*seq*/1,
        /*session*/0,
        /*flags*/0,
        BytesView{req_bytes.data(), req_bytes.size()}
    );

    auto outs1 = sess.on_packet(from, BytesView{pkt_req.data(), pkt_req.size()});
    auto pp_ack_opt = find_first_parsed(outs1, MsgType::CONNECT_ACK);
    TEST_CHECK(pp_ack_opt.has_value());

    const ParsedPacket& pp_ack = *pp_ack_opt;
    TEST_EQ(pp_ack.hdr.payload_len, static_cast<std::uint32_t>(sizeof(ConnectAck)));
    TEST_CHECK(pp_ack.hdr.session_id != 0);

    ConnectAck ca{};
    std::memcpy(&ca, pp_ack.payload.data, sizeof(ca));
    TEST_EQ(ca.gcs_nonce_echo, req.gcs_nonce);
    TEST_CHECK(ca.rov_nonce != 0);

    if (rov_nonce_out) *rov_nonce_out = ca.rov_nonce;

    const std::uint64_t sid = pp_ack.hdr.session_id;

    // 2) CONNECT_CONFIRM (ACK_REQ)
    ConnectConfirm cc{};
    cc.rov_nonce_echo = ca.rov_nonce;
    auto cc_bytes = comm_gcs::codec::to_bytes_vec(cc);

    auto pkt_cc = build_pkt(
        MsgType::CONNECT_CONFIRM,
        /*seq*/2,
        /*session*/sid,
        /*flags*/FLAG_ACK_REQ,
        BytesView{cc_bytes.data(), cc_bytes.size()}
    );

    auto outs2 = sess.on_packet(from, BytesView{pkt_cc.data(), pkt_cc.size()});
    auto pp2_opt = find_first_parsed(outs2, MsgType::ACK);
    TEST_CHECK(pp2_opt.has_value());
    (void)expect_ack(*pp2_opt, /*ack_seq*/2u, AckCode::OK);

    return sid;
}

// ======================== tests ========================

static int test_handshake_establish()
{
    using namespace comm_gcs::session;

    bool cb_established = false;
    std::uint64_t cb_sid = 0;

    GcsSessionConfig cfg{};
    cfg.telem_hz = 10;
    cfg.require_session_for_commands = true;
    cfg.strict_session_check = true;

    GcsSessionEvents ev{};
    ev.on_session_established = [&](std::uint64_t sid, const comm_gcs::UdpAddress& peer) {
        (void)peer;
        cb_established = true;
        cb_sid = sid;
    };

    GcsSession sess(cfg, ev);
    const UdpAddress from{"127.0.0.1", 50000};

    const std::uint64_t sid = do_handshake(sess, from);

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

    auto pkt = build_pkt(
        MsgType::ESTOP,
        /*seq*/10,
        /*session*/0,
        /*flags*/FLAG_ACK_REQ,
        BytesView{cmd_bytes.data(), cmd_bytes.size()}
    );

    auto outs = sess.on_packet(from, BytesView{pkt.data(), pkt.size()});
    auto ack_opt = find_first_parsed(outs, MsgType::ACK);
    TEST_CHECK(ack_opt.has_value());

    return expect_ack(*ack_opt, /*ack_seq*/10u, AckCode::INVALID_SESSION);
}

static int test_dedup_seq_old_or_dup()
{
    using namespace comm_gcs::session;
    using namespace rovctrl::io::gcs;

    GcsSessionConfig cfg{};
    cfg.telem_hz = 10;
    cfg.require_session_for_commands = true;
    cfg.strict_session_check = true;

    GcsSessionEvents ev{};
    GcsSession sess(cfg, ev);

    const UdpAddress from{"127.0.0.1", 50002};
    const std::uint64_t sid = do_handshake(sess, from);

    // 发一个 SET_MODE seq=100
    SetModeCmd sm{};
    sm.mode = static_cast<std::uint8_t>(WireControlMode::Manual);
    auto sm_bytes = comm_gcs::codec::to_bytes_vec(sm);

    auto pkt1 = build_pkt(
        MsgType::SET_MODE,
        /*seq*/100,
        /*session*/sid,
        /*flags*/FLAG_ACK_REQ,
        BytesView{sm_bytes.data(), sm_bytes.size()}
    );

    auto outs1 = sess.on_packet(from, BytesView{pkt1.data(), pkt1.size()});
    auto ack1_opt = find_first_parsed(outs1, MsgType::ACK);
    TEST_CHECK(ack1_opt.has_value());
    (void)expect_ack(*ack1_opt, /*ack_seq*/100u, AckCode::OK);

    // 重复 seq=100 -> 应 SEQ_OLD_OR_DUP
    auto pkt2 = build_pkt(
        MsgType::SET_MODE,
        /*seq*/100,
        /*session*/sid,
        /*flags*/FLAG_ACK_REQ,
        BytesView{sm_bytes.data(), sm_bytes.size()}
    );

    auto outs2 = sess.on_packet(from, BytesView{pkt2.data(), pkt2.size()});
    auto ack2_opt = find_first_parsed(outs2, MsgType::ACK);
    TEST_CHECK(ack2_opt.has_value());

    return expect_ack(*ack2_opt, /*ack_seq*/100u, AckCode::SEQ_OLD_OR_DUP);
}

static int test_tick_status_rate_limit()
{
    using namespace comm_gcs::session;
    using namespace rovctrl::io::gcs;

    // 不追求严格时间学意义，只验证“不是每次 tick 都产出包”
    GcsSessionConfig cfg{};
    cfg.telem_hz = 5; // 5Hz -> 200ms/包
    cfg.require_session_for_commands = false;
    cfg.strict_session_check = false;

    GcsSessionEvents ev{};
    GcsSession sess(cfg, ev);

    // 让 session 有 peer：发送 CONNECT_REQ 即可
    const UdpAddress from{"127.0.0.1", 50003};
    ConnectReq req{};
    req.gcs_nonce = 1;
    auto req_bytes = comm_gcs::codec::to_bytes_vec(req);

    auto pkt_req = build_pkt(
        MsgType::CONNECT_REQ,
        /*seq*/1,
        /*session*/0,
        /*flags*/0,
        BytesView{req_bytes.data(), req_bytes.size()}
    );

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
