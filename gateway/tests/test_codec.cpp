#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "gateway/bytes.hpp"
#include "gateway/codec/gcs_codec.hpp"
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
                      << " EQ(" #a ", " #b ") failed: " << _va << " vs " << _vb << "\n";          \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

struct DummyPod {
    std::uint32_t a{0};
    std::uint32_t b{0};
};

static int test_build_and_parse_ok()
{
    using namespace comm_gcs::codec;
    using namespace rovctrl::io::gcs;

    DummyPod pod{};
    pod.a = 0x11223344u;
    pod.b = 0x55667788u;

    auto payload = to_bytes_vec(pod);
    PacketHeader h = make_header(
        static_cast<std::uint8_t>(MsgType::HEARTBEAT), // msg type arbitrary
        123,                                           // seq
        456,                                           // session_id
        0,                                             // flags
        static_cast<std::uint32_t>(payload.size())
    );

    auto pkt = build_packet(h, comm_gcs::BytesView{payload.data(), payload.size()});

    AckCode ec{};
    std::string emsg;
    auto pp = parse_and_validate(comm_gcs::BytesView{pkt.data(), pkt.size()}, &ec, &emsg);
    TEST_CHECK(pp.has_value());
    TEST_EQ(static_cast<std::uint16_t>(ec), static_cast<std::uint16_t>(AckCode::OK));
    TEST_CHECK(emsg.empty());

    TEST_EQ(pp->hdr.seq, 123u);
    TEST_EQ(pp->hdr.session_id, 456ull);
    TEST_EQ(pp->hdr.payload_len, static_cast<std::uint32_t>(sizeof(DummyPod)));
    TEST_CHECK(payload_size_is(pp->payload, sizeof(DummyPod)));

    DummyPod pod2{};
    std::memcpy(&pod2, pp->payload.data, sizeof(DummyPod));
    TEST_EQ(pod2.a, pod.a);
    TEST_EQ(pod2.b, pod.b);

    return 0;
}

static int test_payload_crc_fail_when_payload_tampered()
{
    using namespace comm_gcs::codec;
    using namespace rovctrl::io::gcs;

    DummyPod pod{};
    pod.a = 1;
    pod.b = 2;

    auto payload = to_bytes_vec(pod);
    PacketHeader h = make_header(
        static_cast<std::uint8_t>(MsgType::SET_DOF_CMD),
        10,
        0xABCDEFu,
        0,
        static_cast<std::uint32_t>(payload.size())
    );

    auto pkt = build_packet(h, comm_gcs::BytesView{payload.data(), payload.size()});
    TEST_CHECK(pkt.size() >= sizeof(PacketHeader) + 1);

    // 篡改 payload 的一个字节
    pkt[sizeof(PacketHeader) + 0] ^= 0xFF;

    AckCode ec{};
    std::string emsg;
    auto pp = parse_and_validate(comm_gcs::BytesView{pkt.data(), pkt.size()}, &ec, &emsg);
    TEST_CHECK(!pp.has_value());
    TEST_EQ(static_cast<std::uint16_t>(ec), static_cast<std::uint16_t>(AckCode::CRC_FAIL));
    TEST_CHECK(!emsg.empty());

    return 0;
}

static int test_header_crc_fail_when_header_crc_tampered()
{
    using namespace comm_gcs::codec;
    using namespace rovctrl::io::gcs;

    DummyPod pod{};
    pod.a = 7;
    pod.b = 9;

    auto payload = to_bytes_vec(pod);
    PacketHeader h = make_header(
        static_cast<std::uint8_t>(MsgType::SET_MODE),
        77,
        88,
        0,
        static_cast<std::uint32_t>(payload.size())
    );

    auto pkt = build_packet(h, comm_gcs::BytesView{payload.data(), payload.size()});
    TEST_CHECK(pkt.size() >= sizeof(PacketHeader));

    // 篡改 header_crc32c 字段（位于 PacketHeader 内部）
    PacketHeader hh{};
    std::memcpy(&hh, pkt.data(), sizeof(PacketHeader));
    hh.header_crc32c ^= 0x12345678u;
    std::memcpy(pkt.data(), &hh, sizeof(PacketHeader));

    AckCode ec{};
    std::string emsg;
    auto pp = parse_and_validate(comm_gcs::BytesView{pkt.data(), pkt.size()}, &ec, &emsg);
    TEST_CHECK(!pp.has_value());
    TEST_EQ(static_cast<std::uint16_t>(ec), static_cast<std::uint16_t>(AckCode::CRC_FAIL));
    TEST_CHECK(!emsg.empty());

    return 0;
}

} // namespace

int main()
{
    int rc = 0;

    rc = test_build_and_parse_ok();
    if (rc != 0) return rc;

    rc = test_payload_crc_fail_when_payload_tampered();
    if (rc != 0) return rc;

    rc = test_header_crc_fail_when_header_crc_tampered();
    if (rc != 0) return rc;

    std::cout << "[test_codec] all tests passed.\n";
    return 0;
}
