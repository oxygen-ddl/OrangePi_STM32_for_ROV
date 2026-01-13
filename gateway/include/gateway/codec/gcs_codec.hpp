#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>
#include <chrono>
#include <string_view>

#include "gateway/bytes.hpp"
#include "proto_gcs/gcs_protocol.hpp"

namespace comm_gcs::codec {

// 引入 GCS 协议基础类型
using rovctrl::io::gcs::PacketHeader;
using rovctrl::io::gcs::MsgType;
using rovctrl::io::gcs::AckCode;

struct ParsedPacket {
    PacketHeader hdr{};
    comm_gcs::BytesView payload{};
};

/// POD payload -> std::vector<uint8_t>
template <class T>
inline std::vector<comm_gcs::Byte> to_bytes_vec(const T& pod) {
    static_assert(std::is_trivially_copyable<T>::value, "payload must be trivially copyable");
    std::vector<comm_gcs::Byte> v(sizeof(T));
    std::memcpy(v.data(), &pod, sizeof(T));
    return v;
}

inline std::uint32_t now_steady_ms() {
    using namespace std::chrono;
    const auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    return static_cast<std::uint32_t>(ms);
}

inline std::uint64_t now_steady_ns() {
    using namespace std::chrono;
    const auto ns = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    return static_cast<std::uint64_t>(ns);
}

inline PacketHeader make_header(std::uint8_t msg_type,
                                std::uint32_t seq,
                                std::uint64_t session_id,
                                std::uint16_t flags,
                                std::uint32_t payload_len)
{
    PacketHeader h{};
    h.magic   = rovctrl::io::gcs::kMagic;
    h.version = rovctrl::io::gcs::kProtoVersion;
    h.msg_type    = msg_type;
    h.reserved0   = 0;
    h.flags       = flags;
    h.reserved1   = 0;
    h.seq         = seq;
    h.session_id  = session_id;
    h.payload_len = payload_len;

    h.ack_seq      = 0;     // IMPORTANT: default
    h.send_time_ms = 0;

    h.header_crc32c  = 0;
    h.payload_crc32c = 0;
    return h;
}


inline std::vector<comm_gcs::Byte> build_packet(const PacketHeader& h_in, comm_gcs::BytesView payload)
{
    PacketHeader h = h_in;

    // payload crc
    if (h.payload_len == 0) {
        h.payload_crc32c = 0;
    } else {
        h.payload_crc32c = rovctrl::io::gcs::crc32c(payload.data, payload.size);
    }

    // header crc (with CRC fields zeroed)
    h.header_crc32c = rovctrl::io::gcs::calc_header_crc(h);

    std::vector<comm_gcs::Byte> out(sizeof(PacketHeader) + payload.size);
    std::memcpy(out.data(), &h, sizeof(PacketHeader));
    if (payload.size > 0) {
        std::memcpy(out.data() + sizeof(PacketHeader), payload.data, payload.size);
    }
    return out;
}

inline std::vector<comm_gcs::Byte> build_packet_no_payload(PacketHeader h)
{
    h.payload_len = 0;
    return build_packet(h, comm_gcs::BytesView{nullptr, 0});
}

/// 解析 + basic 校验 + CRC 校验
inline std::optional<ParsedPacket> parse_and_validate(comm_gcs::BytesView bytes,
                                                      AckCode* out_err_code = nullptr,
                                                      std::string* out_err = nullptr)
{
    if (out_err_code) *out_err_code = AckCode::BAD_FORMAT;

    if (bytes.size < sizeof(PacketHeader)) {
        if (out_err) *out_err = "packet too small for header";
        return std::nullopt;
    }

    PacketHeader h{};
    std::memcpy(&h, bytes.data, sizeof(PacketHeader));

    if (!rovctrl::io::gcs::header_basic_valid(h)) {
        if (out_err) *out_err = "header_basic_valid failed";
        if (out_err_code) *out_err_code = AckCode::BAD_FORMAT;
        return std::nullopt;
    }

    const std::size_t total = sizeof(PacketHeader) + static_cast<std::size_t>(h.payload_len);
    if (bytes.size != total) {
        if (out_err) *out_err = "size mismatch: bytes.size != header+payload_len";
        if (out_err_code) *out_err_code = AckCode::BAD_FORMAT;
        return std::nullopt;
    }

    // header crc
    const std::uint32_t hc = rovctrl::io::gcs::calc_header_crc(h);
    if (hc != h.header_crc32c) {
        if (out_err) *out_err = "header CRC32C mismatch";
        if (out_err_code) *out_err_code = AckCode::CRC_FAIL;
        return std::nullopt;
    }

    // payload crc
    if (h.payload_len == 0) {
        if (h.payload_crc32c != 0) {
            if (out_err) *out_err = "payload_len=0 but payload_crc32c != 0";
            if (out_err_code) *out_err_code = AckCode::CRC_FAIL;
            return std::nullopt;
        }
    } else {
        const auto* p = bytes.data + sizeof(PacketHeader);
        const std::uint32_t pc = rovctrl::io::gcs::crc32c(p, h.payload_len);
        if (pc != h.payload_crc32c) {
            if (out_err) *out_err = "payload CRC32C mismatch";
            if (out_err_code) *out_err_code = AckCode::CRC_FAIL;
            return std::nullopt;
        }
    }

    ParsedPacket pp{};
    pp.hdr = h;
    pp.payload = comm_gcs::BytesView{
        bytes.data + sizeof(PacketHeader),
        static_cast<std::size_t>(h.payload_len)
    };

    if (out_err_code) *out_err_code = AckCode::OK;
    return pp;
}

/// 生成 ACK 包（payload=AckPayload，ack_seq 放在 PacketHeader::ack_seq）
inline std::vector<comm_gcs::Byte> build_ack(std::uint32_t tx_seq,
                                            std::uint64_t session_id,
                                            std::uint32_t ack_seq,
                                            rovctrl::io::gcs::AckCode code,
                                            std::uint16_t flags_extra = 0)
{
    using namespace rovctrl::io::gcs;

    // payload: AckPayload（当前仅含 ack_code + reserved(或padding)）
    rovctrl::io::gcs::AckPayload ap{}; // value-init -> 所有字段置 0
    ap.ack_code = static_cast<std::uint16_t>(code);


    PacketHeader h = make_header(
        static_cast<std::uint8_t>(MsgType::ACK),
        tx_seq,
        session_id,
        static_cast<std::uint16_t>(rovctrl::io::gcs::FLAG_IS_ACK | flags_extra),
        static_cast<std::uint32_t>(sizeof(ap))
    );

    // NEW: ack_seq 放 header，而不是放 payload
    h.ack_seq = ack_seq;

    auto payload = to_bytes_vec(ap);
    return build_packet(h, comm_gcs::BytesView{payload.data(), payload.size()});
}



inline bool payload_size_is(comm_gcs::BytesView p, std::size_t expect) noexcept {
    return p.size == expect;
}

} // namespace comm_gcs::codec
