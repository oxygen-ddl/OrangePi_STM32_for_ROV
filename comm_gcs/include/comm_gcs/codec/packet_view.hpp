#pragma once
#ifndef COMM_GCS_CODEC_PACKET_VIEW_HPP
#define COMM_GCS_CODEC_PACKET_VIEW_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "comm_gcs/bytes.hpp"

// wire protocol (PacketHeader, constants, header_basic_valid, etc.)
#include "proto_gcs/gcs_protocol.hpp"

namespace comm_gcs::codec {

/**
 * @brief 解析失败的原因（适合用于测试/日志）。
 *
 * 注意：CRC 校验不在这里做（通常由 gcs_codec 负责）。
 */
enum class PacketViewError : std::uint8_t {
    Ok = 0,
    TooSmallForHeader,
    HeaderBasicInvalid,
    PayloadLenTooLarge,
    SizeMismatch,
};

/**
 * @brief PacketHeader + payload 的零拷贝视图。
 *
 * - 仅保存指针，不拥有内存；
 * - PacketHeader 是从 bytes 中 memcpy 得到的一个副本（避免对齐/别名问题）；
 * - payload 是指向原始 buffer 的 BytesView。
 */
struct PacketView final {
    rovctrl::io::gcs::PacketHeader header{};
    comm_gcs::BytesView payload{};

    constexpr std::uint8_t  msg_type() const noexcept { return header.msg_type; }
    constexpr std::uint16_t flags()    const noexcept { return header.flags; }
    constexpr std::uint32_t seq()      const noexcept { return header.seq; }
    constexpr std::uint64_t session_id() const noexcept { return header.session_id; }
    constexpr std::uint32_t payload_len() const noexcept { return header.payload_len; }

    constexpr bool has_payload() const noexcept { return payload.size != 0; }
};

/**
 * @brief 结果：成功得到 PacketView；失败给出错误码与可选错误字符串。
 *
 * 设计：
 * - out_err 可为空；为空时不产生字符串分配；
 * - out_code 可为空；为空时只用 bool 表示成功失败。
 */
struct PacketViewResult final {
    PacketView view{};
    PacketViewError err{PacketViewError::Ok};
    std::string msg{};
    bool ok() const noexcept { return err == PacketViewError::Ok; }
};

inline const char* to_string(PacketViewError e) noexcept
{
    switch (e) {
    case PacketViewError::Ok:               return "Ok";
    case PacketViewError::TooSmallForHeader:return "TooSmallForHeader";
    case PacketViewError::HeaderBasicInvalid:return "HeaderBasicInvalid";
    case PacketViewError::PayloadLenTooLarge:return "PayloadLenTooLarge";
    case PacketViewError::SizeMismatch:     return "SizeMismatch";
    default:                                return "Unknown";
    }
}

/**
 * @brief 从原始 bytes 构造 PacketView（只做基础校验 + size 校验，不做 CRC 校验）。
 */
inline PacketViewResult make_packet_view(comm_gcs::BytesView bytes)
{
    using namespace rovctrl::io::gcs;

    PacketViewResult r{};

    if (bytes.size < sizeof(PacketHeader)) {
        r.err = PacketViewError::TooSmallForHeader;
        r.msg = "packet too small for header";
        return r;
    }

    PacketHeader h{};
    std::memcpy(&h, bytes.data, sizeof(PacketHeader));

    // basic header validation: magic/version/msg_type/payload_len<=kMaxPayloadBytes
    if (!header_basic_valid(h)) {
        r.err = PacketViewError::HeaderBasicInvalid;
        r.msg = "header_basic_valid failed";
        return r;
    }

    if (h.payload_len > kMaxPayloadBytes) {
        r.err = PacketViewError::PayloadLenTooLarge;
        r.msg = "payload_len exceeds kMaxPayloadBytes";
        return r;
    }

    const std::size_t expect_total = sizeof(PacketHeader) + static_cast<std::size_t>(h.payload_len);
    if (bytes.size != expect_total) {
        r.err = PacketViewError::SizeMismatch;
        r.msg = "size mismatch: bytes.size != sizeof(header)+payload_len";
        return r;
    }

    r.view.header = h;
    r.view.payload = comm_gcs::BytesView{
        bytes.data + sizeof(PacketHeader),
        static_cast<std::size_t>(h.payload_len)
    };

    r.err = PacketViewError::Ok;
    r.msg.clear();
    return r;
}

/**
 * @brief 若 payload 预期是某个 POD，检查长度并 memcpy 出来。
 *
 * 返回：成功则 std::optional<T> 有值；失败返回空。
 */
template <class T>
inline std::optional<T> read_pod_payload(const PacketView& pv)
{
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");

    if (pv.payload.size != sizeof(T)) return std::nullopt;

    T out{};
    if constexpr (sizeof(T) > 0) {
        std::memcpy(&out, pv.payload.data, sizeof(T));
    }
    return out;
}

/**
 * @brief payload size 判断工具（便于 session 中做长度判定）
 */
inline bool payload_size_is(const PacketView& pv, std::size_t expect) noexcept
{
    return pv.payload.size == expect;
}

} // namespace comm_gcs::codec

#endif // COMM_GCS_CODEC_PACKET_VIEW_HPP
