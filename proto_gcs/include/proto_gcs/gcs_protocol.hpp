#pragma once
#ifndef ROVCTRL_IO_GCS_PROTOCOL_HPP
#define ROVCTRL_IO_GCS_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>

namespace rovctrl::io::gcs {

// ============================================================================
// Protocol constants / version
// ============================================================================

inline constexpr std::uint32_t kMagic        = 0x524F5647u;  // 'ROVG'
inline constexpr std::uint16_t kProtoVersion = 1;

// Payload sizing: keep UDP packets modest; avoid fragmentation.
inline constexpr std::size_t kMaxPayloadBytes = 1024;

// NOTE: header is 48 bytes (packed). Keep this derived from sizeof(PacketHeader)
// via kHeaderBytes below to avoid silent drift.
inline constexpr std::size_t kAutoNameMaxLen = 16; // SetModeCmd::auto_controller
inline constexpr std::size_t kCtrlNameMaxLen = 16; // StatusTelemetry controller names

// Flags (wire)
inline constexpr std::uint16_t FLAG_ACK_REQ = 0x0001;
inline constexpr std::uint16_t FLAG_IS_ACK  = 0x0002;

// ============================================================================
// Wire enums (fixed underlying types)
// ============================================================================

enum class MsgType : std::uint8_t {
    // handshake
    CONNECT_REQ     = 1,
    CONNECT_ACK     = 2,
    CONNECT_CONFIRM = 3,

    // heartbeat
    HEARTBEAT       = 10,

    // commands (GCS -> ROV)
    SET_MODE        = 20,
    SET_DOF_CMD     = 21,
    ESTOP           = 22,

    // telemetry (ROV -> GCS)
    STATUS          = 40,

    // ack
    ACK             = 250
};

enum class AckCode : std::uint16_t {
    OK               = 0,
    BAD_FORMAT       = 1,
    CRC_FAIL         = 2,
    INVALID_SESSION  = 3,
    SEQ_OLD_OR_DUP   = 4,
    NOT_SUPPORTED    = 5
};

// This enum exists to satisfy mapping code.
// Keep numeric values aligned with your control modes.
enum class WireControlMode : std::uint8_t {
    Unknown  = 0,
    Manual   = 1,
    Auto     = 2,
    Failsafe = 3
};

// ============================================================================
// Wire structs (POD, packed, fixed-size)
// Endianness: CURRENTLY HOST ENDIAN. This is only safe when both ends share
// the same endianness. Upgrade to explicit LE/BE later if needed.
// ============================================================================

#pragma pack(push, 1)

struct PacketHeader final {
    std::uint32_t magic   = kMagic;        // 4
    std::uint16_t version = kProtoVersion; // 2

    std::uint8_t  msg_type  = 0;           // 1
    std::uint8_t  reserved0 = 0;           // 1

    std::uint16_t flags     = 0;           // 2
    std::uint16_t reserved1 = 0;           // 2

    std::uint32_t seq        = 0;          // 4
    std::uint64_t session_id = 0;          // 8

    std::uint32_t payload_len  = 0;        // 4
    std::uint32_t ack_seq      = 0;        // 4

    std::uint32_t send_time_ms = 0;        // 4

    // NEW: pad/reserved to reach 48 bytes total
    std::uint32_t reserved2    = 0;        // 4

    std::uint32_t header_crc32c  = 0;      // 4
    std::uint32_t payload_crc32c = 0;      // 4
};
static_assert(sizeof(PacketHeader) == 48, "PacketHeader size must be 48 bytes");


struct ConnectReq final {
    std::uint64_t gcs_nonce = 0;
};

struct ConnectAck final {
    std::uint64_t gcs_nonce_echo = 0;
    std::uint64_t rov_nonce      = 0;
    std::uint32_t rov_caps       = 0;   // reserved
    std::uint32_t result_code    = 0;   // 0=OK
};

struct ConnectConfirm final {
    std::uint64_t rov_nonce_echo = 0;
};

struct Heartbeat final {
    std::uint32_t now_ms = 0;
};

struct SetModeCmd final {
    std::uint8_t  mode      = 0; // WireControlMode numeric value
    std::uint8_t  reserved0 = 0;
    std::uint16_t reserved1 = 0;

    char auto_controller[kAutoNameMaxLen]{}; // null-terminated, zero padded
};

struct SetDofCmd final {
    float dof[6]{};
};

struct EstopCmd final {
    std::uint8_t  enable    = 0;
    std::uint8_t  reserved0 = 0;
    std::uint16_t reserved1 = 0;
};

// ACK payload: keep tiny. ack_seq is in PacketHeader.ack_seq.
struct AckPayload final {
    std::uint16_t ack_code = 0; // AckCode
    std::uint16_t reason   = 0; // reserved for future use
};

struct StatusTelemetry final {
    std::uint8_t  session_established = 0;
    std::uint8_t  link_alive          = 0;
    std::uint8_t  estop               = 0;
    std::uint8_t  reserved0           = 0;

    std::uint8_t  mode                = 0; // WireControlMode
    std::uint8_t  reserved1           = 0;
    std::uint16_t reserved2           = 0;

    char active_controller[kCtrlNameMaxLen]{};
    char desired_controller[kCtrlNameMaxLen]{};

    std::uint32_t consecutive_failures = 0;
    std::uint32_t auto_fail_limit      = 0;

    std::uint64_t t_ns = 0;
};

// New in v1: MotorTestCmd
struct MotorTestCmd {
    std::uint8_t enable;
    std::uint8_t motor_id;
    std::uint8_t mode;
    std::uint8_t reserved0;
    float        value;
    std::uint16_t duration_ms;
    std::uint16_t reserved1;
    std::uint32_t cmd_id;
};


#pragma pack(pop)

// ============================================================================
// Derived constants (keep in sync automatically)
// ============================================================================

inline constexpr std::size_t kHeaderBytes   = sizeof(PacketHeader);
inline constexpr std::size_t kMaxPacketSize = kHeaderBytes + kMaxPayloadBytes;

// ============================================================================
// Layout invariants (do NOT break silently)
// ============================================================================

static_assert(sizeof(PacketHeader)  == 48, "PacketHeader size must be 48 bytes");
static_assert(alignof(PacketHeader) == 1,  "PacketHeader must be packed (align=1)");
static_assert(std::is_trivially_copyable<PacketHeader>::value, "PacketHeader must be trivially copyable");
static_assert(std::is_standard_layout<PacketHeader>::value, "PacketHeader must be standard layout");

static_assert(sizeof(AckPayload)     == 4,  "AckPayload size must be 4 bytes");
static_assert(sizeof(ConnectReq)     == 8,  "ConnectReq size must be 8");
static_assert(sizeof(ConnectAck)     == 24, "ConnectAck size must be 24");
static_assert(sizeof(ConnectConfirm) == 8,  "ConnectConfirm size must be 8");
static_assert(sizeof(SetModeCmd)     == 20, "SetModeCmd size must be 20");
static_assert(sizeof(SetDofCmd)      == 24, "SetDofCmd size must be 24");
static_assert(sizeof(EstopCmd)       == 4,  "EstopCmd size must be 4 bytes");

// ============================================================================
// CRC32C API (implementation in .cpp)
// ============================================================================

std::uint32_t crc32c(const void* data, std::size_t len);

inline PacketHeader header_zero_crc(PacketHeader h) noexcept
{
    h.header_crc32c  = 0;
    h.payload_crc32c = 0;
    return h;
}

inline std::uint32_t calc_header_crc(const PacketHeader& h) noexcept
{
    const PacketHeader hz = header_zero_crc(h);
    return crc32c(&hz, sizeof(PacketHeader));
}

// ============================================================================
// Basic validation & utilities
// ============================================================================

inline bool msg_type_known(std::uint8_t mt) noexcept
{
    switch (static_cast<MsgType>(mt)) {
    case MsgType::CONNECT_REQ:
    case MsgType::CONNECT_ACK:
    case MsgType::CONNECT_CONFIRM:
    case MsgType::HEARTBEAT:
    case MsgType::SET_MODE:
    case MsgType::SET_DOF_CMD:
    case MsgType::ESTOP:
    case MsgType::STATUS:
    case MsgType::ACK:
        return true;
    default:
        return false;
    }
}

inline bool header_basic_valid(const PacketHeader& h) noexcept
{
    if (h.magic   != kMagic)        return false;
    if (h.version != kProtoVersion) return false;
    if (!msg_type_known(h.msg_type)) return false;
    if (h.payload_len > kMaxPayloadBytes) return false;
    return true;
}

inline void write_cstr(char* dst, std::size_t dst_cap, std::string_view s) noexcept
{
    if (!dst || dst_cap == 0) return;
    std::memset(dst, 0, dst_cap);
    const std::size_t n = (s.size() < (dst_cap - 1)) ? s.size() : (dst_cap - 1);
    if (n > 0) std::memcpy(dst, s.data(), n);
}

} // namespace rovctrl::io::gcs

#endif // ROVCTRL_IO_GCS_PROTOCOL_HPP
