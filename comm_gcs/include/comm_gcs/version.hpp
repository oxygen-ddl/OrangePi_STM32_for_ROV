#pragma once

#include <cstdint>
#include <string_view>

namespace comm_gcs {

// ============================================================================
// 1. comm_gcs module version (code version)
// ============================================================================
//
// 用于日志 / 调试 / 构建信息。
// 不用于通信兼容性判断。
//
struct ModuleVersion {
    std::uint8_t major;
    std::uint8_t minor;
    std::uint8_t patch;
};

constexpr ModuleVersion kModuleVersion{
    0, 1, 0
};

constexpr std::string_view module_version_string() noexcept {
    return "comm_gcs/0.1.0";
}

// ============================================================================
// 2. Wire protocol version (MOST IMPORTANT)
// ============================================================================
//
// 用于 GCS ↔ Control 通信兼容性判断。
// 规则：
//   - major 不同 → 不兼容，必须拒绝通信
//   - minor 不同 → 向后兼容（可选功能）
//
struct WireVersion {
    std::uint16_t major;
    std::uint16_t minor;
};

constexpr WireVersion kWireVersion{
    1, 0
};

// 比较工具（供 adapter / 握手使用）
constexpr bool is_wire_compatible(WireVersion remote) noexcept {
    return remote.major == kWireVersion.major;
}

// ============================================================================
// 3. Capability flags (optional but future-proof)
// ============================================================================
//
// 用于：
//   - 同一 wire version 下，协商是否支持某些功能
//   - 避免通过“magic field existence”猜能力
//
enum class Capability : std::uint32_t {
    None              = 0,

    // Direction
    RxControlCommand  = 1u << 0,  // GCS -> Control
    TxTelemetry       = 1u << 1,  // Control -> GCS

    // Telemetry content
    TelemetryNav      = 1u << 8,
    TelemetryControl  = 1u << 9,
    TelemetryPWM      = 1u << 10,
    TelemetryPower    = 1u << 11,

    // Control features
    ModeSwitch        = 1u << 16,
    EmergencyStop     = 1u << 17,
};

using CapabilityMask = std::uint32_t;

constexpr CapabilityMask operator|(Capability a, Capability b) noexcept {
    return static_cast<CapabilityMask>(a) | static_cast<CapabilityMask>(b);
}

constexpr bool has_capability(CapabilityMask mask, Capability cap) noexcept {
    return (mask & static_cast<CapabilityMask>(cap)) != 0;
}

// 本端支持的能力（控制侧）
constexpr CapabilityMask kLocalCapabilities =
      static_cast<CapabilityMask>(Capability::RxControlCommand)
    | static_cast<CapabilityMask>(Capability::TxTelemetry)
    | static_cast<CapabilityMask>(Capability::TelemetryNav)
    | static_cast<CapabilityMask>(Capability::TelemetryControl)
    | static_cast<CapabilityMask>(Capability::TelemetryPWM);

} // namespace comm_gcs
