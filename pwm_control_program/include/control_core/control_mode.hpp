#pragma once
#ifndef ROVCTRL_CONTROL_CORE_CONTROL_MODE_HPP
#define ROVCTRL_CONTROL_CORE_CONTROL_MODE_HPP

#include <cstdint>
#include <cstddef>
#include <string_view>

namespace rovctrl::control_core {

/**
 * @brief 控制算法层的“控制模式”枚举（唯一真源）
 *
 * 语义边界：
 *  - kNone：输入/请求层语义（no request / keep current mode）
 *  - kUnknown：内部状态语义（uninitialized / invalid），尽量不要作为外部“请求值”使用
 *
 * 设计约束：
 *  - 必须稳定（跨模块/跨进程日志/序列化不轻易改数值）
 *  - 固定底层类型（uint8_t）
 */
enum class ControlMode : std::uint8_t {
    kNone     = 0,   ///< no request / keep current mode

    kManual   = 1,   ///< teleop / manual
    kAuto     = 2,   ///< closed-loop controller (PID/MPC/SMC...)

    kFailsafe = 3,   ///< output zero / estop / degraded policy

    kUnknown  = 255  ///< internal: uninitialized / invalid
};

// -----------------------------------------------------------------------------
// Compatibility aliases (temporary migration aid)
// 建议：全仓替换完成后删掉 compat，避免长期“二义入口”。
// -----------------------------------------------------------------------------
namespace compat {
inline constexpr ControlMode None     = ControlMode::kNone;
inline constexpr ControlMode Manual   = ControlMode::kManual;
inline constexpr ControlMode Auto     = ControlMode::kAuto;
inline constexpr ControlMode Failsafe = ControlMode::kFailsafe;
inline constexpr ControlMode Unknown  = ControlMode::kUnknown;
} // namespace compat

/// @brief 是否是“请求层枚举值”（None 属于请求层；Unknown 属于内部态，不建议作为外部请求）
constexpr bool is_request(ControlMode m) noexcept
{
    return m != ControlMode::kUnknown;
}

/// @brief 是否是“会改变系统状态的请求”（None 不改变；Manual/Auto/Failsafe 会改变）
constexpr bool is_actionable_request(ControlMode m) noexcept
{
    return (m == ControlMode::kManual) || (m == ControlMode::kAuto) || (m == ControlMode::kFailsafe);
}

constexpr std::string_view to_string(ControlMode m) noexcept
{
    switch (m) {
    case ControlMode::kNone:     return "none";
    case ControlMode::kManual:   return "manual";
    case ControlMode::kAuto:     return "auto";
    case ControlMode::kFailsafe: return "failsafe";
    case ControlMode::kUnknown:
    default:                     return "unknown";
    }
}

namespace detail {

constexpr char tolower_ascii(char c) noexcept
{
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

constexpr bool iequals(std::string_view a, std::string_view b) noexcept
{
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (tolower_ascii(a[i]) != tolower_ascii(b[i])) return false;
    }
    return true;
}

} // namespace detail

/**
 * @brief 从字符串解析 ControlMode（用于 YAML / CLI / GCS 文本命令）
 *
 * 支持（大小写无关）：
 *  - "none" / "no" / "keep"          -> kNone（不改变当前模式）
 *  - "manual" / "teleop"            -> kManual
 *  - "auto" / "pid" / "mpc" / "smc" -> kAuto（算法细分由“控制器选择/业务模式”处理）
 *  - "failsafe" / "safe" / "estop"  -> kFailsafe
 *  - "unknown"                      -> kUnknown（不建议外部使用，仅调试）
 */
constexpr bool parse_control_mode(std::string_view s, ControlMode& out) noexcept
{
    using detail::iequals;

    if (iequals(s, "none") || iequals(s, "no") || iequals(s, "keep")) {
        out = ControlMode::kNone;
        return true;
    }
    if (iequals(s, "manual") || iequals(s, "teleop")) {
        out = ControlMode::kManual;
        return true;
    }
    if (iequals(s, "auto") || iequals(s, "pid") || iequals(s, "mpc") || iequals(s, "smc")) {
        out = ControlMode::kAuto;
        return true;
    }
    if (iequals(s, "failsafe") || iequals(s, "safe") || iequals(s, "estop")) {
        out = ControlMode::kFailsafe;
        return true;
    }
    if (iequals(s, "unknown")) {
        out = ControlMode::kUnknown;
        return true;
    }
    return false;
}

/// @brief 运行态可输出控制（Manual/Auto）
constexpr bool is_operational(ControlMode m) noexcept
{
    return (m == ControlMode::kManual) || (m == ControlMode::kAuto);
}

/// @brief 明确有效值（排除 unknown）
constexpr bool is_valid(ControlMode m) noexcept
{
    return m != ControlMode::kUnknown;
}

} // namespace rovctrl::control_core

// -----------------------------------------------------------------------------
// Optional transitional alias to prevent duplicate ControlMode definitions in io
// 目的：彻底杜绝 rovctrl::io 再定义一份 ControlMode。
// -----------------------------------------------------------------------------


#endif // ROVCTRL_CONTROL_CORE_CONTROL_MODE_HPP
