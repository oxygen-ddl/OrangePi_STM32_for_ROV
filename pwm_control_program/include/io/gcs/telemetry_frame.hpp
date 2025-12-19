#pragma once
#ifndef ROVCTRL_IO_TELEMETRY_FRAME_HPP
#define ROVCTRL_IO_TELEMETRY_FRAME_HPP

#include <cstdint>
#include <string_view>

#include "io/gcs/gcs_protocol.hpp"
#include "control_core/control_mode.hpp"

namespace rovctrl::io {

/**
 * @brief 内部语义遥测帧（不等同于 wire struct）
 *
 * 设计意图：
 *  - 作为“系统内部状态 -> GCS wire struct”的中间语义层；
 *  - 未来若扩展更多 telemetry 类型（电源/推进器/导航），可继续加 V2/V3 或新增 FrameKind；
 *  - 这里尽量只用标量 + string_view，避免依赖复杂对象。
 */
struct TelemetryFrameV1 {
    bool session_established = false;
    bool link_alive          = false;
    bool estop               = false;

    rovctrl::control_core::ControlMode mode = rovctrl::control_core::ControlMode::kUnknown;

    std::string_view active_controller;
    std::string_view desired_controller;

    std::uint32_t consecutive_failures = 0;
    std::uint32_t auto_fail_limit      = 0;

    std::uint64_t t_ns = 0; // steady ns or nav_t_ns
};

/// ControlMode -> WireControlMode（显式映射，避免枚举数值漂移）
inline constexpr std::uint8_t to_wire_control_mode(rovctrl::control_core::ControlMode m) noexcept
{
    using rovctrl::io::gcs::WireControlMode;

    switch (m) {
    case rovctrl::control_core::ControlMode::kManual:
        return static_cast<std::uint8_t>(WireControlMode::Manual);
    case rovctrl::control_core::ControlMode::kAuto:
        return static_cast<std::uint8_t>(WireControlMode::Auto);
    case rovctrl::control_core::ControlMode::kFailsafe:
        return static_cast<std::uint8_t>(WireControlMode::Failsafe);
    case rovctrl::control_core::ControlMode::kNone:
        // none 不应出现在“当前模式”，视为 Unknown
        return static_cast<std::uint8_t>(WireControlMode::Unknown);
    case rovctrl::control_core::ControlMode::kUnknown:
    default:
        return static_cast<std::uint8_t>(WireControlMode::Unknown);
    }
}

/**
 * @brief 将语义帧打包为 wire: gcs::StatusTelemetry（唯一真源）
 *
 * 注意：
 *  - 这里负责 write_cstr()/截断/清零 reserved，避免 adapter/发送层重复写。
 */
inline void pack_status_telemetry(const TelemetryFrameV1& in,
                                 rovctrl::io::gcs::StatusTelemetry& out) noexcept
{
    using namespace rovctrl::io::gcs;

    out = StatusTelemetry{}; // 清零全部字段（含 reserved）

    out.session_established = in.session_established ? 1 : 0;
    out.link_alive          = in.link_alive ? 1 : 0;
    out.estop               = in.estop ? 1 : 0;

    out.mode = to_wire_control_mode(in.mode);

    write_cstr(out.active_controller,  kCtrlNameMaxLen, in.active_controller);
    write_cstr(out.desired_controller, kCtrlNameMaxLen, in.desired_controller);

    out.consecutive_failures = in.consecutive_failures;
    out.auto_fail_limit      = in.auto_fail_limit;

    out.t_ns = in.t_ns;
}

} // namespace rovctrl::io

#endif // ROVCTRL_IO_TELEMETRY_FRAME_HPP
