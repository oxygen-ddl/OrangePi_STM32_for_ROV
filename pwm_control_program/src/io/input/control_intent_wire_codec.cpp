#include "io/input/control_intent_wire_codec.hpp"

#include <cstdint>
#include <iostream>

namespace rovctrl::io::input {

// =============================================================================
// Helpers
// =============================================================================

static inline bool flag_has(std::uint32_t flags, std::uint32_t f) noexcept
{
    return (flags & f) != 0u;
}

static inline void flag_set(std::uint32_t& flags, std::uint32_t f, bool on) noexcept
{
    if (on) flags |= f;
    else    flags &= ~f;
}

// -----------------------------------------------------------------------------
// Mode mapping (Wire <-> Core)
//
// Engineering policy:
//  - Wire may contain values that core does not implement yet (e.g., kHold).
//  - Keep ABI/compat-safety: never crash / UB due to unknown enum values.
//  - Fallback strategy: map unsupported wire mode to safe core mode.
// -----------------------------------------------------------------------------

static inline rovctrl::control_core::ControlMode
map_mode_from_wire(shared::msg::ControlMode m) noexcept
{
    using WM = shared::msg::ControlMode;
    using CM = rovctrl::control_core::ControlMode;

    switch (m) {
    case WM::kManual: return CM::kManual;
    case WM::kAuto:   return CM::kAuto;
    case WM::kNone:   return CM::kNone;

    // Forward-compat: core may not implement Hold yet.
    case WM::kHold:   return CM::kManual;

    default:          return CM::kNone;
    }
}

static inline shared::msg::ControlMode
map_mode_to_wire(rovctrl::control_core::ControlMode m) noexcept
{
    using WM = shared::msg::ControlMode;
    using CM = rovctrl::control_core::ControlMode;

    switch (m) {
    case CM::kManual: return WM::kManual;
    case CM::kAuto:   return WM::kAuto;
    case CM::kNone:   return WM::kNone;

    default:          return WM::kNone;
    }
}

// =============================================================================
// Decode: wire -> core
// =============================================================================

bool decode_control_intent(const shared::msg::ControlIntent& w,
                           rovctrl::control_core::ControlIntent& c) noexcept
{
    // 0) Wire version gate
    if (w.version != shared::msg::kControlIntentWireVersion) {
        c.clear_all();
        return false;
    }
    //打印输出调试信息
    // std::cout << "[decode][ARM] version=" << w.version
    //           << " flags=0x" << std::hex << w.flags
    //           << " mask_arm=0x"
    //           << static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasArmCmd)
    //           << std::dec
    //           << " arm="    << int(w.arm)
    //           << " disarm=" << int(w.disarm)
    //           << "\n";

    if (w.version != shared::msg::kControlIntentWireVersion) {
        c.clear_all();
        return false;
    }

    // 1) Start from clean state (avoid leaking stale fields across frames)
    c.clear_all();

    // 2) Meta / staleness fields (always copied)
    c.cmd_seq  = w.cmd_seq;
    c.stamp_ns = w.stamp_ns;
    c.ttl_ms   = w.ttl_ms;

    // 3) Lifecycle
    c.request_exit = (w.request_exit != 0);

    // 4) E-Stop group (flag-driven)
    c.has_estop_cmd = flag_has(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasEStopCmd));
    if (c.has_estop_cmd) {
        c.estop       = (w.estop != 0);
        c.clear_estop = (w.clear_estop != 0);
    }

    // 5) Arm/Disarm group (flag-driven)
    c.has_arm_cmd = flag_has(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasArmCmd));
    if (c.has_arm_cmd) {
        c.arm    = (w.arm != 0);
        c.disarm = (w.disarm != 0);
    }

    // 6) Mode request (flag-driven)
    c.has_mode_request = flag_has(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasModeRequest));
    if (c.has_mode_request) {
        c.mode_request = map_mode_from_wire(w.mode_request);
    }

    // 7) Teleop DOF (flag-driven)
    c.has_teleop_dof = flag_has(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasTeleopDof));
    if (c.has_teleop_dof) {
        c.teleop_dof_cmd.surge = w.teleop_dof_cmd.surge;
        c.teleop_dof_cmd.sway  = w.teleop_dof_cmd.sway;
        c.teleop_dof_cmd.heave = w.teleop_dof_cmd.heave;
        c.teleop_dof_cmd.roll  = w.teleop_dof_cmd.roll;
        c.teleop_dof_cmd.pitch = w.teleop_dof_cmd.pitch;
        c.teleop_dof_cmd.yaw   = w.teleop_dof_cmd.yaw;
    }

    // 8) Motor test (flag-driven)  // NEW
    c.has_motor_test = flag_has(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasMotorTest));
    if (c.has_motor_test) {
        // Normalize enable to strict bool
        c.motor_test.enable      = (w.motor_test.enable != 0);
        c.motor_test.motor_id    = w.motor_test.motor_id;
        c.motor_test.mode        = w.motor_test.mode;
        c.motor_test.value       = w.motor_test.value;
        c.motor_test.duration_ms = w.motor_test.duration_ms;
        c.motor_test.cmd_id      = w.motor_test.cmd_id;
    }

    // 9) Ref / RefDelta: wire v1 has reserved flags but no payload.
    // Keep core defaults (do not set has_ref/has_ref_delta here).

    return true;
}

// =============================================================================
// Encode: core -> wire
// =============================================================================

bool encode_control_intent(const rovctrl::control_core::ControlIntent& c,
                           shared::msg::ControlIntent& w) noexcept
{
    // Stable baseline (also resets reserved/padding predictably).
    w.clear_all();
    w.version = shared::msg::kControlIntentWireVersion;

    // 1) Meta / staleness fields (always copied)
    w.cmd_seq  = c.cmd_seq;
    w.stamp_ns = c.stamp_ns;
    w.ttl_ms   = c.ttl_ms;

    // 2) Lifecycle
    w.request_exit = c.request_exit ? 1u : 0u;

    // 3) E-Stop group (flag-driven)
    flag_set(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasEStopCmd), c.has_estop_cmd);
    if (c.has_estop_cmd) {
        w.estop       = c.estop ? 1u : 0u;
        w.clear_estop = c.clear_estop ? 1u : 0u;
    }

    // 4) Arm/Disarm group (flag-driven)
    flag_set(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasArmCmd), c.has_arm_cmd);
    if (c.has_arm_cmd) {
        w.arm    = c.arm ? 1u : 0u;
        w.disarm = c.disarm ? 1u : 0u;
    }

    // 5) Mode request (flag-driven)
    flag_set(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasModeRequest), c.has_mode_request);
    if (c.has_mode_request) {
        w.mode_request = map_mode_to_wire(c.mode_request);
    }

    // 6) Teleop DOF (flag-driven)
    flag_set(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasTeleopDof), c.has_teleop_dof);
    if (c.has_teleop_dof) {
        w.teleop_dof_cmd.surge = c.teleop_dof_cmd.surge;
        w.teleop_dof_cmd.sway  = c.teleop_dof_cmd.sway;
        w.teleop_dof_cmd.heave = c.teleop_dof_cmd.heave;
        w.teleop_dof_cmd.roll  = c.teleop_dof_cmd.roll;
        w.teleop_dof_cmd.pitch = c.teleop_dof_cmd.pitch;
        w.teleop_dof_cmd.yaw   = c.teleop_dof_cmd.yaw;
    }

    // 7) Motor test (flag-driven)  // NEW
    flag_set(w.flags, static_cast<std::uint32_t>(shared::msg::IntentFlags::kHasMotorTest), c.has_motor_test);
    if (c.has_motor_test) {
        w.motor_test.enable      = c.motor_test.enable ? 1u : 0u;
        w.motor_test.motor_id    = c.motor_test.motor_id;
        w.motor_test.mode        = c.motor_test.mode;
        w.motor_test.value       = c.motor_test.value;
        w.motor_test.duration_ms = c.motor_test.duration_ms;
        w.motor_test.cmd_id      = c.motor_test.cmd_id;
    }

    // Ref / RefDelta: do not set flags in wire v1 until payload exists.

    return true;
}

} // namespace rovctrl::io::input
