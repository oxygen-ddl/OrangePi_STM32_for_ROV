#include "io/input/control_intent_wire_adapter.hpp"

#include "control_core/control_intent.hpp"
#include "platform/timebase.hpp"

#include <cstdint>

namespace rovctrl::io {

namespace {

using WireMode = shared::msg::ControlMode;
using IntMode  = rovctrl::control_core::ControlMode;

inline bool map_mode(WireMode w, IntMode& out) noexcept
{
    switch (w) {
    case WireMode::kNone:   out = IntMode::kNone;   return true;
    case WireMode::kManual: out = IntMode::kManual; return true;
    case WireMode::kAuto:   out = IntMode::kAuto;   return true;

    // Compatibility: wire has Hold, internal may not.
    // Choose a safe semantic mapping:
    // - map to Auto: allow controller side to implement "hold" policy
    // - OR map to None: "do not change mode"
    case WireMode::kHold:   out = IntMode::kAuto;   return true;

    default:
        // Unknown enum value: degrade gracefully to "no mode change".
        out = IntMode::kNone;
        return true;  // note: not a hard failure
    }
}


inline std::uint64_t ensure_stamp_ns(std::uint64_t stamp_ns) noexcept
{
    if (stamp_ns != 0) return stamp_ns;
    return static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());
}

} // namespace

bool to_internal_intent(const shared::msg::ControlIntent& w,
                        rovctrl::control_core::ControlIntent& out) noexcept
{
    // ---------------------------------------------------------------------
    // 1) Wire version gate
    // ---------------------------------------------------------------------
    if (w.version != shared::msg::kControlIntentWireVersion) {
        return false;
    }

    // ---------------------------------------------------------------------
    // 2) Start from clean state
    //    - We overwrite seq/stamp/ttl because they are meaningful for staleness.
    // ---------------------------------------------------------------------
    out.clear_all();
    out.seq      = w.seq;
    out.stamp_ns = ensure_stamp_ns(w.stamp_ns);
    out.ttl_ms   = w.ttl_ms;

    // ---------------------------------------------------------------------
    // 3) Lifecycle
    // ---------------------------------------------------------------------
    out.request_exit = w.request_exit;

    // ---------------------------------------------------------------------
    // 4) Safety / arm-disarm: only effective when corresponding flag is set
    // ---------------------------------------------------------------------
    if ((w.flags & shared::msg::IntentFlags::kHasEStopCmd) != 0u) {
        out.has_estop_cmd = true;
        out.estop         = w.estop;
        out.clear_estop   = w.clear_estop;
    }

    if ((w.flags & shared::msg::IntentFlags::kHasArmCmd) != 0u) {
        out.has_arm_cmd = true;
        out.arm         = w.arm;
        out.disarm      = w.disarm;
    }

    // ---------------------------------------------------------------------
    // 5) Mode request
    // ---------------------------------------------------------------------
    if ((w.flags & shared::msg::IntentFlags::kHasModeRequest) != 0u) {
        rovctrl::control_core::ControlMode m{};
        if (!map_mode(w.mode_request, m)) {
            // Wire says "has mode request" but enum value is unknown -> reject
            return false;
        }
        out.has_mode_request = true;
        out.mode_request     = m;
    }

    // ---------------------------------------------------------------------
    // 6) Teleop DOF (manual)
    // ---------------------------------------------------------------------
    if ((w.flags & shared::msg::IntentFlags::kHasTeleopDof) != 0u) {
        out.has_teleop_dof         = true;
        out.teleop_dof_cmd.surge   = w.teleop_dof_cmd.surge;
        out.teleop_dof_cmd.sway    = w.teleop_dof_cmd.sway;
        out.teleop_dof_cmd.heave   = w.teleop_dof_cmd.heave;
        out.teleop_dof_cmd.roll    = w.teleop_dof_cmd.roll;
        out.teleop_dof_cmd.pitch   = w.teleop_dof_cmd.pitch;
        out.teleop_dof_cmd.yaw     = w.teleop_dof_cmd.yaw;
    }

    // ---------------------------------------------------------------------
    // 7) Ref / RefDelta (wire v1 currently does not carry them)
    //    - We ignore those flags for forward compatibility.
    // ---------------------------------------------------------------------

    return true;
}

} // namespace rovctrl::io
