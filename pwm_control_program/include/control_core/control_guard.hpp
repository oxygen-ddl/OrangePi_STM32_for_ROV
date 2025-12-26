#pragma once
#ifndef ROVCTRL_CONTROL_CORE_CONTROL_GUARD_HPP
#define ROVCTRL_CONTROL_CORE_CONTROL_GUARD_HPP

#include <cstdint>

#include "control_core/control_intent.hpp"   // ControlIntent / ControlState / ControlReference / ControlMode

// forward declarations (avoid leaking shared/msg and nav headers)
namespace shared::msg {
struct NavState;
}

namespace rovctrl::control_core {

struct ControlGuardConfig {
    // ---- Input timeout (ms) ----
    std::uint32_t default_ttl_ms = 200;   // 0 => disable stale check

    // ---- Ref delta limits ----
    double max_depth_step_m = 0.05;
    double max_yaw_step_rad = 0.10;

    // ---- Teleop DOF clamp ----
    double teleop_dof_min = -1.0;
    double teleop_dof_max =  1.0;

    // ---- Safety behavior ----
    bool estop_latch = true;

    // ---- Mode gating policy ----
    bool enable_mode_gating = true;
};

enum class FailsafeAction : std::uint8_t {
    kNone = 0,
    kHoldOutput,
    kZeroOutput,
    kEmergencyStop
};

struct GuardResult {
    // Safety state
    bool armed = false;
    bool estop_latched = false;

    // Mode decision
    ControlMode effective_mode = ControlMode::kManual;
    bool mode_changed = false;

    // Effective input payload (post-arbitration)
    ControlIntent effective_intent{};

    // Failsafe action suggestion
    FailsafeAction failsafe = FailsafeAction::kNone;

    // Debug/status
    bool input_stale = false;
    std::uint64_t last_intent_ns = 0;
};

class ControlGuard {
public:
    explicit ControlGuard(ControlGuardConfig cfg = {});
    void reset();

    GuardResult step(std::uint64_t now_ns,
                     const ControlState& state,
                     const shared::msg::NavState* nav,
                     const ControlIntent& intent);

    const ControlGuardConfig& config() const noexcept { return cfg_; }

    // =========================================================
    // Read-only accessors (for telemetry / debug)
    // =========================================================
    bool armed() const noexcept { return armed_; }
    bool estop_latched() const noexcept { return estop_latched_; }
    ControlMode mode() const noexcept { return mode_; }

    std::uint64_t last_intent_ns() const noexcept { return last_intent_ns_; }
    std::uint64_t last_intent_seq() const noexcept { return last_intent_seq_; }
    std::uint32_t input_age_ms() const noexcept { return input_age_ms_; }

private:
    bool is_intent_stale(std::uint64_t now_ns, const ControlIntent& intent) const;
    bool nav_ok_for_mode(const shared::msg::NavState* nav, ControlMode m) const;
    ControlMode downgrade_mode(ControlMode requested) const;

    void clamp_teleop(ControlIntent& inout) const;
    void clamp_ref_delta(ControlIntent& inout) const;

private:
    ControlGuardConfig cfg_{};

    bool armed_{false};
    bool estop_latched_{false};

    ControlMode mode_{ControlMode::kManual};

    std::uint64_t last_intent_ns_{0};
    std::uint64_t last_intent_seq_{0};
    std::uint32_t input_age_ms_{0};
};

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_CONTROL_GUARD_HPP
