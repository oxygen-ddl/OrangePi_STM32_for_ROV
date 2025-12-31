#pragma once
#ifndef ROVCTRL_CONTROL_CORE_CONTROL_GUARD_HPP
#define ROVCTRL_CONTROL_CORE_CONTROL_GUARD_HPP

#include <cstdint>

#include "control_core/control_intent.hpp"   // ControlIntent / ControlState / ControlReference / ControlMode
#include "shared/msg/nav_state_view.hpp"  // 新增
// #include "shared/msg/nav_state.hpp"    // 旧的如果只剩这里用，可以删

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
    // ==== NEW: 急停解除的“长按”时间阈值（毫秒） ====
    //
    // 语义：
    //  - 当 estop 已经生效，且收到 clear_estop 命令时，
    //    Guard 只有在 clear_estop 持续保持 >= estop_clear_hold_ms
    //    之后才真正解除急停；
    //  - 0 表示使用 cpp 中的默认值（目前是 2000ms）；
    //  - 也可以在 YAML 中显式配置一个值。
    std::uint32_t estop_clear_hold_ms = 0;
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
                     const shared::msg::NavStateView* nav,
                     const ControlIntent& intent);

    const ControlGuardConfig& config() const noexcept { return cfg_; }

    // =========================================================
    // Read-only accessors (telemetry / debug)
    // =========================================================
    bool armed() const noexcept { return armed_; }
    bool estop_latched() const noexcept { return estop_latched_; }
    ControlMode mode() const noexcept { return mode_; }

    std::uint64_t last_intent_ns() const noexcept { return last_intent_ns_; }
    std::uint64_t last_intent_cmd_seq() const noexcept { return last_intent_cmd_seq_; }
    std::uint32_t input_age_ms() const noexcept { return input_age_ms_; }

    // Optional: expose clear-hold diagnostic for UI
    std::uint32_t clear_hold_ms() const noexcept { return clear_hold_ms_; }

private:
    // freshness / ttl
    bool is_intent_stale(std::uint64_t now_ns, const ControlIntent& intent) const;

    // mode gating based on nav availability/health
    bool nav_ok_for_mode(const shared::msg::NavStateView* nav, ControlMode requested) const;
    ControlMode downgrade_mode(ControlMode requested) const;

    // safety: S2 clear conditions
    bool is_neutral_for_clear_(const ControlIntent& intent) const noexcept;

    // payload bounds
    void clamp_teleop(ControlIntent& inout) const;
    void clamp_ref_delta(ControlIntent& inout) const;

private:
    ControlGuardConfig cfg_{};

    bool armed_{false};
    bool estop_latched_{false};
    ControlMode mode_{ControlMode::kManual};

    std::uint64_t last_intent_ns_{0};
    std::uint64_t last_intent_cmd_seq_{0};
    std::uint32_t input_age_ms_{0};

    // S2 clear (hold-to-clear) state
    std::uint64_t clear_hold_start_ns_{0};
    std::uint32_t clear_hold_ms_{0};
};

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_CONTROL_GUARD_HPP
