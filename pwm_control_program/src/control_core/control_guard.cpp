#include "control_core/control_guard.hpp"

#include <algorithm>
#include <cstdint>

// 只在 cpp 里依赖 nav_state 的完整定义
#include "shared/msg/nav_state.hpp"

// 做法 A：ControlIntent/ControlMode 真源在 control_core
#include "control_core/control_intent.hpp"
#include "control_core/control_mode.hpp"

namespace rovctrl::control_core {

static inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

ControlGuard::ControlGuard(ControlGuardConfig cfg)
    : cfg_(cfg)
{
}

void ControlGuard::reset()
{
    armed_          = false;
    estop_latched_  = false;

    // 做法 A：ControlMode 在 control_core
    mode_           = ControlMode::kManual;

    last_intent_ns_  = 0;
    last_intent_seq_ = 0;
    input_age_ms_    = 0;
}

// -----------------------------------------------------------------------------
// “输入过期”判定（做法 A 推荐实现）：
//  - 优先使用 intent.stamp_ns + ttl_ms（若 stamp_ns==0，则退化到 now_ns）
//  - ttl_ms==0：使用 intent.ttl_ms=0 表示“让 Guard 用 cfg_.default_ttl_ms”
//  - cfg_.default_ttl_ms==0：禁用超时
// -----------------------------------------------------------------------------
bool ControlGuard::is_intent_stale(std::uint64_t now_ns,
                                   const ControlIntent& intent) const
{
    // 1) ttl 决策
    const std::uint32_t ttl_ms =
        (intent.ttl_ms != 0) ? intent.ttl_ms : cfg_.default_ttl_ms;

    if (ttl_ms == 0) {
        return false; // ttl disabled
    }

    // 2) 起始时间
    const std::uint64_t t0 = (intent.stamp_ns != 0) ? intent.stamp_ns : last_intent_ns_;
    const std::uint64_t t1 = (now_ns != 0) ? now_ns : t0;

    if (t1 < t0) {
        // steady ns 理论不应倒退；保守处理为“不过期”
        return false;
    }

    const std::uint64_t age_ns = (t1 - t0);
    const std::uint64_t age_ms = age_ns / 1000000ull;

    return age_ms > static_cast<std::uint64_t>(ttl_ms);
}

// -----------------------------------------------------------------------------
// 模式门控（做法 A 收敛）：
//  - Manual 永远允许
//  - Auto 依赖 nav（若 enable_mode_gating==true）
//  - Failsafe 永远允许（安全态）
// -----------------------------------------------------------------------------
bool ControlGuard::nav_ok_for_mode(const shared::msg::NavState* nav,
                                   ControlMode m) const
{
    if (!cfg_.enable_mode_gating) return true;

    if (m == ControlMode::kManual)   return true;
    if (m == ControlMode::kFailsafe) return true;
    if (m == ControlMode::kNone)     return true; // "keep" request, not a real mode
    if (m == ControlMode::kUnknown)  return false;

    // Auto：依赖导航健康状态
    if (m == ControlMode::kAuto) {
        if (!nav) return false;
        if (nav->health == shared::msg::NavHealth::INVALID) return false;

        const bool eskf_ok = (nav->status_flags & shared::msg::NAV_FLAG_ESKF_OK) != 0;
        if (!eskf_ok) return false;

        return true;
    }

    // 其它未知值：保守拒绝
    return false;
}

ControlMode ControlGuard::downgrade_mode(ControlMode requested) const
{
    // 做法 A：只允许降级到 Manual（或 Failsafe，取决于你策略）
    // 这里采用更保守的：无法满足 gating 时 -> Manual
    (void)requested;
    return ControlMode::kManual;
}

void ControlGuard::clamp_teleop(ControlIntent& inout) const
{
    if (!inout.has_teleop_dof) return;

    auto& c = inout.teleop_dof_cmd;
    c.surge = clampd(c.surge, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.sway  = clampd(c.sway,  cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.heave = clampd(c.heave, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.roll  = clampd(c.roll,  cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.pitch = clampd(c.pitch, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.yaw   = clampd(c.yaw,   cfg_.teleop_dof_min, cfg_.teleop_dof_max);
}

void ControlGuard::clamp_ref_delta(ControlIntent& inout) const
{
    if (!inout.has_ref_delta) return;

    // 这里严格依赖你们 ControlReference 的字段布局。
    // 建议你后续把 ref_delta 从 ControlReference 提取为 RefDelta 结构，便于 clamp 和语义约束。
    // 当前保持钩子，不做假设。
}

GuardResult ControlGuard::step(std::uint64_t now_ns,
                              const ControlState& /*state*/,
                              const shared::msg::NavState* nav,
                              const ControlIntent& intent)
{
    GuardResult out{};
    out.last_intent_ns = last_intent_ns_;

    // 1) 复制 raw intent
    out.effective_intent = intent;
    const bool req_exit = intent.request_exit;

    // 2) 更新“输入新鲜度”状态
    // - seq 变化：认为新输入，刷新 last_intent_ns_
    // - 否则：不主动累加 age（由 is_intent_stale 基于时间戳判定）
    if (intent.seq != 0 && intent.seq != last_intent_seq_) {
        last_intent_seq_ = intent.seq;
        last_intent_ns_  = (intent.stamp_ns != 0) ? intent.stamp_ns : now_ns;
        input_age_ms_    = 0; // 保留字段：可用于日志/调试
    }

    // 3) stale 判定
    const bool stale = is_intent_stale(now_ns, intent);
    out.input_stale = stale;

    if (stale) {
        out.effective_intent.clear_payload();
        out.effective_intent.request_exit = req_exit;
        out.failsafe = FailsafeAction::kZeroOutput;
    }

    // 4) 安全仲裁：E-STOP latch
    // - 有效位 has_estop_cmd 控制是否采纳 estop/clear_estop
    if (out.effective_intent.has_estop_cmd) {
        if (out.effective_intent.estop) {
            if (cfg_.estop_latch) {
                estop_latched_ = true;
            } else {
                estop_latched_ = true; // 即使不 latch，这里也可用 level；按你策略调整
            }
        }
        if (out.effective_intent.clear_estop) {
            // 是否允许解除由策略控制：这里允许解除
            estop_latched_ = false;
        }
    }
    out.estop_latched = estop_latched_;

    // 5) Arm/Disarm
    if (out.effective_intent.has_arm_cmd) {
        if (out.effective_intent.disarm) {
            armed_ = false;
        } else if (out.effective_intent.arm) {
            if (!estop_latched_) {
                armed_ = true;
            }
        }
    }

    out.armed = armed_ && !estop_latched_;

    if (!out.armed) {
        out.failsafe = estop_latched_ ? FailsafeAction::kEmergencyStop
                                      : FailsafeAction::kZeroOutput;

        // 失能时：丢弃所有控制 payload（保留 request_exit）
        out.effective_intent.has_teleop_dof = false;
        out.effective_intent.has_ref       = false;
        out.effective_intent.has_ref_delta = false;
    }

    // 6) 模式门控
    // 说明：ControlMode::kNone 表示“无请求/不改变”，不应触发 mode_changed。
    if (out.effective_intent.has_mode_request &&
        out.effective_intent.mode_request != ControlMode::kNone) {

        ControlMode requested = out.effective_intent.mode_request;

        if (!nav_ok_for_mode(nav, requested)) {
            requested = downgrade_mode(requested);
        }

        out.mode_changed = (requested != mode_);
        mode_ = requested;
    }

    out.effective_mode = mode_;

    // 7) clamp payload
    clamp_teleop(out.effective_intent);
    clamp_ref_delta(out.effective_intent);

    return out;
}

} // namespace rovctrl::control_core
