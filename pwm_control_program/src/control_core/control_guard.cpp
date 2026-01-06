#include "control_core/control_guard.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>   // <<< 新增
#include <cstring>


// 只在 cpp 里依赖 nav_state 的完整定义
#include "shared/msg/nav_state.hpp"

// 做法 A：ControlIntent/ControlMode 真源在 control_core
#include "control_core/control_intent.hpp"
#include "control_core/control_mode.hpp"

namespace rovctrl::control_core {

static inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
    
}

static inline double absd(double x) noexcept { return x < 0 ? -x : x; }

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
    last_intent_cmd_seq_ = 0;
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
bool ControlGuard::nav_ok_for_mode(const shared::msg::NavStateView* nav,
                                  rovctrl::control_core::ControlMode requested) const
{
    // 1) 没有 nav 直接不 OK（通常用于 AUTO / stabilized 等模式）
    if (!nav) return false;

    // 2) 基础可用性：gateway 的 nav_viewd 已经给了 valid/health
    //    valid==1 表示字段有限且 health OK/DEGRADED（按你的 builder 策略）
    if (nav->valid == 0) return false;

    // 3) 你原来用 reserved1 来存 ESKF OK 等状态位 —— 现在 view.reserved1 里就是 status_flags（由 nav_viewd 填）
    //    只要你在 nav_view_builder.cpp 里做了：
    //      v.reserved1 = static_cast<uint32_t>(s.status_flags);
    //    这里就可以继续用 reserved1 做 bitmask。
    const std::uint32_t flags = nav->reserved1;
    const bool eskf_ok = (flags & shared::msg::NAV_FLAG_ESKF_OK) != 0;

    // 4) 根据模式决定是否强依赖 ESKF（示例：你按自己模式要求改）
    switch (requested) {
    case ControlMode::kManual:
        // 手动模式一般不强依赖导航
        return true;

    case ControlMode::kAuto:
        // 自动/闭环依赖导航
        return eskf_ok;

    default:
        return eskf_ok;
    }
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

// =========================================================
// ControlGuard helpers (private)
// =========================================================
bool ControlGuard::is_neutral_for_clear_(const ControlIntent& intent) const noexcept
{
    // 你可以把 eps 做成 cfg 参数；先给一个工程上相对保守的默认值
    constexpr double kEps = 0.05; // “基本回中立”的阈值，按你的摇杆映射调整

    const auto& d = intent.teleop_dof_cmd;

    return absd(d.surge) < kEps &&
           absd(d.sway)  < kEps &&
           absd(d.heave) < kEps &&
           absd(d.roll)  < kEps &&
           absd(d.pitch) < kEps &&
           absd(d.yaw)   < kEps;
}

GuardResult ControlGuard::step(std::uint64_t now_ns,
                               const ControlState& /*state*/,
                               const shared::msg::NavStateView* nav,
                               const ControlIntent& intent)
{
    GuardResult out{};
    out.last_intent_ns = last_intent_ns_;

    // 0) 先把原始 intent 拷贝一份作为“可修改的有效意图”
    out.effective_intent = intent;
    out.effective_mode   = mode_;
    out.mode_changed     = false;
    out.input_stale      = false;
    out.estop_latched    = estop_latched_;
    out.armed            = armed_;                 // 真正的值在 ARM 逻辑后统一更新
    out.failsafe         = FailsafeAction::kNone;

    const bool req_exit = (intent.request_exit != 0);

    // 1) 更新“输入新鲜度”（基于 cmd_seq + stamp_ns）
    if (intent.cmd_seq != 0 && intent.cmd_seq != last_intent_cmd_seq_) {
        last_intent_cmd_seq_ = intent.cmd_seq;
        last_intent_ns_      = (intent.stamp_ns != 0) ? intent.stamp_ns : now_ns;
        input_age_ms_        = 0;  // 目前仅用于调试
    }

    // 2) stale 判定
    const bool stale = is_intent_stale(now_ns, intent);
    out.input_stale  = stale;

    if (stale) {
        // TTL 过期：保留 request_exit 语义，其它字段后面由 failsafe 统一处理
        out.effective_intent.request_exit = req_exit ? 1 : 0;
    }

    // 3) E-STOP 锁存与解除（S2：按住中立一段时间才允许解除）
    {
        auto& eff = out.effective_intent;

        const bool has_estop_level = (eff.estop != 0);
        const bool has_clear_req   = (eff.clear_estop != 0);

        if (has_estop_level) {
            // (A) 任何时刻收到 estop=1：立即锁存，并重置“解除计时”
            estop_latched_       = true;
            clear_hold_start_ns_ = 0;
            clear_hold_ms_       = 0;
        } else if (estop_latched_ && has_clear_req) {
            // (B) 只有在已锁存时，才考虑解除
            if (is_neutral_for_clear_(eff)) {
                if (clear_hold_start_ns_ == 0) {
                    clear_hold_start_ns_ = now_ns;
                    clear_hold_ms_       = 0;
                } else if (now_ns >= clear_hold_start_ns_) {
                    clear_hold_ms_ = static_cast<std::uint32_t>(
                        (now_ns - clear_hold_start_ns_) / 1000000ull
                    );
                }

                const std::uint32_t hold_threshold_ms =
                    (cfg_.estop_clear_hold_ms > 0) ? cfg_.estop_clear_hold_ms : 2000;

                if (clear_hold_ms_ >= hold_threshold_ms) {
                    estop_latched_       = false;
                    clear_hold_start_ns_ = 0;
                    clear_hold_ms_       = 0;
                }
            } else {
                // 不在中立态：重置计时，防误解锁
                clear_hold_start_ns_ = 0;
                clear_hold_ms_       = 0;
            }
        } else {
            // (C) 没有 clear 请求或未锁存：计时清零
            clear_hold_start_ns_ = 0;
            clear_hold_ms_       = 0;
        }

        out.estop_latched = estop_latched_;
    }

    // 4) 处理 ARM / DISARM（解锁 / 上锁）
    //
    // 说明：
    //  - 只依赖 effective_intent（经过 TTL / failsafe 基础清洗后的意图）；
    //  - 有急停锁存时强制上锁，ARM 请求在急停期间无效；
    //  - 一帧中若 arm 和 disarm 同时为真，则以 disarm 优先（保守策略）。
    {
        auto& eff = out.effective_intent;

        const bool has_arm_cmd = eff.has_arm_cmd;   // 来自 core::ControlIntent
        const bool prev_armed  = armed_;            // 用于检测状态是否变化

        if (estop_latched_) {
            // 急停锁存：无条件上锁
            armed_ = false;
        } else if (has_arm_cmd) {
            const bool req_arm    = (eff.arm    != 0);
            const bool req_disarm = (eff.disarm != 0);

            // 调试：仅在这一帧确实带 ARM 命令时打印
            std::cout << "[ControlGuard][ARM] has_arm_cmd=1"
                      << " estop_latched=" << int(estop_latched_)
                      << " req_arm="      << int(req_arm)
                      << " req_disarm="   << int(req_disarm)
                      << " prev_armed="   << int(prev_armed)
                      << "\n";

            if (req_disarm) {
                // 优先 DISARM：一旦收到上锁请求，立即上锁
                armed_ = false;
            } else if (req_arm) {
                // 当前无急停锁存的前提下，可以解锁
                armed_ = true;
            }
            // 若 has_arm_cmd=1 但 arm/disarm 都为 0，则保持原状态不变
        }

        // 仅在状态真正变化时打印一条变更日志（避免刷屏）
        if (armed_ != prev_armed) {
            std::cout << "[ControlGuard][ARM] armed_ changed "
                      << prev_armed << " -> " << armed_ << "\n";
        }

        // 内部状态同步到输出
        out.armed = armed_;
    }

    // 5) 未解锁时禁止 DOF/Ref 输出（保留 exit / estop / mode 请求）
    {
        auto& eff = out.effective_intent;

        if (!out.armed) {
            if (eff.has_teleop_dof) {
                eff.teleop_dof_cmd = rovctrl::control_core::DofCommand{};
                eff.has_teleop_dof = false;
            }

            if (eff.has_ref) {
                eff.has_ref = false;
            }
            if (eff.has_ref_delta) {
                eff.has_ref_delta = false;
            }
        }
    }

    // 6) 模式门控
    {
        auto& eff = out.effective_intent;

        if (eff.has_mode_request && eff.mode_request != ControlMode::kNone) {
            ControlMode requested = eff.mode_request;

            if (!nav_ok_for_mode(nav, requested)) {
                requested = downgrade_mode(requested);
            }

            out.mode_changed = (requested != mode_);
            mode_            = requested;
        }

        out.effective_mode = mode_;
    }

    // 7) 数值限幅
    clamp_teleop(out.effective_intent);
    clamp_ref_delta(out.effective_intent);

    // 8) failsafe 决策
    FailsafeAction fs = FailsafeAction::kNone;

    if (estop_latched_) {
        fs = FailsafeAction::kEmergencyStop;   // 3：急停
    } else if (stale) {
        fs = FailsafeAction::kZeroOutput;      // 2：输入过期 → 零输出
    } else if (!out.armed) {
        // 未 ARM 时统一 ZeroOutput，确保推进器被拉回中立
        fs = FailsafeAction::kZeroOutput;
    }

    out.failsafe = fs;

    // 9) 调试输出：仅在关键状态变化时打印（armed / estop / mode / has_nav / failsafe）
    std::uint32_t intent_age_ms = 0;
    if (intent.stamp_ns != 0 && now_ns >= intent.stamp_ns) {
        intent_age_ms = static_cast<std::uint32_t>(
            (now_ns - intent.stamp_ns) / 1000000ull
        );
    }

    const auto& eff = out.effective_intent;

    // 仅关注“关键状态”：
    //   - armed         : 是否解锁
    //   - estop_latched : 急停是否锁存
    //   - mode          : 当前控制模式
    //   - has_nav       : 是否有导航数据
    //   - failsafe      : 是否处于失效保护模式
    struct GuardDebugSnapshot {
        std::uint8_t armed;
        std::uint8_t estop_latched;
        std::uint8_t mode;
        std::uint8_t has_nav;
        std::uint8_t failsafe;
    };

    GuardDebugSnapshot cur{
        static_cast<std::uint8_t>(out.armed ? 1 : 0),
        static_cast<std::uint8_t>(out.estop_latched ? 1 : 0),
        static_cast<std::uint8_t>(static_cast<int>(out.effective_mode)),
        static_cast<std::uint8_t>(nav ? 1 : 0),
        static_cast<std::uint8_t>(static_cast<int>(out.failsafe)),
    };

    static GuardDebugSnapshot s_last{};
    static bool s_have_last = false;

    const bool changed = !s_have_last ||
                         std::memcmp(&cur, &s_last, sizeof(GuardDebugSnapshot)) != 0;

    if (changed) {
        std::cout << "[ControlGuard][STATE] "
                  << "armed="            << int(out.armed)
                  << " estop_latched="   << int(out.estop_latched)
                  << " mode="            << static_cast<int>(out.effective_mode)
                  << " has_nav="         << (nav ? 1 : 0)
                  << " intent_has_dof="  << int(intent.has_teleop_dof)
                  << " eff_has_dof="     << int(eff.has_teleop_dof)
                  << " intent_ttl_ms="   << intent.ttl_ms
                  << " intent_age_ms="   << intent_age_ms
                  << " stale="           << int(stale)
                  << " failsafe="        << static_cast<int>(out.failsafe)
                  << "\n";

        s_last      = cur;
        s_have_last = true;
    }

    return out;

}



} // namespace rovctrl::control_core
