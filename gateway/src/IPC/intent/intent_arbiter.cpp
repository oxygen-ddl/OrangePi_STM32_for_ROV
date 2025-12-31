// gateway/src/IPC/intent/intent_arbiter.cpp

#include "gateway/IPC/intent/intent_arbiter.hpp"

#include <algorithm>
#include <cstdint>

namespace comm_gcs::ipc::intent {

// ------------------------------
// helpers
// ------------------------------


void IntentArbiter::set_sources(std::vector<SourcePolicy> sources)
{
    sources_ = std::move(sources);
    std::sort(sources_.begin(), sources_.end(),
              [](const SourcePolicy& a, const SourcePolicy& b) {
                  if (a.priority != b.priority) return a.priority < b.priority;
                  return static_cast<int>(a.src) < static_cast<int>(b.src);
              });
}

const SourcePolicy* IntentArbiter::find_policy(SourceId src) const noexcept
{
    for (const auto& sp : sources_) {
        if (sp.src == src) return &sp;
    }
    return nullptr;
}

bool IntentArbiter::is_stale(const SourcePolicy& sp,
                             std::uint64_t now_mono_ns,
                             std::uint64_t t0_mono_ns,
                             const IntentSample& smp,
                             std::uint64_t* out_age_ms) const noexcept
{
    if (out_age_ms) *out_age_ms = 0;

    // pub_mono_ns == 0 => treat stale (unknown publisher time)
    if (smp.pub_mono_ns == 0) {
        if (out_age_ms) *out_age_ms = UINT64_MAX;
        return true;
    }

    const std::uint64_t age_ms = ns_to_ms(now_mono_ns - smp.pub_mono_ns);
    if (out_age_ms) *out_age_ms = age_ms;

    // warmup grace: during early startup, we do not mark stale
    const std::uint64_t elapsed_ms = ns_to_ms(now_mono_ns - t0_mono_ns);
    if (elapsed_ms < static_cast<std::uint64_t>(sp.warmup_ms)) return false;

    return age_ms > static_cast<std::uint64_t>(sp.max_age_ms);
}

bool IntentArbiter::is_expired_payload(const SourcePolicy& sp,
                                       std::uint64_t now_mono_ns,
                                       const IntentSample& smp,
                                       std::uint64_t* out_left_ms) const noexcept
{
    if (out_left_ms) *out_left_ms = 0;
    if (!policy_.enable_payload_ttl) return false;

    if (!smp.intent.has_value()) return true;
    const auto& it = *smp.intent;

    // stamp_ns semantics
    if (it.stamp_ns == 0) {
        return policy_.stamp0_is_expired;
    }

    const std::uint32_t ttl_ms = (it.ttl_ms != 0) ? it.ttl_ms : sp.default_ttl_ms;
    if (ttl_ms == 0) {
        // ttl disabled => never expire
        return false;
    }

    const std::uint64_t deadline_ns = it.stamp_ns + static_cast<std::uint64_t>(ttl_ms) * 1000000ull;
    if (now_mono_ns >= deadline_ns) {
        if (out_left_ms) *out_left_ms = 0;
        return true;
    }

    if (out_left_ms) {
        *out_left_ms = ns_to_ms(deadline_ns - now_mono_ns);
    }
    return false;
}

// ------------------------------
// decide (route-A)
// ------------------------------
ArbiterDecision IntentArbiter::decide(std::uint64_t now_mono_ns,
                                      std::uint64_t t0_mono_ns,
                                      const std::vector<IntentSample>& samples,
                                      const shared::msg::ControlIntent* last_good) const
{
    ArbiterDecision d{};
    d.should_publish = true;
    d.publish_intent = shared::msg::ControlIntent{}; // default cleared

    if (sources_.empty()) {
        d.should_publish = policy_.publish_when_no_fresh;
        d.has_winner = false;
        d.degraded = true;
        d.reason = "no source policy configured";
        if (policy_.publish_when_no_fresh && last_good && policy_.hold_last_good_on_degrade) {
            d.publish_intent = *last_good;
            if (policy_.clear_payload_on_degrade) d.publish_intent.clear_payload();
        } else {
            if (policy_.clear_payload_on_degrade) d.publish_intent.clear_payload();
        }
        // normalize stamp/ttl for downstream (optional; intentd can also set)
        d.publish_intent.stamp_ns = now_mono_ns;
        return d;
    }

    auto find_sample = [&](SourceId src) -> const IntentSample* {
        for (const auto& s : samples) {
            if (s.src == src) return &s;
        }
        return nullptr;
    };

    // ------------------------------
    // Pass 0: EStop priority (assert only)
    // ------------------------------
    if (policy_.estop_priority) {
        for (const auto& sp : sources_) {
            if (!sp.enable) continue;

            const auto* smp = find_sample(sp.src);
            if (!smp || !smp->intent.has_value()) continue;
            if (policy_.require_initialized && !smp->subscriber_initialized) continue;

            const auto& it = *smp->intent;
            const bool has_estop_flag = (it.flags & shared::msg::kHasEStopCmd) != 0;
            (void)has_estop_flag; // keep for future diag if needed

            if (it.estop == 0) continue;
            if (is_expired_payload(sp, now_mono_ns, *smp, nullptr)) continue;

            std::uint64_t age_ms = 0;
            const bool stale = is_stale(sp, now_mono_ns, t0_mono_ns, *smp, &age_ms);

            d.has_winner = true;
            d.winner = sp.src;
            d.winner_sample = *smp;
            d.degraded = stale;
            d.winner_age_ms = age_ms;
            d.reason = stale ? "estop winner (stale)" : "estop winner (fresh)";

            d.estop_assert = true;
            d.estop_clear  = false;

            d.publish_intent = it;
            d.publish_intent.stamp_ns = now_mono_ns;
            return d;
        }
    }

    // ------------------------------
    // Pass 1: pick best fresh winner (priority order)
    // ------------------------------
    for (const auto& sp : sources_) {
        if (!sp.enable) continue;

        const auto* smp = find_sample(sp.src);
        if (!smp || !smp->intent.has_value()) continue;
        if (policy_.require_initialized && !smp->subscriber_initialized) continue;

        if (is_expired_payload(sp, now_mono_ns, *smp, nullptr)) continue;

        std::uint64_t age_ms = 0;
        const bool stale = is_stale(sp, now_mono_ns, t0_mono_ns, *smp, &age_ms);
        if (stale) continue;

        d.has_winner = true;
        d.winner = sp.src;
        d.winner_sample = *smp;
        d.degraded = false;
        d.winner_age_ms = age_ms;
        d.reason = "fresh winner";

        const auto& it = *smp->intent;
        d.publish_intent = it;
        d.publish_intent.stamp_ns = now_mono_ns;
         // ------------------------------
        // Exit semantics (Local only):
        // request_exit => publish EStop + request gateway shutdown
        // ------------------------------
        if (d.winner == sp.src && sp.src == SourceId::Local) {
            const bool has_exit = ((it.flags & shared::msg::kHasExitRequest) != 0u) && (it.request_exit != 0);
            if (has_exit) {
                d.request_shutdown = true;

                // Safety: ensure an EStop pulse is published in the same frame
                d.publish_intent.estop = 1;
                d.publish_intent.flags |= shared::msg::kHasEStopCmd;

                // Optional: do NOT forward request_exit to control side if you prefer
                // d.publish_intent.request_exit = 0;
                // d.publish_intent.flags &= ~shared::msg::kHasExitRequest;
            }
        }

        // estop clear flag (diagnostic)
        if ((it.flags & shared::msg::kHasEStopCmd) && it.clear_estop != 0) {
            d.estop_clear = true;
        }
        return d;
    }

    // ------------------------------
    // Pass 2: allow stale winner if policy allows (degraded selection)
    // ------------------------------
    const IntentSample* best_stale = nullptr;
    const SourcePolicy* best_sp    = nullptr;
    std::uint64_t best_age_ms      = 0;

    for (const auto& sp : sources_) {
        if (!sp.enable) continue;
        if (!sp.allow_stale) continue;

        const auto* smp = find_sample(sp.src);
        if (!smp || !smp->intent.has_value()) continue;
        if (policy_.require_initialized && !smp->subscriber_initialized) continue;

        if (is_expired_payload(sp, now_mono_ns, *smp, nullptr)) continue;

        std::uint64_t age_ms = 0;
        const bool stale = is_stale(sp, now_mono_ns, t0_mono_ns, *smp, &age_ms);
        if (!stale) continue;

        // sources_ already sorted by priority, so first stale hit is best
        best_stale = smp;
        best_sp = &sp;
        best_age_ms = age_ms;
        break;
    }

    if (best_stale && best_sp) {
        d.has_winner = true;
        d.winner = best_sp->src;
        d.winner_sample = *best_stale;
        d.degraded = true;
        d.winner_age_ms = best_age_ms;
        d.reason = "stale winner (allow_stale)";

        d.publish_intent = *best_stale->intent;
        d.publish_intent.stamp_ns = now_mono_ns;

        const auto& it = *best_stale->intent;
        if ((it.flags & shared::msg::kHasEStopCmd) && it.clear_estop != 0) {
            d.estop_clear = true;
        }
        return d;
    }

    if (d.has_winner && d.winner == SourceId::Local) {
        const auto& wi = d.publish_intent;
        if ((wi.flags & shared::msg::kHasExitRequest) != 0u) {
            d.request_shutdown = true;

            // 保险：即使上游没置 estop，这里也补一个停机脉冲
            d.publish_intent.estop = 1;
            // EStop flag
            d.publish_intent.flags |= shared::msg::kHasEStopCmd;
        }
    }

    // ------------------------------
    // Pass 3: no winner => degrade publish decision produced here
    // ------------------------------
    d.has_winner = false;
    d.winner = SourceId::Unknown;
    d.winner_sample = IntentSample{};
    d.degraded = true;
    d.winner_age_ms = 0;
    d.reason = "no eligible winner";

    if (!policy_.publish_when_no_fresh) {
        d.should_publish = false;
        return d;
    }

    // Degrade publish strategy
    if (policy_.hold_last_good_on_degrade && last_good) {
        d.publish_intent = *last_good;
        d.reason = "degraded publish: hold_last_good";
    } else {
        d.publish_intent = shared::msg::ControlIntent{};
        d.reason = "degraded publish: cleared";
    }

    if (policy_.clear_payload_on_degrade) {
        d.publish_intent.clear_payload();
    }

    // normalize stamp/ttl for downstream safety
    d.publish_intent.source_id   = static_cast<std::uint8_t>(shared::msg::IntentSource::kUnknown);
    d.publish_intent.source_prio = 0;
    d.publish_intent.stamp_ns    = now_mono_ns;
    return d;
}



} // namespace comm_gcs::ipc::intent
