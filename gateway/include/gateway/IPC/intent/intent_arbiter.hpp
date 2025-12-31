#pragma once
#ifndef GATEWAY_IPC_INTENT_INTENT_ARBITER_HPP
#define GATEWAY_IPC_INTENT_INTENT_ARBITER_HPP

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "shared/msg/control_intent.hpp"

namespace comm_gcs::ipc::intent {

/**
 * @brief Intent source identifier (pipeline-level).
 */
enum class SourceId : std::uint8_t {
    Remote  = 0,   ///< From GCS over network
    Local   = 1,   ///< From local keyboard / tools
    Auto    = 2,   ///< From autonomy / controller
    Unknown = 255
};

inline const char* to_string(SourceId s) noexcept
{
    switch (s) {
    case SourceId::Remote: return "remote";
    case SourceId::Local:  return "local";
    case SourceId::Auto:   return "auto";
    default:               return "unknown";
    }
}

/**
 * @brief A sampled intent + timing metadata (decoupled from shm layout).
 *
 * NOTE:
 *  - intent: payload snapshot cached by mux (std::optional)
 *  - pub_mono_ns/pub_wall_ns: timestamps from shm header
 *  - recv_mono_ns: gateway time when mux polled
 */
struct IntentSample final {
    SourceId src = SourceId::Unknown;

    std::optional<shared::msg::ControlIntent> intent;

    std::uint64_t pub_mono_ns  = 0;
    std::uint64_t pub_wall_ns  = 0;
    std::uint64_t recv_mono_ns = 0;

    bool subscriber_initialized = false;
};

/**
 * @brief Arbiter configuration per source.
 */
struct SourcePolicy final {
    SourceId src = SourceId::Unknown;

    // smaller value => higher priority (0 is highest)
    int priority = 100;

    // stale if age_ms > max_age_ms (age based on pub_mono_ns)
    std::uint32_t max_age_ms = 250;

    // warmup grace window since daemon start (ms)
    std::uint32_t warmup_ms = 1500;

    // default ttl for payload (ms) when wire ttl_ms==0
    // 0 means "no default ttl" (payload TTL check can still be enabled but ttl==0 => never expire)
    std::uint32_t default_ttl_ms = 0;

    // whether allow this source when stale (degraded selection)
    bool allow_stale = true;

    // whether this source is enabled in arbitration
    bool enable = true;
};

/**
 * @brief Decision result.
 */
struct ArbiterDecision final {
    // --- selection result ---
    bool has_winner = false;

    SourceId winner = SourceId::Unknown;       // 胜者来源（如果 degraded 且无 winner，可为 Unknown）
    IntentSample winner_sample{};             // 诊断用：胜者样本（或空样本）

    bool degraded = false;                    // winner 是否 stale 或本轮是否为“降级发布”
    std::uint64_t winner_age_ms = 0;
    std::string reason;

    // --- event semantics (诊断位) ---
    bool estop_assert = false;
    bool estop_clear  = false;

    // --- publish plan (关键新增) ---
    bool should_publish = true;               // 对应 policy.publish_when_no_fresh
    shared::msg::ControlIntent publish_intent{}; // 本轮应发布的最终意图（永远可用）
    bool request_shutdown = false;            // 请求守护进程停机

};


/**
 * @brief IntentArbiter: pure arbitration logic (no shm, no IO).
 */
class IntentArbiter final {
public:
    struct Policy final {
        
        bool require_initialized   = false;

        // enable payload ttl check (stamp_ns + ttl_ms/default_ttl_ms)
        bool enable_payload_ttl = true;

        // if stamp_ns==0, treat as expired? recommended true
        bool stamp0_is_expired = true;

        // whether to prioritize EStop over everything
        bool estop_priority = true;
        // ----- degrade publish policy -----
        // 当没有 fresh winner 时，是否仍然发布一个“降级 intent”
        bool publish_when_no_fresh = true;

        // 降级时是否保留 last_good（由调用者传入），否则发布清空 intent
        bool hold_last_good_on_degrade = true;

        // 降级发布时是否清空 payload（推荐 true：避免旧 DOF 残留）
        bool clear_payload_on_degrade = true;     

    };

    IntentArbiter() = default;

    void set_policy(const Policy& p) { policy_ = p; }
    const Policy& policy() const noexcept { return policy_; }

    void set_sources(std::vector<SourcePolicy> sources);
    const std::vector<SourcePolicy>& sources() const noexcept { return sources_; }

    ArbiterDecision decide(std::uint64_t now_mono_ns,
                       std::uint64_t t0_mono_ns,
                       const std::vector<IntentSample>& samples,
                       const shared::msg::ControlIntent* last_good) const;
    // 兼容旧调用：不关心上一次已发布意图时，可以调用 3 参数版本
    ArbiterDecision decide(std::uint64_t now_mono_ns,
                           std::uint64_t stale_ttl_ms,
                           const std::vector<IntentSample>& samples) const
    {
        return decide(now_mono_ns, stale_ttl_ms, samples, nullptr);
    }



private:
    static std::uint64_t ns_to_ms(std::uint64_t ns) noexcept { return ns / 1000000ull; }

    const SourcePolicy* find_policy(SourceId src) const noexcept;

    bool is_stale(const SourcePolicy& sp,
                  std::uint64_t now_mono_ns,
                  std::uint64_t t0_mono_ns,
                  const IntentSample& smp,
                  std::uint64_t* out_age_ms) const noexcept;

    bool is_expired_payload(const SourcePolicy& sp,
                            std::uint64_t now_mono_ns,
                            const IntentSample& smp,
                            std::uint64_t* out_left_ms) const noexcept;

private:
    Policy policy_{};
    std::vector<SourcePolicy> sources_{};
};

} // namespace comm_gcs::ipc::intent

#endif // GATEWAY_IPC_INTENT_INTENT_ARBITER_HPP
