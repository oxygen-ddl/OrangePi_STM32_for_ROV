// gateway/include/gateway/IPC/intent/intent_mux.hpp
#pragma once
#ifndef GATEWAY_IPC_INTENT_INTENT_MUX_HPP
#define GATEWAY_IPC_INTENT_INTENT_MUX_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "gateway/IPC/intent/intent_arbiter.hpp"          // SourceId, IntentSample
#include "gateway/IPC/intent/intent_subscriber_shm.hpp"   // GcsIntentSubscriberShm


namespace comm_gcs::ipc::intent {

/**
 * @brief Mux source configuration: describes one upstream intent SHM.
 *
 * 典型三路（默认建议）：
 *   - Remote (GCS): /rovctrl_intent_remote_v1
 *   - Local  (teleop_local / local tools): /rovctrl_intent_local_v1
 *   - Auto   (planner/MPC/RL): /rovctrl_intent_auto_v1
 *
 * Mux 只负责“采样与缓存”，不做胜者裁决（裁决由 IntentArbiter）。
 */
struct MuxSourceConfig final {
    SourceId     src       = SourceId::Unknown;
    std::string  shm_name  = "";

    bool         enable    = true;
    bool         lazy_init = true;

    // Optional: fixed shm size override (0 => sizeof(shared::shm::IntentShmLayout) or fstat size)
    std::size_t  shm_size  = 0;

    // Optional: assign a fixed source priority at mux level (0..255).
    // If you希望 prio 完全由 wire.intent.source_prio 决定，也可以不用这个字段。
    std::uint8_t source_prio = 0;
};

/**
 * @brief IntentMux: owns multiple shm subscribers, polls them, caches last samples.
 *
 * Behavior:
 *  - init(): create entries for all sources; init subscribers (supports lazy init).
 *  - poll_all(): poll each enabled source once; update cached IntentSample if hit.
 *  - samples_ref(): returns cached samples (one per configured source).
 *
 * Note:
 *  - Mux 不做 stale 判定；它只记录 sample 的 recv_mono_ns / pub_mono_ns，交给 Arbiter 根据 policy 决策。
 */
class IntentMux final {
public:
    IntentMux() = default;
    ~IntentMux() noexcept { shutdown(); }

    IntentMux(const IntentMux&) = delete;
    IntentMux& operator=(const IntentMux&) = delete;

    IntentMux(IntentMux&&) noexcept = default;
    IntentMux& operator=(IntentMux&&) noexcept = default;

    bool init(const std::vector<MuxSourceConfig>& sources);
    void shutdown() noexcept;

    void poll_all(std::uint64_t now_mono_ns);

    // Avoid copying vector each time.
    const std::vector<IntentSample>& samples_ref() const noexcept { return samples_; }

    struct Stats {
        std::uint64_t poll_calls = 0;
        std::uint64_t poll_hits  = 0;
    };
    const Stats& stats() const noexcept { return stats_; }

private:
    struct Entry final {
           MuxSourceConfig cfg{};
           GcsIntentSubscriberShm sub{};

           std::uint64_t last_pub_mono_ns = 0;
           std::uint64_t last_pub_wall_ns = 0;

        bool ready = false;
    };


private:
    std::vector<Entry> entries_;
    std::vector<IntentSample> samples_; // cached outputs aligned with entries_
    Stats stats_{};
};

} // namespace comm_gcs::ipc::intent

#endif // GATEWAY_IPC_INTENT_INTENT_MUX_HPP
