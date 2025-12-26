#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace shared::msg {
struct ControlIntent;   // wire struct (POD)
} // namespace shared::msg

namespace rovctrl::io::input {

/**
 * @brief Subscribe GCS ControlIntent from POSIX shared memory.
 *
 * Intended collaboration in this project:
 *  - comm_gcs runs as a separate process:
 *      UDP/session/protocol -> build shared::msg::ControlIntent -> publish to shm.
 *  - pwm_control_program subscribes shm:
 *      read wire intent -> (cpp) convert to internal control_core::ControlIntent -> feed MultiInputProvider.
 *
 * Important:
 *  - This header intentionally does NOT include control_core/control_intent.hpp
 *    to avoid large include fan-out. The conversion happens in .cpp.
 *  - This class focuses on IPC subscription only (no TTL policy; no guard logic).
 */
class GcsIntentSubscriberShm final {
public:
    // ------------------------------------------------------------------------
    // Config
    // ------------------------------------------------------------------------
    struct Config {
        bool        enable   = true;
        std::string shm_name = "/rovctrl_gcs_intent_v1";
        std::size_t shm_size = 0;   // 0 => sizeof(ShmLayout) in .cpp

        /**
         * @brief If true, subscriber will try to (re)init on first poll even if init() wasn't called.
         * Useful when you want pwm_control_program to start before comm_gcs.
         */
        bool lazy_init = true;
    };

    // ------------------------------------------------------------------------
    // Shared header contract (must match comm_gcs::IntentPublisherShm::ShmHeader layout)
    // Keep it identical: seq (atomic), mono_ns, wall_ns, magic, version.
    // ------------------------------------------------------------------------
    struct ShmHeader {
        std::atomic<std::uint64_t> seq{0};  // odd=writing, even=stable
        std::uint64_t mono_ns = 0;
        std::uint64_t wall_ns = 0;
        std::uint32_t magic   = 0;
        std::uint32_t version = 0;
    };

    // ------------------------------------------------------------------------
    // Lifecycle / move-only
    // ------------------------------------------------------------------------
    GcsIntentSubscriberShm() = default;
    ~GcsIntentSubscriberShm() noexcept;

    GcsIntentSubscriberShm(const GcsIntentSubscriberShm&)            = delete;
    GcsIntentSubscriberShm& operator=(const GcsIntentSubscriberShm&) = delete;

    GcsIntentSubscriberShm(GcsIntentSubscriberShm&& other) noexcept;
    GcsIntentSubscriberShm& operator=(GcsIntentSubscriberShm&& other) noexcept;

    // ------------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------------
    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    /**
     * @brief Poll one snapshot from shm.
     *
     * Semantics:
     *  - Returns std::nullopt if disabled / not initialized / no valid snapshot / read failure.
     *  - Returns a wire intent snapshot when seqlock read succeeds and header matches.
     *
     * Note:
     *  - TTL / stale determination is intentionally NOT done here, because the control layer
     *    (ControlGuard / ControlLoop) already enforces TTL and safety policy centrally.
     *  - You can optionally use out_mono_ns/out_wall_ns for debug/telemetry.
     */
    std::optional<shared::msg::ControlIntent> poll_wire(std::uint64_t* out_mono_ns = nullptr,
                                                        std::uint64_t* out_wall_ns = nullptr);

    /**
     * @brief Convenience: last seen publisher timestamp (mono ns) for diagnostics.
     */
    std::uint64_t last_pub_mono_ns() const noexcept { return last_pub_mono_ns_; }

private:
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;

    bool read_once(shared::msg::ControlIntent& out,
                   std::uint64_t* out_mono_ns,
                   std::uint64_t* out_wall_ns) noexcept;

private:
    bool enabled_     = false;
    bool initialized_ = false;
    bool error_flag_  = false;

    Config cfg_{};

    std::string shm_name_;
    std::size_t shm_size_ = 0;

    std::uint64_t last_pub_mono_ns_ = 0;

#ifndef _WIN32
    int shm_fd_ = -1;

    // defined in .cpp:
    // struct ShmLayout { ShmHeader hdr; shared::msg::ControlIntent intent; };
    struct ShmLayout;
    ShmLayout* shm_ptr_ = nullptr;
#else
    void* shm_handle_ = nullptr;
    void* shm_ptr_    = nullptr;
#endif
};

} // namespace rovctrl::io::input
