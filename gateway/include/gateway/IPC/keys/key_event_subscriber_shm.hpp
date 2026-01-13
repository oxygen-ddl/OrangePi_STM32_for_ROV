#pragma once
#ifndef GATEWAY_IPC_KEYS_KEY_EVENT_SUBSCRIBER_SHM_HPP
#define GATEWAY_IPC_KEYS_KEY_EVENT_SUBSCRIBER_SHM_HPP

#include <cstddef>
#include <cstdint>
#include <string>

#include "shared/msg/key_event.hpp"
#include "shared/shm/key_event_ring_shm.hpp"

namespace comm_gcs::ipc::keys {

class KeyEventSubscriberShm final {
public:
    struct Config {
        bool        enable      = true;
        std::string shm_name    = "/rovctrl_key_event_local_v1";
        bool        lazy_init   = true;

        // 0 => accept publisher capacity; otherwise strict match.
        std::uint32_t expect_capacity = 0;
    };

    struct Stats final {
        std::uint64_t read_count    = 0;  // events delivered to caller
        std::uint64_t lost_count    = 0;  // overwrite loss computed by subscriber
        std::uint64_t seqlock_retry = 0;  // retry loops due to writer in progress
        std::uint64_t abi_mismatch  = 0;  // header ABI mismatch after mapping
        std::uint64_t open_fail     = 0;  // shm_open/mmap failures (lazy init mode)

        std::uint64_t last_write_idx = 0;
        std::uint64_t last_mono_ns   = 0;
        std::uint64_t last_wall_ns   = 0;
        std::uint64_t last_drop_count = 0;
    };

    KeyEventSubscriberShm() = default;
    ~KeyEventSubscriberShm() noexcept;

    KeyEventSubscriberShm(KeyEventSubscriberShm&& other) noexcept;
    KeyEventSubscriberShm& operator=(KeyEventSubscriberShm&& other) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool initialized() const noexcept { return initialized_; }
    bool enabled() const noexcept { return enabled_; }

    // Read up to max events into out[]; non-blocking.
    std::size_t drain(shared::msg::KeyEvent* out, std::size_t max) noexcept;

    // Read exactly one event if available.
    bool poll(shared::msg::KeyEvent& out) noexcept;

    // Skip all pending events; cursor jumps to latest write_idx.
    void flush() noexcept;

    // Reset stats + cursor; if start_from_latest=true, cursor jumps to current write_idx.
    void reset_cursor(bool start_from_latest = true) noexcept;

    const Stats& stats() const noexcept { return stats_; }

    // Publisher-side drop_count snapshot (does NOT include overwrite loss).
    std::uint64_t publisher_drop_count() const noexcept;

    // Stable header snapshot for debugging.
    bool snapshot_header(shared::shm::KeyEventRingShmHeader& out_hdr) noexcept;

private:
#ifndef _WIN32
    bool try_open_map_ro_() noexcept;
    void close_shm_() noexcept;
#endif

    bool read_header_stable_(shared::shm::KeyEventRingShmHeader& out_hdr) noexcept;
    bool validate_abi_(const shared::shm::KeyEventRingShmHeader& hdr) noexcept;

private:
    Config cfg_{};

    bool enabled_{false};
    bool initialized_{false};
    bool error_flag_{false};

#ifndef _WIN32
    int shm_fd_{-1};
#endif

    std::size_t shm_size_{0};
    const shared::shm::KeyEventRingShmLayout* shm_ptr_{nullptr};

    std::uint64_t last_read_idx_{0};
    std::uint32_t capacity_{0};

    Stats stats_{};
};

} // namespace comm_gcs::ipc::keys

#endif // GATEWAY_IPC_KEYS_KEY_EVENT_SUBSCRIBER_SHM_HPP
