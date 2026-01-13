#pragma once
#ifndef ROVCTRL_IO_INPUT_KEY_EVENT_SUBSCRIBER_SHM_HPP
#define ROVCTRL_IO_INPUT_KEY_EVENT_SUBSCRIBER_SHM_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "shared/msg/key_event.hpp"

namespace rovctrl::io {

/**
 * @brief Subscribe KeyEvent ring buffer via POSIX shared memory (Linux).
 *
 * Design goals:
 *  - Allow "lazy init": consumer can start before publisher creates shm.
 *  - Hard ABI check: magic/layout/payload(size/align/ver)/capacity must match.
 *  - Ring buffer semantics: reader keeps its own last_read_seq_ (monotonic).
 *  - Seqlock-consistent reads to avoid torn payload.
 *
 * Typical usage:
 *  - init(cfg{ enable=true, lazy_init=true })
 *  - in poll loop: poll_many(buf, N) -> for each event: handle_key(event.key)
 */
class KeyEventSubscriberShm final {
public:
    struct Config {
        bool        enable    = true;

        // Default shm name for "local keyboard events published by gateway/teleop_local".
        // Must start with '/' on POSIX.
        std::string shm_name  = "/rovctrl_key_event_local_v1";

        // If non-zero, clamp mmap size to this (<= real shm size). Usually keep 0.
        std::size_t shm_size  = 0;

        // If true: init() succeeds even if shm not yet present; poll will retry init.
        bool        lazy_init = true;
    };

public:
    KeyEventSubscriberShm() = default;
    ~KeyEventSubscriberShm() noexcept { shutdown(); }

    KeyEventSubscriberShm(const KeyEventSubscriberShm&) = delete;
    KeyEventSubscriberShm& operator=(const KeyEventSubscriberShm&) = delete;

    KeyEventSubscriberShm(KeyEventSubscriberShm&& other) noexcept;
    KeyEventSubscriberShm& operator=(KeyEventSubscriberShm&& other) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }
    bool error() const noexcept { return error_flag_; }

    // Returns publisher timestamps from the shm header, if available/valid.
    std::uint64_t last_pub_mono_ns() const noexcept { return last_pub_mono_ns_; }
    std::uint64_t last_pub_wall_ns() const noexcept { return last_pub_wall_ns_; }

    /**
     * @brief Read up to max_out pending events into out[].
     *
     * Semantics:
     *  - If shm not ready (lazy mode), returns 0.
     *  - If writer has advanced too far (reader lag), old events are dropped
     *    and last_read_seq_ is fast-forwarded to (write_seq - capacity).
     *  - Returns number of events copied (0..max_out).
     */
    int poll_many(shared::msg::KeyEvent* out, int max_out,
                  std::uint64_t* out_mono_ns = nullptr,
                  std::uint64_t* out_wall_ns = nullptr);

private:
    // Implementation details (Linux-only)
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;

    bool read_header_snapshot(std::uint64_t& out_write_seq,
                              std::uint64_t& out_mono_ns,
                              std::uint64_t& out_wall_ns) noexcept;

    bool read_event_at_seq(std::uint64_t seq, shared::msg::KeyEvent& out) noexcept;

private:
    Config cfg_{};

    bool enabled_{false};
    bool initialized_{false};
    bool error_flag_{false};

    std::string shm_name_{};
    std::size_t shm_size_{0};

    std::uint64_t last_pub_mono_ns_{0};
    std::uint64_t last_pub_wall_ns_{0};

    // Reader cursor
    std::uint64_t last_read_seq_{0};

#ifndef _WIN32
    int shm_fd_{-1};
#endif

    // Opaque mapped layout pointer (cast in .cpp)
    void* shm_ptr_{nullptr};
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_INPUT_KEY_EVENT_SUBSCRIBER_SHM_HPP
