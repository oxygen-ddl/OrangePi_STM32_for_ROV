#pragma once
#ifndef GATEWAY_IPC_KEYS_KEY_EVENT_PUBLISHER_SHM_HPP
#define GATEWAY_IPC_KEYS_KEY_EVENT_PUBLISHER_SHM_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include "shared/msg/key_event.hpp"
#include "shared/shm/key_event_ring_shm.hpp"

namespace comm_gcs::ipc::keys {

/**
 * @brief POSIX SHM publisher for KeyEvent ring buffer.
 *
 * Contract:
 *  - Publisher OWNS creation of shm and layout initialization.
 *  - Subscriber(s) open read-only and perform ABI checks + ring consumption.
 *
 * History policy:
 *  - For keyboard events, we do NOT want to keep "historical" events across restarts.
 *  - Therefore, the default is truncate=true, i.e. publisher unlinks and recreates shm
 *    so the ring is clean on every start.
 */
class KeyEventPublisherShm final {
public:
    struct Config {
        bool        enable      = true;

        // Default: local keyboard events (teleop_local).
        // Use a distinct name for other sources if needed (e.g., remote keyboard gateway).
        std::string shm_name    = "/rovctrl_key_event_local_v1";

        // Ring capacity (number of KeyEvent slots).
        // Default: 1024 (comfortable for bursty terminal auto-repeat + scheduling jitter).
        // Must satisfy shared contract constraints in shared::shm::key_event_ring_shm.hpp.
        std::uint32_t capacity  = 1024;

        // POSIX shm mode; publisher owns creation.
        // Typical: 0666 so different processes/users can read in dev environment.
        std::uint32_t create_mode = 0666;

        // If true: shm_unlink(name) before create, ensuring a clean ring each start.
        // For keyboard events we prefer TRUE (no historical events).
        bool truncate = true;
    };

    KeyEventPublisherShm() = default;
    ~KeyEventPublisherShm() noexcept;

    KeyEventPublisherShm(KeyEventPublisherShm&& other) noexcept;
    KeyEventPublisherShm& operator=(KeyEventPublisherShm&& other) noexcept;
    
    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool initialized() const noexcept { return initialized_; }

    // Total dropped events recorded by the ring header (publisher-side increments).
    std::uint64_t dropped() const noexcept;

    // Publish one key event into ring (non-blocking).
    bool publish(const shared::msg::KeyEvent& ev) noexcept;

    // Publish multiple events; returns number accepted.
    std::size_t publish_many(const shared::msg::KeyEvent* evs, std::size_t n) noexcept;

private:
    bool open_or_create_shm_(const Config& cfg);
    void close_shm_() noexcept;

    // Seqlock begin/end on header seq (odd=writing, even=stable).
    void begin_write_() noexcept;
    void end_write_() noexcept;

private:
    Config cfg_{};

    bool enabled_{false};
    bool initialized_{false};
    bool error_flag_{false};

#ifndef _WIN32
    int shm_fd_{-1};
#else
    void* shm_handle_{nullptr};
#endif

    std::size_t shm_size_{0};
    shared::shm::KeyEventRingShmLayout* shm_ptr_{nullptr};
};

} // namespace comm_gcs::ipc::keys

#endif // GATEWAY_IPC_KEYS_KEY_EVENT_PUBLISHER_SHM_HPP
