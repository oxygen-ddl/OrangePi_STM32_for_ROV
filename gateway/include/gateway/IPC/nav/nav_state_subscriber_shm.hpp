#pragma once
#ifndef COMM_GCS_IPC_NAV_NAV_STATE_SUBSCRIBER_SHM_HPP
#define COMM_GCS_IPC_NAV_NAV_STATE_SUBSCRIBER_SHM_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace shared::msg {
struct NavState;
} // namespace shared::msg

namespace comm_gcs::ipc::nav {

/**
 * @brief Subscribe shared::msg::NavState from POSIX shared memory (seqlock).
 *
 * Contract:
 *   - Layout and ABI contract is defined in shared/shm/nav_state_shm.hpp
 *   - Reader validates hdr.magic/layout_ver/payload_ver/payload_size/payload_align
 *   - Payload is copied via memcpy after seqlock snapshot is stable
 *
 * Notes:
 *   - No TTL/stale policy here; that belongs to control/guard or higher-level logic.
 */
class NavStateSubscriberShm final {
public:
    struct Config {
        bool        enable   = true;
        std::string shm_name = "/rovctrl_nav_state_v1";
        std::size_t shm_size = 0;   // 0 => use fstat() size
        bool        lazy_init = true;
    };

    NavStateSubscriberShm() = default;
    ~NavStateSubscriberShm() noexcept;

    NavStateSubscriberShm(const NavStateSubscriberShm&)            = delete;
    NavStateSubscriberShm& operator=(const NavStateSubscriberShm&) = delete;

    NavStateSubscriberShm(NavStateSubscriberShm&& other) noexcept;
    NavStateSubscriberShm& operator=(NavStateSubscriberShm&& other) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    /**
     * @brief Poll one NavState snapshot from shm.
     *
     * @param out_pub_mono_ns Optional: publisher mono timestamp stored in hdr.
     * @param out_pub_wall_ns Optional: publisher wall timestamp stored in hdr.
     * @return std::nullopt if disabled/not-ready/read-failed/ABI mismatch.
     */
    std::optional<shared::msg::NavState> poll(std::uint64_t* out_pub_mono_ns = nullptr,
                                             std::uint64_t* out_pub_wall_ns = nullptr);

    std::uint64_t last_pub_mono_ns() const noexcept { return last_pub_mono_ns_; }

private:
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;

    bool read_once(shared::msg::NavState& out,
                   std::uint64_t* out_pub_mono_ns,
                   std::uint64_t* out_pub_wall_ns) noexcept;

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

    // Will point to shared::shm::ShmLayout (from shared/shm/nav_state_shm.hpp).
    void* shm_ptr_ = nullptr;
#else
    void* shm_handle_ = nullptr;
    void* shm_ptr_    = nullptr;
#endif
};

} // namespace comm_gcs::ipc::nav

#endif // COMM_GCS_IPC_NAV_NAV_STATE_SUBSCRIBER_SHM_HPP
