#pragma once
#ifndef COMM_GCS_IPC_NAV_NAV_VIEW_PUBLISHER_SHM_HPP
#define COMM_GCS_IPC_NAV_NAV_VIEW_PUBLISHER_SHM_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

namespace shared::msg {
struct NavStateView;
} // namespace shared::msg

namespace comm_gcs::ipc::nav {

/**
 * @brief Publish shared::msg::NavStateView to POSIX shared memory (seqlock).
 *
 * Contract:
 *   - Layout and ABI contract is defined in shared/shm/nav_state_view_shm.hpp
 *   - Writer initializes hdr.magic/layout_ver/payload_* and keeps them stable.
 *   - publish() updates timestamps and payload under seqlock.
 */
class NavViewPublisherShm final {
public:
    struct Config {
        bool        enable   = true;
        std::string shm_name = "/rovctrl_nav_view_v1";
        std::size_t shm_size = 0;  // 0 => sizeof(shared::shm::ShmLayout)
    };

    NavViewPublisherShm() = default;
    ~NavViewPublisherShm() noexcept;

    NavViewPublisherShm(const NavViewPublisherShm&)            = delete;
    NavViewPublisherShm& operator=(const NavViewPublisherShm&) = delete;

    NavViewPublisherShm(NavViewPublisherShm&& other) noexcept;
    NavViewPublisherShm& operator=(NavViewPublisherShm&& other) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    bool publish(const shared::msg::NavStateView& view);

    const void* debug_ptr()  const noexcept { return shm_ptr_; }
    std::size_t debug_size() const noexcept { return shm_size_; }

private:
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;

#ifndef _WIN32
    void*        shm_ptr_  = nullptr;
    std::size_t  shm_size_ = 0;
    int          shm_fd_   = -1;
#else
    void*        shm_handle_ = nullptr;
    void*        shm_ptr_    = nullptr;
    std::size_t  shm_size_   = 0;
#endif

    bool        enabled_     = false;
    bool        initialized_ = false;
    bool        error_flag_  = false;

    std::string shm_name_;
};

} // namespace comm_gcs::ipc::nav

#endif // COMM_GCS_IPC_NAV_NAV_VIEW_PUBLISHER_SHM_HPP
