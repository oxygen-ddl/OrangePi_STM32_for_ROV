#pragma once
#ifndef ROVCTRL_IO_NAV_NAV_VIEW_SUBSCRIBER_SHM_HPP
#define ROVCTRL_IO_NAV_NAV_VIEW_SUBSCRIBER_SHM_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "shared/msg/nav_state_view.hpp"
#include "shared/shm/nav_state_view_shm.hpp"  // provides shared::shm::ShmLayout + hdr contract

namespace rovctrl::io::nav {

/**
 * @brief Subscribe NavStateView from POSIX shared memory (seqlock style).
 *
 * Responsibilities:
 *  - SHM open/mmap lifecycle (RO)
 *  - ABI contract verification (magic/layout/payload)
 *  - Seqlock consistent snapshot read
 *  - Optional repeat suppression (based on seq)
 *
 * Non-responsibilities:
 *  - max-age / validity policy (handled by NavViewShmSource or ControlLoop)
 */
class NavViewSubscriberShm final {
public:
    struct Config {
        bool        enable       = true;
        std::string shm_name     = "/rovctrl_nav_view_v1";
        std::size_t shm_size     = 0;     // 0 => sizeof(shared::shm::ShmLayout)
        bool        lazy_init    = true;  // allow start before publisher exists
        bool        allow_repeat = false; // if false: suppress same seq frame
    };

    NavViewSubscriberShm() = default;
    ~NavViewSubscriberShm() noexcept;

    NavViewSubscriberShm(const NavViewSubscriberShm&)            = delete;
    NavViewSubscriberShm& operator=(const NavViewSubscriberShm&) = delete;

    NavViewSubscriberShm(NavViewSubscriberShm&& o) noexcept;
    NavViewSubscriberShm& operator=(NavViewSubscriberShm&& o) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    /**
     * @brief Poll one consistent snapshot from shm.
     *
     * @param out_pub_mono_ns optional publisher monotonic timestamp (from hdr)
     * @param out_pub_wall_ns optional publisher wall timestamp (from hdr)
     * @return latest NavStateView snapshot; std::nullopt if not ready/unchanged/fail
     */
    std::optional<shared::msg::NavStateView> poll_wire(std::uint64_t* out_pub_mono_ns = nullptr,
                                                       std::uint64_t* out_pub_wall_ns = nullptr) noexcept;

private:
    bool try_open_() noexcept;
    bool abi_check_() noexcept;
    void close_() noexcept;

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

    void*       map_{nullptr};
    std::size_t map_size_{0};

    // NOTE: nav_state_view_shm.hpp defines this as shared::shm::ShmLayout
    shared::shm::ShmLayout* shm_{nullptr};

    std::uint64_t last_seq_{0};
};

} // namespace rovctrl::io::nav

#endif // ROVCTRL_IO_NAV_NAV_VIEW_SUBSCRIBER_SHM_HPP
