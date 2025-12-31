#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "shared/shm/control_intent_shm.hpp"




namespace shared::msg {
struct ControlIntent; // forward decl; .cpp includes full header
} // namespace shared::msg

namespace comm_gcs {

/**
 * @brief Publish shared::msg::ControlIntent to shared memory (POSIX shm).
 *
 * Goals:
 *  - Writer is robust and low-latency (seqlock).
 *  - Payload is POD / trivially-copyable and written via memcpy.
 *  - No session/UDP/protocol logic here.
 */
class IntentPublisherShm final {
public:
    struct Config {
        bool        enable   = true;
        std::string shm_name = "/rovctrl_gcs_intent_v1";
        std::size_t shm_size = 0;  // 0 => sizeof(ShmLayout) in .cpp
    };



    IntentPublisherShm() = default;
    ~IntentPublisherShm() noexcept;

    IntentPublisherShm(const IntentPublisherShm&)            = delete;
    IntentPublisherShm& operator=(const IntentPublisherShm&) = delete;

    IntentPublisherShm(IntentPublisherShm&& other) noexcept;
    IntentPublisherShm& operator=(IntentPublisherShm&& other) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    bool publish(const shared::msg::ControlIntent& intent);

    // Debug helpers (read-only)
    const void*  debug_ptr()  const noexcept { return shm_ptr_; }
    std::size_t  debug_size() const noexcept { return shm_size_; }

private:
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;

#ifndef _WIN32
    // defined in .cpp:
    // struct ShmLayout { ShmHeader hdr; shared::msg::ControlIntent intent; };
    using ShmLayout = shared::shm::IntentShmLayout;

    ShmLayout*   shm_ptr_  = nullptr;
    std::size_t  shm_size_ = 0;
    int          shm_fd_   = -1;
#else
    // Windows not supported in this project currently
    void*        shm_handle_ = nullptr;
    void*        shm_ptr_    = nullptr;
    std::size_t  shm_size_   = 0;
#endif

    bool        enabled_     = false;
    bool        initialized_ = false;
    bool        error_flag_  = false;

    std::string shm_name_;
};

} // namespace comm_gcs
