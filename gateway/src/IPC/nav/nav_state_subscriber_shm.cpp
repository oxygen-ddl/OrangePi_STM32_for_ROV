#include "gateway/IPC/nav/nav_state_subscriber_shm.hpp"

#include "shared/msg/nav_state.hpp"
#include "shared/shm/nav_state_shm.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace comm_gcs::ipc::nav {

namespace {

using ShmLayout = shared::shm::ShmLayout;

// Limit seqlock retry count to avoid busy loop in worst-case.
constexpr int kMaxSeqLockRetries = 8;

inline bool check_header_contract(const shared::shm::ShmHeader& h) noexcept
{
    if (h.magic != shared::shm::kNavStateMagic) return false;
    if (h.layout_ver != shared::shm::kNavStateLayoutVersion) return false;

    // ABI pinning for payload
    if (h.payload_ver != shared::shm::kNavStatePayloadVersion) return false;
    if (h.payload_size != static_cast<std::uint32_t>(sizeof(shared::shm::Payload))) return false;
    if (h.payload_align != static_cast<std::uint32_t>(alignof(shared::shm::Payload))) return false;

    return true;
}

} // namespace

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
NavStateSubscriberShm::~NavStateSubscriberShm() noexcept
{
    shutdown();
}

NavStateSubscriberShm::NavStateSubscriberShm(NavStateSubscriberShm&& other) noexcept
{
    *this = std::move(other);
}

NavStateSubscriberShm&
NavStateSubscriberShm::operator=(NavStateSubscriberShm&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    enabled_     = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_  = other.error_flag_;
    cfg_         = other.cfg_;

    shm_name_    = std::move(other.shm_name_);
    shm_size_    = other.shm_size_;
    last_pub_mono_ns_ = other.last_pub_mono_ns_;

#ifndef _WIN32
    shm_fd_        = other.shm_fd_;
    shm_ptr_       = other.shm_ptr_;
    other.shm_fd_  = -1;
    other.shm_ptr_ = nullptr;
#else
    shm_handle_        = other.shm_handle_;
    shm_ptr_           = other.shm_ptr_;
    other.shm_handle_  = nullptr;
    other.shm_ptr_     = nullptr;
#endif

    other.enabled_     = false;
    other.initialized_ = false;
    other.error_flag_  = false;
    other.shm_size_    = 0;
    other.last_pub_mono_ns_ = 0;

    return *this;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool NavStateSubscriberShm::init(const Config& cfg)
{
    cfg_     = cfg;
    enabled_ = cfg.enable;

    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }

    // lazy init: allow comm_gcs to start before nav process creates shm.
    if (cfg_.lazy_init) {
        initialized_ = false;
        error_flag_  = false;
        shm_name_    = cfg_.shm_name;
        shm_size_    = 0;
        return true;
    }

    return init_shm(cfg_);
}

void NavStateSubscriberShm::shutdown() noexcept
{
    close_shm();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;
    last_pub_mono_ns_ = 0;
}

std::optional<shared::msg::NavState>
NavStateSubscriberShm::poll(std::uint64_t* out_pub_mono_ns,
                            std::uint64_t* out_pub_wall_ns)
{
    if (!enabled_) return std::nullopt;

    // Attempt lazy init if needed.
    if (!initialized_) {
        if (!cfg_.lazy_init) return std::nullopt;

        // Try to init; if shm not ready, do not latch error.
        if (!init_shm(cfg_)) {
            return std::nullopt;
        }
    }

    if (error_flag_ || !shm_ptr_) return std::nullopt;

    shared::msg::NavState out{};
    std::uint64_t pub_mono_ns = 0;
    std::uint64_t pub_wall_ns = 0;

    if (!read_once(out, &pub_mono_ns, &pub_wall_ns)) {
        return std::nullopt;
    }

    if (out_pub_mono_ns) *out_pub_mono_ns = pub_mono_ns;
    if (out_pub_wall_ns) *out_pub_wall_ns = pub_wall_ns;

    last_pub_mono_ns_ = pub_mono_ns;
    return out;
}

// -----------------------------------------------------------------------------
// Internal: seqlock read
// -----------------------------------------------------------------------------
bool NavStateSubscriberShm::read_once(shared::msg::NavState& out,
                                     std::uint64_t* out_pub_mono_ns,
                                     std::uint64_t* out_pub_wall_ns) noexcept
{
    if (!shm_ptr_) return false;

    auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    const auto& hdr = layout->hdr;

    for (int i = 0; i < kMaxSeqLockRetries; ++i) {
        const std::uint64_t s1 = hdr.seq.load(std::memory_order_acquire);
        if (s1 & 1u) {
            // writer in progress
            continue;
        }

        const std::uint64_t pub_mono_ns = hdr.mono_ns;
        const std::uint64_t pub_wall_ns = hdr.wall_ns;

        // contract / ABI check
        if (!check_header_contract(hdr)) {
            return false;
        }

        // payload copy
        std::memcpy(&out, &layout->payload, sizeof(out));

        const std::uint64_t s2 = hdr.seq.load(std::memory_order_acquire);
        if (s1 == s2 && ((s2 & 1u) == 0)) {
            if (out_pub_mono_ns) *out_pub_mono_ns = pub_mono_ns;
            if (out_pub_wall_ns) *out_pub_wall_ns = pub_wall_ns;
            return true;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// Internal: shm init/close
// -----------------------------------------------------------------------------
bool NavStateSubscriberShm::init_shm(const Config& cfg)
{
#ifdef _WIN32
    std::cerr << "[NavStateSubscriberShm] Shared memory is not supported on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rovctrl_nav_state_v1" : cfg.shm_name;

    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[NavStateSubscriberShm] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // Open existing shm. Do NOT create here (publisher owns creation).
    shm_fd_ = ::shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ < 0) {
        // In lazy mode, ENOENT is expected if nav not started yet.
        if (cfg.lazy_init && (errno == ENOENT)) {
            initialized_ = false;
            error_flag_  = false;
            return false;
        }
        std::perror("[NavStateSubscriberShm] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // Determine shm size via fstat() (match publisher's actual mapping size).
    struct ::stat st {};
    if (::fstat(shm_fd_, &st) != 0) {
        std::perror("[NavStateSubscriberShm] fstat failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    const std::size_t min_size = sizeof(ShmLayout);
    const std::size_t real_size = static_cast<std::size_t>(st.st_size);
    if (real_size < min_size) {
        std::cerr << "[NavStateSubscriberShm] shm size too small: " << real_size
                  << " < " << min_size << "\n";
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // If user provided shm_size, clamp to real size (optional).
    if (cfg.shm_size != 0) {
        shm_size_ = (cfg.shm_size <= real_size) ? cfg.shm_size : real_size;
        if (shm_size_ < min_size) shm_size_ = min_size;
    } else {
        shm_size_ = real_size;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[NavStateSubscriberShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = addr;

    // Basic sanity check: header contract must match (non-fatal in lazy mode).
    const auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    if (!check_header_contract(layout->hdr)) {
        // allow retry later (publisher may still be initializing)
        close_shm();
        initialized_ = false;
        error_flag_  = false;
        return false;
    }

    initialized_ = true;
    error_flag_  = false;
    return true;
#endif
}

void NavStateSubscriberShm::close_shm() noexcept
{
#ifndef _WIN32
    if (shm_ptr_) {
        ::munmap(shm_ptr_, shm_size_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    shm_size_ = 0;
#else
    shm_ptr_    = nullptr;
    shm_handle_ = nullptr;
    shm_size_   = 0;
#endif
}

} // namespace comm_gcs::ipc::nav
