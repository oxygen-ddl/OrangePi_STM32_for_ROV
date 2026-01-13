#include "gateway/IPC/intent/intent_subscriber_shm.hpp"

#include "shared/msg/control_intent.hpp"
#include "shared/shm/control_intent_shm.hpp"

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

namespace comm_gcs::ipc::intent {

namespace {

using WireLayout = shared::shm::IntentShmLayout;
// Limit seqlock retry count to avoid busy loop in worst-case.
constexpr int kMaxSeqLockRetries = 8;

inline bool check_header_contract(const decltype(WireLayout::hdr)& h) noexcept
{
    if (h.magic != shared::shm::kIntentShmMagic) return false;
    if (h.layout_ver != shared::shm::kIntentShmLayoutVersion) return false;

    if (h.payload_ver != shared::msg::kControlIntentWireVersion) return false;
    if (h.payload_size != static_cast<std::uint32_t>(sizeof(shared::msg::ControlIntent))) return false;
    if (h.payload_align != static_cast<std::uint32_t>(alignof(shared::msg::ControlIntent))) return false;

    return true;
}

} // namespace


// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
GcsIntentSubscriberShm::~GcsIntentSubscriberShm() noexcept
{
    shutdown();
}

GcsIntentSubscriberShm::GcsIntentSubscriberShm(GcsIntentSubscriberShm&& other) noexcept
{
    *this = std::move(other);
}

GcsIntentSubscriberShm&
GcsIntentSubscriberShm::operator=(GcsIntentSubscriberShm&& other) noexcept
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
bool GcsIntentSubscriberShm::init(const Config& cfg)
{
    cfg_     = cfg;
    enabled_ = cfg.enable;

    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }

    // lazy init: allow comm_gcs to start before publisher creates shm.
    if (cfg_.lazy_init) {
        initialized_ = false;
        error_flag_  = false;
        shm_name_    = cfg_.shm_name;
        shm_size_    = 0;
        return true;
    }

    return init_shm(cfg_);
}

void GcsIntentSubscriberShm::shutdown() noexcept
{
    close_shm();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;
    last_pub_mono_ns_ = 0;
}

std::optional<shared::msg::ControlIntent>
GcsIntentSubscriberShm::poll_wire(std::uint64_t* out_mono_ns,
                                  std::uint64_t* out_wall_ns)
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

    shared::msg::ControlIntent wire{};
    std::uint64_t mono_ns = 0;
    std::uint64_t wall_ns = 0;

    if (!read_once(wire, &mono_ns, &wall_ns)) {
        return std::nullopt;
    }

    if (out_mono_ns) *out_mono_ns = mono_ns;
    if (out_wall_ns) *out_wall_ns = wall_ns;

    last_pub_mono_ns_ = mono_ns;
    return wire;
}

// -----------------------------------------------------------------------------
// Internal: seqlock read (matches shared::shm::IntentShmLayout contract)
// -----------------------------------------------------------------------------
bool GcsIntentSubscriberShm::read_once(shared::msg::ControlIntent& out,
                                      std::uint64_t* out_mono_ns,
                                      std::uint64_t* out_wall_ns) noexcept
{
    if (!shm_ptr_) return false;

    const auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    const auto& hdr    = layout->hdr;

    for (int i = 0; i < kMaxSeqLockRetries; ++i) {
        const std::uint64_t s1 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 & 1u) {
            // writer in progress
            continue;
        }

        const std::uint64_t mono_ns = hdr.mono_ns;
        const std::uint64_t wall_ns = hdr.wall_ns;

        // hard reject if contract/ABI mismatch
        if (!check_header_contract(hdr)) {
            return false;
        }

        // payload snapshot
        std::memcpy(&out, &layout->intent, sizeof(out));

        const std::uint64_t s2 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 == s2 && ((s2 & 1u) == 0)) {
            if (out_mono_ns) *out_mono_ns = mono_ns;
            if (out_wall_ns) *out_wall_ns = wall_ns;
            return true;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// Internal: shm init/close
// -----------------------------------------------------------------------------
bool GcsIntentSubscriberShm::init_shm(const Config& cfg)
{
#ifdef _WIN32
    std::cerr << "[GcsIntentSubscriberShm] Shared memory is not supported on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rovctrl_gcs_intent_v1" : cfg.shm_name;

    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[GcsIntentSubscriberShm] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // Open existing shm. Do NOT create here (publisher owns creation).
    shm_fd_ = ::shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ < 0) {
        // In lazy mode, ENOENT is expected if publisher not started yet.
        if (cfg.lazy_init && (errno == ENOENT)) {
            initialized_ = false;
            error_flag_  = false;
            return false;
        }
        std::perror("[GcsIntentSubscriberShm] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // Determine shm size via fstat() to match publisher mapping.
    struct ::stat st {};
    if (::fstat(shm_fd_, &st) != 0) {
        std::perror("[GcsIntentSubscriberShm] fstat failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    const std::size_t min_size  = sizeof(ShmLayout);
    const std::size_t real_size = static_cast<std::size_t>(st.st_size);
    if (real_size < min_size) {
        std::cerr << "[GcsIntentSubscriberShm] shm size too small: " << real_size
                  << " < " << min_size << "\n";
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // Optional clamp if user provided shm_size
    if (cfg.shm_size != 0) {
        shm_size_ = (cfg.shm_size <= real_size) ? cfg.shm_size : real_size;
        if (shm_size_ < min_size) shm_size_ = min_size;
    } else {
        shm_size_ = real_size;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[GcsIntentSubscriberShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);

    // Sanity check: contract must match; in lazy mode allow retry later.
    const auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    if (!check_header_contract(layout->hdr)) {
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

void GcsIntentSubscriberShm::close_shm() noexcept
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

} // namespace comm_gcs::ipc::intent
