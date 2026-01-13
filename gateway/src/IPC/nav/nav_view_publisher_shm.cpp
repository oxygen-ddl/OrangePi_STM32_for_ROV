#include "gateway/IPC/nav/nav_view_publisher_shm.hpp"

#include "shared/msg/nav_state_view.hpp"
#include "shared/shm/nav_state_view_shm.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <utility>
#include <cstdio>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace comm_gcs::ipc::nav {

namespace {

using ShmLayout = shared::shm::ShmLayout;

inline std::uint64_t now_mono_ns()
{
    using Clock = std::chrono::steady_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count());
}

inline std::uint64_t now_wall_ns()
{
    using Clock = std::chrono::system_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count());
}

inline bool header_matches_contract(const shared::shm::ShmHeader& h) noexcept
{
    if (h.magic != shared::shm::kNavViewMagic) return false;
    if (h.layout_ver != shared::shm::kNavViewLayoutVersion) return false;

    if (h.payload_ver != shared::msg::kNavStateViewWireVersion) return false;
    if (h.payload_size != static_cast<std::uint32_t>(sizeof(shared::shm::Payload))) return false;
    if (h.payload_align != static_cast<std::uint32_t>(alignof(shared::shm::Payload))) return false;

    return true;
}

} // namespace

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
NavViewPublisherShm::~NavViewPublisherShm() noexcept
{
    shutdown();
}

NavViewPublisherShm::NavViewPublisherShm(NavViewPublisherShm&& other) noexcept
{
    *this = std::move(other);
}

NavViewPublisherShm&
NavViewPublisherShm::operator=(NavViewPublisherShm&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    enabled_     = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_  = other.error_flag_;
    shm_name_    = std::move(other.shm_name_);
    shm_size_    = other.shm_size_;

#ifndef _WIN32
    shm_fd_        = other.shm_fd_;
    shm_ptr_       = other.shm_ptr_;
    other.shm_fd_  = -1;
    other.shm_ptr_ = nullptr;
#else
    shm_handle_       = other.shm_handle_;
    shm_ptr_          = other.shm_ptr_;
    other.shm_handle_ = nullptr;
    other.shm_ptr_    = nullptr;
#endif

    other.enabled_     = false;
    other.initialized_ = false;
    other.error_flag_  = false;
    other.shm_size_    = 0;

    return *this;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool NavViewPublisherShm::init(const Config& cfg)
{
    // If re-init, ensure clean slate
    shutdown();

    enabled_ = cfg.enable;
    std::cerr << "[NavViewPublisherShm] init(): enable=" << cfg.enable
              << " shm_name=" << (cfg.shm_name.empty() ? "<empty>" : cfg.shm_name)
              << " shm_size=" << cfg.shm_size << "\n";

    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }
    return init_shm(cfg);
}

void NavViewPublisherShm::shutdown() noexcept
{
    close_shm();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;
    shm_name_.clear();
    shm_size_ = 0;
}

bool NavViewPublisherShm::publish(const shared::msg::NavStateView& view)
{
    if (!enabled_) return true;
    if (!initialized_ || error_flag_ || !shm_ptr_) return false;

    auto* layout = static_cast<ShmLayout*>(shm_ptr_);
    auto& hdr    = layout->hdr;

    // begin write: make it odd
    const std::uint64_t s0 = hdr.seq.load(std::memory_order_relaxed);
    hdr.seq.store(s0 + 1, std::memory_order_release);

    hdr.mono_ns = now_mono_ns();
    hdr.wall_ns = now_wall_ns();
    // magic/layout_ver/payload_* are set at init and should stay stable.

    std::memcpy(&layout->payload, &view, sizeof(view));

    // end write: even
    hdr.seq.store(s0 + 2, std::memory_order_release);
    return true;
}

// -----------------------------------------------------------------------------
// Shared memory init
// -----------------------------------------------------------------------------
bool NavViewPublisherShm::init_shm(const Config& cfg)
{
#ifdef _WIN32
    std::cerr << "[NavViewPublisherShm] Shared memory not supported on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rovctrl_nav_view_v1" : cfg.shm_name;

    std::cerr << "[NavViewPublisherShm] init_shm(): shm_name=" << shm_name_
              << " cfg.shm_size=" << cfg.shm_size
              << " min_size=" << sizeof(ShmLayout) << "\n";

    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[NavViewPublisherShm] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    const std::size_t min_size = sizeof(ShmLayout);
    shm_size_ = (cfg.shm_size == 0 || cfg.shm_size < min_size) ? min_size : cfg.shm_size;

    shm_fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::perror("[NavViewPublisherShm] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }
    std::cerr << "[NavViewPublisherShm] shm_open ok: fd=" << shm_fd_ << "\n";

    if (::ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
        std::perror("[NavViewPublisherShm] ftruncate failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[NavViewPublisherShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = addr;
    std::cerr << "[NavViewPublisherShm] mmap ok: ptr=" << shm_ptr_
              << " size=" << shm_size_ << "\n";

    // Contract mismatch -> reset deterministically (with seqlock semantics)
    auto* layout = static_cast<ShmLayout*>(shm_ptr_);
    const bool mismatch = !header_matches_contract(layout->hdr);

    if (mismatch) {
        layout->hdr.seq.store(1, std::memory_order_relaxed);

        // reset payload
        layout->payload = shared::msg::NavStateView{};

        // reset header contract tags
        layout->hdr.magic       = shared::shm::kNavViewMagic;
        layout->hdr.layout_ver  = shared::shm::kNavViewLayoutVersion;
        layout->hdr.payload_ver = shared::msg::kNavStateViewWireVersion;
        layout->hdr.payload_size  = static_cast<std::uint32_t>(sizeof(shared::shm::Payload));
        layout->hdr.payload_align = static_cast<std::uint32_t>(alignof(shared::shm::Payload));

        layout->hdr.mono_ns = now_mono_ns();
        layout->hdr.wall_ns = now_wall_ns();

        std::atomic_thread_fence(std::memory_order_release);
        layout->hdr.seq.store(2, std::memory_order_release);

        std::cerr << "[NavViewPublisherShm] layout mismatch -> reset\n";
    }

    initialized_ = true;
    error_flag_  = false;
    return true;
#endif
}

// -----------------------------------------------------------------------------
// Close shm
// -----------------------------------------------------------------------------
void NavViewPublisherShm::close_shm() noexcept
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
#else
    shm_ptr_    = nullptr;
    shm_handle_ = nullptr;
#endif
}

} // namespace comm_gcs::ipc::nav
