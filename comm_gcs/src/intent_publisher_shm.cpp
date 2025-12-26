#include <comm_gcs/intent_publisher_shm.hpp>
#include "shared/msg/control_intent.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace comm_gcs {

namespace {

// 'I''N''T''1'
constexpr std::uint32_t INTENT_MAGIC =
    (static_cast<std::uint32_t>('I') << 24) |
    (static_cast<std::uint32_t>('N') << 16) |
    (static_cast<std::uint32_t>('T') << 8)  |
    (static_cast<std::uint32_t>('1'));

constexpr std::uint32_t INTENT_VERSION = 1;

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

} // namespace

// -----------------------------------------------------------------------------
// Shared memory layout
// -----------------------------------------------------------------------------
struct IntentPublisherShm::ShmLayout {
    ShmHeader                   hdr;
    shared::msg::ControlIntent  intent;
};

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
IntentPublisherShm::~IntentPublisherShm() noexcept
{
    shutdown();
}

IntentPublisherShm::IntentPublisherShm(IntentPublisherShm&& other) noexcept
{
    *this = std::move(other);
}

IntentPublisherShm&
IntentPublisherShm::operator=(IntentPublisherShm&& other) noexcept
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
    shm_handle_        = other.shm_handle_;
    shm_ptr_           = other.shm_ptr_;
    other.shm_handle_  = nullptr;
    other.shm_ptr_     = nullptr;
#endif

    other.enabled_     = false;
    other.initialized_ = false;
    other.error_flag_  = false;
    return *this;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool IntentPublisherShm::init(const Config& cfg)
{
    enabled_ = cfg.enable;
    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }
    return init_shm(cfg);
}

void IntentPublisherShm::shutdown() noexcept
{
    close_shm();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;
}

bool IntentPublisherShm::publish(const shared::msg::ControlIntent& intent)
{
    if (!enabled_) return true;
    if (!initialized_ || error_flag_ || !shm_ptr_) return false;

    ShmLayout* layout = shm_ptr_;
    auto& hdr = layout->hdr;
    auto& seq = hdr.seq;

    // seqlock begin (odd)
    const std::uint64_t old   = seq.load(std::memory_order_relaxed);
    const std::uint64_t start = (old & 1u) ? (old + 2u) : (old + 1u);
    seq.store(start, std::memory_order_relaxed);

    // header
    hdr.mono_ns = now_mono_ns();
    hdr.wall_ns = now_wall_ns();
    hdr.magic   = INTENT_MAGIC;
    hdr.version = INTENT_VERSION;

    // payload (POD)
    std::memcpy(&layout->intent, &intent, sizeof(shared::msg::ControlIntent));

    // seqlock end (even)
    seq.store(start + 1u, std::memory_order_release);
    return true;
}

// -----------------------------------------------------------------------------
// Shared memory init
// -----------------------------------------------------------------------------
bool IntentPublisherShm::init_shm(const Config& cfg)
{
#ifdef _WIN32
    std::cerr << "[IntentPublisherShm] Shared memory not supported on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rovctrl_gcs_intent_v1" : cfg.shm_name;

    if (shm_name_.front() != '/') {
        std::cerr << "[IntentPublisherShm] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    const std::size_t min_size = sizeof(ShmLayout);
    shm_size_ = (cfg.shm_size == 0 || cfg.shm_size < min_size)
                  ? min_size
                  : cfg.shm_size;

    shm_fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::perror("[IntentPublisherShm] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    if (::ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
        std::perror("[IntentPublisherShm] ftruncate failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    void* addr = ::mmap(nullptr, shm_size_,
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[IntentPublisherShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);

    // protocol mismatch -> reset payload
    if (shm_ptr_->hdr.magic != INTENT_MAGIC ||
        shm_ptr_->hdr.version != INTENT_VERSION)
    {
        shm_ptr_->intent.version = shared::msg::kControlIntentWireVersion;
        shm_ptr_->intent.flags   = 0;
        shm_ptr_->hdr.magic   = INTENT_MAGIC;
        shm_ptr_->hdr.version = INTENT_VERSION;
        shm_ptr_->hdr.mono_ns = now_mono_ns();
        shm_ptr_->hdr.wall_ns = now_wall_ns();
        shm_ptr_->hdr.seq.store(0, std::memory_order_relaxed);
    }

    initialized_ = true;
    error_flag_  = false;
    return true;
#endif
}

// -----------------------------------------------------------------------------
// Close shm
// -----------------------------------------------------------------------------
void IntentPublisherShm::close_shm() noexcept
{
#ifndef _WIN32
    if (shm_ptr_) {
        ::munmap(static_cast<void*>(shm_ptr_), shm_size_);
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

} // namespace comm_gcs
