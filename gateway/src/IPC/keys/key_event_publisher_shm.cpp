#include "gateway/IPC/keys/key_event_publisher_shm.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <utility>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace comm_gcs::ipc::keys {

namespace {

inline std::uint64_t now_mono_ns()
{
    using Clock = std::chrono::steady_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

inline std::uint64_t now_wall_ns()
{
    using Clock = std::chrono::system_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

inline bool valid_shm_name(const std::string& s)
{
    return !s.empty() && s.front() == '/';
}

} // namespace

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
KeyEventPublisherShm::~KeyEventPublisherShm() noexcept
{
    shutdown();
}

KeyEventPublisherShm::KeyEventPublisherShm(KeyEventPublisherShm&& other) noexcept
{
    *this = std::move(other);
}

KeyEventPublisherShm& KeyEventPublisherShm::operator=(KeyEventPublisherShm&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    cfg_ = std::move(other.cfg_);
    enabled_     = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_  = other.error_flag_;

#ifndef _WIN32
    shm_fd_      = other.shm_fd_;
    other.shm_fd_ = -1;
#else
    shm_handle_      = other.shm_handle_;
    other.shm_handle_ = nullptr;
#endif

    shm_size_    = other.shm_size_;
    shm_ptr_     = other.shm_ptr_;
    other.shm_ptr_  = nullptr;
    other.shm_size_ = 0;

    other.enabled_     = false;
    other.initialized_ = false;
    other.error_flag_  = false;
    other.cfg_         = Config{};

    return *this;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool KeyEventPublisherShm::init(const Config& cfg)
{
    shutdown();

    cfg_     = cfg;
    enabled_ = cfg.enable;

    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }

#ifdef _WIN32
    std::cerr << "[KeyEventPublisherShm] Shared memory not supported on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    if (!valid_shm_name(cfg_.shm_name)) {
        std::cerr << "[KeyEventPublisherShm] Invalid shm_name: " << cfg_.shm_name
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    if (cfg_.capacity == 0) {
        std::cerr << "[KeyEventPublisherShm] Invalid capacity=0\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    if (!open_or_create_shm_(cfg_)) {
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    initialized_ = true;
    error_flag_  = false;

    std::cerr << "[KeyEventPublisherShm] init ok"
              << " shm=" << cfg_.shm_name
              << " cap=" << cfg_.capacity
              << " size=" << shm_size_
              << " ptr=" << static_cast<void*>(shm_ptr_)
              << "\n";
    return true;
#endif
}

void KeyEventPublisherShm::shutdown() noexcept
{
    close_shm_();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;
    cfg_         = Config{};
}

std::uint64_t KeyEventPublisherShm::dropped() const noexcept
{
    if (!shm_ptr_) return 0;
    return shm_ptr_->hdr.drop_count;
}

bool KeyEventPublisherShm::publish(const shared::msg::KeyEvent& ev) noexcept
{
    return publish_many(&ev, 1) == 1;
}

std::size_t KeyEventPublisherShm::publish_many(const shared::msg::KeyEvent* evs, std::size_t n) noexcept
{
    if (!enabled_) return n;                 // disabled treated as "ok"
    if (!initialized_ || error_flag_) return 0;
    if (!shm_ptr_ || !evs || n == 0) return 0;

    auto& hdr = shm_ptr_->hdr;
    if (hdr.capacity == 0) {
        begin_write_();
        hdr.drop_count += n;
        end_write_();
        return 0;
    }

    begin_write_();

    hdr.mono_ns = now_mono_ns();
    hdr.wall_ns = now_wall_ns();

    auto* ring = shared::shm::key_event_ring_ptr(&hdr);
    const std::uint32_t cap = hdr.capacity;
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint64_t idx  = hdr.write_idx;
        const std::uint32_t slot = static_cast<std::uint32_t>(idx % cap);

        shared::msg::KeyEvent tmp = evs[i];
        // A 路线：防御性补齐（只在缺省时补）
        if (tmp.version == 0) tmp.version = shared::msg::kKeyEventWireVersion;
        if (tmp.stamp_mono_ns == 0) tmp.stamp_mono_ns = hdr.mono_ns;

        std::memcpy(&ring[slot], &tmp, sizeof(tmp));
        hdr.write_idx = idx + 1;
    }

    end_write_();
    return n;
}

// -----------------------------------------------------------------------------
// Internal: seqlock begin/end
// -----------------------------------------------------------------------------
void KeyEventPublisherShm::begin_write_() noexcept
{
    auto& hdr = shm_ptr_->hdr;
    const std::uint64_t s0 = hdr.seqlock.load(std::memory_order_relaxed);
    hdr.seqlock.store(s0 + 1, std::memory_order_release); // odd
}

void KeyEventPublisherShm::end_write_() noexcept
{
    auto& hdr = shm_ptr_->hdr;
    const std::uint64_t s1 = hdr.seqlock.load(std::memory_order_relaxed);
    // If called correctly, s1 should be odd. Force next even.
    hdr.seqlock.store((s1 | 1ULL) + 1ULL, std::memory_order_release);
}

// -----------------------------------------------------------------------------
// Internal: open/create + init header
// -----------------------------------------------------------------------------
bool KeyEventPublisherShm::open_or_create_shm_(const Config& cfg)
{
#ifdef _WIN32
    (void)cfg;
    return false;
#else
    if (cfg.truncate) {
        // Best-effort unlink before create; ignore ENOENT.
        ::shm_unlink(cfg.shm_name.c_str());
    }

    const std::size_t want_size = shared::shm::key_event_ring_bytes(cfg.capacity);
    shm_size_ = want_size;

    shm_fd_ = ::shm_open(cfg.shm_name.c_str(), O_CREAT | O_RDWR, static_cast<mode_t>(cfg.create_mode));
    if (shm_fd_ < 0) {
        std::perror("[KeyEventPublisherShm] shm_open failed");
        return false;
    }

    if (::ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
        std::perror("[KeyEventPublisherShm] ftruncate failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[KeyEventPublisherShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_ptr_ = reinterpret_cast<shared::shm::KeyEventRingShmLayout*>(addr);

    // Initialize / validate header and reset deterministically on mismatch.
    auto& hdr = shm_ptr_->hdr;

    const bool mismatch =
        (hdr.magic != shared::shm::kKeyEventRingMagic) ||
        (hdr.layout_ver != shared::shm::kKeyEventRingLayoutVersion) ||
        (hdr.payload_ver != shared::shm::kKeyEventPayloadVersion) ||
        (hdr.payload_size != sizeof(shared::shm::Payload)) ||
        (hdr.payload_align != alignof(shared::shm::Payload)) ||
        (hdr.capacity != cfg.capacity);

    if (mismatch) {
        // seqlock: mark write in progress
        hdr.seqlock.store(1, std::memory_order_relaxed);

        // Reset ring header fields
        hdr.mono_ns = now_mono_ns();
        hdr.wall_ns = now_wall_ns();

        hdr.magic      = shared::shm::kKeyEventRingMagic;
        hdr.layout_ver = shared::shm::kKeyEventRingLayoutVersion;

        hdr.payload_ver   = shared::shm::kKeyEventPayloadVersion;
        hdr.payload_size  = static_cast<std::uint32_t>(sizeof(shared::shm::Payload));
        hdr.payload_align = static_cast<std::uint32_t>(alignof(shared::shm::Payload));

        hdr.capacity   = cfg.capacity;
        hdr.reserved0  = 0;

        hdr.write_idx  = 0;
        hdr.drop_count = 0;
        hdr.reserved1  = 0;

        // Clear ring memory to deterministic zero
        auto* ring = shared::shm::key_event_ring_ptr(&hdr);
        // 原来：memset(ring, 0, capacity * sizeof(Payload));
        for (std::uint32_t i = 0; i < hdr.capacity; ++i) {
            ring[i] = shared::shm::Payload{};  // 或 shared::msg::KeyEvent{}，看 typedef
        }

        std::atomic_thread_fence(std::memory_order_release);
        hdr.seqlock.store(2, std::memory_order_release); // stable even

        std::cerr << "[KeyEventPublisherShm] layout mismatch -> reset"
                  << " shm=" << cfg.shm_name
                  << " cap=" << cfg.capacity
                  << "\n";
    }

    return true;
#endif
}

// -----------------------------------------------------------------------------
// Internal: close shm
// -----------------------------------------------------------------------------
void KeyEventPublisherShm::close_shm_() noexcept
{
#ifndef _WIN32
    if (shm_ptr_) {
        ::munmap(reinterpret_cast<void*>(shm_ptr_), shm_size_);
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

} // namespace comm_gcs::ipc::keys
