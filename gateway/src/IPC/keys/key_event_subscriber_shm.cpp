#include "gateway/IPC/keys/key_event_subscriber_shm.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <utility>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace comm_gcs::ipc::keys {

namespace {

// seqlock stable read: read seq before/after and ensure even + unchanged
inline bool seqlock_read_begin(const std::atomic<std::uint64_t>& seq, std::uint64_t& out) noexcept
{
    out = seq.load(std::memory_order_acquire);
    return (out % 2u) == 0u;
}

inline bool seqlock_read_end(const std::atomic<std::uint64_t>& seq, std::uint64_t begin) noexcept
{
    const std::uint64_t end = seq.load(std::memory_order_acquire);
    return (begin == end) && ((end % 2u) == 0u);
}

} // namespace

KeyEventSubscriberShm::~KeyEventSubscriberShm() noexcept
{
    shutdown();
}

KeyEventSubscriberShm::KeyEventSubscriberShm(KeyEventSubscriberShm&& other) noexcept
{
    *this = std::move(other);
}

KeyEventSubscriberShm& KeyEventSubscriberShm::operator=(KeyEventSubscriberShm&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    cfg_ = std::move(other.cfg_);
    enabled_ = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_ = other.error_flag_;

#ifndef _WIN32
    shm_fd_ = other.shm_fd_;
    other.shm_fd_ = -1;
#endif

    shm_size_ = other.shm_size_;
    shm_ptr_  = other.shm_ptr_;
    other.shm_size_ = 0;
    other.shm_ptr_ = nullptr;

    last_read_idx_ = other.last_read_idx_;
    capacity_ = other.capacity_;

    stats_ = other.stats_;

    other.enabled_ = false;
    other.initialized_ = false;
    other.error_flag_ = false;
    other.last_read_idx_ = 0;
    other.capacity_ = 0;
    other.stats_ = Stats{};

    return *this;
}

bool KeyEventSubscriberShm::init(const Config& cfg)
{
    shutdown();

    cfg_ = cfg;
    enabled_ = cfg_.enable;

    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }

#ifdef _WIN32
    std::cerr << "[KeyEventSubscriberShm] Shared memory not supported on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    // In lazy_init mode, we don't fail hard if shm isn't present yet.
    if (cfg_.lazy_init) {
        initialized_ = false;
        error_flag_ = false;
        return true;
    }

    if (!try_open_map_ro_()) {
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    // Initialize cursor to latest to avoid stale events on startup.
    reset_cursor(true);
    initialized_ = true;
    error_flag_ = false;
    return true;
#endif
}

void KeyEventSubscriberShm::shutdown() noexcept
{
#ifndef _WIN32
    close_shm_();
#endif
    enabled_ = false;
    initialized_ = false;
    error_flag_ = false;
    cfg_ = Config{};
    last_read_idx_ = 0;
    capacity_ = 0;
    stats_ = Stats{};
}

#ifndef _WIN32
bool KeyEventSubscriberShm::try_open_map_ro_() noexcept
{
    if (shm_ptr_) return true;

    if (cfg_.shm_name.empty() || cfg_.shm_name.front() != '/') {
        stats_.open_fail++;
        std::cerr << "[KeyEventSubscriberShm] invalid shm_name=" << cfg_.shm_name << "\n";
        return false;
    }

    shm_fd_ = ::shm_open(cfg_.shm_name.c_str(), O_RDONLY, 0);
    if (shm_fd_ < 0) {
        stats_.open_fail++;
        return false;
    }

    struct ::stat st {};
    if (::fstat(shm_fd_, &st) != 0) {
        stats_.open_fail++;
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_size_ = static_cast<std::size_t>(st.st_size);
    if (shm_size_ < sizeof(shared::shm::KeyEventRingShmHeader)) {
        stats_.open_fail++;
        ::close(shm_fd_);
        shm_fd_ = -1;
        shm_size_ = 0;
        return false;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        stats_.open_fail++;
        ::close(shm_fd_);
        shm_fd_ = -1;
        shm_size_ = 0;
        return false;
    }

    shm_ptr_ = reinterpret_cast<const shared::shm::KeyEventRingShmLayout*>(addr);

    // Validate ABI using a stable header snapshot.
    shared::shm::KeyEventRingShmHeader hdr{};
    if (!read_header_stable_(hdr) || !validate_abi_(hdr)) {
        stats_.abi_mismatch++;
        close_shm_();
        return false;
    }

    capacity_ = hdr.capacity;

    // Optional strict capacity check
    if (cfg_.expect_capacity != 0 && capacity_ != cfg_.expect_capacity) {
        stats_.abi_mismatch++;
        std::cerr << "[KeyEventSubscriberShm] capacity mismatch expect=" << cfg_.expect_capacity
                  << " got=" << capacity_ << "\n";
        close_shm_();
        return false;
    }

    // Optional: check mapping size matches expected ring bytes (strict ABI hygiene)
    const std::size_t expect_size = shared::shm::key_event_ring_bytes(capacity_);
    if (shm_size_ < expect_size) {
        stats_.abi_mismatch++;
        std::cerr << "[KeyEventSubscriberShm] shm_size too small size=" << shm_size_
                  << " expect>=" << expect_size << "\n";
        close_shm_();
        return false;
    }

    std::cerr << "[KeyEventSubscriberShm] map ok shm=" << cfg_.shm_name
              << " size=" << shm_size_ << " cap=" << capacity_ << "\n";
    return true;
}

void KeyEventSubscriberShm::close_shm_() noexcept
{
    if (shm_ptr_) {
        ::munmap(const_cast<shared::shm::KeyEventRingShmLayout*>(
                     reinterpret_cast<const shared::shm::KeyEventRingShmLayout*>(shm_ptr_)),
                 shm_size_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    shm_size_ = 0;
}
#endif

bool KeyEventSubscriberShm::validate_abi_(const shared::shm::KeyEventRingShmHeader& hdr) noexcept
{
    if (hdr.magic != shared::shm::kKeyEventRingMagic) return false;
    if (hdr.layout_ver != shared::shm::kKeyEventRingLayoutVersion) return false;

    if (hdr.payload_ver != shared::shm::kKeyEventPayloadVersion) return false;
    if (hdr.payload_size != sizeof(shared::shm::Payload)) return false;
    if (hdr.payload_align != alignof(shared::shm::Payload)) return false;

    if (hdr.capacity == 0) return false;
    return true;
}

bool KeyEventSubscriberShm::read_header_stable_(shared::shm::KeyEventRingShmHeader& out_hdr) noexcept
{
    if (!shm_ptr_) return false;

    // Retry bounded times to avoid spinning forever if writer is active.
    for (int attempt = 0; attempt < 50; ++attempt) {
        std::uint64_t s0 = 0;
        if (!seqlock_read_begin(shm_ptr_->hdr.seqlock, s0)) {
            stats_.seqlock_retry++;
            continue;
        }

        // Copy header as bytes。
        // 这里头部包含 std::atomic 等非平凡成员，不能用 operator=，
        // 但作为 SHM 协议头，我们只做一次性快照，用 memcpy 是合理的。
        // 为了不被 -Wclass-memaccess 警告刷屏，这里局部屏蔽一下。
        #if defined(__GNUG__) && !defined(__clang__)
        #  pragma GCC diagnostic push
        #  pragma GCC diagnostic ignored "-Wclass-memaccess"
        #endif
                std::memcpy(&out_hdr, &shm_ptr_->hdr, sizeof(out_hdr));
        #if defined(__GNUG__) && !defined(__clang__)
        #  pragma GCC diagnostic pop
        #endif

        if (seqlock_read_end(shm_ptr_->hdr.seqlock, s0)) {
            return true;
        }
        stats_.seqlock_retry++;
    }
    return false;
}


std::size_t KeyEventSubscriberShm::drain(shared::msg::KeyEvent* out, std::size_t max) noexcept
{
    if (!enabled_) return 0;
    if (error_flag_) return 0;
    if (!out || max == 0) return 0;

#ifdef _WIN32
    return 0;
#else
    // Lazy init: try open/map when first used.
    if (!shm_ptr_) {
        if (!cfg_.lazy_init) return 0;
        if (!try_open_map_ro_()) return 0;
        initialized_ = true;
        // start from latest to avoid replaying old ring content
        reset_cursor(true);
    }

    shared::shm::KeyEventRingShmHeader hdr{};
    if (!read_header_stable_(hdr) || !validate_abi_(hdr)) {
        stats_.abi_mismatch++;
        error_flag_ = true;
        return 0;
    }

    // Cache diag snapshots
    stats_.last_write_idx  = hdr.write_idx;
    stats_.last_mono_ns    = hdr.mono_ns;
    stats_.last_wall_ns    = hdr.wall_ns;
    stats_.last_drop_count = hdr.drop_count;

    const std::uint32_t cap = hdr.capacity;
    const std::uint64_t w   = hdr.write_idx;

    if (cap == 0) return 0;

    // Initialize cursor if needed
    if (last_read_idx_ == 0 && w != 0) {
        // If reset_cursor(true) was called, last_read_idx_ should already be w.
        // Keep conservative behavior otherwise.
    }

    std::uint64_t avail = (w >= last_read_idx_) ? (w - last_read_idx_) : 0;
    if (avail == 0) return 0;

    // Overwrite loss detection
    if (avail > cap) {
        const std::uint64_t lost = avail - cap;
        stats_.lost_count += lost;
        // fast-forward cursor to the oldest still available
        last_read_idx_ = w - cap;
        avail = cap;
    }

    const std::size_t take = (avail < max) ? static_cast<std::size_t>(avail) : max;

    const auto* ring = shared::shm::key_event_ring_ptr(&shm_ptr_->hdr);
    for (std::size_t i = 0; i < take; ++i) {
        const std::uint64_t idx  = last_read_idx_;
        const std::uint32_t slot = static_cast<std::uint32_t>(idx % cap);

        std::memcpy(&out[i], &ring[slot], sizeof(shared::msg::KeyEvent));
        last_read_idx_ = idx + 1;
    }

    stats_.read_count += take;
    return take;
#endif
}

bool KeyEventSubscriberShm::poll(shared::msg::KeyEvent& out) noexcept
{
    return drain(&out, 1) == 1;
}

void KeyEventSubscriberShm::flush() noexcept
{
#ifdef _WIN32
    (void)0;
#else
    shared::shm::KeyEventRingShmHeader hdr{};
    if (!shm_ptr_) {
        if (!cfg_.lazy_init || !try_open_map_ro_()) return;
        initialized_ = true;
    }
    if (!read_header_stable_(hdr) || !validate_abi_(hdr)) return;
    last_read_idx_ = hdr.write_idx;
#endif
}

void KeyEventSubscriberShm::reset_cursor(bool start_from_latest) noexcept
{
    stats_ = Stats{};
    last_read_idx_ = 0;

#ifdef _WIN32
    (void)start_from_latest;
#else
    if (!start_from_latest) return;
    if (!shm_ptr_) {
        if (!cfg_.lazy_init || !try_open_map_ro_()) return;
        initialized_ = true;
    }
    shared::shm::KeyEventRingShmHeader hdr{};
    if (!read_header_stable_(hdr) || !validate_abi_(hdr)) return;
    capacity_ = hdr.capacity;
    last_read_idx_ = hdr.write_idx;
#endif
}

std::uint64_t KeyEventSubscriberShm::publisher_drop_count() const noexcept
{
    if (!shm_ptr_) return 0;
    return shm_ptr_->hdr.drop_count;
}

bool KeyEventSubscriberShm::snapshot_header(shared::shm::KeyEventRingShmHeader& out_hdr) noexcept
{
#ifdef _WIN32
    (void)out_hdr;
    return false;
#else
    if (!shm_ptr_) {
        if (!cfg_.lazy_init || !try_open_map_ro_()) return false;
    }
    return read_header_stable_(out_hdr);
#endif
}

} // namespace comm_gcs::ipc::keys
