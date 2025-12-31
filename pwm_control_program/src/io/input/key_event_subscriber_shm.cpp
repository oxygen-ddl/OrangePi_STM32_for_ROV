#include "io/input/key_event_subscriber_shm.hpp"

#include "shared/shm/key_event_shm.hpp"

#include <atomic>
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

namespace rovctrl::io {

namespace {

// 固定 ring 容量：publisher 与 subscriber 必须一致。
// 如果你希望可配置：也可以做成 template 或编译期宏，但建议固定以保持 ABI 严格。
constexpr std::size_t kRingCapacity = 256;

// seqlock 读取有限重试，避免极端情况下忙等。
constexpr int kMaxSeqLockRetries = 16;

// Layout type
using ShmLayout = shared::shm::ShmLayoutT<kRingCapacity>;

static inline bool starts_with_slash(const std::string& s) {
    return !s.empty() && s.front() == '/';
}

} // namespace

KeyEventSubscriberShm::KeyEventSubscriberShm(KeyEventSubscriberShm&& other) noexcept
{
    *this = std::move(other);
}

KeyEventSubscriberShm&
KeyEventSubscriberShm::operator=(KeyEventSubscriberShm&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    cfg_            = other.cfg_;
    enabled_        = other.enabled_;
    initialized_    = other.initialized_;
    error_flag_     = other.error_flag_;

    shm_name_       = std::move(other.shm_name_);
    shm_size_       = other.shm_size_;

    last_pub_mono_ns_ = other.last_pub_mono_ns_;
    last_pub_wall_ns_ = other.last_pub_wall_ns_;
    last_read_seq_    = other.last_read_seq_;

#ifndef _WIN32
    shm_fd_         = other.shm_fd_;
    other.shm_fd_   = -1;
#endif
    shm_ptr_        = other.shm_ptr_;
    other.shm_ptr_  = nullptr;

    other.enabled_     = false;
    other.initialized_ = false;
    other.error_flag_  = false;
    other.shm_size_    = 0;
    other.last_pub_mono_ns_ = 0;
    other.last_pub_wall_ns_ = 0;
    other.last_read_seq_    = 0;

    return *this;
}

bool KeyEventSubscriberShm::init(const Config& cfg)
{
    cfg_     = cfg;
    enabled_ = cfg.enable;

    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }

    shm_name_ = cfg_.shm_name;

#ifndef _WIN32
    if (!starts_with_slash(shm_name_)) {
        std::cerr << "[KeyEventSubscriberShm] Invalid shm_name (must start with '/'): "
                  << shm_name_ << "\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // lazy init: do not fail if shm doesn't exist yet
    if (cfg_.lazy_init) {
        initialized_ = false;
        error_flag_  = false;
        shm_size_    = 0;
        last_read_seq_ = 0;
        return true;
    }

    return init_shm(cfg_);
#else
    std::cerr << "[KeyEventSubscriberShm] Shared memory is not supported on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#endif
}

void KeyEventSubscriberShm::shutdown() noexcept
{
    close_shm();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;

    last_pub_mono_ns_ = 0;
    last_pub_wall_ns_ = 0;
    last_read_seq_    = 0;
}

int KeyEventSubscriberShm::poll_many(shared::msg::KeyEvent* out, int max_out,
                                     std::uint64_t* out_mono_ns,
                                     std::uint64_t* out_wall_ns)
{
    if (!enabled_ || !out || max_out <= 0) return 0;

#ifndef _WIN32
    // Lazy init attempt if needed
    if (!initialized_) {
        if (!cfg_.lazy_init) return 0;
        if (!init_shm(cfg_)) {
            // shm not ready yet, not an error in lazy mode
            return 0;
        }
    }

    if (error_flag_ || !shm_ptr_) return 0;

    std::uint64_t write_seq = 0;
    std::uint64_t mono_ns   = 0;
    std::uint64_t wall_ns   = 0;

    if (!read_header_snapshot(write_seq, mono_ns, wall_ns)) {
        return 0;
    }

    if (out_mono_ns) *out_mono_ns = mono_ns;
    if (out_wall_ns) *out_wall_ns = wall_ns;

    last_pub_mono_ns_ = mono_ns;
    last_pub_wall_ns_ = wall_ns;

    // No new events
    if (write_seq <= last_read_seq_) {
        return 0;
    }

    // If reader lags too far, drop old events (fast-forward).
    // Available = write_seq - last_read_seq_
    const std::uint64_t available = write_seq - last_read_seq_;
    if (available > kRingCapacity) {
        // drop (available - capacity) events
        last_read_seq_ = write_seq - kRingCapacity;
    }

    int n_out = 0;
    while (n_out < max_out) {
        // refresh write_seq occasionally is possible, but not required
        if (last_read_seq_ >= write_seq) break;

        const std::uint64_t next_seq = last_read_seq_ + 1;

        shared::msg::KeyEvent ev{};
        if (!read_event_at_seq(next_seq, ev)) {
            // If we cannot read consistently (writer in progress), stop this cycle.
            break;
        }

        out[n_out++] = ev;
        last_read_seq_ = next_seq;
    }

    return n_out;
#else
    (void)out_mono_ns; (void)out_wall_ns;
    return 0;
#endif
}

#ifndef _WIN32

bool KeyEventSubscriberShm::init_shm(const Config& cfg)
{
    // Open existing shm (publisher owns creation)
    shm_fd_ = ::shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ < 0) {
        if (cfg.lazy_init && errno == ENOENT) {
            initialized_ = false;
            error_flag_  = false;
            return false;
        }
        std::perror("[KeyEventSubscriberShm] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // Determine actual shm size via fstat
    struct ::stat st {};
    if (::fstat(shm_fd_, &st) != 0) {
        std::perror("[KeyEventSubscriberShm] fstat failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    const std::size_t min_size  = sizeof(ShmLayout);
    std::size_t real_size = static_cast<std::size_t>(st.st_size);
    if (real_size < min_size) {
        std::cerr << "[KeyEventSubscriberShm] shm size too small: " << real_size
                  << " < " << min_size << "\n";
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    if (cfg.shm_size != 0) {
        shm_size_ = (cfg.shm_size <= real_size) ? cfg.shm_size : real_size;
        if (shm_size_ < min_size) shm_size_ = min_size;
    } else {
        shm_size_ = real_size;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[KeyEventSubscriberShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = addr;

    // ABI check (hard reject)
    auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    const auto& hdr = layout->hdr;

    // We must read hdr fields safely: seqlock snapshot
    std::uint64_t write_seq = 0, mono_ns = 0, wall_ns = 0;
    if (!read_header_snapshot(write_seq, mono_ns, wall_ns)) {
        // publisher might be initializing; allow retry in lazy mode
        close_shm();
        initialized_ = false;
        error_flag_  = cfg.lazy_init ? false : true;
        return false;
    }

    // Validate contract tags / ABI metadata
    // Note: hdr.capacity is part of seqlock-protected header snapshot; re-read directly is OK but we use hdr.
    // We enforce exact capacity match for strict ABI.
    const bool abi_ok =
        (hdr.magic      == shared::shm::kKeyEventMagic) &&
        (hdr.layout_ver == shared::shm::kKeyEventLayoutVersion) &&
        (hdr.payload_ver   == shared::msg::kKeyEventWireVersion) &&
        (hdr.payload_size  == static_cast<std::uint32_t>(sizeof(shared::msg::KeyEvent))) &&
        (hdr.payload_align == static_cast<std::uint32_t>(alignof(shared::msg::KeyEvent))) &&
        (hdr.capacity      == static_cast<std::uint32_t>(kRingCapacity));

    if (!abi_ok) {
        std::cerr << "[KeyEventSubscriberShm] ABI mismatch; reject mapping.\n"
                  << "  magic=" << std::hex << hdr.magic << std::dec
                  << " layout_ver=" << hdr.layout_ver
                  << " payload_ver=" << hdr.payload_ver
                  << " payload_size=" << hdr.payload_size
                  << " payload_align=" << hdr.payload_align
                  << " capacity=" << hdr.capacity
                  << " (expect capacity=" << kRingCapacity << ")\n";
        close_shm();
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // Good to go
    last_pub_mono_ns_ = mono_ns;
    last_pub_wall_ns_ = wall_ns;

    // Initialize reader cursor to "current tail": start from latest write_seq,
    // so first poll only receives events after subscriber started.
    last_read_seq_ = write_seq;

    initialized_ = true;
    error_flag_  = false;
    return true;
}

void KeyEventSubscriberShm::close_shm() noexcept
{
    if (shm_ptr_) {
        ::munmap(shm_ptr_, shm_size_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    shm_size_ = 0;
}

bool KeyEventSubscriberShm::read_header_snapshot(std::uint64_t& out_write_seq,
                                                 std::uint64_t& out_mono_ns,
                                                 std::uint64_t& out_wall_ns) noexcept
{
    if (!shm_ptr_) return false;

    const auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    const auto& hdr = layout->hdr;

    for (int i = 0; i < kMaxSeqLockRetries; ++i) {
        const std::uint64_t s1 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 & 1u) continue; // writer in progress

        // Snapshot (plain reads are okay under seqlock protocol)
        const std::uint64_t mono_ns = hdr.mono_ns;
        const std::uint64_t wall_ns = hdr.wall_ns;
        const std::uint64_t write_seq = layout->write_seq.load(std::memory_order_acquire);

        // Verify stable
        const std::uint64_t s2 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 == s2 && ((s2 & 1u) == 0)) {
            out_mono_ns   = mono_ns;
            out_wall_ns   = wall_ns;
            out_write_seq = write_seq;
            return true;
        }
    }
    return false;
}

bool KeyEventSubscriberShm::read_event_at_seq(std::uint64_t seq, shared::msg::KeyEvent& out) noexcept
{
    if (!shm_ptr_) return false;

    const auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    const auto& hdr = layout->hdr;

    const std::size_t idx = static_cast<std::size_t>(seq % kRingCapacity);

    for (int i = 0; i < kMaxSeqLockRetries; ++i) {
        const std::uint64_t s1 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 & 1u) continue;

        // Copy payload (memcpy avoids UB with potentially unaligned loads)
        shared::msg::KeyEvent tmp{};
        std::memcpy(&tmp, &layout->ring[idx], sizeof(tmp));

        const std::uint64_t s2 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 == s2 && ((s2 & 1u) == 0)) {
            out = tmp;
            return true;
        }
    }

    return false;
}

#endif // !_WIN32

} // namespace rovctrl::io
