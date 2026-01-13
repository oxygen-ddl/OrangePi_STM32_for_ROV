#include "gateway/intent_publisher_shm.hpp"
#include "shared/msg/control_intent.hpp"
#include "shared/shm/control_intent_shm.hpp"

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

namespace comm_gcs {

namespace {

// 'I''N''T''1'
using ShmLayout = shared::shm::IntentShmLayout;

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
// NOTE:
// ShmLayout is a type alias declared in intent_publisher_shm.hpp:
//   using ShmLayout = shared::shm::IntentShmLayout;
// Do NOT define IntentPublisherShm::ShmLayout here.
// The canonical layout is shared::shm::IntentShmLayout (hdr + intent).


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

IntentPublisherShm& IntentPublisherShm::operator=(IntentPublisherShm&& other) noexcept
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
bool IntentPublisherShm::init(const Config& cfg)
{
    // If re-init, ensure clean slate
    shutdown();

    enabled_ = cfg.enable;
    std::cerr << "[IntentPublisherShm] init(): enable=" << cfg.enable
              << " shm_name=" << (cfg.shm_name.empty() ? "<empty>" : cfg.shm_name)
              << " shm_size=" << cfg.shm_size << "\n";

    if (!enabled_) {
        // disabled is treated as "ok"
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
    shm_name_.clear();
    shm_size_ = 0;
}

bool IntentPublisherShm::publish(const shared::msg::ControlIntent& intent)
{
    if (!enabled_) return true;
    if (!initialized_ || error_flag_ || !shm_ptr_) return false;

    auto* layout = static_cast<ShmLayout*>(shm_ptr_);
    auto& hdr    = layout->hdr;

    // ---- seqlock: begin write（置奇数，表示正在写）----
    const std::uint64_t s0 = hdr.seqlock.load(std::memory_order_relaxed);
    hdr.seqlock.store(s0 + 1, std::memory_order_release);

    // 时间戳更新
    hdr.mono_ns = now_mono_ns();
    hdr.wall_ns = now_wall_ns();
    // magic / layout_ver 等在 init 时已写好，这里不动

    // ---- 关键：整结构赋值，避免 memcpy + sizeof 带来的 ABI/对齐问题 ----
    layout->intent = intent;

    // ---- seqlock: end write（置偶数，表示写完）----
    hdr.seqlock.store(s0 + 2, std::memory_order_release);

    // ---- 调试：直接打印 SHM 里落盘后的内容 ----
    // const auto& snap = layout->intent;

    // if (snap.flags & shared::msg::kHasTeleopDof) {
    //     const auto& c = snap.teleop_dof_cmd;
    //     std::cerr << "[IntentPublisherShm][RAW_AFTER_WRITE] teleop_dof"
    //               << " s="  << c.surge
    //               << " sw=" << c.sway
    //               << " h="  << c.heave
    //               << " r="  << c.roll
    //               << " p="  << c.pitch
    //               << " y="  << c.yaw
    //               << " flags=0x" << std::hex << snap.flags << std::dec
    //               << "\n";
    // } else {
    //     std::cerr << "[IntentPublisherShm][RAW_AFTER_WRITE] no teleop_dof, flags=0x"
    //               << std::hex << snap.flags << std::dec
    //               << "\n";
    // }

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

    std::cerr << "[IntentPublisherShm] init_shm(): shm_name=" << shm_name_
              << " cfg.shm_size=" << cfg.shm_size
              << " min_size=" << sizeof(ShmLayout) << "\n";

    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[IntentPublisherShm] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    const std::size_t min_size = sizeof(ShmLayout);
    shm_size_ = (cfg.shm_size == 0 || cfg.shm_size < min_size) ? min_size : cfg.shm_size;

    shm_fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::perror("[IntentPublisherShm] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }
    std::cerr << "[IntentPublisherShm] shm_open ok: fd=" << shm_fd_ << "\n";

    if (::ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
        std::perror("[IntentPublisherShm] ftruncate failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[IntentPublisherShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);
    std::cerr << "[IntentPublisherShm] mmap ok: ptr=" << static_cast<void*>(shm_ptr_)
              << " size=" << shm_size_ << "\n";

    // protocol mismatch -> reset deterministically (with seqlock semantics)
    auto* layout = static_cast<ShmLayout*>(shm_ptr_);
    const bool mismatch =
        (layout->hdr.magic != shared::shm::kIntentShmMagic) ||
        (layout->hdr.layout_ver != shared::shm::kIntentShmLayoutVersion) ||
        (layout->hdr.payload_ver != shared::msg::kControlIntentWireVersion) ||
        (layout->hdr.payload_size != sizeof(shared::msg::ControlIntent)) ||
        (layout->hdr.payload_align != alignof(shared::msg::ControlIntent));

    if (mismatch) {
        layout->hdr.seqlock.store(1, std::memory_order_relaxed);

        layout->intent = shared::msg::ControlIntent{};
        layout->hdr.magic        = shared::shm::kIntentShmMagic;
        layout->hdr.layout_ver   = shared::shm::kIntentShmLayoutVersion;
        layout->hdr.payload_ver  = shared::msg::kControlIntentWireVersion;
        layout->hdr.payload_size = sizeof(shared::msg::ControlIntent);
        layout->hdr.payload_align= alignof(shared::msg::ControlIntent);
        layout->hdr.mono_ns      = now_mono_ns();
        layout->hdr.wall_ns      = now_wall_ns();

        std::atomic_thread_fence(std::memory_order_release);
        layout->hdr.seqlock.store(2, std::memory_order_release);

        std::cerr << "[IntentPublisherShm] layout mismatch -> reset\n";
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
