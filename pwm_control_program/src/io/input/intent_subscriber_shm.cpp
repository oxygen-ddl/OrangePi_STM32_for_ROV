#include "io/input/intent_subscriber_shm.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rovctrl::io::input {

static inline std::size_t layout_size_or_default(std::size_t cfg_size) noexcept
{
    const std::size_t min_size = sizeof(shared::shm::IntentShmLayout);
    if (cfg_size == 0) return min_size;
    return (cfg_size < min_size) ? min_size : cfg_size;
}

IntentSubscriberShm::~IntentSubscriberShm() noexcept { shutdown(); }

IntentSubscriberShm::IntentSubscriberShm(IntentSubscriberShm&& o) noexcept { *this = std::move(o); }

IntentSubscriberShm& IntentSubscriberShm::operator=(IntentSubscriberShm&& o) noexcept
{
    if (this == &o) return *this;
    shutdown();

    cfg_ = o.cfg_;
    enabled_ = o.enabled_;
    initialized_ = o.initialized_;
    error_flag_ = o.error_flag_;
    shm_size_ = o.shm_size_;
    map_ptr_ = o.map_ptr_;
    shm_ = o.shm_;

#ifndef _WIN32
    shm_fd_ = o.shm_fd_;
    o.shm_fd_ = -1;
#else
    shm_handle_ = o.shm_handle_;
    o.shm_handle_ = nullptr;
#endif

    o.enabled_ = false;
    o.initialized_ = false;
    o.error_flag_ = false;
    o.shm_size_ = 0;
    o.map_ptr_ = nullptr;
    o.shm_ = nullptr;
    return *this;
}

bool IntentSubscriberShm::init(const Config& cfg)
{
    shutdown();
    cfg_ = cfg;
    std::cout << "[IntentSubscriberShm] init: shm_name=" << cfg_.shm_name
              << " enable=" << cfg_.enable
              << " lazy_init=" << cfg_.lazy_init
              << " auto_recover=" << cfg_.auto_recover << "\n";
    enabled_ = cfg_.enable;

    if (!enabled_) {
        initialized_ = true;
        return true;
    }

    shm_size_ = layout_size_or_default(cfg_.shm_size);

    if (!try_open_()) {
        if (cfg_.lazy_init) {
            // 不可用：允许后续 poll() 再尝试
            initialized_ = false;
            return true;
        }
        return false;
    }

    initialized_ = true;

    return true;
}

void IntentSubscriberShm::shutdown() noexcept
{
    close_();
    enabled_ = false;
    initialized_ = false;
    error_flag_ = false;
    shm_size_ = 0;
    cfg_ = Config{};
}

bool IntentSubscriberShm::try_open_() noexcept
{
#ifndef _WIN32
    // read-only open
    shm_fd_ = ::shm_open(cfg_.shm_name.c_str(), O_RDONLY, 0);
    if (shm_fd_ < 0) {
        return false;
    }

    // optional: use fstat to learn actual size (support future extension)
    struct stat st{};
    if (::fstat(shm_fd_, &st) == 0) {
        if (static_cast<std::size_t>(st.st_size) >= sizeof(shared::shm::IntentShmLayout)) {
            shm_size_ = static_cast<std::size_t>(st.st_size);
        }
    }

    if (!map_()) {
        close_();
        return false;
    }

    if (!validate_contract_()) {
        std::cerr << "[IntentSub][ERR] contract mismatch for shm=" << cfg_.shm_name << "\n";
        error_flag_ = true;
        close_();
        return false;
    }

    return true;
#else
    return false;
#endif
}

bool IntentSubscriberShm::map_() noexcept
{
#ifndef _WIN32
    map_ptr_ = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (map_ptr_ == MAP_FAILED) {
        map_ptr_ = nullptr;
        return false;
    }
    shm_ = reinterpret_cast<shared::shm::IntentShmLayout*>(map_ptr_);
    return true;
#else
    return false;
#endif
}

void IntentSubscriberShm::close_() noexcept
{
#ifndef _WIN32
    if (map_ptr_) {
        ::munmap(map_ptr_, shm_size_);
        map_ptr_ = nullptr;
        shm_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
#else
    map_ptr_ = nullptr;
    shm_ = nullptr;
#endif
}

bool IntentSubscriberShm::validate_contract_() noexcept
{
    if (!shm_) return false;

    const auto& h = shm_->hdr;

    if (h.magic != shared::shm::kIntentShmMagic) return false;
    if (h.layout_ver != shared::shm::kIntentShmLayoutVersion) return false;

    if (h.payload_ver != shared::msg::kControlIntentWireVersion) return false;

    const std::uint32_t expect_size  = static_cast<std::uint32_t>(sizeof(shared::msg::ControlIntent));
    const std::uint32_t expect_align = static_cast<std::uint32_t>(alignof(shared::msg::ControlIntent));

    if (h.payload_size != expect_size) return false;
    if (h.payload_align != expect_align) return false;

    return true;
}

bool IntentSubscriberShm::read_consistent_(shared::msg::ControlIntent& out,
                                          std::uint64_t* out_mono_ns,
                                          std::uint64_t* out_wall_ns) noexcept
{
    if (!shm_) return false;

    // bounded retries: seqlock may change during copy
    for (int attempt = 0; attempt < 3; ++attempt) {
        const std::uint64_t s0 = shm_->hdr.seqlock.load(std::memory_order_acquire);
        if (s0 & 1ULL) {
            // writer in progress
            continue;
        }

        const std::uint64_t mono = shm_->hdr.mono_ns;
        const std::uint64_t wall = shm_->hdr.wall_ns;

        // payload copy (plain struct)
        out = shm_->intent;

        const std::uint64_t s1 = shm_->hdr.seqlock.load(std::memory_order_acquire);
        if (s0 == s1 && ((s1 & 1ULL) == 0)) {
            if (out_mono_ns) *out_mono_ns = mono;
            if (out_wall_ns) *out_wall_ns = wall;
            return true;
        }
    }
    return false;
}

std::optional<shared::msg::ControlIntent> IntentSubscriberShm::poll(std::uint64_t* out_mono_ns,
                                                                    std::uint64_t* out_wall_ns) noexcept
{
    if (!enabled_) return std::nullopt;

    // lazy init: not opened yet
    if (!shm_) {
        if (error_flag_ && !cfg_.auto_recover) return std::nullopt;

        if (!try_open_()) {
            return std::nullopt;
        }
    }

    shared::msg::ControlIntent snap{};
    if (!read_consistent_(snap, out_mono_ns, out_wall_ns)) {
        return std::nullopt;
    }

    // version gate (payload 内部也有 version)
    if (snap.version != shared::msg::kControlIntentWireVersion) {
        return std::nullopt;
    }
    
    // ===== 在这里插入调试代码（BEGIN） =====
    static int dbg_cnt = 0;
    if (++dbg_cnt % 20 == 0) {  // 每 20 次 poll 打一条，避免刷屏
        if (snap.flags & shared::msg::kHasTeleopDof) {
            const auto& c = snap.teleop_dof_cmd;
            std::cerr << "[IntentSubscriberShm][POLL] teleop_dof"
                      << " s="  << c.surge
                      << " sw=" << c.sway
                      << " h="  << c.heave
                      << " r="  << c.roll
                      << " p="  << c.pitch
                      << " y="  << c.yaw
                      << " flags=0x" << std::hex << snap.flags << std::dec
                      << "\n";
        } else {
            std::cerr << "[IntentSubscriberShm][POLL] no teleop_dof, flags=0x"
                      << std::hex << snap.flags << std::dec << "\n";
        }
    }
    // ===== 调试代码（END） =====

    return snap;
}

} // namespace rovctrl::io::input
