// src/io/nav/nav_state_subscriber.cpp
#include "io/nav/nav_state_subscriber.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>


#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#endif

namespace rovctrl::io {

namespace {

// 与 nav_core::NavStatePublisher 中保持完全一致
constexpr std::uint32_t NAV_STATE_MAGIC =
    (static_cast<std::uint32_t>('N') << 24) |
    (static_cast<std::uint32_t>('A') << 16) |
    (static_cast<std::uint32_t>('V') << 8)  |
    (static_cast<std::uint32_t>('1'));

constexpr std::uint32_t NAV_STATE_VERSION = 1;

// 1s 限频
constexpr std::chrono::milliseconds kWarnEvery{1000};

// 进程内限频告警：同一类告警 1s 仅打印一次
inline void rate_limited_warn(const char* tag, const std::string& msg)
{
    // 说明：简单起见，这里对所有 warn 共用一个节流器；
    // 如果你希望“不同错误各自 1s 一次”，可以把 key 做成 map（版本 2 可扩展）。
    static auto last = std::chrono::steady_clock::time_point::min();

    const auto now = std::chrono::steady_clock::now();
    if (now - last < kWarnEvery) return;
    last = now;

    std::cerr << tag << " " << msg << "\n";
}

#ifndef _WIN32
inline std::string errno_str()
{
    // strerror_r 不同平台签名差异较大，简单用 strerror 即可（我们只用于日志）
    return std::string(std::strerror(errno));
}
#endif

} // namespace

NavStateSubscriber::~NavStateSubscriber() noexcept
{
    shutdown();
}

NavStateSubscriber::NavStateSubscriber(NavStateSubscriber&& other) noexcept
{
    *this = std::move(other);
}

NavStateSubscriber&
NavStateSubscriber::operator=(NavStateSubscriber&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

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

    other.initialized_ = false;
    other.error_flag_  = false;
    return *this;
}

bool NavStateSubscriber::init(const std::string& shm_name)
{
    shutdown();

    shm_name_    = shm_name;
    error_flag_  = false;
    initialized_ = false;

    if (shm_name_.empty() || shm_name_.front() != '/') {
        rate_limited_warn("[NavStateSubscriber]",
                          "Invalid shm_name: " + shm_name_ + " (must start with '/').");
        error_flag_ = true;
        return false;
    }

    if (!open_shm(shm_name_)) {
        error_flag_ = true;
        return false;
    }

    initialized_ = true;
    error_flag_  = false;
    return true;
}

void NavStateSubscriber::shutdown() noexcept
{
    close_shm();
    initialized_ = false;
    error_flag_  = false;
}

bool NavStateSubscriber::read_latest(shared::msg::NavState& out_state) const
{
    if (!initialized_ || error_flag_ || !shm_ptr_) return false;

    const ShmLayout* layout = shm_ptr_;

    const std::uint64_t seq1 = __atomic_load_n(&layout->hdr.seq, __ATOMIC_ACQUIRE);
    if (seq1 & 1ULL) return false;

    shared::msg::NavState tmp;
    std::memcpy(&tmp, &layout->state, sizeof(tmp));

    const std::uint64_t seq2 = __atomic_load_n(&layout->hdr.seq, __ATOMIC_ACQUIRE);
    if (seq1 != seq2 || (seq2 & 1ULL)) return false;

    out_state = tmp;
    return true;
}

bool NavStateSubscriber::open_shm(const std::string& shm_name)
{
#ifndef _WIN32
    shm_fd_ = ::shm_open(shm_name.c_str(), O_RDONLY, 0666);
    if (shm_fd_ < 0) {
        rate_limited_warn("[NavStateSubscriber]",
                          "shm_open failed for " + shm_name + ": " + errno_str());
        return false;
    }

    struct stat st {};
    if (::fstat(shm_fd_, &st) != 0) {
        rate_limited_warn("[NavStateSubscriber]",
                          "fstat failed: " + errno_str());
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_size_ = static_cast<std::size_t>(st.st_size);
    if (shm_size_ < sizeof(ShmLayout)) {
        rate_limited_warn("[NavStateSubscriber]",
                          "shm size too small: " + std::to_string(shm_size_) +
                          " < " + std::to_string(sizeof(ShmLayout)));
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        rate_limited_warn("[NavStateSubscriber]",
                          "mmap failed: " + errno_str());
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);

    if (shm_ptr_->hdr.magic != NAV_STATE_MAGIC || shm_ptr_->hdr.version != NAV_STATE_VERSION) {
        std::ostringstream oss;
        oss << "shm header mismatch: magic=0x" << std::hex << shm_ptr_->hdr.magic
            << " version=" << std::dec << shm_ptr_->hdr.version
            << " (expected magic=0x" << std::hex << NAV_STATE_MAGIC
            << " version=" << std::dec << NAV_STATE_VERSION << ")";
        rate_limited_warn("[NavStateSubscriber]", oss.str());

        ::munmap(static_cast<void*>(shm_ptr_), shm_size_);
        ::close(shm_fd_);
        shm_ptr_ = nullptr;
        shm_fd_  = -1;
        return false;
    }

    return true;
#else
    rate_limited_warn("[NavStateSubscriber]",
                      "Shared memory subscriber is not implemented on Windows.");
    return false;
#endif
}

void NavStateSubscriber::close_shm() noexcept
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

} // namespace rovctrl::io
