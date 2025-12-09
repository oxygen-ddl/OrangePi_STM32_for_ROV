// src/io/nav_state_subscriber.cpp
#include "io/nav_state_subscriber.hpp"

#include <iostream>
#include <cstring>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
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

} // namespace

// ==================== 生命周期管理 ====================

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
    if (this == &other) {
        return *this;
    }

    shutdown();

    initialized_ = other.initialized_;
    error_flag_  = other.error_flag_;
    shm_name_    = std::move(other.shm_name_);
    shm_size_    = other.shm_size_;

#ifndef _WIN32
    shm_fd_      = other.shm_fd_;
    shm_ptr_     = other.shm_ptr_;
    other.shm_fd_  = -1;
    other.shm_ptr_ = nullptr;
#else
    shm_handle_  = other.shm_handle_;
    shm_ptr_     = other.shm_ptr_;
    other.shm_handle_ = nullptr;
    other.shm_ptr_    = nullptr;
#endif

    other.initialized_ = false;
    other.error_flag_  = false;

    return *this;
}

// ==================== 公共接口 ====================

bool NavStateSubscriber::init(const std::string& shm_name)
{
    shutdown();  // 确保干净起步

    shm_name_    = shm_name;
    error_flag_  = false;
    initialized_ = false;

    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[NavStateSubscriber] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        error_flag_  = true;
        initialized_ = false;
        return false;
    }

    if (!open_shm(shm_name_)) {
        error_flag_  = true;
        initialized_ = false;
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
    if (!initialized_ || error_flag_ || !shm_ptr_) {
        return false;
    }

    const ShmLayout* layout = shm_ptr_;

    // 无锁读取：seq 奇偶 + 前后一致检查
    const std::uint64_t seq1 = layout->hdr.seq;
    if (seq1 & 1u) {
        // 奇数：写入中，当前帧不使用
        return false;
    }

    shared::msg::NavState tmp = layout->state;

    const std::uint64_t seq2 = layout->hdr.seq;
    if (seq1 != seq2 || (seq2 & 1u)) {
        // 期间被写入过，或结束时仍为奇数，认为不稳定
        return false;
    }

    out_state = tmp;
    return true;
}

// ==================== 内部实现：共享内存打开/关闭 ====================

bool NavStateSubscriber::open_shm(const std::string& shm_name)
{
#ifndef _WIN32
    // 只读方式打开既有共享内存，不创建
    shm_fd_ = ::shm_open(shm_name.c_str(), O_RDONLY, 0666);
    if (shm_fd_ < 0) {
        std::perror("[NavStateSubscriber] shm_open failed");
        return false;
    }

    struct stat st {};
    if (::fstat(shm_fd_, &st) != 0) {
        std::perror("[NavStateSubscriber] fstat failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_size_ = static_cast<std::size_t>(st.st_size);
    if (shm_size_ < sizeof(ShmLayout)) {
        std::cerr << "[NavStateSubscriber] shm size too small: "
                  << shm_size_ << " < " << sizeof(ShmLayout) << "\n";
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    void* addr = ::mmap(nullptr,
                        shm_size_,
                        PROT_READ,
                        MAP_SHARED,
                        shm_fd_,
                        0);
    if (addr == MAP_FAILED) {
        std::perror("[NavStateSubscriber] mmap failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);

    // 校验 magic/version 是否匹配 nav_core 端
    if (shm_ptr_->hdr.magic   != NAV_STATE_MAGIC ||
        shm_ptr_->hdr.version != NAV_STATE_VERSION)
    {
        std::cerr << "[NavStateSubscriber] shm header mismatch: "
                  << "magic=0x" << std::hex << shm_ptr_->hdr.magic
                  << " version=" << std::dec << shm_ptr_->hdr.version
                  << " (expected magic=0x" << std::hex << NAV_STATE_MAGIC
                  << " version=" << std::dec << NAV_STATE_VERSION << ")\n";

        ::munmap(static_cast<void*>(shm_ptr_), shm_size_);
        ::close(shm_fd_);
        shm_ptr_ = nullptr;
        shm_fd_  = -1;
        return false;
    }

    return true;
#else
    std::cerr << "[NavStateSubscriber] Shared memory subscriber "
              << "is not implemented on Windows.\n";
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
