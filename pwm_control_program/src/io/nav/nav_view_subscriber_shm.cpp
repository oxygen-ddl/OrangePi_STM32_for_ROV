#include "io/nav/nav_view_subscriber_shm.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <iostream>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rovctrl::io::nav {

// nav_state_view_shm.hpp 在 namespace shared::shm 里定义 struct ShmLayout
using NavViewShmLayout = shared::shm::ShmLayout;

namespace {

#ifndef _WIN32
static void* map_ro(int fd, std::size_t size) noexcept
{
    void* p = ::mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    return (p == MAP_FAILED) ? nullptr : p;
}

static void unmap(void* p, std::size_t size) noexcept
{
    if (!p || size == 0) return;
    ::munmap(p, size);
}
#endif

// Seqlock read helper:
//  - publisher uses hdr.seq as seqlock: odd=writing, even=stable
//  - reader retries if changed or odd
static inline bool read_seqlock_snapshot(const NavViewShmLayout* shm,
                                        shared::msg::NavStateView& out_view,
                                        std::uint64_t& out_pub_mono_ns,
                                        std::uint64_t& out_pub_wall_ns,
                                        std::uint64_t& out_seq_even) noexcept
{
    if (!shm) return false;

    constexpr int kMaxRetry = 8;

    for (int i = 0; i < kMaxRetry; ++i) {
        const std::uint64_t s0 = shm->hdr.seq.load(std::memory_order_acquire);
        if (s0 & 1ULL) continue; // writer in progress

        // timestamps (hdr)
        out_pub_mono_ns = shm->hdr.mono_ns;
        out_pub_wall_ns = shm->hdr.wall_ns;

        // payload snapshot
        std::memcpy(&out_view, &shm->payload, sizeof(shared::msg::NavStateView));

        const std::uint64_t s1 = shm->hdr.seq.load(std::memory_order_acquire);
        if (s0 == s1 && ((s1 & 1ULL) == 0)) {
            out_seq_even = s1;
            return true;
        }
    }
    return false;
}

} // namespace

// ----------------------------------------------------------------------------
// Lifecycle
// ----------------------------------------------------------------------------

NavViewSubscriberShm::~NavViewSubscriberShm() noexcept
{
    shutdown();
}

NavViewSubscriberShm::NavViewSubscriberShm(NavViewSubscriberShm&& o) noexcept
{
    *this = std::move(o);
}

NavViewSubscriberShm& NavViewSubscriberShm::operator=(NavViewSubscriberShm&& o) noexcept
{
    if (this == &o) return *this;

    shutdown();

    cfg_         = std::move(o.cfg_);
    enabled_     = o.enabled_;
    initialized_ = o.initialized_;
    error_flag_  = o.error_flag_;

#ifndef _WIN32
    shm_fd_ = o.shm_fd_;
    o.shm_fd_ = -1;
#else
    shm_handle_ = o.shm_handle_;
    o.shm_handle_ = nullptr;
#endif

    map_      = o.map_;
    map_size_ = o.map_size_;
    shm_      = o.shm_;

    last_seq_ = o.last_seq_;

    o.map_        = nullptr;
    o.map_size_   = 0;
    o.shm_        = nullptr;
    o.enabled_    = false;
    o.initialized_= false;
    o.error_flag_ = false;
    o.last_seq_   = 0;

    return *this;
}

// ----------------------------------------------------------------------------
// Init / Shutdown
// ----------------------------------------------------------------------------

bool NavViewSubscriberShm::init(const Config& cfg)
{
    shutdown();

    cfg_     = cfg;
    enabled_ = cfg_.enable;

    if (!enabled_) {
        initialized_ = true;
        return true;
    }

    // eager open if not lazy
    if (!cfg_.lazy_init) {
        if (!try_open_()) {
            std::cerr << "[NavViewSub][ERR] open shm failed (lazy_init=0)\n";
            return false;
        }
        if (!abi_check_()) {
            std::cerr << "[NavViewSub][ERR] ABI check failed\n";
            close_();
            return false;
        }
    }

    initialized_ = true;
    return true;
}

void NavViewSubscriberShm::shutdown() noexcept
{
    close_();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;
    last_seq_    = 0;
}

// ----------------------------------------------------------------------------
// Internal close
// ----------------------------------------------------------------------------

void NavViewSubscriberShm::close_() noexcept
{
#ifndef _WIN32
    if (map_) {
        unmap(map_, map_size_);
        map_ = nullptr;
    }
    map_size_ = 0;
    shm_      = nullptr;

    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
#else
    map_ = nullptr;
    map_size_ = 0;
    shm_ = nullptr;
    shm_handle_ = nullptr;
#endif
}

// ----------------------------------------------------------------------------
// Open / ABI check
// ----------------------------------------------------------------------------

bool NavViewSubscriberShm::try_open_() noexcept
{
    if (shm_) return true; // already open/mapped

#ifndef _WIN32
    // Use local fd and commit on success to avoid leaving half-open state.
    const int fd = ::shm_open(cfg_.shm_name.c_str(), O_RDONLY, 0);
    if (fd < 0) {
        if (cfg_.lazy_init && errno == ENOENT) {
            return false; // not ready yet (no error_flag_)
        }
        std::cerr << "[NavViewSub][ERR] shm_open(" << cfg_.shm_name
                  << ") failed: " << std::strerror(errno) << "\n";
        error_flag_ = true;
        return false;
    }

    std::size_t size = cfg_.shm_size;
    if (size == 0) {
        size = sizeof(NavViewShmLayout); // unify contract size
    } else if (size < sizeof(NavViewShmLayout)) {
        std::cerr << "[NavViewSub][ERR] shm_size too small: " << size
                  << " < sizeof(layout)=" << sizeof(NavViewShmLayout) << "\n";
        ::close(fd);
        error_flag_ = true;
        return false;
    }

    void* map = map_ro(fd, size);
    if (!map) {
        std::cerr << "[NavViewSub][ERR] mmap failed: " << std::strerror(errno) << "\n";
        ::close(fd);
        error_flag_ = true;
        return false;
    }

    // Commit
    shm_fd_   = fd;
    map_      = map;
    map_size_ = size;
    shm_      = reinterpret_cast<NavViewShmLayout*>(map_);

    return true;

#else
    error_flag_ = true;
    return false;
#endif
}

bool NavViewSubscriberShm::abi_check_() noexcept
{
    if (!shm_) return false;

    const auto& h = shm_->hdr;

    if (h.magic != shared::shm::kNavViewMagic) {
        std::cerr << "[NavViewSub][ERR] bad magic: got=0x" << std::hex << h.magic
                  << " expect=0x" << shared::shm::kNavViewMagic << std::dec << "\n";
        return false;
    }
    if (h.layout_ver != shared::shm::kNavViewLayoutVersion) {
        std::cerr << "[NavViewSub][ERR] bad layout_ver: got=" << h.layout_ver
                  << " expect=" << shared::shm::kNavViewLayoutVersion << "\n";
        return false;
    }
    if (h.payload_ver != shared::msg::kNavStateViewWireVersion) {
        std::cerr << "[NavViewSub][ERR] bad payload_ver: got=" << h.payload_ver
                  << " expect=" << shared::msg::kNavStateViewWireVersion << "\n";
        return false;
    }
    if (h.payload_size != sizeof(shared::msg::NavStateView)) {
        std::cerr << "[NavViewSub][ERR] bad payload_size: got=" << h.payload_size
                  << " expect=" << sizeof(shared::msg::NavStateView) << "\n";
        return false;
    }
    if (h.payload_align != alignof(shared::msg::NavStateView)) {
        std::cerr << "[NavViewSub][ERR] bad payload_align: got=" << h.payload_align
                  << " expect=" << alignof(shared::msg::NavStateView) << "\n";
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
// Poll
// ----------------------------------------------------------------------------

std::optional<shared::msg::NavStateView>
NavViewSubscriberShm::poll_wire(std::uint64_t* out_pub_mono_ns,
                               std::uint64_t* out_pub_wall_ns) noexcept
{
    if (!enabled_ || !initialized_) return std::nullopt;

    // Lazy open
    if (!shm_) {
        if (!try_open_()) return std::nullopt; // not ready yet (or hard fail already flagged)

        if (!abi_check_()) {
            std::cerr << "[NavViewSub][ERR] ABI check failed; disabling subscriber.\n";
            error_flag_ = true;
            close_();
            return std::nullopt;
        }
    }

    shared::msg::NavStateView view{};
    std::uint64_t pub_mono_ns = 0;
    std::uint64_t pub_wall_ns = 0;
    std::uint64_t seq_even    = 0;

    if (!read_seqlock_snapshot(shm_, view, pub_mono_ns, pub_wall_ns, seq_even)) {
        return std::nullopt;
    }

    // Change detection
    if (!cfg_.allow_repeat) {
        if (seq_even == last_seq_) return std::nullopt;
        last_seq_ = seq_even;
    }

    if (out_pub_mono_ns) *out_pub_mono_ns = pub_mono_ns;
    if (out_pub_wall_ns) *out_pub_wall_ns = pub_wall_ns;

    return view;
}

} // namespace rovctrl::io::nav
