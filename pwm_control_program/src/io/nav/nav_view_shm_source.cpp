#include "io/nav/nav_view_shm_source.hpp"

#include <chrono>
#include <cstdint>

namespace rovctrl::io::nav {

NavViewShmSource::NavViewShmSource() = default;

NavViewShmSource::~NavViewShmSource() noexcept
{
    shutdown();
}

bool NavViewShmSource::init(const Config& cfg)
{
    shutdown();

    cfg_     = cfg;
    enabled_ = cfg_.enable;

    if (!enabled_) {
        ok_ = true;   // disabled means "not an error"
        return true;
    }

    NavViewSubscriberShm::Config scfg{};
    scfg.enable       = true;
    scfg.shm_name     = cfg_.shm_name;
    scfg.shm_size     = 0;             // use contract sizeof(layout)
    scfg.lazy_init    = cfg_.lazy_init;
    scfg.allow_repeat = true;          // let Source apply policy each call

    ok_ = sub_.init(scfg);
    return ok_;
}

bool NavViewShmSource::init()
{
    return init(cfg_);
}

bool NavViewShmSource::ok() const noexcept
{
    return ok_;
}

bool NavViewShmSource::read_latest(rovctrl::io::NavStateView& out)
{
    if (!enabled_ || !ok_) return false;

    std::uint64_t pub_mono_ns = 0;
    std::uint64_t pub_wall_ns = 0;

    auto opt = sub_.poll_wire(&pub_mono_ns, &pub_wall_ns);
    if (!opt.has_value()) return false;

    // Fill (Scheme B)
    out.wire         = *opt;
    out.pub_mono_ns  = pub_mono_ns;
    out.pub_wall_ns  = pub_wall_ns;
    out.age_ms_local = 0;

    // Policy: age gate based on publisher mono_ns
    if (cfg_.max_age_ms > 0 && pub_mono_ns != 0) {
        const std::uint64_t now_ns = now_mono_ns_();
        const std::uint64_t age_ms =
            (now_ns >= pub_mono_ns) ? (now_ns - pub_mono_ns) / 1000000ULL : 0ULL;

        out.age_ms_local = static_cast<std::uint32_t>(age_ms);
        if (age_ms > cfg_.max_age_ms) return false;

        // Optional diag sync into payload (keeps wire.age_ms meaningful)
        out.wire.age_ms = out.age_ms_local;
    }

    // Policy: require valid
    if (cfg_.require_valid && out.wire.valid == 0) return false;

    // Contract: mono_ns in wire equals shm header mono_ns (Convention 1)
    out.wire.mono_ns = out.pub_mono_ns;

    return true;
}

void NavViewShmSource::shutdown() noexcept
{
    sub_.shutdown();
    enabled_ = false;
    ok_      = true;
}

std::uint64_t NavViewShmSource::now_mono_ns_() noexcept
{
    using Clock = std::chrono::steady_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count());
}

} // namespace rovctrl::io::nav
