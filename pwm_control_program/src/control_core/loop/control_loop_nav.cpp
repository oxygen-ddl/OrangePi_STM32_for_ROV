/**
 * @file   control_loop_nav.cpp
 * @brief  NavSub PIMPL + navigation feedback update (B2: NavStateView)
 */

#include "control_core/control_loop.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "io/nav/nav_state_view.hpp"
#include "io/nav/nav_view_shm_source.hpp"   // rovctrl::io::nav::NavViewShmSource

namespace rovctrl::control_core {

// ============================================================================
// NavSub (PIMPL) —— only depends on nav shm source in this TU
// ============================================================================

struct ControlLoop::NavSub {
    rovctrl::io::nav::NavViewShmSource src{};
    rovctrl::io::nav::NavViewShmSource::Config cfg{};
    bool inited{false};

    bool ensure_init() {
        if (inited) return src.ok();
        inited = src.init(cfg);
        return inited && src.ok();
    }

    bool read_latest(rovctrl::io::NavStateView& out) {
        if (!ensure_init()) return false;
        return src.read_latest(out);
    }

    void shutdown() noexcept { src.shutdown(); inited = false; }
};


// custom deleter for unique_ptr<NavSub, NavSubDeleter>
void ControlLoop::NavSubDeleter::operator()(NavSub* p) noexcept { delete p; }

// ============================================================================
// Navigation feedback update (B2: NavStateView)
// ============================================================================

bool ControlLoop::update_nav_feedback_(rovctrl::io::NavStateView& nav_view_out)
{
    state_.nav_valid = false;

    if (!nav_sub_) {
        nav_sub_.reset(new NavSub());
        // 可在此用 cfg_ 覆盖 nav_sub_->cfg.*
    }

    if (!nav_sub_->read_latest(nav_view_out)) {
        last_nav_valid_ = false;
        return false;
    }

    last_nav_valid_   = true;
    state_.nav_valid  = true;

    // readability: use accessor
    const auto& v = nav_view_out.payload();

    // nav timestamp (fusion time)
    state_.nav_t_ns = v.stamp_ns;

    for (std::size_t i = 0; i < 3; ++i) {
        state_.nav_pos_ned[i] = v.pos[i];
        state_.nav_vel_ned[i] = v.vel[i];
        state_.nav_rpy[i]     = v.rpy[i];
        state_.nav_omega_b[i] = v.omega_b[i];
        state_.nav_acc_b[i]   = v.acc_b[i];
    }

    state_.nav_depth = v.depth_m;

    // flags uint32 -> uint16 safe truncate (or upgrade state_ field type)
    state_.nav_status_flags = static_cast<std::uint16_t>(v.flags & 0xFFFFu);

    // （可选）如果 state_ 有诊断字段，再写入：
    // state_.nav_pub_mono_ns = nav_view_out.pub_mono_ns;
    // state_.nav_pub_wall_ns = nav_view_out.pub_wall_ns;
    // state_.nav_age_ms      = nav_view_out.age_ms_local;

    return true;
}


// IMPORTANT: define ~ControlLoop() here (NavSub complete in this TU)
ControlLoop::~ControlLoop() noexcept = default;

} // namespace rovctrl::control_core
