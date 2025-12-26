/**
 * @file   control_loop_nav.cpp
 * @brief  NavSub PIMPL + navigation feedback update
 */

#include "control_core/control_loop.hpp"

#include <cstddef>
#include <string>

// cpp 内部依赖：导航订阅器 + NavState
#include "io/nav/nav_state_subscriber.hpp"
#include "shared/msg/nav_state.hpp"

namespace rovctrl::control_core {

// ============================================================================
// NavSub (PIMPL) —— 只在 .cpp 依赖 shared/msg 与共享内存订阅实现
// ============================================================================

struct ControlLoop::NavSub {
    rovctrl::io::NavStateSubscriber sub{};
    std::string shm_name{"/rov_nav_state_v1"};
    bool inited{false};

    bool ensure_init()
    {
        if (inited) return sub.ok();
        inited = sub.init(shm_name);
        return inited && sub.ok();
    }

    bool read_latest(shared::msg::NavState& out)
    {
        if (!ensure_init()) return false;
        return sub.read_latest(out);
    }

    bool ok() const noexcept { return sub.ok(); }
};

// ============================================================================
// custom deleter for unique_ptr<NavSub, NavSubDeleter>
// ============================================================================

void ControlLoop::NavSubDeleter::operator()(NavSub* p) noexcept
{
    delete p;
}

// ============================================================================
// 导航反馈更新（并返回 nav 是否有效）
// ============================================================================

bool ControlLoop::update_nav_feedback_(shared::msg::NavState& nav_out)
{
    state_.nav_valid = false;

    if (!nav_sub_) {
        nav_sub_.reset(new NavSub());
    }

    if (!nav_sub_->read_latest(nav_out)) {
        last_nav_valid_ = false;
        return false;
    }

    last_nav_valid_ = true;

    const auto& nav = nav_out;

    state_.nav_valid = true;
    state_.nav_t_ns  = nav.t_ns;

    for (std::size_t i = 0; i < 3; ++i) {
        state_.nav_pos_ned[i] = nav.pos[i];
        state_.nav_vel_ned[i] = nav.vel[i];
        state_.nav_rpy[i]     = nav.rpy[i];
        state_.nav_omega_b[i] = nav.omega_b[i];
        state_.nav_acc_b[i]   = nav.acc_b[i];
    }

    state_.nav_depth        = nav.depth;
    state_.nav_status_flags = nav.status_flags;

    return true;
}

// ============================================================================
// IMPORTANT: define ~ControlLoop() here (NavSub is complete in this TU)
// ============================================================================

ControlLoop::~ControlLoop() noexcept = default;

} // namespace rovctrl::control_core
