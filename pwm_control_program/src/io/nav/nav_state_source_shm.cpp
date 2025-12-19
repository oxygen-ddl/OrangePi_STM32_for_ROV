#include "io/nav/nav_state_source_shm.hpp"

#include <algorithm> // std::copy_n

#include "io/nav/nav_state_subscriber.hpp"
#include "shared/msg/nav_state.hpp"

namespace rovctrl::io {

struct NavStateSourceShm::Impl {
    NavStateSubscriber sub;
};

NavStateSourceShm::NavStateSourceShm(const std::string& name)
    : shm_name_(name)
    , impl_(std::make_unique<Impl>())
{
}

NavStateSourceShm::~NavStateSourceShm() = default;

bool NavStateSourceShm::init()
{
    return impl_ && impl_->sub.init(shm_name_);
}

bool NavStateSourceShm::ok() const noexcept
{
    return impl_ && impl_->sub.ok();
}

bool NavStateSourceShm::read_latest(NavStateView& out)
{
    if (!impl_ || !impl_->sub.ok()) {
        out.valid = false;
        return false;
    }

    shared::msg::NavState nav{};
    if (!impl_->sub.read_latest(nav)) {
        // 当前周期没读到稳定数据：不算致命
        out.valid = false;
        return false;
    }

    out.t_ns = nav.t_ns;

    std::copy_n(nav.pos,     3, out.pos);
    std::copy_n(nav.vel,     3, out.vel);
    std::copy_n(nav.rpy,     3, out.rpy);

    out.depth = nav.depth;

    std::copy_n(nav.omega_b, 3, out.omega_b);
    std::copy_n(nav.acc_b,   3, out.acc_b);

    out.status_flags = nav.status_flags;
    out.health       = static_cast<std::uint8_t>(nav.health);

    // 统一的“有效性”判定：控制侧可据此决定是否使用 nav 数据
    out.valid = (nav.health != shared::msg::NavHealth::UNINITIALIZED) &&
                (nav.health != shared::msg::NavHealth::INVALID);

    return true;
}

void NavStateSourceShm::reset() noexcept
{
    // reset 语义：回到“未初始化/等待 navd”的状态
    if (impl_) {
        impl_->sub.shutdown();
        // 也可以选择保留 shm_name_ 并允许稍后再次 init()
    }
}

} // namespace rovctrl::io
