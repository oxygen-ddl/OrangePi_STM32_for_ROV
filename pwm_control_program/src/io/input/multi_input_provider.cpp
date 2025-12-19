#include "io/input/multi_input_provider.hpp"

#include "platform/timebase.hpp"  // now_ns()

#include <utility>  // std::move

namespace rovctrl::io {
namespace cc = rovctrl::control_core;

MultiInputProvider::MultiInputProvider(InputProviderPtr teleop,
                                       InputProviderPtr gcs)
    : MultiInputProvider(std::move(teleop), std::move(gcs), Config{})
{
}

MultiInputProvider::MultiInputProvider(InputProviderPtr teleop,
                                       InputProviderPtr gcs,
                                       Config cfg)
    : teleop_(std::move(teleop))
    , gcs_(std::move(gcs))
    , cfg_(cfg)
{
}

bool MultiInputProvider::init()
{
    bool ok = true;
    if (teleop_) ok = teleop_->init() && ok;
    if (gcs_)    ok = gcs_->init() && ok;
    return ok;
}

void MultiInputProvider::reset()
{
    if (teleop_) teleop_->reset();
    if (gcs_)    gcs_->reset();
    seq_ = 0;
}

bool MultiInputProvider::has_payload_(const cc::ControlIntent& in) noexcept
{
    return in.request_exit ||
           in.has_estop_cmd ||
           in.has_arm_cmd ||
           in.has_mode_request ||
           in.has_teleop_dof ||
           in.has_ref ||
           in.has_ref_delta;
}

bool MultiInputProvider::poll(cc::ControlState& state, cc::ControlIntent& out)
{
    cc::ControlIntent t{};
    cc::ControlIntent g{};

    if (teleop_ && !teleop_->poll(state, t)) return false;
    if (gcs_    && !gcs_->poll(state, g))    return false;

    const bool t_has = has_payload_(t);
    const bool g_has = has_payload_(g);

    const cc::ControlIntent* primary   = nullptr;
    const cc::ControlIntent* secondary = nullptr;

    if (cfg_.gcs_priority) {
        primary   = g_has ? &g : (t_has ? &t : nullptr);
        secondary = (primary == &g) ? &t : &g;
    } else {
        primary   = t_has ? &t : (g_has ? &g : nullptr);
        secondary = (primary == &t) ? &g : &t;
    }

    out.clear_all();
    out.seq      = ++seq_;
    out.stamp_ns = static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());
    out.ttl_ms   = cfg_.default_ttl_ms;

    if (!primary) {
        return true; // no payload
    }

    // 1) 安全类字段：primary + secondary 取并集（更安全）
    out.request_exit = primary->request_exit || secondary->request_exit;

    if (primary->has_estop_cmd || secondary->has_estop_cmd) {
        out.has_estop_cmd = true;
        out.estop         = primary->estop       || secondary->estop;
        out.clear_estop   = primary->clear_estop || secondary->clear_estop;
    }

    if (primary->has_arm_cmd || secondary->has_arm_cmd) {
        out.has_arm_cmd = true;
        out.arm         = primary->arm    || secondary->arm;
        out.disarm      = primary->disarm || secondary->disarm;
    }

    // 2) 载荷类字段：以 primary 为主（避免“同时控制”混乱）
    if (primary->has_mode_request) {
        out.has_mode_request = true;
        out.mode_request     = primary->mode_request;
    }

    if (primary->has_ref) {
        out.has_ref = true;
        out.ref     = primary->ref;
    }

    if (primary->has_ref_delta) {
        out.has_ref_delta = true;
        out.ref_delta     = primary->ref_delta;
    }

    if (primary->has_teleop_dof) {
        out.has_teleop_dof = true;
        out.teleop_dof_cmd = primary->teleop_dof_cmd;
    }

    return true;
}

} // namespace rovctrl::io
