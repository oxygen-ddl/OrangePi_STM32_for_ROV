#include "io/gcs/gcs_telemetry_adapter.hpp"

#include "controllers/controller_manager.hpp" // 这里拿到完整类型 + status() 定义

#include <utility>

namespace rovctrl::io {

GcsTelemetryAdapter::GcsTelemetryAdapter(const GcsSession& session,
                                         const rovctrl::control_core::ControllerManager& cmgr,
                                         const rovctrl::control_core::ControlGuard& guard,
                                         SendStatusFn send_status,
                                         Config cfg)
    : session_(session)
    , cmgr_(cmgr)
    , guard_(guard)
    , send_status_(std::move(send_status))
    , cfg_(cfg)
{
}

bool GcsTelemetryAdapter::should_send_(std::uint64_t now_ns) const noexcept
{
    if (cfg_.status_hz == 0) return true;
    const std::uint64_t period_ns = static_cast<std::uint64_t>(1000000000ULL / cfg_.status_hz);
    if (last_send_ns_ == 0) return true;
    if (now_ns < last_send_ns_) return true;
    return (now_ns - last_send_ns_) >= period_ns;
}

void GcsTelemetryAdapter::tick(std::uint64_t now_ns)
{
    if (!send_status_) return;
    if (!should_send_(now_ns)) return;

    const auto& st = cmgr_.status();

    TelemetryFrameV1 frame{};
    frame.t_ns = now_ns;

    frame.session_established = session_.session_established();
    frame.link_alive          = session_.is_alive(now_ns);
    frame.estop               = guard_.estop_latched(); // 下面第 3 步加 getter

    frame.mode                 = st.mode;
    frame.active_controller    = st.active_controller;
    frame.desired_controller   = st.desired_controller;
    frame.consecutive_failures = st.consecutive_failures;
    frame.auto_fail_limit      = 0; // 没 getter 就先 0

    rovctrl::io::gcs::StatusTelemetry out{};
    pack_status_telemetry(frame, out);

    send_status_(out);
    last_send_ns_ = now_ns;
}

} // namespace rovctrl::io
