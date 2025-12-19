#pragma once
#ifndef ROVCTRL_IO_GCS_TELEMETRY_ADAPTER_HPP
#define ROVCTRL_IO_GCS_TELEMETRY_ADAPTER_HPP

#include <cstdint>
#include <functional>
#include <utility>

#include "io/gcs/gcs_protocol.hpp"
#include "io/gcs/telemetry_frame.hpp"
#include "io/gcs/gcs_session.hpp"
#include "control_core/control_guard.hpp"

namespace rovctrl::control_core {
class ControllerManager;   // ✅ 正确命名空间：control_core
}

namespace rovctrl::io {

using SendStatusFn = std::function<void(const rovctrl::io::gcs::StatusTelemetry&)>;

class GcsTelemetryAdapter final {
public:
    struct Config {
        std::uint16_t status_hz = 10;
        Config() = default;
    };

    GcsTelemetryAdapter(const GcsSession& session,
                        const rovctrl::control_core::ControllerManager& cmgr,
                        const rovctrl::control_core::ControlGuard& guard,
                        SendStatusFn send_status)
        : GcsTelemetryAdapter(session, cmgr, guard, std::move(send_status), Config{}) {}

    GcsTelemetryAdapter(const GcsSession& session,
                        const rovctrl::control_core::ControllerManager& cmgr,
                        const rovctrl::control_core::ControlGuard& guard,
                        SendStatusFn send_status,
                        Config cfg);

    void tick(std::uint64_t now_ns);

private:
    bool should_send_(std::uint64_t now_ns) const noexcept;

private:
    const GcsSession& session_;
    const rovctrl::control_core::ControllerManager& cmgr_;  // ✅
    const rovctrl::control_core::ControlGuard& guard_;
    SendStatusFn send_status_;
    Config cfg_{};

    std::uint64_t last_send_ns_ = 0;
};

} // namespace rovctrl::io
#endif
