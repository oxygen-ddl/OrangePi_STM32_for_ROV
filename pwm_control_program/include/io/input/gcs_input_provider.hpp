#pragma once
#ifndef ROVCTRL_IO_GCS_INPUT_PROVIDER_HPP
#define ROVCTRL_IO_GCS_INPUT_PROVIDER_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <memory>

#include "io/input/input_provider.hpp"
#include "io/gcs/gcs_link_udp.hpp"
#include "io/gcs/gcs_session.hpp"
#include "io/input/gcs_input_adapter.hpp"

#include "platform/timebase.hpp"              // now_ns()
#include "control_core/control_intent.hpp"

namespace rovctrl::io {

class GcsInputProvider final : public IInputProvider, public IIntentSink {
public:
    struct Config {
        // UDP link
        std::uint16_t bind_port = 14600;
        std::string   allow_gcs_ip;     // optional whitelist
        std::size_t   rx_buf_size = 2048;

        // adapter/session policy
        bool enable_handshake = true;
        bool enable_ack       = true;
        bool require_session_for_commands = false;
        bool ack_on_dup        = true;

        std::uint32_t default_ttl_ms = 200;

        // poll behavior
        bool keep_last_intent = true;  // true: 持续输出最后一条命令；false: poll 后消费清空

        Config() = default;  // 关键：显式默认构造
        
    };

    GcsInputProvider();
    explicit GcsInputProvider(Config cfg);
    ~GcsInputProvider() override;

    bool init() override;
    bool poll(rovctrl::control_core::ControlState&  state,
              rovctrl::control_core::ControlIntent& intent) override;
    void reset() override;

    // IIntentSink
    void submit_gcs_intent(const rovctrl::control_core::ControlIntent& intent) override;

    bool ok() const noexcept { return initialized_.load(); }

private:
    static std::uint64_t now_mono_ns_() {
        return static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());
    }

private:
    Config cfg_;

    std::atomic_bool initialized_{false};

    // Link + session + adapter
    GcsSession session_{};
    GcsLinkUdp link_{};
    std::unique_ptr<GcsInputAdapter> adapter_{};

    // Latest intent mailbox
    mutable std::mutex mtx_;
    std::optional<rovctrl::control_core::ControlIntent> latest_;
    std::uint64_t local_seq_{0};
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_GCS_INPUT_PROVIDER_HPP
