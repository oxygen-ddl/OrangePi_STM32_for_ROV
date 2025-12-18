#pragma once
#ifndef ROVCTRL_IO_GCS_INPUT_ADAPTER_HPP
#define ROVCTRL_IO_GCS_INPUT_ADAPTER_HPP

#include <cstddef>
#include <cstdint>
#include <functional>

#include "io/gcs/gcs_protocol.hpp"
#include "io/gcs/gcs_session.hpp"

// ControlIntent / ControlMode / DofCommand 等
#include "control_core/control_mode.hpp"
#include "control_core/control_types.hpp"
#include "control_core/control_intent.hpp"

namespace rovctrl::io {

// InputProvider 侧实现该接口即可接入
class IIntentSink {
public:
    virtual ~IIntentSink() = default;
    virtual void submit_gcs_intent(const rovctrl::control_core::ControlIntent& intent) = 0;
};

// 发送 raw bytes 的函数：由 app_main 绑定到 GcsLinkUdp::send_to(...)
using SendFn = std::function<void(std::uint32_t ip_be,
                                  std::uint16_t port_be,
                                  const void* data,
                                  std::size_t len)>;

class GcsInputAdapter {
public:
    struct Config {
        bool enable_handshake = true;
        bool enable_ack       = true;

        // 若开启：必须 session_established 才接收命令（更严格）
        bool require_session_for_commands = false;

        // 对重复/旧 seq 是否回应 ACK（建议 true，便于上位机停止重发）
        bool ack_on_dup = true;

        // 收到命令时默认 TTL（ms），若 GCS 不提供 stamp/ttl 可用本值
        std::uint32_t default_ttl_ms = 200;
    };

    GcsInputAdapter(GcsSession& session, IIntentSink& sink, SendFn send)
        : GcsInputAdapter(session, sink, std::move(send), Config{}) {}

    GcsInputAdapter(GcsSession& session, IIntentSink& sink, SendFn send, Config cfg)
        : session_(session), sink_(sink), send_(std::move(send)), cfg_(cfg) {}

    void on_packet(const std::uint8_t* data,
                   std::size_t len,
                   std::uint32_t src_ip_be,
                   std::uint16_t src_port_be,
                   std::uint64_t now_ns);

private:
    static bool read_header(const std::uint8_t* data, std::size_t len,
                            rovctrl::io::gcs::PacketHeader& out);

    bool validate_packet(const rovctrl::io::gcs::PacketHeader& h,
                         const std::uint8_t* payload,
                         std::size_t plen,
                         rovctrl::io::gcs::AckCode& err);

    void maybe_send_ack(UdpPeer peer,
                        const rovctrl::io::gcs::PacketHeader& h,
                        rovctrl::io::gcs::AckCode code);

    void send_ack(UdpPeer peer,
                  std::uint32_t ack_seq,
                  rovctrl::io::gcs::AckCode code);

    // handshake
    void handle_connect_req(UdpPeer peer,
                            const rovctrl::io::gcs::PacketHeader& h,
                            const std::uint8_t* payload,
                            std::size_t plen);

    void handle_connect_confirm(const rovctrl::io::gcs::PacketHeader& h,
                                const std::uint8_t* payload,
                                std::size_t plen);

    // messages
    void handle_heartbeat(std::uint64_t now_ns);

    void handle_set_mode(const rovctrl::io::gcs::PacketHeader& h,
                         const std::uint8_t* payload,
                         std::size_t plen,
                         std::uint64_t now_ns);

    void handle_set_dof(const rovctrl::io::gcs::PacketHeader& h,
                        const std::uint8_t* payload,
                        std::size_t plen,
                        std::uint64_t now_ns);

    void handle_estop(const rovctrl::io::gcs::PacketHeader& h,
                      const std::uint8_t* payload,
                      std::size_t plen,
                      std::uint64_t now_ns);

private:
    GcsSession& session_;
    IIntentSink& sink_;
    SendFn send_;
    Config cfg_;

    // minimal handshake state
    std::uint64_t last_gcs_nonce_ = 0;
    std::uint64_t rov_nonce_      = 0;
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_GCS_INPUT_ADAPTER_HPP
