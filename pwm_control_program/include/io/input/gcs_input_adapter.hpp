#pragma once
#ifndef ROVCTRL_IO_INPUT_GCS_INPUT_ADAPTER_HPP
#define ROVCTRL_IO_INPUT_GCS_INPUT_ADAPTER_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>

#include "io/gcs/gcs_protocol.hpp"   // shim -> proto_gcs/gcs_protocol.hpp (PacketHeader/AckCode/MsgType/...)
#include "io/gcs/gcs_link_udp.hpp"   // UdpPeer
#include "io/gcs/gcs_session.hpp"    // GcsSession (rx/tx stats, seq dedup, session policy, etc.)

#include "control_core/control_mode.hpp"
#include "control_core/control_types.hpp"
#include "control_core/control_intent.hpp"

namespace rovctrl::io {

/**
 * @brief Adapter 将解析出的 ControlIntent 交给上层（Provider/mailbox/manager）。
 */
class IIntentSink {
public:
    virtual ~IIntentSink() = default;
    virtual void submit_gcs_intent(const rovctrl::control_core::ControlIntent& intent) = 0;
};

/**
 * @brief 发送 raw bytes 的函数（由 Provider 绑定到 GcsLinkUdp::send_to(...)）。
 * @note ip_be/port_be 均为网络序（big-endian），保持与 UdpPeer 一致。
 */
using SendFn = std::function<void(std::uint32_t ip_be,
                                  std::uint16_t port_be,
                                  const void*   data,
                                  std::size_t   len)>;

/**
 * @brief GCS 输入协议解析器（纯解析/会话/ACK/握手逻辑），不直接持有 socket。
 *
 * 依赖关系：
 * - PacketHeader/AckCode/MsgType/ConnectReq... 来自 rovctrl::io::gcs
 * - UdpPeer 来自 gcs_link_udp.hpp
 * - 会话去重/统计由 GcsSession 提供（你们已有实现）
 */
class GcsInputAdapter final {
public:
    struct Config final {
        bool enable_handshake             = true;
        bool enable_ack                   = true;
        bool require_session_for_commands = false;
        bool ack_on_dup                   = true;

        std::uint32_t default_ttl_ms      = 200;

        Config() = default;
    };

    static_assert(std::is_trivially_copyable<Config>::value,
                  "GcsInputAdapter::Config should remain trivially copyable");

public:
    // 推荐使用：显式给 cfg
    GcsInputAdapter(GcsSession& session, IIntentSink& sink, SendFn send, const Config& cfg);

    // 便捷构造：使用默认 cfg（避免 default-arg cfg = {} 的坑）
    GcsInputAdapter(GcsSession& session, IIntentSink& sink, SendFn send)
        : GcsInputAdapter(session, sink, std::move(send), Config{}) {}

    ~GcsInputAdapter() = default;

    GcsInputAdapter(const GcsInputAdapter&)            = delete;
    GcsInputAdapter& operator=(const GcsInputAdapter&) = delete;
    GcsInputAdapter(GcsInputAdapter&&) noexcept        = default;
    GcsInputAdapter& operator=(GcsInputAdapter&&) noexcept = default;

    /**
     * @brief 收到一个 UDP 数据包（完整 packet：header + payload），进行校验、会话、去重与解析。
     *
     * @param data        UDP payload 起始地址
     * @param len         UDP payload 字节数
     * @param src_ip_be   源 IP（网络序）
     * @param src_port_be 源端口（网络序）
     * @param now_ns      当前时间戳（mono ns，来自 timebase::now_ns）
     */
    void on_packet(const std::uint8_t* data,
                   std::size_t len,
                   std::uint32_t src_ip_be,
                   std::uint16_t src_port_be,
                   std::uint64_t now_ns);

private:
    // ---- helpers ----
    static bool read_header(const std::uint8_t* data, std::size_t len,
                            rovctrl::io::gcs::PacketHeader& out);

    bool validate_packet(const rovctrl::io::gcs::PacketHeader& h,
                         const std::uint8_t* payload,
                         std::size_t plen,
                         rovctrl::io::gcs::AckCode& err) const;

    void maybe_send_ack(const UdpPeer& peer,
                        const rovctrl::io::gcs::PacketHeader& h,
                        rovctrl::io::gcs::AckCode code);

    void send_ack(const UdpPeer& peer,
                  std::uint32_t ack_seq,
                  rovctrl::io::gcs::AckCode code);

    // ---- handshake ----
    void handle_connect_req(const UdpPeer& peer,
                            const rovctrl::io::gcs::PacketHeader& h,
                            const std::uint8_t* payload,
                            std::size_t plen);

    void handle_connect_confirm(const rovctrl::io::gcs::PacketHeader& h,
                                const std::uint8_t* payload,
                                std::size_t plen);

    // ---- heartbeat ----
    void handle_heartbeat(std::uint64_t now_ns);

    // ---- commands ----
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
    GcsSession&  session_;
    IIntentSink& sink_;
    SendFn       send_;
    Config       cfg_{};

    // minimal handshake state
    std::uint64_t last_gcs_nonce_ = 0;
    std::uint64_t rov_nonce_      = 0;
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_INPUT_GCS_INPUT_ADAPTER_HPP
