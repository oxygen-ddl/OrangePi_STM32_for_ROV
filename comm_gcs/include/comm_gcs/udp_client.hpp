#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "comm_gcs/udp_endpoint.hpp"
#include "comm_gcs/bytes.hpp"

namespace comm_gcs {

struct UdpClientConfig {
    // 可选：绑定本地地址（指定源端口/网卡）
    // 若不 bind，则由 OS 分配临时端口
    std::optional<UdpAddress> bind_local{std::nullopt};

    // 发送缓冲区（bytes），0 表示不改系统默认
    int send_buf_bytes{0};

    // TTL（IP_TTL），0 表示不设置
    int ttl{0};

    // 是否允许广播（发送到 255.255.255.255 或子网广播）
    bool enable_broadcast{false};

    // 是否 connect 到默认远端（connect 后 sendto 可以变为 send，且可接收 ICMP 错误）
    bool connect_default_remote{false};
};

class UdpClient {
public:
    UdpClient() = default;
    ~UdpClient() noexcept = default;

    UdpClient(const UdpClient&) = delete;
    UdpClient& operator=(const UdpClient&) = delete;

    UdpClient(UdpClient&&) noexcept = default;
    UdpClient& operator=(UdpClient&&) noexcept = default;

    // 打开 socket 并应用配置
    bool open(const UdpClientConfig& cfg = {}, std::string* err = nullptr);

    // 设置默认远端（可选）
    void set_default_remote(const UdpAddress& remote) { remote_ = remote; }
    const std::optional<UdpAddress>& default_remote() const noexcept { return remote_; }

    // 发送到默认远端（必须先 set_default_remote）
    bool send(BytesView payload, std::string* err = nullptr);

    // 发送到指定远端
    bool send_to(const UdpAddress& remote, BytesView payload, std::string* err = nullptr);

    // 便捷：发送 std::vector<uint8_t>
    bool send_vec(const std::vector<Byte>& v, std::string* err = nullptr) {
        return send(BytesView{v.data(), v.size()}, err);
    }
    bool send_vec_to(const UdpAddress& remote, const std::vector<Byte>& v, std::string* err = nullptr) {
        return send_to(remote, BytesView{v.data(), v.size()}, err);
    }

    bool valid() const noexcept { return sock_.valid(); }
    int  fd() const noexcept { return sock_.fd(); }

private:
    bool apply_config_(const UdpClientConfig& cfg, std::string* err);

private:
    UdpEndpoint sock_{};
    std::optional<UdpAddress> remote_{std::nullopt};
    UdpClientConfig cfg_{};
};

} // namespace comm_gcs
