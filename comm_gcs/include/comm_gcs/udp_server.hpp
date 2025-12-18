#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "comm_gcs/udp_endpoint.hpp"

namespace comm_gcs {

struct UdpServerConfig {
    UdpAddress local;
    std::size_t recv_buf_bytes{2048};
    int recv_timeout_ms{50};      // small timeout helps responsive stop
    bool reuse_addr{true};
};

class UdpServer {
public:
    using PacketHandler = std::function<void(const UdpAddress& from, BytesView payload)>;

    UdpServer() = default;
    ~UdpServer() noexcept;

    UdpServer(const UdpServer&) = delete;
    UdpServer& operator=(const UdpServer&) = delete;

    bool start(const UdpServerConfig& cfg, PacketHandler on_packet, std::string* err = nullptr);
    void stop();

    bool running() const noexcept { return running_.load(); }

private:
    void run_loop_();

private:
    UdpServerConfig cfg_{};
    PacketHandler on_packet_{};

    UdpEndpoint sock_{};
    std::vector<Byte> buf_{};

    std::atomic<bool> running_{false};
    std::thread th_{};
};

} // namespace comm_gcs
