#include "comm_gcs/udp_server.hpp"

#include <utility>

namespace comm_gcs {

UdpServer::~UdpServer() noexcept {
    stop();
}

bool UdpServer::start(const UdpServerConfig& cfg, PacketHandler on_packet, std::string* err) {
    if (running_.load()) {
        if (err) *err = "UdpServer already running";
        return false;
    }
    if (!on_packet) {
        if (err) *err = "on_packet callback is empty";
        return false;
    }

    cfg_ = cfg;
    on_packet_ = std::move(on_packet);
    buf_.assign(cfg_.recv_buf_bytes, Byte{0});

    std::string local_err;
    if (!sock_.open_sender(&local_err)) {
        if (err) *err = local_err;
        return false;
    }
    if (cfg_.reuse_addr) {
        sock_.set_reuseaddr(true, &local_err); // best effort
    }
    sock_.set_recv_timeout_ms(cfg_.recv_timeout_ms, &local_err); // best effort

    if (!sock_.bind(cfg_.local, &local_err)) {
        if (err) *err = local_err;
        return false;
    }

    running_.store(true);
    th_ = std::thread([this]() { run_loop_(); });
    return true;
}

void UdpServer::stop() {
    const bool was = running_.exchange(false);
    if (was) {
        if (th_.joinable()) th_.join();
    }
}

void UdpServer::run_loop_() {
    while (running_.load()) {
        UdpAddress from{};
        std::string err;
        auto nopt = sock_.recv_from(buf_, &from, &err);
        if (!nopt.has_value()) {
            continue; // timeout/no data or transient
        }
        const std::size_t n = *nopt;
        if (n == 0) continue;

        BytesView view{buf_.data(), n};
        on_packet_(from, view);
    }
}

} // namespace comm_gcs
