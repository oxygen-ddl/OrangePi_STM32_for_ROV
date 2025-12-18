#include "comm_gcs/udp_client.hpp"

#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace comm_gcs {

static inline void set_err(std::string* err, const char* msg) {
    if (!err) return;
    *err = msg;
}

static inline void set_errno_err(std::string* err, const char* msg) {
    if (!err) return;
    *err = std::string(msg) + ": " + std::strerror(errno);
}

bool UdpClient::open(const UdpClientConfig& cfg, std::string* err)
{
    cfg_ = cfg;

    // open socket if not already
    if (!sock_.valid()) {
        if (!sock_.open_sender(err)) return false;
    }

    // bind local if requested
    if (cfg_.bind_local.has_value()) {
        // 注意：UdpEndpoint::bind() 内部也会 open_sender，
        // 但这里 sock_ 已 open 也没问题。
        if (!sock_.bind(*cfg_.bind_local, err)) return false;
    }

    // apply additional socket options
    if (!apply_config_(cfg_, err)) return false;

    // optional connect to default remote
    if (cfg_.connect_default_remote) {
        if (!remote_.has_value()) {
            set_err(err, "connect_default_remote is true but default remote is not set");
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(remote_->port);
        if (::inet_pton(AF_INET, remote_->ip.c_str(), &addr.sin_addr) != 1) {
            set_err(err, "inet_pton failed (invalid default remote ip?)");
            return false;
        }

        if (::connect(sock_.fd(),
                      reinterpret_cast<const sockaddr*>(&addr),
                      sizeof(addr)) < 0) {
            set_errno_err(err, "connect() failed");
            return false;
        }
    }

    return true;
}

bool UdpClient::apply_config_(const UdpClientConfig& cfg, std::string* err)
{
    const int fd = sock_.fd();
    if (fd < 0) {
        set_err(err, "socket not open");
        return false;
    }

    // SO_SNDBUF
    if (cfg.send_buf_bytes > 0) {
        int v = cfg.send_buf_bytes;
        if (::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &v, sizeof(v)) < 0) {
            set_errno_err(err, "setsockopt(SO_SNDBUF) failed");
            return false;
        }
    }

    // IP_TTL (IPv4)
    if (cfg.ttl > 0) {
        int v = cfg.ttl;
        if (::setsockopt(fd, IPPROTO_IP, IP_TTL, &v, sizeof(v)) < 0) {
            set_errno_err(err, "setsockopt(IP_TTL) failed");
            return false;
        }
    }

    // SO_BROADCAST
    if (cfg.enable_broadcast) {
        int v = 1;
        if (::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &v, sizeof(v)) < 0) {
            set_errno_err(err, "setsockopt(SO_BROADCAST) failed");
            return false;
        }
    }

    return true;
}

bool UdpClient::send(BytesView payload, std::string* err)
{
    if (!remote_.has_value()) {
        set_err(err, "default remote not set");
        return false;
    }
    return send_to(*remote_, payload, err);
}

bool UdpClient::send_to(const UdpAddress& remote, BytesView payload, std::string* err)
{
    if (!sock_.valid()) {
        // lazy open with default cfg
        if (!open(cfg_, err)) return false;
    }
    return sock_.send_to(remote, payload, err);
}

} // namespace comm_gcs
