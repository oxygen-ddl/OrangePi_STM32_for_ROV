#include "io/gcs/gcs_link_udp.hpp"

#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rovctrl::io {

static inline void set_err(std::string* err, const char* msg) {
    if (err) *err = msg;
}
static inline void set_errno_err(std::string* err, const char* msg) {
    if (err) *err = std::string(msg) + ": " + std::strerror(errno);
}

GcsLinkUdp::~GcsLinkUdp() {
    stop();
}

bool GcsLinkUdp::init(const Config& cfg, std::string* err) {
    cfg_ = cfg;

    if (sock_fd_ >= 0) {
        ok_.store(true);
        return true;
    }

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        set_errno_err(err, "socket() failed");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(cfg_.bind_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        set_errno_err(err, "bind() failed");
        close_socket();
        return false;
    }

    ok_.store(true);
    return true;
}

bool GcsLinkUdp::start(RxCallback cb, std::string* err) {
    if (!ok_.load()) {
        set_err(err, "GcsLinkUdp not initialized");
        return false;
    }
    if (running_.load()) {
        return true; // 幂等
    }
    if (!cb) {
        set_err(err, "RxCallback is null");
        return false;
    }

    rx_cb_ = std::move(cb);
    running_.store(true);
    rx_thread_ = std::thread(&GcsLinkUdp::rx_thread_main, this);
    return true;
}

void GcsLinkUdp::stop() {
    running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    close_socket();
    ok_.store(false);
}

void GcsLinkUdp::close_socket() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

bool GcsLinkUdp::send_to(std::uint32_t dst_ip_be,
                         std::uint16_t dst_port_be,
                         const void* data,
                         std::size_t len,
                         std::string* err) {
    if (sock_fd_ < 0) {
        set_err(err, "socket not open");
        return false;
    }

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = dst_ip_be;
    dst.sin_port = dst_port_be;

    ssize_t n = ::sendto(sock_fd_, data, len, 0,
                         reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        if (n >= 0) {
            stats_.tx_packets++;
            stats_.tx_bytes += static_cast<std::uint64_t>(n);
        } else {
            stats_.tx_failed++;
        }
    }

    if (n < 0) {
        set_errno_err(err, "sendto() failed");
        return false;
    }
    return true;
}

bool GcsLinkUdp::send_to_last_peer(const void* data,
                                   std::size_t len,
                                   std::string* err) {
    std::uint32_t ip;
    std::uint16_t port;
    {
        std::lock_guard<std::mutex> lk(peer_mtx_);
        if (!has_peer_) {
            set_err(err, "no last peer");
            return false;
        }
        ip   = peer_ip_be_;
        port = peer_port_be_;
    }
    return send_to(ip, port, data, len, err);
}

bool GcsLinkUdp::has_last_peer() const noexcept {
    std::lock_guard<std::mutex> lk(peer_mtx_);
    return has_peer_;
}

void GcsLinkUdp::get_last_peer(std::uint32_t& ip_be, std::uint16_t& port_be) const {
    std::lock_guard<std::mutex> lk(peer_mtx_);
    ip_be   = peer_ip_be_;
    port_be = peer_port_be_;
}

GcsLinkUdp::Stats GcsLinkUdp::stats() const {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    return stats_;
}

void GcsLinkUdp::rx_thread_main() {
    std::vector<std::uint8_t> buf(cfg_.rx_buf_size);

    while (running_.load()) {
        sockaddr_in src{};
        socklen_t slen = sizeof(src);
        ssize_t n = ::recvfrom(sock_fd_, buf.data(), buf.size(), 0,
                               reinterpret_cast<sockaddr*>(&src), &slen);
        if (n < 0) {
            if (errno == EINTR) continue;
            {
                std::lock_guard<std::mutex> lk(stats_mtx_);
                stats_.rx_dropped++;
            }
            continue;
        }

        // IP 白名单（可选）
        if (!cfg_.allow_gcs_ip.empty()) {
            char ip_str[INET_ADDRSTRLEN]{};
            ::inet_ntop(AF_INET, &src.sin_addr, ip_str, sizeof(ip_str));
            if (cfg_.allow_gcs_ip != ip_str) {
                std::lock_guard<std::mutex> lk(stats_mtx_);
                stats_.rx_dropped++;
                continue;
            }
        }

        {
            std::lock_guard<std::mutex> lk(peer_mtx_);
            has_peer_ = true;
            peer_ip_be_   = src.sin_addr.s_addr;
            peer_port_be_ = src.sin_port;
        }

        {
            std::lock_guard<std::mutex> lk(stats_mtx_);
            stats_.rx_packets++;
            stats_.rx_bytes += static_cast<std::uint64_t>(n);
        }

        // 仅把 raw bytes 交给上层
        rx_cb_(buf.data(), static_cast<std::size_t>(n),
               src.sin_addr.s_addr, src.sin_port);
    }
}

} // namespace rovctrl::io
