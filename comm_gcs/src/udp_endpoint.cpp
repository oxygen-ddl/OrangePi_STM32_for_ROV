#include "comm_gcs/udp_endpoint.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
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

UdpEndpoint::~UdpEndpoint() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

UdpEndpoint::UdpEndpoint(UdpEndpoint&& o) noexcept : fd_(o.fd_) { o.fd_ = -1; }
UdpEndpoint& UdpEndpoint::operator=(UdpEndpoint&& o) noexcept {
    if (this == &o) return *this;
    if (fd_ >= 0) ::close(fd_);
    fd_ = o.fd_;
    o.fd_ = -1;
    return *this;
}

bool UdpEndpoint::open_sender(std::string* err) {
    if (fd_ >= 0) return true;
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        set_errno_err(err, "socket() failed");
        return false;
    }
    return true;
}

bool UdpEndpoint::set_reuseaddr(bool on, std::string* err) {
    if (fd_ < 0) { set_err(err, "socket not open"); return false; }
    int v = on ? 1 : 0;
    if (::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &v, sizeof(v)) < 0) {
        set_errno_err(err, "setsockopt(SO_REUSEADDR) failed");
        return false;
    }
    return true;
}

bool UdpEndpoint::set_nonblocking(bool on, std::string* err) {
    if (fd_ < 0) { set_err(err, "socket not open"); return false; }
    int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0) { set_errno_err(err, "fcntl(F_GETFL) failed"); return false; }
    if (on) flags |= O_NONBLOCK;
    else    flags &= ~O_NONBLOCK;
    if (::fcntl(fd_, F_SETFL, flags) < 0) { set_errno_err(err, "fcntl(F_SETFL) failed"); return false; }
    return true;
}

bool UdpEndpoint::set_recv_timeout_ms(int ms, std::string* err) {
    if (fd_ < 0) { set_err(err, "socket not open"); return false; }
    timeval tv{};
    tv.tv_sec  = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    if (::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        set_errno_err(err, "setsockopt(SO_RCVTIMEO) failed");
        return false;
    }
    return true;
}

bool UdpEndpoint::bind(const UdpAddress& local, std::string* err) {
    if (!open_sender(err)) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(local.port);
    if (::inet_pton(AF_INET, local.ip.c_str(), &addr.sin_addr) != 1) {
        set_err(err, "inet_pton failed (invalid ip?)");
        return false;
    }

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        set_errno_err(err, "bind() failed");
        return false;
    }
    return true;
}

bool UdpEndpoint::send_to(const UdpAddress& remote, BytesView payload, std::string* err) {
    if (fd_ < 0) { set_err(err, "socket not open"); return false; }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(remote.port);
    if (::inet_pton(AF_INET, remote.ip.c_str(), &addr.sin_addr) != 1) {
        set_err(err, "inet_pton failed (invalid remote ip?)");
        return false;
    }

    const auto n = ::sendto(fd_,
                            payload.data,
                            payload.size,
                            0,
                            reinterpret_cast<const sockaddr*>(&addr),
                            sizeof(addr));
    if (n < 0) {
        set_errno_err(err, "sendto() failed");
        return false;
    }
    return true;
}

std::optional<std::size_t> UdpEndpoint::recv_from(std::vector<Byte>& out_buf,
                                                  UdpAddress* from,
                                                  std::string* err) {
    if (fd_ < 0) { set_err(err, "socket not open"); return std::nullopt; }
    if (out_buf.empty()) { set_err(err, "recv buffer is empty"); return std::nullopt; }

    sockaddr_in src{};
    socklen_t slen = sizeof(src);

    const auto n = ::recvfrom(fd_,
                              out_buf.data(),
                              out_buf.size(),
                              0,
                              reinterpret_cast<sockaddr*>(&src),
                              &slen);

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return std::nullopt; // timeout / non-blocking no data
        }
        set_errno_err(err, "recvfrom() failed");
        return std::nullopt;
    }

    if (from) {
        char ipbuf[INET_ADDRSTRLEN]{};
        ::inet_ntop(AF_INET, &src.sin_addr, ipbuf, sizeof(ipbuf));
        from->ip = ipbuf;
        from->port = ntohs(src.sin_port);
    }
    return static_cast<std::size_t>(n);
}

} // namespace comm_gcs
