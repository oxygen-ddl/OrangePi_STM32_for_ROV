#include "UdpSender.h"

#include <cerrno>
#include <cstring>
#include <sstream>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

namespace {
inline std::string errnoStr(const char* prefix) {
    std::ostringstream oss;
    oss << prefix << " (errno=" << errno << "): " << std::strerror(errno);
    return oss.str();
}
} // namespace

UdpSender::UdpSender()
    : sockfd_(-1),
      target_port_(0),
      is_initialized_(false),
      peer_(nullptr),
      local_port_(0),
      rcvbuf_bytes_(0),
      sndbuf_bytes_(0) {}

UdpSender::~UdpSender() {
    close();
    // 释放 peer_
    if (peer_) {
        delete peer_;
        peer_ = nullptr;
    }
}

void UdpSender::setLocalBind(const std::string& local_ip, uint16_t local_port) {
    local_ip_   = local_ip;
    local_port_ = local_port;
}

void UdpSender::setSocketBuffers(int rcvbuf_bytes, int sndbuf_bytes) {
    rcvbuf_bytes_ = rcvbuf_bytes;
    sndbuf_bytes_ = sndbuf_bytes;
}

bool UdpSender::setNonBlocking(bool nb) {
    if (sockfd_ < 0) return false;
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags < 0) {
        last_error_ = errnoStr("fcntl(F_GETFL) failed");
        return false;
    }
    if (nb) flags |= O_NONBLOCK;
    else    flags &= ~O_NONBLOCK;
    if (fcntl(sockfd_, F_SETFL, flags) < 0) {
        last_error_ = errnoStr("fcntl(F_SETFL) failed");
        return false;
    }
    return true;
}

bool UdpSender::initialize(const std::string& target_ip, uint16_t target_port, int timeout_ms) {
    close(); // 若已存在则关闭
    if (peer_) { delete peer_; peer_ = nullptr; }

    target_ip_   = target_ip;
    target_port_ = target_port;

    // 1) 创建 socket
    sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        last_error_ = errnoStr("socket(AF_INET, SOCK_DGRAM) failed");
        return false;
    }

    // 2) 可选本地绑定
    if (!bindLocalIfNeeded()) {
        ::close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    // 3) 目标地址缓存
    peer_ = new sockaddr_in();
    std::memset(peer_, 0, sizeof(sockaddr_in));
    peer_->sin_family = AF_INET;
    peer_->sin_port   = htons(target_port_);
    if (::inet_pton(AF_INET, target_ip_.c_str(), &peer_->sin_addr) != 1) {
        last_error_ = "inet_pton failed for target_ip=" + target_ip_;
        ::close(sockfd_);
        sockfd_ = -1;
        delete peer_; peer_ = nullptr;
        return false;
    }

    // 4) 套接字选项
    if (!applySocketOptions(timeout_ms)) {
        ::close(sockfd_);
        sockfd_ = -1;
        delete peer_; peer_ = nullptr;
        return false;
    }

    is_initialized_ = true;
    last_error_.clear();
    return true;
}

bool UdpSender::applySocketOptions(int timeout_ms) {
    // SO_REUSEADDR
    int on = 1;
    if (::setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
        last_error_ = errnoStr("setsockopt(SO_REUSEADDR) failed");
        return false;
    }

    // RCVBUF/SNDBUF（可选）
    if (rcvbuf_bytes_ > 0) {
        if (::setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_bytes_, sizeof(rcvbuf_bytes_)) < 0) {
            last_error_ = errnoStr("setsockopt(SO_RCVBUF) failed");
            return false;
        }
    }
    if (sndbuf_bytes_ > 0) {
        if (::setsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &sndbuf_bytes_, sizeof(sndbuf_bytes_)) < 0) {
            last_error_ = errnoStr("setsockopt(SO_SNDBUF) failed");
            return false;
        }
    }

    // 设置非阻塞（推荐）
    if (!setNonBlocking(true)) {
        // last_error_ 已在 setNonBlocking 内设置
        return false;
    }

    // 可选：设置 SO_RCVTIMEO / SO_SNDTIMEO（阻塞 I/O 时有效；我们默认非阻塞+poll，不强依赖）
    if (timeout_ms > 0) {
        timeval tv{};
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        // 不强制，失败也无所谓
        ::setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        ::setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    return true;
}

bool UdpSender::bindLocalIfNeeded() {
    if (local_ip_.empty() && local_port_ == 0) return true;

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port   = htons(local_port_);
    if (local_ip_.empty()) {
        local.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (::inet_pton(AF_INET, local_ip_.c_str(), &local.sin_addr) != 1) {
            last_error_ = "inet_pton failed for local_ip=" + local_ip_;
            return false;
        }
    }

    if (::bind(sockfd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        last_error_ = errnoStr("bind local failed");
        return false;
    }
    return true;
}

bool UdpSender::sendHexData(const std::vector<uint8_t>& data) {
    return sendRawData(data.data(), data.size());
}

bool UdpSender::sendStringData(const std::string& data) {
    return sendRawData(data.data(), data.size());
}

bool UdpSender::sendRawData(const void* data, size_t data_size) {
    if (!is_initialized_ || sockfd_ < 0 || !peer_) {
        last_error_ = "UdpSender not initialized";
        return false;
    }
    if (data == nullptr || data_size == 0) {
        last_error_ = "sendRawData: empty payload";
        return false;
    }

    ssize_t n = ::sendto(sockfd_, data, data_size, 0,
                         reinterpret_cast<sockaddr*>(peer_), sizeof(*peer_));
    if (n < 0) {
        last_error_ = errnoStr("sendto failed");
        return false;
    }
    if (static_cast<size_t>(n) != data_size) {
        last_error_ = "sendto truncated";
        return false;
    }
    return true;
}

bool UdpSender::receiveData(std::vector<uint8_t>& buffer, int timeout_ms) {
    std::string ip;
    uint16_t port = 0;
    return receiveFrom(buffer, ip, port, timeout_ms);
}

bool UdpSender::receiveDataWithSize(std::vector<uint8_t>& buffer, size_t expected_size, int timeout_ms) {
    if (!receiveData(buffer, timeout_ms)) return false;
    if (buffer.size() != expected_size) {
        last_error_ = "receive size mismatch: expected=" + std::to_string(expected_size)
                    + " actual=" + std::to_string(buffer.size());
        return false;
    }
    return true;
}

bool UdpSender::receiveFrom(std::vector<uint8_t>& buffer, std::string& src_ip, uint16_t& src_port, int timeout_ms) {
    buffer.clear();
    src_ip.clear();
    src_port = 0;

    if (!is_initialized_ || sockfd_ < 0) {
        last_error_ = "UdpSender not initialized";
        return false;
    }

    // poll 短超时轮询
    struct pollfd pfd{};
    pfd.fd = sockfd_;
    pfd.events = POLLIN;

    int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr == 0) {
        // 超时：不是错误
        last_error_.clear();
        return false;
    }
    if (pr < 0) {
        if (errno == EINTR) {
            last_error_.clear();
            return false;
        }
        last_error_ = errnoStr("poll failed");
        return false;
    }

    // 尝试接收
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    std::vector<uint8_t> tmp(2048); // 常见 UDP MTU 足够；现场可调整
    ssize_t n = ::recvfrom(sockfd_, tmp.data(), tmp.size(), 0,
                           reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            last_error_.clear();
            return false;
        }
        last_error_ = errnoStr("recvfrom failed");
        return false;
    }

    tmp.resize(static_cast<size_t>(n));
    buffer.swap(tmp);

    char ipbuf[INET_ADDRSTRLEN] = {0};
    if (::inet_ntop(AF_INET, &from.sin_addr, ipbuf, sizeof(ipbuf))) {
        src_ip = ipbuf;
    }
    src_port = ntohs(from.sin_port);

    last_error_.clear();
    return true;
}

void UdpSender::close() {
    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = -1;
    }
    is_initialized_ = false;
}

std::string UdpSender::getLastError() const {
    return last_error_;
}
