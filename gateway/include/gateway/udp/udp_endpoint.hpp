#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>

#include "gateway/bytes.hpp"

namespace comm_gcs {

struct UdpAddress {
    std::string ip{"127.0.0.1"};
    std::uint16_t port{0};
};

class UdpEndpoint {
public:
    UdpEndpoint() = default;
    ~UdpEndpoint() noexcept;

    UdpEndpoint(const UdpEndpoint&) = delete;
    UdpEndpoint& operator=(const UdpEndpoint&) = delete;

    UdpEndpoint(UdpEndpoint&&) noexcept;
    UdpEndpoint& operator=(UdpEndpoint&&) noexcept;

    // Bind as a receiver (server side)
    bool bind(const UdpAddress& local, std::string* err = nullptr);

    // Open for sending (client side). No bind required; OS assigns ephemeral port.
    bool open_sender(std::string* err = nullptr);

    // Send to remote
    bool send_to(const UdpAddress& remote, BytesView payload, std::string* err = nullptr) const;

    // Receive (non-blocking if configured by server wrapper)
    // Returns payload size, and optionally the sender.
    std::optional<std::size_t> recv_from(std::vector<Byte>& out_buf,
                                         UdpAddress* from = nullptr,
                                         std::string* err = nullptr);

    // Set socket options
    bool set_nonblocking(bool on, std::string* err = nullptr);
    bool set_reuseaddr(bool on, std::string* err = nullptr);
    bool set_recv_timeout_ms(int ms, std::string* err = nullptr);

    int fd() const noexcept { return fd_; }
    bool valid() const noexcept { return fd_ >= 0; }

private:
    int fd_{-1};
};

} // namespace comm_gcs
