#pragma once
#ifndef ROVCTRL_IO_GCS_LINK_UDP_HPP
#define ROVCTRL_IO_GCS_LINK_UDP_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace rovctrl::io {

/**
 * @brief GCS <-> OrangePi UDP Transport Wrapper (Path A compliant)
 *
 * 只负责：
 *  - UDP socket 打开/关闭
 *  - RX 线程
 *  - 发送 raw bytes
 *  - 维护最近一次对端（last peer）
 *
 * 不负责：
 *  - 协议解析（ACK/CRC/SEQ/HEARTBEAT）
 *  - 会话/握手
 *  - 控制语义（ControlMode / DofCommand）
 *  - mailbox / failsafe 判定
 */
class GcsLinkUdp {
public:
    struct Config {
        std::uint16_t bind_port = 14600;   ///< 本地监听端口
        std::string   allow_gcs_ip;        ///< 可选：白名单 IP（空表示不限制）
        std::size_t   rx_buf_size = 2048;  ///< RX buffer 大小
    };

    struct Stats {
        std::uint64_t rx_packets = 0;
        std::uint64_t rx_bytes   = 0;
        std::uint64_t tx_packets = 0;
        std::uint64_t tx_bytes   = 0;
        std::uint64_t rx_dropped = 0;      ///< 例如长度/系统错误
        std::uint64_t tx_failed  = 0;
    };

    /// 收包回调：仅传 raw bytes + 对端地址（network byte order）
    using RxCallback = std::function<void(const std::uint8_t* data,
                                          std::size_t len,
                                          std::uint32_t src_ip_be,
                                          std::uint16_t src_port_be)>;

public:
    GcsLinkUdp() = default;
    ~GcsLinkUdp();

    GcsLinkUdp(const GcsLinkUdp&)            = delete;
    GcsLinkUdp& operator=(const GcsLinkUdp&) = delete;

    // =========================================================================
    // 生命周期
    // =========================================================================

    /// 初始化 socket（bind 端口），不启动线程
    bool init(const Config& cfg, std::string* err = nullptr);

    /// 启动 RX 线程（幂等）
    bool start(RxCallback cb, std::string* err = nullptr);

    /// 停止 RX 线程并关闭 socket（幂等）
    void stop();

    bool ok() const noexcept { return ok_.load(); }
    bool running() const noexcept { return running_.load(); }

    // =========================================================================
    // 发送
    // =========================================================================

    /// 发送 raw bytes 到指定对端（network byte order）
    bool send_to(std::uint32_t dst_ip_be,
                 std::uint16_t dst_port_be,
                 const void* data,
                 std::size_t len,
                 std::string* err = nullptr);

    /// 发送到最近一次收到数据的对端（若不存在则失败）
    bool send_to_last_peer(const void* data,
                           std::size_t len,
                           std::string* err = nullptr);

    // =========================================================================
    // 对端信息
    // =========================================================================

    bool has_last_peer() const noexcept;
    void get_last_peer(std::uint32_t& ip_be, std::uint16_t& port_be) const;

    // =========================================================================
    // 统计
    // =========================================================================

    Stats stats() const;

private:
    void rx_thread_main();
    void close_socket();

private:
    Config cfg_{};

    std::atomic_bool ok_{false};
    std::atomic_bool running_{false};
    std::thread      rx_thread_;

    int sock_fd_ = -1;

    // 回调
    RxCallback rx_cb_{};

    // 最近一次对端
    mutable std::mutex peer_mtx_;
    bool       has_peer_{false};
    std::uint32_t peer_ip_be_{0};
    std::uint16_t peer_port_be_{0};

    // 统计
    mutable std::mutex stats_mtx_;
    Stats stats_{};
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_GCS_LINK_UDP_HPP
