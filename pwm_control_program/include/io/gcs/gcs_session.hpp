#pragma once
#ifndef ROVCTRL_IO_GCS_SESSION_HPP
#define ROVCTRL_IO_GCS_SESSION_HPP

#include <cstdint>
#include <mutex>

namespace rovctrl::io {

// UDP 对端（network byte order: ip_be/port_be）
struct UdpPeer {
    std::uint32_t ip_be   = 0;
    std::uint16_t port_be = 0;

    bool valid() const noexcept { return ip_be != 0 && port_be != 0; }

    friend bool operator==(const UdpPeer& a, const UdpPeer& b) noexcept {
        return a.ip_be == b.ip_be && a.port_be == b.port_be;
    }
    friend bool operator!=(const UdpPeer& a, const UdpPeer& b) noexcept {
        return !(a == b);
    }
};

struct GcsSessionStats {
    std::uint64_t rx_ok          = 0;
    std::uint64_t rx_crc_fail    = 0;
    std::uint64_t rx_bad_format  = 0;
    std::uint64_t rx_bad_session = 0;
    std::uint64_t rx_dup_or_old  = 0;

    std::uint64_t tx_ok          = 0;
    std::uint64_t tx_fail        = 0;
};

struct GcsSessionSnapshot {
    bool session_established = false;
    UdpPeer peer{};

    std::uint64_t last_rx_t_ns  = 0;  // 最近一次“合法包”(adapter判定后)到达时间
    std::uint64_t last_cmd_t_ns = 0;  // 最近一次“命令包”到达时间
    std::uint64_t last_hb_t_ns  = 0;  // 最近一次 heartbeat 到达时间

    std::uint32_t last_seq      = 0;
    bool has_last_seq           = false;

    bool peer_locked            = false;

    GcsSessionStats stats{};
};

class GcsSession {
public:
    struct Config {
        std::uint32_t link_timeout_ms = 800;  // 用于 is_alive() 的事实判定
        bool lock_first_peer          = true; // 首次看到合法包后锁定 peer（除非 reset）
        bool allow_peer_switch        = false;// 是否允许切换 peer（调试场景）
        Config() = default;  // 关键：显式默认构造
    };

    GcsSession();                     // 默认配置：使用 Config 的默认成员初始化器
    explicit GcsSession(Config cfg); // 自定义配置

    // 清空状态（建议在启动/重连/上位机切换时调用）
    void reset();

    // ---- 会话/对端管理（adapter 调用） ----

    // 适配器收到合法包后调用（会更新 peer / last_rx）
    // 返回：是否接受该 peer（若不允许切换，会拒绝不同 peer）
    bool on_rx_packet(UdpPeer peer, std::uint64_t now_ns);

    // 适配器确认这是“命令包”后调用（更新 last_cmd）
    void on_rx_cmd(std::uint64_t now_ns);

    // 适配器确认这是“heartbeat包”后调用（更新 last_hb）
    void on_rx_heartbeat(std::uint64_t now_ns);

    // handshake 状态机在 adapter 中实现；session 只存“是否建立”
    void set_session_established(bool v);
    bool session_established() const;

    // 仅 peer 管理（需要时）
    bool update_peer_if_allowed(UdpPeer peer);
    UdpPeer peer() const;

    // ---- seq 去重（简化：严格单调递增） ----
    // 返回 true：接受；false：重复或旧包
    bool accept_seq(std::uint32_t seq);

    // ---- 链路事实判定（不做控制策略） ----
    // 使用 last_rx（或 last_hb）与 link_timeout_ms 计算
    bool is_alive(std::uint64_t now_ns) const;

    // ---- 统计（adapter/telemetry 调用） ----
    void inc_rx_ok();
    void inc_rx_crc_fail();
    void inc_rx_bad_format();
    void inc_rx_bad_session();
    void inc_rx_dup_or_old();
    void inc_tx_ok();
    void inc_tx_fail();

    GcsSessionSnapshot snapshot() const;

private:
    static std::uint64_t ms_to_ns(std::uint32_t ms) noexcept {
        return static_cast<std::uint64_t>(ms) * 1000ULL * 1000ULL;
    }

private:
    Config cfg_;
    mutable std::mutex mtx_;

    bool session_established_ = false;

    bool peer_locked_ = false;
    UdpPeer peer_{};

    std::uint64_t last_rx_t_ns_  = 0;
    std::uint64_t last_cmd_t_ns_ = 0;
    std::uint64_t last_hb_t_ns_  = 0;

    std::uint32_t last_seq_    = 0;
    bool has_last_seq_         = false;
    

    GcsSessionStats stats_{};
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_GCS_SESSION_HPP
