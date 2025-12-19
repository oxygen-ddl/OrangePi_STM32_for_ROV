#include "io/gcs/gcs_session.hpp"

#include <mutex>   // std::lock_guard
#include <utility> // std::move

namespace rovctrl::io {

namespace {

// 需要在一个锁内完成的 peer 更新逻辑：
// 返回 true 表示接受并更新/保持 peer；false 表示拒绝
static bool update_peer_if_allowed_unlocked(GcsSession::Config const& cfg,
                                            bool&                    peer_locked,
                                            UdpPeer&                 peer_cur,
                                            UdpPeer                  peer_new)
{
    if (!peer_new.valid()) return false;

    // 未锁定：接受并可选择锁定
    if (!peer_locked) {
        peer_cur = peer_new;
        if (cfg.lock_first_peer) {
            peer_locked = true;
        }
        return true;
    }

    // 已锁定：同一 peer 允许
    if (peer_cur == peer_new) return true;

    // 不允许切换则拒绝
    if (!cfg.allow_peer_switch) return false;

    // 允许切换：更新 peer（保持 locked=true）
    peer_cur    = peer_new;
    peer_locked = true;
    return true;
}

} // namespace

// ========================
// Ctors (out-of-line)
// ========================

GcsSession::GcsSession() : cfg_{} {}

GcsSession::GcsSession(Config cfg) : cfg_(std::move(cfg)) {}

// ========================
// State ops
// ========================

void GcsSession::reset()
{
    std::lock_guard<std::mutex> lk(mtx_);
    session_established_ = false;

    peer_locked_ = false;
    peer_        = {};

    last_rx_t_ns_  = 0;
    last_cmd_t_ns_ = 0;
    last_hb_t_ns_  = 0;

    last_seq_     = 0;
    has_last_seq_ = false;

    stats_ = {};
}

bool GcsSession::update_peer_if_allowed(UdpPeer peer)
{
    std::lock_guard<std::mutex> lk(mtx_);
    return update_peer_if_allowed_unlocked(cfg_, peer_locked_, peer_, peer);
}

bool GcsSession::on_rx_packet(UdpPeer peer, std::uint64_t now_ns)
{
    std::lock_guard<std::mutex> lk(mtx_);

    // 一次锁内完成 peer 接受性判定 + last_rx 更新
    if (!update_peer_if_allowed_unlocked(cfg_, peer_locked_, peer_, peer)) {
        return false;
    }

    last_rx_t_ns_ = now_ns;
    return true;
}

void GcsSession::on_rx_cmd(std::uint64_t now_ns)
{
    std::lock_guard<std::mutex> lk(mtx_);
    last_cmd_t_ns_ = now_ns;
}

void GcsSession::on_rx_heartbeat(std::uint64_t now_ns)
{
    std::lock_guard<std::mutex> lk(mtx_);
    last_hb_t_ns_ = now_ns;
}

void GcsSession::set_session_established(bool v)
{
    std::lock_guard<std::mutex> lk(mtx_);
    session_established_ = v;
}

bool GcsSession::session_established() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return session_established_;
}

UdpPeer GcsSession::peer() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return peer_;
}

bool GcsSession::accept_seq(std::uint32_t seq)
{
    std::lock_guard<std::mutex> lk(mtx_);

    // 初次：接受
    if (!has_last_seq_) {
        has_last_seq_ = true;
        last_seq_     = seq;
        return true;
    }

    // 简化策略：严格单调递增
    if (seq > last_seq_) {
        last_seq_ = seq;
        return true;
    }

    // seq <= last_seq_：重复或旧包
    return false;
}

bool GcsSession::is_alive(std::uint64_t now_ns) const
{
    std::lock_guard<std::mutex> lk(mtx_);

    const std::uint64_t timeout_ns = ms_to_ns(cfg_.link_timeout_ms);

    // 优先使用 last_rx（adapter 认为“合法包”才更新 last_rx）
    if (last_rx_t_ns_ != 0) {
        return (now_ns >= last_rx_t_ns_) && ((now_ns - last_rx_t_ns_) <= timeout_ns);
    }

    // 次选 heartbeat
    if (last_hb_t_ns_ != 0) {
        return (now_ns >= last_hb_t_ns_) && ((now_ns - last_hb_t_ns_) <= timeout_ns);
    }

    return false;
}

// ========================
// Stats
// ========================

void GcsSession::inc_rx_ok()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++stats_.rx_ok;
}

void GcsSession::inc_rx_crc_fail()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++stats_.rx_crc_fail;
}

void GcsSession::inc_rx_bad_format()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++stats_.rx_bad_format;
}

void GcsSession::inc_rx_bad_session()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++stats_.rx_bad_session;
}

void GcsSession::inc_rx_dup_or_old()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++stats_.rx_dup_or_old;
}

void GcsSession::inc_tx_ok()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++stats_.tx_ok;
}

void GcsSession::inc_tx_fail()
{
    std::lock_guard<std::mutex> lk(mtx_);
    ++stats_.tx_fail;
}

GcsSessionSnapshot GcsSession::snapshot() const
{
    std::lock_guard<std::mutex> lk(mtx_);

    GcsSessionSnapshot s{};
    s.session_established = session_established_;
    s.peer                = peer_;
    s.peer_locked         = peer_locked_;
    s.last_rx_t_ns        = last_rx_t_ns_;
    s.last_cmd_t_ns       = last_cmd_t_ns_;
    s.last_hb_t_ns        = last_hb_t_ns_;
    s.last_seq            = last_seq_;
    s.has_last_seq        = has_last_seq_;
    s.stats               = stats_;
    return s;
}

} // namespace rovctrl::io
