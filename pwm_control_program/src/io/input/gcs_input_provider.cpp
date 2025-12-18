#include "io/input/gcs_input_provider.hpp"

#include <iostream>
#include <utility> // std::move

namespace rovctrl::io {

// 默认配置构造：避免在头文件内出现 Config{} 相关表达式
GcsInputProvider::GcsInputProvider()
    : GcsInputProvider(Config())
{
}

GcsInputProvider::GcsInputProvider(Config cfg)
    : cfg_(std::move(cfg))
{
}

GcsInputProvider::~GcsInputProvider()
{
    // 先停 RX 线程，避免回调继续访问 adapter_/session_
    link_.stop();

    // 再清理 adapter（回调停了后更安全）
    adapter_.reset();

    initialized_.store(false);
}

bool GcsInputProvider::init()
{
    // 允许重复 init：先停一次，清状态
    link_.stop();
    adapter_.reset();
    session_.reset();
    {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_.reset();
        local_seq_ = 0;
    }
    initialized_.store(false);

    // 1) init UDP
    GcsLinkUdp::Config lc{};
    lc.bind_port    = cfg_.bind_port;
    lc.allow_gcs_ip = cfg_.allow_gcs_ip;
    lc.rx_buf_size  = cfg_.rx_buf_size;

    std::string err;
    if (!link_.init(lc, &err)) {
        std::cerr << "[GcsInputProvider] link.init failed: " << err << "\n";
        return false;
    }

    // 2) build adapter with SendFn -> link_.send_to
    auto sendfn = [this](std::uint32_t ip_be,
                         std::uint16_t port_be,
                         const void*   data,
                         std::size_t   len)
    {
        std::string e;
        if (!link_.send_to(ip_be, port_be, data, len, &e)) {
            session_.inc_tx_fail();
            // 这里不强制打印每次失败，避免刷屏；需要时可打开
            // std::cerr << "[GcsInputProvider] send_to failed: " << e << "\n";
            return;
        }
        session_.inc_tx_ok();
    };

    GcsInputAdapter::Config ac{};
    ac.enable_handshake            = cfg_.enable_handshake;
    ac.enable_ack                  = cfg_.enable_ack;
    ac.require_session_for_commands = cfg_.require_session_for_commands;
    ac.ack_on_dup                  = cfg_.ack_on_dup;
    ac.default_ttl_ms              = cfg_.default_ttl_ms;

    // 注意：构造 adapter 时不再依赖“Config 默认实参”
    adapter_ = std::make_unique<GcsInputAdapter>(session_, *this, std::move(sendfn), ac);

    // 3) start RX thread
    const bool started = link_.start(
        [this](const std::uint8_t* data,
               std::size_t         len,
               std::uint32_t       ip_be,
               std::uint16_t       port_be)
        {
            const std::uint64_t now_ns = now_mono_ns_();

            // 读取 adapter_ 指针（回调运行期间 adapter_ 可能被析构；stop() 之后应不会再进回调）
            auto* ad = adapter_.get();
            if (ad) {
                ad->on_packet(data, len, ip_be, port_be, now_ns);
            }
        },
        &err
    );

    if (!started) {
        std::cerr << "[GcsInputProvider] link.start failed: " << err << "\n";
        adapter_.reset();
        return false;
    }

    initialized_.store(true);
    return true;
}

void GcsInputProvider::reset()
{
    // reset 不停 link；只清内部状态
    {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_.reset();
        local_seq_ = 0;
    }
    session_.reset();
}

void GcsInputProvider::submit_gcs_intent(const rovctrl::control_core::ControlIntent& intent)
{
    std::lock_guard<std::mutex> lk(mtx_);
    latest_ = intent;
}

bool GcsInputProvider::poll(rovctrl::control_core::ControlState&  /*state*/,
                            rovctrl::control_core::ControlIntent& intent)
{
    intent.clear_all();

    // provider 统一填元信息
    intent.seq = ++local_seq_;

    const std::uint64_t now_ns = now_mono_ns_();
    intent.stamp_ns = now_ns;
    intent.ttl_ms   = cfg_.default_ttl_ms;

    if (!initialized_.load()) {
        // 未初始化就输出空 intent（ControlGuard 负责 stale/ttl 行为）
        return true;
    }

    std::optional<rovctrl::control_core::ControlIntent> tmp;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        tmp = latest_;
        if (tmp && !cfg_.keep_last_intent) {
            latest_.reset();
        }
    }

    if (!tmp) return true;

    // 合并 payload：只拷贝“业务字段”，避免混入 adapter 内部的 seq/stamp/ttl
    const auto& g = *tmp;

    intent.request_exit = g.request_exit;

    intent.has_estop_cmd = g.has_estop_cmd;
    intent.estop         = g.estop;
    intent.clear_estop   = g.clear_estop;

    intent.has_arm_cmd   = g.has_arm_cmd;
    intent.arm           = g.arm;
    intent.disarm        = g.disarm;

    intent.has_mode_request = g.has_mode_request;
    intent.mode_request     = g.mode_request;

    intent.has_teleop_dof   = g.has_teleop_dof;
    intent.teleop_dof_cmd   = g.teleop_dof_cmd;

    intent.has_ref          = g.has_ref;
    intent.ref              = g.ref;

    intent.has_ref_delta    = g.has_ref_delta;
    intent.ref_delta        = g.ref_delta;

    return true;
}

} // namespace rovctrl::io
