// gateway/apps/gcs_server.cpp
//
// GCS UDP <-> ControlIntent SHM 桥接程序（精简版）
//
// 职责：
//   - 解析命令行参数（监听 IP/端口、Telemetry 频率、SHM 名称等）；
//   - 初始化 IntentPublisherShm + ShmHexDumper（用于调试）；
//   - 创建 GcsSession + UdpServer；
//   - 启动 Telemetry 线程；
//   - 把“GCS 报文 → ControlIntent SHM”的业务逻辑，全部委托给
//       gateway::app::attach_default_events(...) 等辅助函数；
//
// 具体业务规则（E-STOP / ARM / 模式切换 / 6DOF / MotorTest 等）
// 已在 gateway/apps/gcs_client.cpp 中实现，头文件 gateway/apps/gcs_client.hpp
// 负责描述业务约定和对外接口。

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "gateway/udp/udp_server.hpp"
#include "gateway/session/gcs_session.hpp"

// shm publisher + shared wire intent
#include "gateway/intent_publisher_shm.hpp"
#include "shared/msg/control_intent.hpp"

// Wire protocol
#include "proto_gcs/gcs_protocol.hpp"
#include "gateway/codec/gcs_codec.hpp"

// 新的业务封装接口（你已经放在 include/gateway/apps/gcs_client.hpp）
#include "gateway/apps/gcs_client.hpp"

using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) { g_stop.store(true); }

} // namespace

int main(int argc, char** argv)
{
    std::cout << "[gcs_server] BUILD_ID=2026-01-05-arm-debug" << std::endl;

    comm_gcs::UdpAddress local{"0.0.0.0", 14550};
    int telem_hz = 10;

    // shm config
    std::string intent_shm    = "/rovctrl_gcs_intent_v1";
    int         intent_ttl_ms = 200;

    // debug switches
    bool dbg_layout           = false;
    bool dbg_shm_hex          = false;
    int  dbg_shm_hex_every_ms = 1000;
    int  dbg_shm_hex_max      = 20;

    // ---------------- CLI 解析 ----------------
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) {
            local.port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else if (a == "--ip" && i + 1 < argc) {
            local.ip = argv[++i];
        } else if (a == "--telem-hz" && i + 1 < argc) {
            telem_hz = std::stoi(argv[++i]);
        } else if (a == "--intent-shm" && i + 1 < argc) {
            intent_shm = argv[++i];
        } else if (a == "--intent-ttl-ms" && i + 1 < argc) {
            intent_ttl_ms = std::stoi(argv[++i]);
        } else if (a == "--dbg-layout") {
            dbg_layout = true;
        } else if (a == "--dbg-shm") {
            dbg_shm_hex = true;
        } else if (a == "--dbg-shm-every-ms" && i + 1 < argc) {
            dbg_shm_hex_every_ms = std::stoi(argv[++i]);
        } else if (a == "--dbg-shm-max" && i + 1 < argc) {
            dbg_shm_hex_max = std::stoi(argv[++i]);
        }
        if (a == "--help" || a == "-h") {
            std::cout <<
              "Usage: gcs_server [options]\n"
              "  --ip <addr>\n"
              "  --port <port>\n"
              "  --telem-hz <hz>\n"
              "  --intent-shm <name>\n"
              "  --intent-ttl-ms <ms>\n"
              "  --dbg-layout\n"
              "  --dbg-shm\n"
              "  --dbg-shm-every-ms <ms>\n"
              "  --dbg-shm-max <n>\n";
            return 0;
        }

    }

    if (telem_hz <= 0) telem_hz = 10;

    std::cout << "[gcs_server] listen " << local.ip << ":" << local.port
              << " telem_hz=" << telem_hz << "\n";
    std::cout << "[gcs_server] intent_shm=" << intent_shm
              << " intent_ttl_ms=" << intent_ttl_ms << "\n";

    // ---------------- IntentPublisherShm 初始化 ----------------
    comm_gcs::IntentPublisherShm pub;
    gateway::app::ShmHexDumper   shm_dump;

    {
        comm_gcs::IntentPublisherShm::Config pcfg{};
        pcfg.enable   = true;
        pcfg.shm_name = intent_shm;
        pcfg.shm_size = 4096; // 0 => sizeof(shared::shm::IntentShmLayout)
        if (!pub.init(pcfg)) {
            std::cerr << "[ERR] IntentPublisherShm init failed.\n";
            return 3;
        }
        const std::size_t sz = pub.debug_size();
        std::cout << "[DBG] pub.debug_size=" << sz << "\n";

        if (sz < 512) { // 先用保守阈值，后续再换成 sizeof(真实布局)
            std::cerr << "[FATAL] SHM size too small: " << sz
                      << " bytes, will likely crash on publish.\n";
            return 3;
        }
    }

    if (dbg_layout) {
        gateway::app::dump_layouts_once();
    }

    shm_dump.enable    = dbg_shm_hex;
    shm_dump.every_ms  = dbg_shm_hex_every_ms;
    shm_dump.max_times = dbg_shm_hex_max;

    // ---------------- 会话管理（GcsSession） ----------------
    comm_gcs::session::GcsSessionConfig scfg{};
    scfg.telem_hz                     = telem_hz;
    scfg.require_session_for_commands = true;
    scfg.strict_session_check         = true;

    comm_gcs::session::GcsSessionEvents sev{};

    // 把“GCS 报文 → ControlIntent SHM”的业务回调统一挂上
    
     gateway::app::IntentContext ictx{
        .pub           = pub,
        .shm_dump      = dbg_shm_hex ? &shm_dump : nullptr,
        .intent_ttl_ms = intent_ttl_ms,
        .armed         = false,
        .arm_log_enable = true,   // 或按你自己的习惯
    };
    std::cout << "[DBG] dbg_shm_hex=" << (dbg_shm_hex ? 1 : 0) << "\n";

    // ARM 状态变化日志开关（旧代码用的是 g_arm_log_enable 全局，这里不强行改动原语义）
    // 如果你未来要把它做成配置项，可以在这里同步：
    // ictx.arm_log_enable = true;

    gateway::app::attach_default_events(sev, ictx);


    comm_gcs::session::GcsSession sess(scfg, sev);

    // ---------------- UDP Server ----------------
    comm_gcs::UdpServer       srv;
    comm_gcs::UdpServerConfig cfg{};
    cfg.local           = local;
    cfg.recv_buf_bytes  = 2048;
    cfg.recv_timeout_ms = 50;
    cfg.reuse_addr      = true;

    std::string err;
    const bool ok = srv.start(
        cfg,
        [&](const comm_gcs::UdpAddress& from, comm_gcs::BytesView payload){
             // ★ 新增：调试打印收到的 GCS 报文类型
            // gateway::app::dump_rx_msg_type(payload);

            auto outs = sess.on_packet(from, payload);

            for (auto& pkt : outs) {
                // 对 ACK 报文做十六进制调试输出（集中封装在 gateway::app 里）
                // gateway::app::dump_ack_hex(pkt);

                std::string e2;
                (void)srv.send_to(from,
                                  comm_gcs::BytesView{pkt.data(), pkt.size()},
                                  &e2);
            }
        },
        &err
    );

    if (!ok) {
        std::cerr << "[ERR] UdpServer start failed: " << err << "\n";
        return 2;
    }

    // ---------------- Telemetry 线程 ----------------
    std::thread telem_th([&](){
        const auto period = std::chrono::milliseconds(1000 / telem_hz);
        while (!g_stop.load()) {
            std::this_thread::sleep_for(period);

            const auto& s = sess.state();
            if (!s.have_peer) continue;

            rovctrl::io::gcs::StatusTelemetry stw{};
            stw.session_established = s.established ? 1 : 0;
            stw.link_alive          = s.link_alive  ? 1 : 0;
            stw.estop               = s.estop       ? 1 : 0;

            stw.mode = static_cast<std::uint8_t>(
                s.established ? rovctrl::io::gcs::WireControlMode::Manual
                              : rovctrl::io::gcs::WireControlMode::Unknown
            );

            rovctrl::io::gcs::write_cstr(stw.active_controller,
                                         rovctrl::io::gcs::kCtrlNameMaxLen,
                                         "demo_active");
            rovctrl::io::gcs::write_cstr(stw.desired_controller,
                                         rovctrl::io::gcs::kCtrlNameMaxLen,
                                         "demo_desired");

            stw.consecutive_failures = 0;
            stw.auto_fail_limit      = 0;
            stw.t_ns                 = comm_gcs::codec::now_steady_ns();

            auto pkt_opt = sess.tick_status(stw);
            if (!pkt_opt) continue;

            std::string e3;
            (void)srv.send_to(s.peer,
                              comm_gcs::BytesView{pkt_opt->data(), pkt_opt->size()},
                              &e3);
        }
    });

    std::cout << "[gcs_server] running. Ctrl+C to quit.\n";
    ::signal(SIGINT, on_sigint);

    while (!g_stop.load()) {
        std::this_thread::sleep_for(200ms);
    }

    srv.stop();
    telem_th.join();

    return 0;
}