#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "comm_gcs/udp_client.hpp"
#include "comm_gcs/udp_server.hpp"
#include "comm_gcs/session/gcs_session.hpp"

// Wire protocol (POD + constants + helpers like write_cstr)
#include "proto_gcs/gcs_protocol.hpp"

// now_steady_ns() is in your comm_gcs codec module
#include "comm_gcs/codec/gcs_codec.hpp"

using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) { g_stop.store(true); }

static inline std::string mode_to_str(std::uint8_t m)
{
    using rovctrl::io::gcs::WireControlMode;
    switch (static_cast<WireControlMode>(m)) {
    case WireControlMode::Manual:   return "Manual";
    case WireControlMode::Auto:     return "Auto";
    case WireControlMode::Failsafe: return "Failsafe";
    case WireControlMode::Unknown:
    default:                        return "Unknown";
    }
}

} // namespace

int main(int argc, char** argv)
{
    comm_gcs::UdpAddress local{"0.0.0.0", 14550};
    int telem_hz = 10;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) {
            local.port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else if (a == "--ip" && i + 1 < argc) {
            local.ip = argv[++i];
        } else if (a == "--telem-hz" && i + 1 < argc) {
            telem_hz = std::stoi(argv[++i]);
        }
    }

    std::cout << "[gcs_server] listen " << local.ip << ":" << local.port
              << " telem_hz=" << telem_hz << "\n";

    // TX client: used to send replies/telemetry to last peer
    comm_gcs::UdpClient tx;
    {
        std::string err;
        if (!tx.open({}, &err)) {
            std::cerr << "[ERR] UdpClient open failed: " << err << "\n";
            return 1;
        }
    }

    // Session + event handlers
    comm_gcs::session::GcsSessionConfig scfg{};
    scfg.telem_hz = telem_hz;
    scfg.require_session_for_commands = true;
    scfg.strict_session_check = true;

    comm_gcs::session::GcsSessionEvents sev{};

    sev.on_session_established = [&](std::uint64_t sid){
        std::cout << "[SESSION] established, session_id=" << sid << "\n";
    };
    sev.on_session_lost = [&](){
        std::cout << "[SESSION] lost/reset\n";
    };

    sev.on_estop = [&](const rovctrl::io::gcs::EstopCmd& cmd){
        std::cout << "[ESTOP] enable=" << int(cmd.enable) << "\n";
    };

    sev.on_set_mode = [&](const rovctrl::io::gcs::SetModeCmd& cmd){
        std::cout << "[SET_MODE] mode=" << mode_to_str(cmd.mode)
                  << " auto_controller=\"" << cmd.auto_controller << "\"\n";
    };

    sev.on_set_dof = [&](const rovctrl::io::gcs::SetDofCmd& cmd){
        std::cout << "[SET_DOF] "
                  << "surge=" << cmd.dof[0] << " sway=" << cmd.dof[1] << " heave=" << cmd.dof[2]
                  << " roll=" << cmd.dof[3] << " pitch=" << cmd.dof[4] << " yaw=" << cmd.dof[5]
                  << "\n";
    };

    comm_gcs::session::GcsSession sess(scfg, sev);

    // RX server
    comm_gcs::UdpServer srv;
    comm_gcs::UdpServerConfig cfg{};
    cfg.local = local;
    cfg.recv_buf_bytes = 2048;
    cfg.recv_timeout_ms = 50;
    cfg.reuse_addr = true;

    std::string err;
    const bool ok = srv.start(
        cfg,
        [&](const comm_gcs::UdpAddress& from, comm_gcs::BytesView payload){
            // Give packet to session (handshake/dedup/ACK routing happens inside)
            auto outs = sess.on_packet(from, payload);

            // Send responses back to the sender
            for (auto& pkt : outs) {
                std::string e2;
                (void)tx.send_to(from, comm_gcs::BytesView{pkt.data(), pkt.size()}, &e2);
            }
        },
        &err
    );

    if (!ok) {
        std::cerr << "[ERR] UdpServer start failed: " << err << "\n";
        return 2;
    }

    // Telemetry thread: tick_status() does rate limiting internally (telem_hz)
    std::thread telem_th([&](){
        while (!g_stop.load()) {
            std::this_thread::sleep_for(20ms); // 50Hz tick; sess.tick_status enforces telem_hz

            const auto& s = sess.state();
            if (!s.have_peer) continue;

            // Build wire-level StatusTelemetry directly (NO cross-project includes)
            rovctrl::io::gcs::StatusTelemetry stw{};
            stw.session_established = s.established ? 1 : 0;
            stw.link_alive          = s.link_alive  ? 1 : 0;
            stw.estop               = s.estop       ? 1 : 0;

            // Demo: report Manual when established, Unknown otherwise
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
            stw.t_ns = comm_gcs::codec::now_steady_ns();

            // Session decides whether to emit STATUS (rate-limited)
            auto pkt_opt = sess.tick_status(stw);
            if (!pkt_opt) continue;

            std::string e3;
            (void)tx.send_to(s.peer, comm_gcs::BytesView{pkt_opt->data(), pkt_opt->size()}, &e3);
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
