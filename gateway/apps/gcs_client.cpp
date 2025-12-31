#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "gateway/udp/udp_endpoint.hpp"
#include "gateway/codec/gcs_codec.hpp"
#include "proto_gcs/gcs_protocol.hpp"

using namespace std::chrono_literals;

namespace {

static std::uint64_t rand_u64()
{
    static thread_local std::mt19937_64 gen{std::random_device{}()};
    return gen();
}

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) { g_stop.store(true); }

struct Args {
    comm_gcs::UdpAddress remote{"127.0.0.1", 14550};

    bool send_heartbeat{true};
    int  hb_hz{2};

    int  handshake_timeout_ms{2000};

    bool do_set_mode{false};
    rovctrl::io::gcs::SetModeCmd mode_cmd{};

    bool do_set_dof{false};
    rovctrl::io::gcs::SetDofCmd dof_cmd{};

    bool do_estop{false};
    rovctrl::io::gcs::EstopCmd estop_cmd{};
};

static void usage()
{
    std::cout
        << "gcs_client options:\n"
        << "  --ip <addr>            remote ip (default 127.0.0.1)\n"
        << "  --port <port>          remote port (default 14550)\n"
        << "  --hb-hz <n>            heartbeat rate (default 2)\n"
        << "  --no-hb                disable heartbeat\n"
        << "  --hs-timeout-ms <ms>   handshake timeout (default 2000)\n"
        << "  --mode <manual|auto|failsafe|unknown>\n"
        << "  --auto-name <name>     for --mode auto\n"
        << "  --dof s sw h r p y      6 floats\n"
        << "  --estop <0|1>\n";
}

static bool parse_args(int argc, char** argv, Args& a)
{
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];

        if (s == "--help" || s == "-h") {
            usage();
            return false;
        } else if (s == "--ip" && i + 1 < argc) {
            a.remote.ip = argv[++i];
        } else if (s == "--port" && i + 1 < argc) {
            a.remote.port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else if (s == "--hb-hz" && i + 1 < argc) {
            a.hb_hz = std::stoi(argv[++i]);
        } else if (s == "--no-hb") {
            a.send_heartbeat = false;
        } else if (s == "--hs-timeout-ms" && i + 1 < argc) {
            a.handshake_timeout_ms = std::stoi(argv[++i]);
        } else if (s == "--mode" && i + 1 < argc) {
            a.do_set_mode = true;
            const std::string m = argv[++i];
            using rovctrl::io::gcs::WireControlMode;
            if (m == "manual")        a.mode_cmd.mode = static_cast<std::uint8_t>(WireControlMode::Manual);
            else if (m == "auto")     a.mode_cmd.mode = static_cast<std::uint8_t>(WireControlMode::Auto);
            else if (m == "failsafe") a.mode_cmd.mode = static_cast<std::uint8_t>(WireControlMode::Failsafe);
            else                      a.mode_cmd.mode = static_cast<std::uint8_t>(WireControlMode::Unknown);
        } else if (s == "--auto-name" && i + 1 < argc) {
            std::string n = argv[++i];
            rovctrl::io::gcs::write_cstr(a.mode_cmd.auto_controller,
                                         rovctrl::io::gcs::kAutoNameMaxLen,
                                         n);
        } else if (s == "--dof" && i + 6 < argc) {
            a.do_set_dof = true;
            for (int k = 0; k < 6; ++k) {
                a.dof_cmd.dof[k] = std::stof(argv[++i]);
            }
        } else if (s == "--estop" && i + 1 < argc) {
            a.do_estop = true;
            int en = std::stoi(argv[++i]);
            a.estop_cmd.enable = (en != 0) ? 1 : 0;
        } else {
            std::cerr << "[ERR] unknown arg: " << s << "\n";
            usage();
            return false;
        }
    }
    return true;
}

static void print_status(const rovctrl::io::gcs::StatusTelemetry& s)
{
    std::cout << "[STATUS] session=" << int(s.session_established)
              << " link=" << int(s.link_alive)
              << " estop=" << int(s.estop)
              << " mode=" << int(s.mode)
              << " active=\"" << s.active_controller << "\""
              << " desired=\"" << s.desired_controller << "\""
              << " t_ns=" << s.t_ns
              << "\n";
}

struct HandshakeResult {
    std::uint64_t session_id{0};
    std::uint64_t rov_nonce{0};
};

static bool send_packet(comm_gcs::UdpEndpoint& sock,
                        const comm_gcs::UdpAddress& remote,
                        const std::vector<comm_gcs::Byte>& pkt)
{
    std::string err;
    const bool ok = sock.send_to(remote, comm_gcs::BytesView{pkt.data(), pkt.size()}, &err);
    if (!ok && !err.empty()) {
        std::cerr << "[ERR] send_to failed: " << err << "\n";
    }
    return ok;
}

template <class T>
static bool send_cmd_pod(comm_gcs::UdpEndpoint& sock,
                         const comm_gcs::UdpAddress& remote,
                         std::uint32_t& seq,
                         std::uint64_t session_id,
                         rovctrl::io::gcs::MsgType mt,
                         const T& pod,
                         bool ack_req = true)
{
    auto p = comm_gcs::codec::to_bytes_vec(pod);
    const std::uint16_t flags = ack_req ? rovctrl::io::gcs::FLAG_ACK_REQ : 0;

    auto h = comm_gcs::codec::make_header(
        static_cast<std::uint8_t>(mt),
        seq++,
        session_id,
        flags,
        static_cast<std::uint32_t>(p.size())
    );

    auto pkt = comm_gcs::codec::build_packet(h, comm_gcs::BytesView{p.data(), p.size()});
    return send_packet(sock, remote, pkt);
}

static std::optional<HandshakeResult> handshake(comm_gcs::UdpEndpoint& sock,
                                                const comm_gcs::UdpAddress& remote,
                                                std::uint32_t& seq,
                                                int timeout_ms)
{
    using namespace rovctrl::io::gcs;

    // CONNECT_REQ
    const std::uint64_t nonce = rand_u64();

    ConnectReq req{};
    req.gcs_nonce = nonce;
    auto pay = comm_gcs::codec::to_bytes_vec(req);

    auto h = comm_gcs::codec::make_header(
        static_cast<std::uint8_t>(MsgType::CONNECT_REQ),
        seq++,
        0,
        FLAG_ACK_REQ,
        static_cast<std::uint32_t>(pay.size())
    );

    auto pkt = comm_gcs::codec::build_packet(h, comm_gcs::BytesView{pay.data(), pay.size()});
    if (!send_packet(sock, remote, pkt)) {
        std::cerr << "[ERR] CONNECT_REQ send failed\n";
        return std::nullopt;
    }
    std::cout << "[TX] CONNECT_REQ\n";

    // Wait CONNECT_ACK
    std::vector<comm_gcs::Byte> rxbuf(2048);
    const auto t0 = std::chrono::steady_clock::now();

    HandshakeResult r{};

    while (!g_stop.load()) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsed > timeout_ms) {
            std::cerr << "[ERR] handshake timeout waiting CONNECT_ACK\n";
            return std::nullopt;
        }

        comm_gcs::UdpAddress from{};
        std::string e;
        auto nopt = sock.recv_from(rxbuf, &from, &e);
        if (!nopt) continue;

        comm_gcs::BytesView bv{rxbuf.data(), *nopt};

        AckCode ec{};
        std::string emsg;
        auto pp = comm_gcs::codec::parse_and_validate(bv, &ec, &emsg);
        if (!pp) continue;

        const auto mt = static_cast<MsgType>(pp->hdr.msg_type);
        if (mt != MsgType::CONNECT_ACK) continue;

        if (!comm_gcs::codec::payload_size_is(pp->payload, sizeof(ConnectAck))) {
            continue;
        }

        ConnectAck ca{};
        std::memcpy(&ca, pp->payload.data, sizeof(ca));
        if (ca.gcs_nonce_echo != nonce) {
            continue;
        }

        r.session_id = pp->hdr.session_id;
        r.rov_nonce  = ca.rov_nonce;

        std::cout << "[RX] CONNECT_ACK session_id=" << r.session_id << "\n";
        if (r.session_id == 0 || r.rov_nonce == 0) {
            std::cerr << "[ERR] bad CONNECT_ACK: session_id/rov_nonce is zero\n";
            return std::nullopt;
        }

        // CONNECT_CONFIRM (ACK_REQ)
        ConnectConfirm cc{};
        cc.rov_nonce_echo = r.rov_nonce;

        auto p2 = comm_gcs::codec::to_bytes_vec(cc);
        auto h2 = comm_gcs::codec::make_header(
            static_cast<std::uint8_t>(MsgType::CONNECT_CONFIRM),
            seq++,
            r.session_id,
            FLAG_ACK_REQ,
            static_cast<std::uint32_t>(p2.size())
        );
        auto pkt2 = comm_gcs::codec::build_packet(h2, comm_gcs::BytesView{p2.data(), p2.size()});
        if (!send_packet(sock, remote, pkt2)) {
            std::cerr << "[ERR] CONNECT_CONFIRM send failed\n";
            return std::nullopt;
        }
        std::cout << "[TX] CONNECT_CONFIRM\n";

        // 可选但强烈建议：等待 ACK(OK)，避免 strict_session_check 下偶发“未建立”
        const auto t1 = std::chrono::steady_clock::now();
        while (!g_stop.load()) {
            const auto el = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t1).count();
            if (el > timeout_ms) {
                std::cerr << "[WARN] timeout waiting ACK for CONNECT_CONFIRM (continue)\n";
                break;
            }

            comm_gcs::UdpAddress from2{};
            std::string e2;
            auto nopt2 = sock.recv_from(rxbuf, &from2, &e2);
            if (!nopt2) continue;

            comm_gcs::BytesView bv2{rxbuf.data(), *nopt2};
            AckCode ec2{};
            std::string emsg2;
            auto pp2 = comm_gcs::codec::parse_and_validate(bv2, &ec2, &emsg2);
            if (!pp2) continue;

            const auto mt2 = static_cast<MsgType>(pp2->hdr.msg_type);
            if (mt2 != MsgType::ACK) continue;

            // ACK 的 code 在 payload，ack_seq 在 header
            if (!comm_gcs::codec::payload_size_is(pp2->payload, sizeof(AckPayload))) continue;

            AckPayload ap{};
            std::memcpy(&ap, pp2->payload.data, sizeof(ap));

            std::cout << "[RX] ACK ack_seq=" << pp2->hdr.ack_seq
                      << " code=" << ap.ack_code << "\n";
            break;
        }

        return r;
    }

    return std::nullopt;
}

} // namespace

int main(int argc, char** argv)
{
    ::signal(SIGINT, on_sigint);

    Args args{};
    if (!parse_args(argc, argv, args)) return 1;

    std::cout << "[gcs_client] remote " << args.remote.ip << ":" << args.remote.port << "\n";

    // Single socket for both TX/RX (ensures replies arrive at the same port).
    comm_gcs::UdpEndpoint sock;
    {
        std::string err;
        if (!sock.open_sender(&err)) {
            std::cerr << "[ERR] open_sender failed: " << err << "\n";
            return 2;
        }
        sock.set_recv_timeout_ms(50, &err);
    }

    std::uint32_t seq = 1;

    // Handshake
    auto hs = handshake(sock, args.remote, seq, args.handshake_timeout_ms);
    if (!hs) {
        std::cerr << "[ERR] handshake failed (no CONNECT_ACK)\n";
        return 3;
    }
    const std::uint64_t session_id = hs->session_id;

    // Optional: send one-shot commands
    if (args.do_estop) {
        send_cmd_pod(sock, args.remote, seq, session_id, rovctrl::io::gcs::MsgType::ESTOP, args.estop_cmd, true);
        std::cout << "[TX] ESTOP enable=" << int(args.estop_cmd.enable) << "\n";
    }
    if (args.do_set_mode) {
        send_cmd_pod(sock, args.remote, seq, session_id, rovctrl::io::gcs::MsgType::SET_MODE, args.mode_cmd, true);
        std::cout << "[TX] SET_MODE mode=" << int(args.mode_cmd.mode) << "\n";
    }
    if (args.do_set_dof) {
        send_cmd_pod(sock, args.remote, seq, session_id, rovctrl::io::gcs::MsgType::SET_DOF_CMD, args.dof_cmd, true);
        std::cout << "[TX] SET_DOF\n";
    }

    // Heartbeat thread
    std::atomic<bool> running{true};
    std::thread hb_th([&](){
        if (!args.send_heartbeat) return;
        const auto period = (args.hb_hz > 0) ? (1000ms / args.hb_hz) : 500ms;

        while (running.load() && !g_stop.load()) {
            std::this_thread::sleep_for(period);

            rovctrl::io::gcs::Heartbeat hb{};
            hb.now_ms = comm_gcs::codec::now_steady_ms();
            auto p = comm_gcs::codec::to_bytes_vec(hb);

            auto h = comm_gcs::codec::make_header(
                static_cast<std::uint8_t>(rovctrl::io::gcs::MsgType::HEARTBEAT),
                seq++,
                session_id,
                0,
                static_cast<std::uint32_t>(p.size())
            );
            auto pkt = comm_gcs::codec::build_packet(h, comm_gcs::BytesView{p.data(), p.size()});
            send_packet(sock, args.remote, pkt);
        }
    });

    std::cout << "[gcs_client] running. Ctrl+C to quit.\n";

    // RX loop
    std::vector<comm_gcs::Byte> rxbuf(2048);

    while (!g_stop.load()) {
        comm_gcs::UdpAddress from{};
        std::string e;
        auto nopt = sock.recv_from(rxbuf, &from, &e);
        if (!nopt) continue;

        comm_gcs::BytesView bv{rxbuf.data(), *nopt};

        rovctrl::io::gcs::AckCode ec{};
        std::string emsg;
        auto pp = comm_gcs::codec::parse_and_validate(bv, &ec, &emsg);
        if (!pp) continue;

        const auto mt = static_cast<rovctrl::io::gcs::MsgType>(pp->hdr.msg_type);

        if (mt == rovctrl::io::gcs::MsgType::ACK) {
            if (comm_gcs::codec::payload_size_is(pp->payload, sizeof(rovctrl::io::gcs::AckPayload))) {
                rovctrl::io::gcs::AckPayload ap{};
                std::memcpy(&ap, pp->payload.data, sizeof(ap));

                // 关键修正：ack_seq 在 header，不在 AckPayload
                std::cout << "[ACK] ack_seq=" << pp->hdr.ack_seq
                          << " code=" << ap.ack_code
                          << " from=" << from.ip << ":" << from.port
                          << "\n";
            }
        } else if (mt == rovctrl::io::gcs::MsgType::STATUS) {
            if (comm_gcs::codec::payload_size_is(pp->payload, sizeof(rovctrl::io::gcs::StatusTelemetry))) {
                rovctrl::io::gcs::StatusTelemetry st{};
                std::memcpy(&st, pp->payload.data, sizeof(st));
                print_status(st);
            }
        } else if (mt == rovctrl::io::gcs::MsgType::CONNECT_ACK) {
            // 可选：如果你运行期间会收到重发/额外的 CONNECT_ACK，这里可以打印一下，方便排障
            std::cout << "[RX] CONNECT_ACK (ignored)\n";
        } else {
            // ignore others
        }
    }

    running.store(false);
    hb_th.join();
    return 0;
}
