// comm_gcs/apps/gcs_client.cpp
//
// GCS UDP <-> ControlIntent SHM 桥接程序（模板版）
// - 监听 UDP（自定义 GCS 协议）
// - 使用 GcsSession 做会话管理 / 报文解析
// - 将控制命令转换为 shared::msg::ControlIntent，写入 SHM
// - 发送状态遥测回 GCS
//
// 说明：
//   1) 所有 ControlIntent 构造都集中在本文件顶部的一组 make_*_intent() 辅助函数中，便于后续扩展。
//   2) MotorTestCmd 已贯通：GcsSessionEvents::on_motor_test -> make_motor_test_intent -> SHM。
//   3) 本文件可作为新人改进 GCS/上位机逻辑的代码范例。

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <cstddef>
#include <cstdio>
#include <type_traits>

#include "gateway/udp/udp_server.hpp"
#include "gateway/session/gcs_session.hpp"

// shm publisher + shared wire intent
#include "gateway/intent_publisher_shm.hpp"
#include "shared/msg/control_intent.hpp"

// Wire protocol
#include "proto_gcs/gcs_protocol.hpp"
#include "gateway/codec/gcs_codec.hpp"

using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) { g_stop.store(true); }

// -----------------------------------------------------------------------------
// Layout dump helpers (debug only)
// -----------------------------------------------------------------------------
template <class Ty>
static void dump_layout_once(const char* name)
{
    std::printf("[LAYOUT] %s sizeof=%zu align=%zu\n", name, sizeof(Ty), alignof(Ty));
#define OFF(x) std::printf("  offsetof(%s, %s) = %zu\n", name, #x, offsetof(Ty, x))

    if constexpr (std::is_same_v<Ty, shared::msg::ControlIntent>) {
        OFF(version);
        OFF(flags);
        OFF(cmd_seq);
        OFF(stamp_ns);
        OFF(ttl_ms);

        OFF(request_exit);
        OFF(estop);
        OFF(clear_estop);
        OFF(arm);
        OFF(disarm);

        OFF(pad0);
        OFF(pad1);
        OFF(pad2);

        OFF(mode_request);
        OFF(pad3);

        OFF(teleop_dof_cmd);
        OFF(motor_test);

        OFF(reserved0);
        OFF(reserved1);
    } else if constexpr (std::is_same_v<Ty, shared::msg::DofCommand>) {
        OFF(surge);
        OFF(sway);
        OFF(heave);
        OFF(roll);
        OFF(pitch);
        OFF(yaw);
    }
#undef OFF
}

static inline void dump_bytes_hex(std::ostream& os, const void* p, std::size_t n)
{
    const auto* b = reinterpret_cast<const unsigned char*>(p);
    os << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < n; ++i) {
        os << std::setw(2) << int(b[i]) << " ";
    }
    os << std::dec;
}

struct ShmHexDumper {
    bool enable      = false;
    int  every_ms    = 1000;
    int  max_times   = 20;

    std::chrono::steady_clock::time_point next_tp =
        std::chrono::steady_clock::now();
    int count = 0;

    void maybe_dump(const comm_gcs::IntentPublisherShm& pub,
                    const char* tag = "[SHM_HEX]")
    {
        if (!enable) return;
        if (count >= max_times) return;

        const auto now = std::chrono::steady_clock::now();
        if (now < next_tp) return;

        const void*       p  = pub.debug_ptr();
        const std::size_t sz = pub.debug_size();
        if (!p || sz == 0) return;

        const std::size_t n = (sz < 128) ? sz : 128;
        std::cerr << tag << " size=" << sz << " head" << n << ": ";
        dump_bytes_hex(std::cerr, p, n);
        std::cerr << "\n";

        ++count;
        next_tp = now + std::chrono::milliseconds(every_ms);
    }
};

// -----------------------------------------------------------------------------
// GCS wire <-> shared::msg::ControlMode 映射（只做模式名转换）
// -----------------------------------------------------------------------------
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

static inline shared::msg::ControlMode
map_wire_mode_to_shared(std::uint8_t m) noexcept
{
    using WCM = rovctrl::io::gcs::WireControlMode;
    switch (static_cast<WCM>(m)) {
    case WCM::Manual:   return shared::msg::ControlMode::kManual;
    case WCM::Auto:     return shared::msg::ControlMode::kAuto;
    case WCM::Failsafe: return shared::msg::ControlMode::kNone; // shared 暂无 failsafe
    case WCM::Unknown:
    default:            return shared::msg::ControlMode::kNone;
    }
}

// -----------------------------------------------------------------------------
// 调试：把即将发送的 ACK 包打印成十六进制，便于抓包对比
// -----------------------------------------------------------------------------
static void dump_hex(const char* tag, const std::vector<comm_gcs::Byte>& v)
{
    std::cerr << tag << " (" << v.size() << " bytes): ";
    std::cerr << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < v.size(); ++i) {
        std::cerr << std::setw(2) << int(v[i]) << " ";
    }
    std::cerr << std::dec << "\n";
}

// -----------------------------------------------------------------------------
// ControlIntent 构造辅助函数（模板化部分）
// -----------------------------------------------------------------------------

// 全局 cmd_seq：让 ESTOP / SET_MODE / SET_DOF / MOTOR_TEST 共享同一计数器
static std::atomic<std::uint64_t> g_cmd_seq{0};

static inline std::uint64_t next_cmd_seq() noexcept
{
    return ++g_cmd_seq;
}

// 统一构造基础 Intent（版本号 / 时间戳 / TTL / cmd_seq）
static shared::msg::ControlIntent
make_base_intent(int intent_ttl_ms)
{
    shared::msg::ControlIntent w{};
    w.clear_all(); // 清 flags + payload，但保留结构体布局稳定

    w.version  = shared::msg::kControlIntentWireVersion;
    w.ttl_ms   = static_cast<std::uint32_t>(intent_ttl_ms);
    w.stamp_ns = static_cast<std::uint64_t>(comm_gcs::codec::now_steady_ns());
    w.cmd_seq  = next_cmd_seq();
    return w;
}

// 会话丢失时的“清零 Intent”
// - 不设置任何 flags / payload，只是告诉控制侧：GCS 会话已断开
static shared::msg::ControlIntent
make_session_reset_intent(int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);
    w.clear_payload();  // 清 request_exit / estop / arm / dof / motor_test 等
    return w;
}

// ESTOP 意图
static shared::msg::ControlIntent
make_estop_intent(const rovctrl::io::gcs::EstopCmd& cmd,
                  int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);

    w.flags |= shared::msg::kHasEStopCmd;
    w.estop       = (cmd.enable != 0) ? 1u : 0u;
    w.clear_estop = 0u;

    return w;
}

// 模式切换意图
static shared::msg::ControlIntent
make_set_mode_intent(const rovctrl::io::gcs::SetModeCmd& cmd,
                     int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);

    w.flags       |= shared::msg::kHasModeRequest;
    w.mode_request = map_wire_mode_to_shared(cmd.mode);

    return w;
}

// 手动 6DOF 意图
static shared::msg::ControlIntent
make_set_dof_intent(const rovctrl::io::gcs::SetDofCmd& cmd,
                    int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);

    w.flags |= shared::msg::kHasTeleopDof;
    w.teleop_dof_cmd.surge = cmd.dof[0];
    w.teleop_dof_cmd.sway  = cmd.dof[1];
    w.teleop_dof_cmd.heave = cmd.dof[2];
    w.teleop_dof_cmd.roll  = cmd.dof[3];
    w.teleop_dof_cmd.pitch = cmd.dof[4];
    w.teleop_dof_cmd.yaw   = cmd.dof[5];

    return w;
}

// 单电机测试意图（MotorTestCmd）
// - 由 GCS 界面的“单电机测试”控件触发
// - 控制侧可以按 cmd.duration_ms（例如 2000ms）执行阻塞测试，并自动归零
static shared::msg::ControlIntent
make_motor_test_intent(const rovctrl::io::gcs::MotorTestCmd& cmd,
                       int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);

    w.flags |= shared::msg::kHasMotorTest;

    w.motor_test.enable      = cmd.enable;
    w.motor_test.motor_id    = cmd.motor_id;
    w.motor_test.mode        = cmd.mode;
    w.motor_test.value       = cmd.value;
    w.motor_test.duration_ms = cmd.duration_ms;
    w.motor_test.cmd_id      = cmd.cmd_id;

    return w;
}

// 自测 demo 意图（无需客户端也能往 SHM 写一帧小的 teleop 命令）
static shared::msg::ControlIntent
make_demo_telem_intent(int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);
    w.flags |= shared::msg::kHasTeleopDof;
    w.teleop_dof_cmd.surge = 0.1;  // demo: 轻微前进
    return w;
}

} // namespace

// ============================================================================
// main
// ============================================================================

int main(int argc, char** argv)
{
    comm_gcs::UdpAddress local{"0.0.0.0", 14550};
    int telem_hz = 10;

    // shm config
    std::string intent_shm     = "/rovctrl_gcs_intent_v1";
    int         intent_ttl_ms  = 200;
    bool        test_publish   = false;

    // debug switches
    bool dbg_layout            = false;
    bool dbg_shm_hex           = false;
    int  dbg_shm_hex_every_ms  = 1000;
    int  dbg_shm_hex_max       = 20;

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
        } else if (a == "--test-publish") {
            test_publish = true;
        } else if (a == "--dbg-layout") {
            dbg_layout = true;
        } else if (a == "--dbg-shm") {
            dbg_shm_hex = true;
        } else if (a == "--dbg-shm-every-ms" && i + 1 < argc) {
            dbg_shm_hex_every_ms = std::stoi(argv[++i]);
        } else if (a == "--dbg-shm-max" && i + 1 < argc) {
            dbg_shm_hex_max = std::stoi(argv[++i]);
        }
    }

    if (telem_hz <= 0) telem_hz = 10;

    std::cout << "[gcs_client] listen " << local.ip << ":" << local.port
              << " telem_hz=" << telem_hz << "\n";
    std::cout << "[gcs_client] intent_shm=" << intent_shm
              << " intent_ttl_ms=" << intent_ttl_ms
              << " test_publish=" << (test_publish ? "ON" : "OFF") << "\n";

    // ---------------- IntentPublisherShm 初始化 ----------------
    comm_gcs::IntentPublisherShm pub;
    ShmHexDumper shm_dump;

    {
        comm_gcs::IntentPublisherShm::Config pcfg{};
        pcfg.enable   = true;
        pcfg.shm_name = intent_shm;
        pcfg.shm_size = 0; // 0 => sizeof(shared::shm::IntentShmLayout)
        if (!pub.init(pcfg)) {
            std::cerr << "[ERR] IntentPublisherShm init failed.\n";
            return 3;
        }
    }

    if (dbg_layout) {
        dump_layout_once<shared::msg::ControlIntent>("shared::msg::ControlIntent");
        dump_layout_once<shared::msg::DofCommand>("shared::msg::DofCommand");
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

    // 会话建立
    sev.on_session_established = [&](std::uint64_t sid, const comm_gcs::UdpAddress& peer){
        std::cout << "[SESSION] established, session_id=" << sid
                  << " peer=" << peer.ip << ":" << peer.port << "\n";
    };

    // 会话丢失：往 SHM 写一帧“清零 intent”
    sev.on_session_lost = [&](){
        std::cout << "[SESSION] lost/reset\n";
        auto w = make_session_reset_intent(intent_ttl_ms);
        (void)pub.publish(w);
        shm_dump.maybe_dump(pub, "[SHM_HEX][SESSION_LOST]");
    };

    // 急停
    sev.on_estop = [&](const rovctrl::io::gcs::EstopCmd& cmd){
        std::cout << "[ESTOP] enable=" << int(cmd.enable) << "\n";
        auto w = make_estop_intent(cmd, intent_ttl_ms);
        (void)pub.publish(w);
        shm_dump.maybe_dump(pub, "[SHM_HEX][ESTOP]");
    };

    // 模式切换
    sev.on_set_mode = [&](const rovctrl::io::gcs::SetModeCmd& cmd){
        std::cout << "[SET_MODE] mode=" << mode_to_str(cmd.mode)
                  << " auto_controller=\"" << cmd.auto_controller << "\"\n";
        auto w = make_set_mode_intent(cmd, intent_ttl_ms);
        (void)pub.publish(w);
        shm_dump.maybe_dump(pub, "[SHM_HEX][SET_MODE]");
    };

    // 6DOF 手动控制
    sev.on_set_dof = [&](const rovctrl::io::gcs::SetDofCmd& cmd){
        std::cout << "[SET_DOF] "
                  << "surge=" << cmd.dof[0]
                  << " sway="  << cmd.dof[1]
                  << " heave=" << cmd.dof[2]
                  << " roll="  << cmd.dof[3]
                  << " pitch=" << cmd.dof[4]
                  << " yaw="   << cmd.dof[5]
                  << "\n";

        auto w = make_set_dof_intent(cmd, intent_ttl_ms);
        // ★ 调试：确认写入 SHM 的 ControlIntent 里 DOF 是否正确 ★
        std::cout << "[INTENT_TX] teleop_dof "
                  << "s="  << w.teleop_dof_cmd.surge
                  << " sw=" << w.teleop_dof_cmd.sway
                  << " h="  << w.teleop_dof_cmd.heave
                  << " r="  << w.teleop_dof_cmd.roll
                  << " p="  << w.teleop_dof_cmd.pitch
                  << " y="  << w.teleop_dof_cmd.yaw
                  << " flags=0x" << std::hex << w.flags << std::dec
                  << "\n";
        (void)pub.publish(w);
        shm_dump.maybe_dump(pub, "[SHM_HEX][SET_DOF]");
    };

    // 单电机测试（新功能）
    sev.on_motor_test = [&](const rovctrl::io::gcs::MotorTestCmd& cmd){
        std::cout << "[MOTOR_TEST] motor=" << int(cmd.motor_id)
                  << " enable="      << int(cmd.enable)
                  << " mode="        << int(cmd.mode)
                  << " value="       << cmd.value
                  << " duration_ms=" << cmd.duration_ms
                  << " cmd_id="      << cmd.cmd_id
                  << "\n";

        auto w = make_motor_test_intent(cmd, intent_ttl_ms);
        (void)pub.publish(w);
        shm_dump.maybe_dump(pub, "[SHM_HEX][MOTOR_TEST]");
    };

    comm_gcs::session::GcsSession sess(scfg, sev);

    // ---------------- UDP Server ----------------
    comm_gcs::UdpServer       srv;
    comm_gcs::UdpServerConfig cfg{};
    cfg.local          = local;
    cfg.recv_buf_bytes = 2048;
    cfg.recv_timeout_ms= 50;
    cfg.reuse_addr     = true;

    std::string err;
    const bool ok = srv.start(
        cfg,
        [&](const comm_gcs::UdpAddress& from, comm_gcs::BytesView payload){
            auto outs = sess.on_packet(from, payload);

            for (auto& pkt : outs) {
                // DEBUG: 对即将发出的 ACK 包做一次解析 / hex dump
                {
                    rovctrl::io::gcs::AckCode ec{};
                    std::string              emsg;
                    auto pp = comm_gcs::codec::parse_and_validate(
                        comm_gcs::BytesView{pkt.data(), pkt.size()},
                        &ec,
                        &emsg
                    );
                    if (pp) {
                        auto mt = static_cast<rovctrl::io::gcs::MsgType>(pp->hdr.msg_type);
                        if (mt == rovctrl::io::gcs::MsgType::ACK) {
                            dump_hex("[TX_ACK_HEX]", pkt);
                        }
                    } else {
                        std::cerr << "[WARN] outgoing pkt parse failed, ec="
                                  << static_cast<std::uint16_t>(ec)
                                  << " msg=" << emsg << "\n";
                    }
                }

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

    // ---------------- 可选：自测线程（无客户端时也往 SHM 写 demo Intent） ----------------
    std::thread test_pub_th([&](){
        if (!test_publish) return;
        while (!g_stop.load()) {
            auto w = make_demo_telem_intent(intent_ttl_ms);
            (void)pub.publish(w);
            shm_dump.maybe_dump(pub, "[SHM_HEX][TEST_PUBLISH]");
            std::this_thread::sleep_for(1000ms);
        }
    });

    std::cout << "[gcs_client] running. Ctrl+C to quit.\n";
    ::signal(SIGINT, on_sigint);

    while (!g_stop.load()) {
        std::this_thread::sleep_for(200ms);
    }

    srv.stop();
    telem_th.join();
    if (test_pub_th.joinable()) test_pub_th.join();

    return 0;
}
