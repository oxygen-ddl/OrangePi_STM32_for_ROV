// gateway/apps/gcs_client.cpp
//
// 实现：
//   - gateway::app::ShmHexDumper::maybe_dump
//   - gateway::app::dump_layouts_once
//   - gateway::app::dump_ack_hex
//   - gateway::app::attach_default_events
//
// 说明：
//   1) 这里集中实现“GCS UDP 会话事件 → ControlIntent SHM”的业务逻辑；
//   2) gcs_server.cpp 只需要：
//        - 初始化 IntentPublisherShm / ShmHexDumper / IntentContext；
//        - 调用 attach_default_events(sev, ictx)；
//        - 在发送 UDP ACK 时调用 dump_ack_hex(pkt)；
//      即可把大部分繁琐逻辑移出 main()。

#include "gateway/apps/gcs_client.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <type_traits>

namespace gateway::app {

using rovctrl::io::gcs::AckCode;
using rovctrl::io::gcs::MsgType;
using rovctrl::io::gcs::WireControlMode;

using rovctrl::io::gcs::EstopCmd;
using rovctrl::io::gcs::ArmCmd;
using rovctrl::io::gcs::SetModeCmd;
using rovctrl::io::gcs::SetDofCmd;
using rovctrl::io::gcs::MotorTestCmd;

using shared::msg::ControlIntent;
using shared::msg::DofCommand;

// =============================
// 内部工具函数（仅本文件使用）
// =============================

namespace {

// ★ 网关本地“解锁状态”视图：
//   - true  : 最近一次 ARM 命令是 enable=1
//   - false : 初始 / 最近一次 ARM 是 enable=0 / 急停等原因本地强制上锁
bool g_armed = false;

// ★ 是否打印 ARM 状态变化相关日志（如需静默可以后面改成 false 或做成 CLI 开关）
bool g_arm_log_enable = true;

template <class Ty>
void dump_layout_once_impl(const char* name)
{
    std::printf("[LAYOUT] %s sizeof=%zu align=%zu\n", name, sizeof(Ty), alignof(Ty));
#define OFF(x) std::printf("  offsetof(%s, %s) = %zu\n", name, #x, offsetof(Ty, x))

    if constexpr (std::is_same_v<Ty, ControlIntent>) {
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
    } else if constexpr (std::is_same_v<Ty, DofCommand>) {
        OFF(surge);
        OFF(sway);
        OFF(heave);
        OFF(roll);
        OFF(pitch);
        OFF(yaw);
    }
#undef OFF
}


inline void dump_bytes_hex(std::ostream& os, const void* p, std::size_t n)
{
    const auto* b = static_cast<const unsigned char*>(p);
    os << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < n; ++i) {
        os << std::setw(2) << int(b[i]) << " ";
    }
    os << std::dec;
}

// GCS wire 模式枚举 → 字符串，仅用于日志
inline std::string mode_to_str(std::uint8_t m)
{
    switch (static_cast<WireControlMode>(m)) {
    case WireControlMode::Manual:   return "Manual";
    case WireControlMode::Auto:     return "Auto";
    case WireControlMode::Failsafe: return "Failsafe";
    case WireControlMode::Unknown:
    default:                        return "Unknown";
    }
}

// wire ControlMode → shared ControlMode
static inline shared::msg::ControlMode
map_wire_mode_to_shared(std::uint8_t m) noexcept
{
    using WCM = WireControlMode;
    using WM  = shared::msg::ControlMode;

    switch (static_cast<WCM>(m)) {
    case WCM::Manual:   return WM::kManual;
    case WCM::Auto:     return WM::kAuto;
    case WCM::Failsafe: return WM::kNone;  // shared 侧暂无 failsafe，先映射到 kNone
    case WCM::Unknown:
    default:            return WM::kNone;
    }
}

// 全局 cmd_seq：让 ESTOP / SET_MODE / SET_DOF / ARM / MOTOR_TEST 共享同一计数器
static std::atomic<std::uint64_t> g_cmd_seq{0};

static inline std::uint64_t next_cmd_seq() noexcept
{
    return ++g_cmd_seq;
}

// 统一构造基础 Intent（版本号 / 时间戳 / TTL / cmd_seq）
static ControlIntent make_base_intent(int intent_ttl_ms)
{
    ControlIntent w{};
    w.clear_all();

    w.version  = shared::msg::kControlIntentWireVersion;
    w.ttl_ms   = static_cast<std::uint32_t>(intent_ttl_ms);
    w.stamp_ns = static_cast<std::uint64_t>(comm_gcs::codec::now_steady_ns());
    w.cmd_seq  = next_cmd_seq();
    return w;
}

// 会话丢失时的“清零 Intent”
static ControlIntent make_session_reset_intent(int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);
    w.clear_payload();  // 清 request_exit / estop / arm / dof / motor_test 等
    return w;
}

// ESTOP 意图
static ControlIntent make_estop_intent(const EstopCmd& cmd, int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);

    w.flags |= shared::msg::kHasEStopCmd;
    w.estop       = (cmd.enable != 0) ? 1u : 0u;
    w.clear_estop = 0u;

    return w;
}

// ARM / DISARM 意图
static ControlIntent make_arm_intent(const ArmCmd& cmd, int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);

    w.flags |= shared::msg::kHasArmCmd;

    if (cmd.enable) {
        w.arm    = 1u;
        w.disarm = 0u;
    } else {
        w.arm    = 0u;
        w.disarm = 1u;
    }

    return w;
}

// 模式切换意图（不再隐含 Arm 语义）
static ControlIntent make_set_mode_intent(const SetModeCmd& cmd, int intent_ttl_ms)
{
    auto w = make_base_intent(intent_ttl_ms);

    w.flags        |= shared::msg::kHasModeRequest;
    w.mode_request  = map_wire_mode_to_shared(cmd.mode);

    return w;
}

// 手动 6DOF 意图
static ControlIntent make_set_dof_intent(const SetDofCmd& cmd, int intent_ttl_ms)
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

// 单电机测试意图
static ControlIntent make_motor_test_intent(const MotorTestCmd& cmd, int intent_ttl_ms)
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

} // namespace

// =============================
// ShmHexDumper 实现
// =============================

void ShmHexDumper::maybe_dump(const comm_gcs::IntentPublisherShm& pub,
                              const char* tag)
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

// =============================
// 调试工具函数实现
// =============================

void dump_layouts_once()
{
    dump_layout_once_impl<ControlIntent>("shared::msg::ControlIntent");
    dump_layout_once_impl<DofCommand>("shared::msg::DofCommand");
}

void dump_ack_hex(const std::vector<comm_gcs::Byte>& pkt)
{
    // 只对 MsgType::ACK 做解析 + hex dump，其余包静默
    rovctrl::io::gcs::AckCode ec{};
    std::string emsg;

    auto pp = comm_gcs::codec::parse_and_validate(
        comm_gcs::BytesView{pkt.data(), pkt.size()},
        &ec,
        &emsg
    );
    if (!pp) {
        std::cerr << "[WARN] outgoing pkt parse failed, ec="
                  << static_cast<std::uint16_t>(ec)
                  << " msg=" << emsg << "\n";
        return;
    }

    auto mt = static_cast<MsgType>(pp->hdr.msg_type);
    if (mt != MsgType::ACK) {
        return;
    }

    std::cerr << "[TX_ACK_HEX] (" << pkt.size() << " bytes): ";
    dump_bytes_hex(std::cerr, pkt.data(), pkt.size());
    std::cerr << "\n";
}
// ===================== 新增：dump_rx_msg_type 实现 =====================
void dump_rx_msg_type(const comm_gcs::BytesView& payload)
{
    using rovctrl::io::gcs::MsgType;
    using rovctrl::io::gcs::AckCode;

    AckCode ec{};
    std::string emsg;
    auto parsed = comm_gcs::codec::parse_and_validate(payload, &ec, &emsg);
    if (!parsed) {
        std::cerr << "[RX] parse failed ec="
                  << static_cast<std::uint16_t>(ec)
                  << " msg=" << emsg << "\n";
        return;
    }

    auto mt = static_cast<MsgType>(parsed->hdr.msg_type);
    std::cout << "[RX] msg_type=" << static_cast<int>(mt)
              << " len=" << parsed->hdr.payload_len << "\n";
}


// =============================
// 事件绑定实现
// =============================

void attach_default_events(comm_gcs::session::GcsSessionEvents& sev,
                           const IntentContext& ictx)
{
    auto& pub   = ictx.pub;
    auto* dump  = ictx.shm_dump;
    const int ttl_ms = ictx.intent_ttl_ms;

    // 会话建立
    sev.on_session_established = [&](std::uint64_t sid, const comm_gcs::UdpAddress& peer){
        std::cout << "[SESSION] established, session_id=" << sid
                  << " peer=" << peer.ip << ":" << peer.port << "\n";
    };

    // 会话丢失：写一帧“清零 Intent”
    sev.on_session_lost = [&](){
        std::cout << "[SESSION] lost/reset\n";
        auto w = make_session_reset_intent(ttl_ms);
        (void)pub.publish(w);
        if (dump) dump->maybe_dump(pub, "[SHM_HEX][SESSION_LOST]");
    };

    // 急停
    sev.on_estop = [&](const EstopCmd& cmd){
        std::cout << "[ESTOP] enable=" << int(cmd.enable) << "\n";
        auto w = make_estop_intent(cmd, ttl_ms);
        (void)pub.publish(w);
        if (dump) dump->maybe_dump(pub, "[SHM_HEX][ESTOP]");
    };

    // ARM / DISARM
    sev.on_arm = [&](const ArmCmd& cmd){
        const bool new_armed = (cmd.enable != 0);
        const bool old_armed = g_armed;
        std::cout << "[ARM] enable=" << int(cmd.enable) << "\n";

        g_armed = new_armed;

        // 只在状态变化时打印一次
        if (g_arm_log_enable && new_armed != old_armed) {
            std::cout << "[ARM] state changed: "
                      << (old_armed ? 1 : 0) << " -> "
                      << (new_armed ? 1 : 0) << "\n";
        }

        auto w = make_arm_intent(cmd, ttl_ms);

        std::cout << "[INTENT_TX][ARM] "
                  << "enable=" << int(cmd.enable)
                  << " arm="   << int(w.arm)
                  << " disarm="<< int(w.disarm)
                  << " flags=0x" << std::hex << w.flags << std::dec
                  << "\n";

        (void)pub.publish(w);
        if (dump) dump->maybe_dump(pub, "[SHM_HEX][ARM]");
    };

    // 模式切换（不再隐含 ARM 语义）
    sev.on_set_mode = [&](const SetModeCmd& cmd){
        std::cout << "[SET_MODE] mode=" << mode_to_str(cmd.mode)
                  << " auto_controller=\"" << cmd.auto_controller << "\"\n";

        auto w = make_set_mode_intent(cmd, ttl_ms);
        (void)pub.publish(w);
        if (dump) dump->maybe_dump(pub, "[SHM_HEX][SET_MODE]");
    };

    // 6DOF 手动控制
    sev.on_set_dof = [&](const SetDofCmd& cmd){

        std::cout << "[SET_DOF] "
                  << "surge=" << cmd.dof[0]
                  << " sway="  << cmd.dof[1]
                  << " heave=" << cmd.dof[2]
                  << " roll="  << cmd.dof[3]
                  << " pitch=" << cmd.dof[4]
                  << " yaw="   << cmd.dof[5]
                  << "\n";

        auto w = make_set_dof_intent(cmd, ttl_ms);
        (void)pub.publish(w);
        if (dump) dump->maybe_dump(pub, "[SHM_HEX][SET_DOF]");
    };


    // 单电机测试
    sev.on_motor_test = [&](const MotorTestCmd& cmd){
        std::cout << "[MOTOR_TEST] motor=" << int(cmd.motor_id)
                  << " enable="      << int(cmd.enable)
                  << " mode="        << int(cmd.mode)
                  << " value="       << cmd.value
                  << " duration_ms=" << cmd.duration_ms
                  << " cmd_id="      << cmd.cmd_id
                  << "\n";

        auto w = make_motor_test_intent(cmd, ttl_ms);
        (void)pub.publish(w);
        if (dump) dump->maybe_dump(pub, "[SHM_HEX][MOTOR_TEST]");
    };
}

} // namespace gateway::app
