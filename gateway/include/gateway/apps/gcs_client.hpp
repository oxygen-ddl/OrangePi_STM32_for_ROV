#pragma once
#ifndef GATEWAY_APPS_GCS_CLIENT_HPP
#define GATEWAY_APPS_GCS_CLIENT_HPP

/**
 * @file gateway/apps/gcs_client.hpp
 * @brief GCS UDP 会话事件 → ControlIntent SHM 的适配层声明
 *
 * 模块定位（业务视角）：
 *   - 本模块不包含 main()，不生成独立可执行程序；
 *   - 专门为 gateway/apps/gcs_server.cpp 提供“业务辅助逻辑”；
 *   - 负责把 GcsSession 解析出的 GCS 命令，转换为 shared::msg::ControlIntent，
 *     并写入 IntentPublisherShm（即 pwm_control_program 侧订阅的意图 SHM）。
 *
 * 主要业务功能：
 *   1) 提供 ShmHexDumper：用于低频十六进制 dump SHM 内容，便于抓包对比 / 排查对齐问题；
 *   2) 提供 IntentContext：封装 Intent SHM 相关上下文（publisher + TTL + 可选调试 dumper）；
 *   3) 提供 attach_default_events(...)：
 *        - 统一为 GcsSessionEvents 挂载一组“默认事件处理逻辑”；
 *        - 事件包括：
 *            - on_session_established：会话建立日志；
 *            - on_session_lost：会话丢失时往 SHM 写一帧“清零 Intent”；
 *            - on_estop：GCS 急停命令 → ControlIntent.kHasEStopCmd；
 *            - on_arm：GCS Arm/Disarm 命令 → ControlIntent.kHasArmCmd + arm/disarm 位；
 *            - on_set_mode：GCS 模式切换 → ControlIntent.kHasModeRequest；
 *            - on_set_dof：GCS 6DOF 手动指令 → ControlIntent.kHasTeleopDof + DofCommand；
 *            - on_motor_test：GCS 单电机测试命令 → ControlIntent.kHasMotorTest；
 *        - 所有 Intent 最终都写入 IntentPublisherShm 供 pwm_control_program 侧消费。
 *
 * 使用方式（典型流程）：
 *   在 gateway/apps/gcs_server.cpp 的 main() 中：
 *
 *     1) 初始化 IntentPublisherShm pub;
 *     2) 构造 ShmHexDumper shm_dump，并按命令行参数配置开关 / 频率；
 *     3) 构造 IntentContext ictx{pub, &shm_dump, intent_ttl_ms};
 *     4) 构造 GcsSessionEvents sev{};
 *     5) 调用 gateway::app::attach_default_events(sev, ictx);
 *     6) 使用 (scfg, sev) 创建 comm_gcs::session::GcsSession sess;
 *
 *   之后：
 *     - UDP 收到的 GCS 数据包经 sess.on_packet(...) 解析；
 *     - sess 内部触发 sev.* 回调；
 *     - 回调通过本模块把命令转成 ControlIntent 并 publish 到 SHM。
 *
 * 注意：
 *   - 本模块只关注“命令 → ControlIntent”的适配逻辑，不负责 PWM / 控制器细节；
 *   - Telemetry（状态回发）仍由 gcs_server.cpp 自行组织；
 *   - 目的就是瘦身 gcs_server.cpp，让 main() 更像“装配脚本”，
 *     而业务细节统一集中在本模块中便于维护与复用。
 */

#include <chrono>
#include <vector>

#include "gateway/intent_publisher_shm.hpp"
#include "gateway/session/gcs_session.hpp"
#include "proto_gcs/gcs_protocol.hpp"
#include "gateway/codec/gcs_codec.hpp"
#include "shared/msg/control_intent.hpp"

namespace gateway::app {

/**
 * @brief SHM 十六进制调试工具
 *
 * 功能：
 *   - 周期性、限次数地把 Intent SHM 开头一段数据以 hex 形式打印到 stderr；
 *   - 主要用于排查对齐 / 版本不一致 / 非预期覆盖等问题；
 *   - 由业务侧在关键事件（ESTOP / ARM / SET_DOF / SESSION_LOST 等）点调用。
 *
 * 使用约定：
 *   - 在 gcs_server.cpp 里配置 enable / every_ms / max_times；
 *   - 调用 maybe_dump(pub, "[TAG]") 即可，无论是否 enable，都不会抛异常。
 */
struct ShmHexDumper {
    bool enable      = false;  ///< 是否启用 hex dump（默认关闭）
    int  every_ms    = 1000;   ///< 最小 dump 间隔（毫秒）
    int  max_times   = 20;     ///< 最多 dump 次数（防止日志爆炸）

    std::chrono::steady_clock::time_point next_tp =
        std::chrono::steady_clock::now();
    int count = 0;

    /**
     * @brief 如到达时间条件，则对 SHM 内容做一次十六进制打印
     *
     * @param pub IntentPublisherShm 实例（必须已 init 成功）
     * @param tag 日志前缀（例如 "[SHM_HEX][ESTOP]"）
     */
    void maybe_dump(const comm_gcs::IntentPublisherShm& pub,
                    const char* tag = "[SHM_HEX]");
};

/**
 * @brief Intent 适配层上下文
 *
 * 聚合：
 *   - IntentPublisherShm：真正的共享内存发布者；
 *   - ShmHexDumper：可选调试工具（允许为 nullptr）；
 *   - intent_ttl_ms：下发给 ControlIntent 的 TTL（毫秒）；
 *   - armed / arm_log_enable：网关侧对 ARM 状态的本地视角。
 */
struct IntentContext {
    comm_gcs::IntentPublisherShm& pub;  ///< SHM publisher（由 gcs_server.cpp 创建并持有）
    ShmHexDumper*                shm_dump;  ///< 可为 nullptr，如不需要 hex dump
    int                          intent_ttl_ms; ///< ControlIntent.ttl_ms 默认值（工程级统一配置）

    // === ARM 状态（网关本地视角）===
    bool armed          = false; ///< 网关认为当前 ROV 是否已解锁（初始为未解锁）
    bool arm_log_enable = false; ///< 是否打印 ARM 状态变化日志（避免刷屏）
};

/**
 * @brief 调试：打印一次 ControlIntent / DofCommand 的内存布局信息
 *
 * 用途：
 *   - 在启动参数中开启 --dbg-layout 时调用；
 *   - 帮助开发者检查结构体 sizeof / alignof / offsetof 是否符合预期；
 *   - 便于在多进程、多语言（如 Python / C）共享内存时做 ABI 验证。
 *
 * 典型用法（gcs_server.cpp）：
 *
 *   if (dbg_layout) {
 *       gateway::app::dump_layouts_once();
 *   }
 */
void dump_layouts_once();

/**
 * @brief 调试：对即将发出的 ACK 包做十六进制打印
 *
 * 行为：
 *   - 仅当 pkt 被解析为 MsgType::ACK 时，才真正打印十六进制内容；
 *   - 若解析失败，会打印一条包含 AckCode 和错误原因的警告；
 *   - 其他类型的报文会静默忽略。
 *
 * 典型用法（gcs_server.cpp）：
 *
 *   for (auto& pkt : outs) {
 *       gateway::app::dump_ack_hex(pkt);
 *       srv.send_to(..., pkt, ...);
 *   }
 *
 * @param pkt 通过 GcsSession 得到的“待发送”完整报文（含头 + payload）
 */
void dump_ack_hex(const std::vector<comm_gcs::Byte>& pkt);

/**
 * @brief 为 GcsSessionEvents 挂载一组“默认事件处理”逻辑
 *
 * 功能概要：
 *   - 将 GcsSession 产生的事件统一转换为 ControlIntent，并写入 Intent SHM：
 *       * 会话事件：
 *           - on_session_established：打印会话建立信息（session_id / peer）；
 *           - on_session_lost：往 SHM 写一帧“清零 Intent”（会话中断信号）；
 *       * 控制命令事件：
 *           - on_estop：急停 / 解除急停 → kHasEStopCmd + estop/clear_estop；
 *           - on_arm：Arm / Disarm → kHasArmCmd + arm/disarm，更新 IntentContext.armed；
 *           - on_set_mode：模式请求 → kHasModeRequest + mode_request；
 *           - on_set_dof：6DOF 手动控制 → kHasTeleopDof + teleop_dof_cmd；
 *           - on_motor_test：单电机测试 → kHasMotorTest + motor_test；
 *   - 如配置了 ShmHexDumper，将在关键事件点调用 maybe_dump() 做低频 hex dump。
 *
 * 安全约定（与控制侧 ControlGuard 协同）：
 *   - ARM / DISARM 仅改变“是否允许推进器响应”的逻辑状态；
 *   - 在未 ARM 时，仍允许下发 E-STOP / 模式切换等命令；
 *   - 6DOF 指令虽然可以写入 Intent SHM，但控制侧会在未 ARM 时强制 ZeroOutput。
 *
 * @param sev   GcsSessionEvents 引用（由 gcs_server.cpp 声明）
 * @param ictx  Intent 上下文（包含 SHM publisher、hex dumper、ttl_ms 等配置）
 *
 * 调用时机：
 *   - 在创建 comm_gcs::session::GcsSession 之前；
 *   - 典型代码片段：
 *
 *       comm_gcs::session::GcsSessionEvents sev{};
 *       gateway::app::IntentContext ictx{pub, &shm_dump, intent_ttl_ms};
 *       ictx.arm_log_enable = true;  // 如需打印 ARM 状态变化日志
 *       gateway::app::attach_default_events(sev, ictx);
 *       comm_gcs::session::GcsSession sess(scfg, sev);
 */
void attach_default_events(comm_gcs::session::GcsSessionEvents& sev,
                           const IntentContext& ictx);
                          
/**
 * @brief 调试：解析并打印收到的 GCS UDP 报文类型和长度
 *
 * 功能：
 *   - 对一帧原始 UDP payload 调用 proto_gcs 的 parse_and_validate；
 *   - 若解析成功，打印 msg_type 和 payload_len；
 *   - 若解析失败，打印错误码和错误信息；
 *
 * 使用场景：
 *   - 在 gcs_server.cpp 的 UDP 回调中，对每个收到的 payload 调用一次；
 *   - 仅用于开发期排障，可根据需要加上命令行开关控制是否启用。
 */
void dump_rx_msg_type(const comm_gcs::BytesView& payload);

} // namespace gateway::app

#endif // GATEWAY_APPS_GCS_CLIENT_HPP
