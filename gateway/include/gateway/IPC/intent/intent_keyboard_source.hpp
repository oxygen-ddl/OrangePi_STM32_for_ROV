#pragma once
#ifndef GATEWAY_IPC_INTENT_INTENT_KEYBOARD_SOURCE_HPP
#define GATEWAY_IPC_INTENT_INTENT_KEYBOARD_SOURCE_HPP

#include <cstddef>
#include <cstdint>
#include <string>

#include "shared/msg/control_intent.hpp"
#include "shared/msg/key_event.hpp"

// Input endpoint: KeyEvent ring subscriber
#include "gateway/IPC/keys/key_event_subscriber_shm.hpp"

// KeyEvent -> TeleopAction mapper (pure semantics)
#include "gateway/IPC/keys/keyboard_mapper.hpp"

// Output endpoint: ControlIntent shm publisher
// 说明：你们当前文件路径是 gateway/include/gateway/intent_publisher_shm.hpp
#include "gateway/intent_publisher_shm.hpp"

namespace comm_gcs::ipc::intent {

/**
 * @brief Keyboard Intent Source（KeyEvent → TeleopAction → ControlIntent）
 *
 * =========================
 * 业务目标（给新人看的）
 * =========================
 * 本模块是 gateway 侧的一个“输入源”（Intent Source），用于把“键盘按键事件”转成可消费的控制意图：
 *
 *  1) 从共享内存 KeyEvent ring（KeyEventSubscriberShm）读取低层按键事件；
 *  2) 通过 keyboard_mapper 将 KeyEvent 归一化为 TeleopAction（控制语义）；
 *  3) 维护键盘遥控的状态机（hold-to-run、timeout、throttle step、特殊姿态互斥等）；
 *  4) 周期性生成 shared::msg::ControlIntent 并发布到 intent shm（IntentPublisherShm）；
 *  5) 后续由 intent_mux / pwm_control_program 等模块订阅该 intent，进入控制链路。
 *
 * =========================
 * 分层边界（必须遵守）
 * =========================
 * - 本模块只负责“产出 ControlIntent”，不负责推进器执行、PWM 输出、推力分配。
 * - 本模块不做安全裁决：E-STOP 锁存、解除门槛、ARM/DISARM 合法性等，
 *   必须由下游 ControlGuard / 模式管理统一处理。
 * - 本模块不把控制语义写进 KeyEvent：KeyEvent 始终保持“最小输入事件模型”。
 *
 * =========================
 * 急停/解急停约定（对应 ControlIntent 字段）
 * =========================
 * - Ctrl + Space → TeleopAction::EStop → intent.estop = 1（脉冲：只在当帧置 1）
 * - Ctrl + M/m   → TeleopAction::ClearEStop → intent.clear_estop = 1（脉冲：只在当帧置 1）
 * - 是否真正进入/解除急停，由 ControlGuard 决定（例如 neutral hold 800ms 等门槛）。
 *
 * =========================
 * 推荐运行方式
 * =========================
 * - 由 gateway 主循环以固定频率调用 tick(now_mono_ns)。
 * - tick() 内部 drain KeyEvent、更新状态机、刷新 cmd_seq/stamp_ns、发布 intent。
 *
 * 可回滚性：
 * - 这是“新增输入源”，不要求立刻删除 pwm_control_program 旧本地键盘路径；
 * - 通过 mux sources 配置开关即可切换/回滚。
 */
class IntentKeyboardSource final {
public:
    struct Config {
        bool enable = true;

        // ------------------------------
        // Input: KeyEvent ring
        // ------------------------------
        comm_gcs::ipc::keys::KeyEventSubscriberShm::Config key_sub{};

        // ------------------------------
        // Output: ControlIntent shm
        // ------------------------------
        comm_gcs::IntentPublisherShm::Config intent_pub{};

        // ------------------------------
        // Source meta (written into ControlIntent)
        // ------------------------------
        std::uint8_t source_id   = 2;   // should map to IntentSource::kKeyboard (if you have)
        std::uint8_t source_prio = 0;   // optional 0..255
        std::uint32_t ttl_ms     = 0;   // 0 => receiver default / policy-defined

        // ------------------------------
        // Teleop policy (state machine knobs)
        // ------------------------------
        std::uint32_t hold_timeout_ms = 120; // no refresh => that DOF -> 0
        std::uint32_t motor_test_cmd_seq_ = 0; // initial cmd_seq for motor test commands

        float throttle_step = 0.05f;   // '=' / '-' step
        float throttle_min  = 0.0f;
        float throttle_max  = 1.0f;

        // Mapper behavior config
        comm_gcs::ipc::keys::KeyboardMapperConfig mapper_cfg{};
    };

    struct Stats final {
        std::uint64_t rx_key_events = 0;
        std::uint64_t tx_intents    = 0;

        // Optional: expose key subscriber stats snapshot (for debugging/monitoring)
        comm_gcs::ipc::keys::KeyEventSubscriberShm::Stats key_sub_stats{};
    };

    IntentKeyboardSource() = default;
    ~IntentKeyboardSource() noexcept = default;

    IntentKeyboardSource(const IntentKeyboardSource&)            = delete;
    IntentKeyboardSource& operator=(const IntentKeyboardSource&) = delete;

    IntentKeyboardSource(IntentKeyboardSource&&) noexcept = default;
    IntentKeyboardSource& operator=(IntentKeyboardSource&&) noexcept = default;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    /**
     * @brief 主循环周期调用：读取 KeyEvent、更新状态机、发布 ControlIntent
     * @param now_mono_ns 单调时钟（ns），建议由 gateway 统一提供（steady_clock）
     * @return true 表示本次 tick 执行成功（enable=false 视为成功）
     */
    bool tick(std::uint64_t now_mono_ns) noexcept;

    const Stats& stats() const noexcept { return stats_; }

private:
    // 状态机：处理一个按键事件（KeyEvent -> TeleopAction -> 更新内部状态/脉冲字段）
    void handle_event_(const shared::msg::KeyEvent& ev, std::uint64_t now_mono_ns) noexcept;

    // 状态机：tick 末尾组合 teleop_dof_cmd（含 hold timeout / 特殊姿态互斥 / throttle scale）
    void compose_intent_(std::uint64_t now_mono_ns) noexcept;

    // 每 tick 清理脉冲字段（estop/clear_estop/arm/disarm/request_exit/mode_request）
    void clear_pulses_() noexcept;

private:
    Config cfg_{};

    bool enabled_{false};
    bool initialized_{false};

    // IPC endpoints
    comm_gcs::ipc::keys::KeyEventSubscriberShm key_sub_{};
    comm_gcs::IntentPublisherShm intent_pub_{};

    // Output intent + sequencing
    shared::msg::ControlIntent intent_{};
    std::uint64_t cmd_seq_{0};

    // ---- teleop internal state (minimal fields in header) ----
    // throttle_cmd_: 语义倍率（0..1），平滑/限速策略在 cpp 的状态机实现里完成
    double throttle_cmd_{0.0};

    // Each DOF last refresh time (ns). Used for hold_timeout.
    // 顺序建议与 DofCommand 保持一致：surge, sway, heave, roll, pitch, yaw
    std::uint64_t last_refresh_ns_[6] = {0, 0, 0, 0, 0, 0};

    // Current directional intent before throttle scaling (typically -1/0/+1 or [-1,1])
    // 与旧 teleop_keyboard.cpp 的 hold-to-run 语义一致
    double dir_[6] = {0, 0, 0, 0, 0, 0};

    Stats stats_{};
    // 根据当前 intent_ 字段重建 flags（每 tick 发布前调用）
    void update_flags_() noexcept;
};

} // namespace comm_gcs::ipc::intent

#endif // GATEWAY_IPC_INTENT_INTENT_KEYBOARD_SOURCE_HPP
