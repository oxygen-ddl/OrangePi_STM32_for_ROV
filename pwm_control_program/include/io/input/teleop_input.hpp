#pragma once
#ifndef ROVCTRL_IO_TELEOP_INPUT_HPP
#define ROVCTRL_IO_TELEOP_INPUT_HPP

/**
 * @file   teleop_input.hpp
 * @brief  TeleopInputProvider – 从 Intent SHM 订阅最终 ControlIntent 的输入源。
 *
 * 当前设计（键盘逻辑已迁移到 gateway 层）：
 *  - 不再直接读取键盘或终端；
 *  - 只从共享内存中订阅由 gateway/intent_mux 合成后的 shared::msg::ControlIntent；
 *  - 利用 control_intent_wire_codec 将 wire intent 解码为 control_core::ControlIntent；
 *  - 控制循环只关心“已经仲裁好”的 ControlIntent。
 */

#include <cstddef>
#include <cstdint>

#include "io/input/input_provider.hpp"        // IInputProvider
#include "control_core/control_intent.hpp"    // ControlIntent
#include "control_core/control_types.hpp"     // ControlState

// 订阅 Intent SHM
#include "io/input/intent_subscriber_shm.hpp"

namespace rovctrl::io {

using input::IntentSubscriberShm;   // ★ 把子命名空间里的类型引进来

class TeleopInputProvider final : public IInputProvider {
public:
    struct Config {
        // 是否启用该输入源。false 时 poll() 仍返回 true，但 intent 为空载荷。
        bool enable = true;

        // 从哪个 SHM 订阅 ControlIntent（应与 gateway/intent_mux 输出一致）
        // 典型命名："/rovctrl_intent_mux_v1"
        const char* shm_name = "/rovctrl_intent_mux_v1";

        // 允许 pwm_control_program 先启动，gateway 稍后再建立 SHM
        bool        lazy_init = true;

        // 如果 wire intent 中 stamp_ns/ttl_ms 为 0，是否在本端补齐
        bool        fill_stamp_if_missing = true;
        bool        fill_ttl_if_missing   = true;

        // 若本端需要强制一个 TTL，可设为非 0；否则沿用 wire/Guard 策略
        std::uint32_t ttl_ms = 0;
    };

public:
    TeleopInputProvider() = default;
    explicit TeleopInputProvider(Config cfg) : cfg_(cfg) {}
    ~TeleopInputProvider() override = default;

    bool init() override;

    bool poll(rovctrl::control_core::ControlState&  state,
              rovctrl::control_core::ControlIntent& intent) override;

    void reset() override;

    // 允许上层在 init() 前修改配置
    void set_config(const Config& cfg) { cfg_ = cfg; }

private:
    using ControlIntent = rovctrl::control_core::ControlIntent;

    // 统一给 intent 打时间戳 / TTL（只在需要补齐时调用）
    void finalize_intent_(ControlIntent& intent);

private:
    Config       cfg_{};
    std::uint64_t seq_{0};

    bool initialized_{false};

    // 目前不再在本层触发请求退出，但保留字段以备未来扩展
    bool exit_requested_{false};

    // 订阅 gateway/intent_mux 写入的 Intent SHM
    IntentSubscriberShm intent_sub_;   // ★ 这里就可以用裸名
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_TELEOP_INPUT_HPP
