// pwm_control_program/src/io/input/teleop_input.cpp
//
// TeleopInputProvider – 通过 Intent SHM 获取最终 ControlIntent。
// 说明：
//   - 不再直接读取键盘或终端；
//   - 键盘/GCS/UI → gateway → intent_mux → Intent SHM → 这里；
//   - 本层只负责：订阅、解码、补时间戳/TTL。

#include "io/input/teleop_input.hpp"

#include <cstdint>
#include <iostream>

#include "platform/timebase.hpp"
#include "io/input/control_intent_wire_codec.hpp"  // decode_control_intent

namespace rovctrl::io {

namespace cc = rovctrl::control_core;
using rovctrl::io::input::decode_control_intent;

bool TeleopInputProvider::init()
{
    if (initialized_) return true;

    if (!cfg_.enable) {
        initialized_ = true; // 依然视作“初始化成功”，但不会输出有效 intent
        std::cerr << "[TeleopInput] disabled by config.\n";
        return true;
    }

    IntentSubscriberShm::Config scfg{};
    scfg.enable    = true;
    scfg.shm_name  = (cfg_.shm_name && cfg_.shm_name[0] != '\0')
                       ? cfg_.shm_name
                       : "/rovctrl_intent_mux_v1";
    scfg.lazy_init = cfg_.lazy_init;
    scfg.shm_size  = 0;  // 0 => 使用默认 sizeof(shared::shm::IntentShmLayout)

    if (!intent_sub_.init(scfg)) {
        // lazy_init=true 时，这里失败是允许的：等后续 poll() 时再重试
        std::cerr << "[TeleopInput] [WARN] IntentSubscriberShm init failed; "
                     "intent will stay empty until SHM is ready.\n";
    }

    initialized_ = true;
    std::cout << "[TeleopInput] ready, consuming ControlIntent from SHM '"
              << scfg.shm_name << "'.\n";
    return true;
}

// =============================================================================
// poll: 读取一帧 ControlIntent（若无数据则返回“空 intent + 正常返回”）
// =============================================================================
bool TeleopInputProvider::poll(cc::ControlState& state, cc::ControlIntent& intent)
{
    (void)state;

    // 每个周期都从干净状态开始，避免上一帧遗留
    intent = cc::ControlIntent{};
    intent.clear_payload();

    if (!initialized_) {
        (void)init();
    }

    // 若配置禁用：只输出带时间戳的“空 intent”
    if (!cfg_.enable) {
        intent.cmd_seq = ++seq_;
        finalize_intent_(intent);
        return true;
    }

    // 从 Intent SHM 拿“最终合成后的 wire intent”
    std::uint64_t pub_mono_ns = 0;
    std::uint64_t pub_wall_ns = 0;
    auto opt = intent_sub_.poll(&pub_mono_ns, &pub_wall_ns);
    if (!opt) {
        // SHM 尚未就绪 / 还没有写入：输出一帧“空 intent”，让 Guard 认为“无外部命令”
        intent.cmd_seq = ++seq_;
        finalize_intent_(intent);
        return true;
    }

    // wire → core 解码
    const bool dec_ok = decode_control_intent(*opt, intent);
    if (!dec_ok) {
        // 版本不匹配或快照损坏：退回到安全的“空 intent”
        intent = cc::ControlIntent{};
        intent.clear_payload();
        intent.cmd_seq = ++seq_;
        finalize_intent_(intent);
        return true;
    }

    // 根据配置，若 stamp/ttl 缺失则在本端补齐
    if ((cfg_.fill_stamp_if_missing && intent.stamp_ns == 0) ||
        (cfg_.fill_ttl_if_missing   && intent.ttl_ms   == 0))
    {
        finalize_intent_(intent);
    }

    return true;
}

void TeleopInputProvider::reset()
{
    exit_requested_ = false;
}

void TeleopInputProvider::finalize_intent_(cc::ControlIntent& intent)
{
    // 时间戳填补：使用本进程 steady ns
    if (intent.stamp_ns == 0) {
        intent.stamp_ns = static_cast<std::uint64_t>(
            rovctrl::platform::timebase::now_ns());
    }

    // TTL 填补：优先使用 Config 中的 ttl_ms，其次用一个工程默认值
    if (intent.ttl_ms == 0) {
        intent.ttl_ms = (cfg_.ttl_ms != 0) ? cfg_.ttl_ms : 200;
    }
}

} // namespace rovctrl::io
