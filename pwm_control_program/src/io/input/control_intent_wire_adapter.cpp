#include "io/input/control_intent_wire_adapter.hpp"
#include "io/input/control_intent_wire_codec.hpp"

#include "control_core/control_intent.hpp"
#include "platform/timebase.hpp"

#include <cstdint>

namespace rovctrl::io {

namespace {

// 如果 wire 侧没填 stamp_ns（=0），在接收侧用本端时间补一份，避免 TTL 逻辑全挂在 0 上。
inline std::uint64_t ensure_stamp_ns(std::uint64_t stamp_ns) noexcept
{
    if (stamp_ns != 0) {
        return stamp_ns;
    }
    return static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());
}

} // namespace

bool to_internal_intent(const shared::msg::ControlIntent& w,
                        rovctrl::control_core::ControlIntent& out) noexcept
{
    // 交给 codec 做唯一一次字段级解析（version / flags / mode / teleop / motor_test 等）
    if (!rovctrl::io::input::decode_control_intent(w, out)) {
        return false;
    }

    // 在接收端做一个统一的时间兜底策略：
    //  - 如果 source 已经填了 stamp_ns，则保留；
    //  - 如果为 0，则用本端 now_ns() 补一个，保证后续 TTL / staleness 判断可用。
    out.stamp_ns = ensure_stamp_ns(out.stamp_ns);

    // TTL 是否兜底留给上层（ControlGuard 或 provider.finalize_intent_）决定：
    //  - 若希望“0 表示用默认 TTL”，可以在控制层统一处理，而不是在适配层写死策略。

    return true;
}

} // namespace rovctrl::io
