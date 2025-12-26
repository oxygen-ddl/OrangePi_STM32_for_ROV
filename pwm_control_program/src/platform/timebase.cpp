// src/platform/timebase.cpp

#include "platform/timebase.hpp"

namespace rovctrl::platform::timebase {

// -------------------------
// 基础时间接口
// -------------------------

int64_t now_ns() noexcept
{
    auto tp = Clock::now().time_since_epoch();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp);
    return ns.count();
}

TimePoint now() noexcept
{
    return Clock::now();
}

// -------------------------
// 默认延迟配置（全局静态）
// -------------------------

static LatencyDefaults g_latency_defaults{};

LatencyDefaults& latency_defaults() noexcept
{
    return g_latency_defaults;
}

// 内部工具：根据事件类型返回默认延迟（纳秒）
static int64_t default_latency_for(SensorKind kind) noexcept
{
    const auto& d = g_latency_defaults;
    switch (kind) {
    case SensorKind::IMU:
        return d.imu_ns;
    case SensorKind::TELEOP:
        return d.teleop_ns;
    case SensorKind::CONTROL_LOOP:
        return d.control_loop_ns;
    case SensorKind::OTHER:
    default:
        return d.other_ns;
    }
}

// -------------------------
// stamp(): 统一时间戳核心方法
// -------------------------

Stamp stamp(const std::string&        sensor_id,
            SensorKind                kind,
            std::optional<int64_t>    sensor_time_ns,
            std::optional<int64_t>    latency_ns)
{
    Stamp s;
    s.sensor_id    = sensor_id;
    s.kind         = kind;
    s.host_time_ns = now_ns();

    // 1) 选择延迟：参数优先，否则使用默认值
    const int64_t used_latency_ns =
        latency_ns.has_value() ? *latency_ns : default_latency_for(kind);
    s.latency_ns = used_latency_ns;

    // 2) 选择“基准时间”：有 sensor_time_ns 用传感器时间，否则用 host_time_ns
    const int64_t base_time_ns =
        sensor_time_ns.has_value() ? *sensor_time_ns : s.host_time_ns;
    s.sensor_time_ns    = sensor_time_ns;
    s.corrected_time_ns = base_time_ns - used_latency_ns;

    // 以后如果要做更复杂的“时钟对齐”（比如 IMU/DVL 时间轴对齐），
    // 可以在这里扩展对 base_time_ns 的计算逻辑。

    return s;
}

} // namespace rovctrl::platform::timebase
