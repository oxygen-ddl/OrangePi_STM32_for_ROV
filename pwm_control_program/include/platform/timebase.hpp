#ifndef ROVCTRL_PLATFORM_TIMEBASE_HPP
#define ROVCTRL_PLATFORM_TIMEBASE_HPP

#pragma once
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace uwnav::timebase {

//-------------------------
// 基础类型
//-------------------------

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = Clock::duration;

// 1. 基础时间接口：返回单调时间
//
// 约定：
// - now_ns()  返回从某个固定 epoch（实现内部）开始的纳秒数，单调递增
// - now()     返回 steady_clock::time_point
int64_t   now_ns() noexcept;
TimePoint now() noexcept;

// 2. 传感器类型（精简版）
//   - 用于按类型选择默认延迟
enum class SensorKind {
    IMU,
    DVL,
    USBL,
    OTHER
};

// 3. 统一的时间戳结构
//
// 语义约定：
// - host_time_ns:
//     使用 now_ns() 记录“本机接收到 / 解析完成该条数据”的时间。
// - sensor_time_ns:
//     传感器内部提供的时间（若有，例如 DVL time tag）。
// - latency_ns:
//     对该条数据使用的“延迟估计”（可以是默认值，也可以是调用方覆写）。
// - corrected_time_ns:
//     统一后的事件时间戳，给 ESKF / Logger 使用。
//     典型策略：
//       a) 如有 sensor_time_ns，则以 sensor_time_ns - latency_ns 作为 corrected；
//       b) 否则以 host_time_ns - latency_ns 作为 corrected。
struct Stamp {
    std::string              sensor_id;         // "imu0", "dvl0", "usbl0", ...
    SensorKind               kind{SensorKind::OTHER};

    int64_t                  host_time_ns{0};
    std::optional<int64_t>   sensor_time_ns;    // 可为空

    int64_t                  latency_ns{0};
    int64_t                  corrected_time_ns{0};
};

// 4. 默认延迟参数（以后可以从配置加载）
//
// 注意：单位为纳秒
struct LatencyDefaults {
    int64_t imu_ns   = 2'000'000;      // 2 ms
    int64_t dvl_ns   = 50'000'000;     // 50 ms
    int64_t usbl_ns  = 150'000'000;    // 150 ms
    int64_t other_ns = 0;
};

// 5. 全局默认延迟配置：
//    - 返回一个可修改的全局引用（线程安全需求目前不强，后续再升级）
//
// 用法示例：
//   auto& ld = latency_defaults();
//   ld.dvl_ns = 40'000'000;
LatencyDefaults& latency_defaults() noexcept;

// 6. 统一打时间戳的工具函数
//
// 调用约定：
//   - sensor_id: "imu0" / "dvl0" / "usbl0" 等
//   - kind:      传感器类型，用于选择默认延迟
//   - sensor_time_ns: 若传感器提供自身时间，则填；否则留空
//   - latency_ns: 如需覆写默认延迟，则传入；否则使用 latency_defaults() 中对应字段
//
// 行为：
//   - 立即调用 now_ns() 作为 host_time_ns
//   - 选取 latency_ns：若参数有值用参数，否则查默认；
//   - 按上述规则计算 corrected_time_ns，返回 Stamp。
Stamp stamp(
    const std::string&              sensor_id,
    SensorKind                      kind,
    std::optional<int64_t>          sensor_time_ns = std::nullopt,
    std::optional<int64_t>          latency_ns     = std::nullopt
);

} // namespace uwnav::timebase


#endif /* ROVCTRL_PLATFORM_TIMEBASE_HPP */