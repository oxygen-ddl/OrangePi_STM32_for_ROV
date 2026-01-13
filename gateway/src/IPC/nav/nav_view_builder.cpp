#include "gateway/IPC/nav/nav_view_builder.hpp"

#include <cmath>
#include <cstdint>

namespace comm_gcs::ipc::nav {

namespace {

inline bool is_finite3(const double v[3]) noexcept
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline std::uint8_t map_health(shared::msg::NavHealth h) noexcept
{
    using shared::msg::NavHealth;
    using shared::msg::NavHealthView; // 来自 nav_state_view.hpp（已改名）

    switch (h) {
    case NavHealth::OK:            return static_cast<std::uint8_t>(NavHealthView::kOk);
    case NavHealth::DEGRADED:      return static_cast<std::uint8_t>(NavHealthView::kDegraded);
    case NavHealth::INVALID:       return static_cast<std::uint8_t>(NavHealthView::kBad);
    case NavHealth::UNINITIALIZED: return static_cast<std::uint8_t>(NavHealthView::kUnknown);
    default:                       return static_cast<std::uint8_t>(NavHealthView::kUnknown);
    }
}

} // namespace

shared::msg::NavStateView NavViewBuilder::build(const shared::msg::NavState& s) noexcept
{
    shared::msg::NavStateView v{};

    // ---- wire header ----
    v.version  = shared::msg::kNavStateViewWireVersion;

    // NavState 的 t_ns 是 steady_clock 的 ns（你在 nav_state.hpp 注释里已经明确）
    // NavStateView 里我们放在 stamp_ns（“导航时间戳”）
    v.stamp_ns = s.t_ns;

    // mono_ns / age_ms 通常由 comm_gcs 在 publish 时填充（这里不强行假设有 now_mono_ns）
    v.mono_ns  = 0;
    v.age_ms   = 0;

    // ---- payload ----
    for (int i = 0; i < 3; ++i) {
        v.pos[i]     = s.pos[i];
        v.vel[i]     = s.vel[i];
        v.rpy[i]     = s.rpy[i];
        v.omega_b[i] = s.omega_b[i];
        v.acc_b[i]   = s.acc_b[i];
    }
    v.depth_m = s.depth;

    // flags：你当前的 view 定义支持这些“字段存在性”bit
    v.flags = shared::msg::kHasPosition |
              shared::msg::kHasVelocity |
              shared::msg::kHasRPY |
              shared::msg::kHasDepth |
              shared::msg::kHasOmegaBody |
              shared::msg::kHasAccBody;

    // health：从 NavState 的健康枚举映射到 View 健康码
    v.health = map_health(s.health);

    // 透传 nav 的细粒度状态位（原 NavState::status_flags）
    v.reserved1 = static_cast<std::uint32_t>(s.status_flags);

    // valid 策略：健康 OK/DEGRADED + 关键字段 finite
    const bool finite_ok =
        (s.t_ns != 0) &&
        is_finite3(s.pos) && is_finite3(s.vel) && is_finite3(s.rpy) &&
        is_finite3(s.omega_b) && is_finite3(s.acc_b) &&
        std::isfinite(s.depth);

    const bool health_ok =
        (s.health == shared::msg::NavHealth::OK) ||
        (s.health == shared::msg::NavHealth::DEGRADED);

    v.valid = (finite_ok && health_ok) ? 1 : 0;

    return v;
}

} // namespace comm_gcs::ipc::nav
