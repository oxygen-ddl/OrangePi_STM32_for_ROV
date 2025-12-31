#include "io/input/gcs_shm_input_provider.hpp"

// Route-1：pwm_control_program 只订阅 shared::msg::ControlIntent 的 SHM wire 数据
#include "io/input/intent_subscriber_shm.hpp"
#include "io/input/control_intent_wire_codec.hpp"
#include "platform/timebase.hpp"   // now_ns()

#include <utility>

namespace rovctrl::io::input {

namespace cc = rovctrl::control_core;

struct GcsShmInputProvider::Impl final {
    Config cfg{};
    IntentSubscriberShm sub{};

    explicit Impl(Config c) : cfg(std::move(c)) {}
};

static inline std::uint64_t now_ns_u64() noexcept
{
    const auto t = rovctrl::platform::timebase::now_ns();
    return (t > 0) ? static_cast<std::uint64_t>(t) : 0ull;
}

// ------------------------------
// GcsShmInputProvider
// ------------------------------
GcsShmInputProvider::GcsShmInputProvider(Config cfg)
    : impl_(std::make_unique<Impl>(std::move(cfg)))
{
}

GcsShmInputProvider::~GcsShmInputProvider() = default;

GcsShmInputProvider::GcsShmInputProvider(GcsShmInputProvider&&) noexcept = default;
GcsShmInputProvider& GcsShmInputProvider::operator=(GcsShmInputProvider&&) noexcept = default;

bool GcsShmInputProvider::init()
{
    // 按你原本语义：enable=false 直接视为 init 成功（不启用该输入源）
    if (!impl_ || !impl_->cfg.enable) return true;

    IntentSubscriberShm::Config c{};
    c.enable    = true;
    c.shm_name  = impl_->cfg.shm_name;
    c.shm_size  = impl_->cfg.shm_size;
    c.lazy_init = impl_->cfg.lazy_init;

    // lazy 模式下，即使 shm 还没 ready，也应返回 true（允许后续 poll 再拿数据）
    return impl_->sub.init(c);
}

void GcsShmInputProvider::reset()
{
    if (!impl_) return;

    // Soft reset: drop shm handle, allow lazy re-init.
    impl_->sub.shutdown();
    (void)init();
}

bool GcsShmInputProvider::poll(cc::ControlState& /*state*/, cc::ControlIntent& out)
{
    // 维持你原先“soft-fail always true”的契约：poll 失败不应让上层崩溃
    out.clear_all();

    if (!impl_ || !impl_->cfg.enable) {
        return true;
    }

    std::uint64_t pub_mono_ns = 0;
    std::uint64_t pub_wall_ns = 0;
    const auto w = impl_->sub.poll(&pub_mono_ns, &pub_wall_ns);

    if (!w.has_value()) {
        return true; // no new data
    }
    // 如果 wire 未携带 stamp_ns，且配置允许：用本地时间补齐，避免上层 TTL/guard 逻辑失效
    if (impl_->cfg.fill_local_stamp_if_missing && w->stamp_ns == 0) {
        auto w2 = *w;               // optional 内对象是 const 视角，复制一份以便修改
        w2.stamp_ns = now_ns_u64();

        if (!decode_control_intent(w2, out)) {
            out.clear_all();
            return true;
        }
        return true;
    }

    // wire -> internal（不成功则清空并 soft-fail）
    if (!decode_control_intent(*w, out)) {
        out.clear_all();
        return true;
    }

    return true;
}

} // namespace rovctrl::io::input
