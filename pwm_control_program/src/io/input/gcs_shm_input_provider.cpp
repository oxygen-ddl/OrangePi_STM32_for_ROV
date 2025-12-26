#include "io/input/gcs_shm_input_provider.hpp"

#include "io/input/gcs_intent_subscriber_shm.hpp"
#include "io/input/control_intent_wire_adapter.hpp"

#include "platform/timebase.hpp" // now_ns() for optional local stamp fill

#include <utility>

namespace rovctrl::io::input {

namespace cc = rovctrl::control_core;

struct GcsShmInputProvider::Impl {
    Config cfg;
    GcsIntentSubscriberShm sub;

    explicit Impl(Config c) : cfg(std::move(c)) {}
};

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
    if (!impl_->cfg.enable) return true;

    GcsIntentSubscriberShm::Config c{};
    c.enable    = true;
    c.shm_name  = impl_->cfg.shm_name;
    c.shm_size  = impl_->cfg.shm_size;
    c.lazy_init = impl_->cfg.lazy_init;

    // In lazy mode, init returns true even if shm not ready yet.
    return impl_->sub.init(c);
}

void GcsShmInputProvider::reset()
{
    // Soft reset: drop mapping, allow lazy re-init.
    impl_->sub.shutdown();
    (void)init();
}

bool GcsShmInputProvider::poll(cc::ControlState& /*state*/, cc::ControlIntent& out)
{
    out.clear_all();

    if (!impl_->cfg.enable) {
        return true;
    }

    const auto w = impl_->sub.poll_wire();
    if (!w.has_value()) {
        return true; // soft-fail: no data
    }

    // Convert wire -> internal (non-fatal on failure)
    if (!rovctrl::io::to_internal_intent(*w, out)) {
        out.clear_all();
        return true;
    }

    return true;
}


} // namespace rovctrl::io::input
