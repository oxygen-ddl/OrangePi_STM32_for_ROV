// gateway/src/IPC/intent/intent_mux.cpp
#include "gateway/IPC/intent/intent_mux.hpp"

#include <utility>

namespace comm_gcs::ipc::intent {

bool IntentMux::init(const std::vector<MuxSourceConfig>& sources)
{
    shutdown();
    entries_.clear();
    samples_.clear();
    stats_ = Stats{};

    entries_.reserve(sources.size());
    samples_.reserve(sources.size());

    for (const auto& sc : sources) {
        Entry e{};
        e.cfg = sc;

        // Pre-create one output slot per entry (aligned with entries_)
        IntentSample s{};
        s.src = sc.src;

        if (!sc.enable) {
            // keep disabled entry + empty sample
            entries_.push_back(std::move(e));
            samples_.push_back(std::move(s));
            continue;
        }

        // init subscriber (supports lazy init)
        GcsIntentSubscriberShm::Config cfg;
        cfg.enable    = true;
        cfg.shm_name  = sc.shm_name;
        cfg.shm_size  = sc.shm_size;
        cfg.lazy_init = sc.lazy_init;

        if (!e.sub.init(cfg)) {
            // If init fails hard (e.g. invalid name in non-lazy), abort.
            return false;
        }

        entries_.push_back(std::move(e));
        samples_.push_back(std::move(s));
    }

    return true;
}

void IntentMux::shutdown() noexcept
{
    for (auto& e : entries_) {
        e.sub.shutdown();
    }
    // keep vectors; init() will clear/rebuild
}

void IntentMux::poll_all(std::uint64_t now_mono_ns)
{
    ++stats_.poll_calls;

    // entries_ and samples_ must be aligned
    const std::size_t n = entries_.size();
    if (samples_.size() != n) {
        // Defensive: keep in a safe state
        samples_.assign(n, IntentSample{});
        for (std::size_t i = 0; i < n; ++i) {
            samples_[i].src = entries_[i].cfg.src;
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        auto& e = entries_[i];
        auto& s = samples_[i];

        s.src         = e.cfg.src;
        s.recv_mono_ns = now_mono_ns;

        if (!e.cfg.enable) {
            s.intent.reset();
            s.pub_mono_ns = 0;
            s.pub_wall_ns = 0;
            s.subscriber_initialized = false;
            continue;
        }

        s.subscriber_initialized = e.sub.initialized();

        std::uint64_t pub_mono_ns = 0;
        std::uint64_t pub_wall_ns = 0;

        auto opt = e.sub.poll_wire(&pub_mono_ns, &pub_wall_ns);
        if (opt.has_value()) {
            ++stats_.poll_hits;
            s.intent      = *opt;
            s.pub_mono_ns = pub_mono_ns;
            s.pub_wall_ns = pub_wall_ns;

            e.last_pub_mono_ns = pub_mono_ns;
            e.last_pub_wall_ns = pub_wall_ns;
            e.ready            = true;
        } else {
            // keep last s.intent (cache last valid). Stale is handled by arbiter.
        }
    }
}

} // namespace comm_gcs::ipc::intent
