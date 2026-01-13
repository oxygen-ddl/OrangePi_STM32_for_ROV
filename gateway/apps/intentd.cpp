// gateway/apps/intentd.cpp

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "gateway/IPC/intent/intent_mux.hpp"
#include "gateway/IPC/intent/intent_arbiter.hpp"
#include "gateway/intent_publisher_shm.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;
std::atomic_bool g_stop{false};
void on_sigint(int) { g_stop.store(true); }

inline std::uint64_t now_mono_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            SteadyClock::now().time_since_epoch()).count());
}

static inline std::chrono::steady_clock::duration period_from_hz(int hz) noexcept
{
    using namespace std::chrono;
    const auto ns = static_cast<std::int64_t>(1'000'000'000LL / hz);
    return duration_cast<steady_clock::duration>(nanoseconds(ns));
}

struct Args {
    std::string shm_remote = "/rovctrl_intent_remote_v1";
    std::string shm_local  = "/rovctrl_intent_local_v1";
    std::string shm_auto   = "/rovctrl_intent_auto_v1";
    std::string shm_final  = "/rovctrl_intent_final_v1";

    int poll_hz  = 50;
    int pub_hz   = 50;
    int print_hz = 2;

    // policy
    bool publish_when_no_fresh      = true;
    bool require_initialized        = false;
    bool enable_payload_ttl         = true;
    bool stamp0_is_expired          = true;
    bool estop_priority             = true;

    bool hold_last_good_on_degrade  = true;
    bool clear_payload_on_degrade   = true;

    // per-source defaults
    std::uint32_t max_age_ms        = 250;
    std::uint32_t warmup_ms         = 1500;
    std::uint32_t default_ttl_ms    = 250;
};

// 你可以继续复用你原来的 parse_args/usage/check_shm_name/sanitize_hz。
// 为了突出减负逻辑，这里略去 parse_args 的实现。
static bool parse_args(int argc, char** argv, Args& a)
{
    // 目前先做“占位实现”，仅使用默认配置，不做真正的命令行解析
    (void)argc;
    (void)argv;
    (void)a;
    return true;
}

} // namespace

// 省略：includes / now_mono_ns / period_from_hz / parse_args 等

int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    Args args;
    if (!parse_args(argc, argv, args)) return 2;

    // ---------------------------- Mux ----------------------------
    comm_gcs::ipc::intent::IntentMux mux;
    {
        using comm_gcs::ipc::intent::MuxSourceConfig;
        using comm_gcs::ipc::intent::SourceId;

        std::vector<MuxSourceConfig> sources;
        sources.reserve(3);

        MuxSourceConfig r{};
        r.src = SourceId::Remote;
        r.shm_name = args.shm_remote;
        r.enable = true;
        r.lazy_init = true;
        sources.push_back(r);

        MuxSourceConfig l{};
        l.src = SourceId::Local;
        l.shm_name = args.shm_local;
        l.enable = true;
        l.lazy_init = true;
        sources.push_back(l);

        MuxSourceConfig a{};
        a.src = SourceId::Auto;
        a.shm_name = args.shm_auto;
        a.enable = true;
        a.lazy_init = true;
        sources.push_back(a);

        if (!mux.init(sources)) {
            std::cerr << "[intentd][ERR] mux.init failed\n";
            return 10;
        }
    }

    // ---------------------------- Arbiter ----------------------------
    comm_gcs::ipc::intent::IntentArbiter arb;
    {
        comm_gcs::ipc::intent::IntentArbiter::Policy p{};
        p.require_initialized        = args.require_initialized;
        p.enable_payload_ttl         = args.enable_payload_ttl;
        p.stamp0_is_expired          = args.stamp0_is_expired;
        p.estop_priority             = args.estop_priority;

        p.publish_when_no_fresh      = args.publish_when_no_fresh;
        p.hold_last_good_on_degrade  = args.hold_last_good_on_degrade;
        p.clear_payload_on_degrade   = args.clear_payload_on_degrade;

        arb.set_policy(p);

        using comm_gcs::ipc::intent::SourcePolicy;
        using comm_gcs::ipc::intent::SourceId;

        std::vector<SourcePolicy> sps;
        sps.reserve(3);

        auto make_sp = [&](SourceId src, int prio) {
            SourcePolicy sp{};
            sp.src = src;
            sp.priority = prio;
            sp.max_age_ms = args.max_age_ms;
            sp.warmup_ms  = args.warmup_ms;
            sp.default_ttl_ms = args.default_ttl_ms;
            sp.allow_stale = true;
            sp.enable = true;
            return sp;
        };

        sps.push_back(make_sp(SourceId::Remote, 10));
        sps.push_back(make_sp(SourceId::Local,  20));
        sps.push_back(make_sp(SourceId::Auto,   30));

        arb.set_sources(std::move(sps));
    }

    // ---------------------------- Final publisher ----------------------------
    comm_gcs::IntentPublisherShm pub_final;
    {
        comm_gcs::IntentPublisherShm::Config cfg;
        cfg.enable   = true;
        cfg.shm_name = args.shm_final;
        cfg.shm_size = 0;
        if (!pub_final.init(cfg)) {
            std::cerr << "[intentd][ERR] final publisher init failed\n";
            return 20;
        }
    }


    // ---------------------------- Timers ----------------------------
    const auto poll_period  = period_from_hz(args.poll_hz);
    const auto pub_period   = period_from_hz(args.pub_hz);
    const auto print_period = period_from_hz(args.print_hz);

    const auto t0_tp = SteadyClock::now();
    const std::uint64_t t0_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t0_tp.time_since_epoch()).count());

    auto next_poll  = SteadyClock::now();
    auto next_pub   = SteadyClock::now();
    auto next_print = SteadyClock::now();

    // ---------------------------- State ----------------------------
    std::optional<shared::msg::ControlIntent> last_good;
    comm_gcs::ipc::intent::ArbiterDecision last_decision{};

    std::uint64_t cnt_poll = 0;
    std::uint64_t cnt_pub  = 0;
    std::uint64_t cnt_degraded_pub = 0;

    while (!g_stop.load()) {
        const auto now_tp = SteadyClock::now();
        const std::uint64_t now_ns = now_mono_ns();

        // ---- poll: sample + decide (pure logic lives in arbiter) ----
        if (now_tp >= next_poll) {
            next_poll = now_tp + poll_period;
            ++cnt_poll;

            mux.poll_all(now_ns);

            const shared::msg::ControlIntent* last_ptr =
                last_good.has_value() ? &(*last_good) : nullptr;

            last_decision = arb.decide(now_ns, t0_ns, mux.samples_ref(), last_ptr);
        }

        // ---- publish: strictly follow arbiter decision ----
        if (now_tp >= next_pub) {
            next_pub = now_tp + pub_period;

            if (last_decision.should_publish) {
                // 重要：不要在 intentd 再做 clear_payload / degrade 决策
                // 只需直接发布 arbiter 给出的 publish_intent
                if (!pub_final.publish(last_decision.publish_intent)) {
                    std::cerr << "[intentd][WARN] publish(final) failed\n";
                        
                        }
                } else {
                    ++cnt_pub;
                    if (last_decision.degraded) ++cnt_degraded_pub;

                    // 更新 last_good：仅 fresh winner
                    if (last_decision.has_winner && !last_decision.degraded) {
                        last_good = last_decision.publish_intent;
                    }
                    if (last_decision.request_shutdown) {
                        std::cerr << "[intentd] shutdown requested (Local exit). published estop, exiting...\n";
                        break;
                    }
            }
        }

        // ---- diagnostics ----
        if (now_tp >= next_print) {
            next_print = now_tp + print_period;

            std::cerr
                << "[intentd] poll=" << cnt_poll
                << " pub=" << cnt_pub
                << " degraded_pub=" << cnt_degraded_pub
                << " mux_hits=" << mux.stats().poll_hits
                << " winner=" << comm_gcs::ipc::intent::to_string(last_decision.winner)
                << " degraded=" << (last_decision.degraded ? 1 : 0)
                << " should_pub=" << (last_decision.should_publish ? 1 : 0)
                << " estop_assert=" << (last_decision.estop_assert ? 1 : 0)
                << " estop_clear=" << (last_decision.estop_clear ? 1 : 0)
                << " reason=" << last_decision.reason
                << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    

    std::cerr << "[intentd] stopping...\n";
    pub_final.shutdown();
    mux.shutdown();
    return 0;
}
