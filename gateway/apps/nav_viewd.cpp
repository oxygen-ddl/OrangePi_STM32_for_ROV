// gateway/apps/nav_viewd.cpp
//
// nav_viewd: NavState(SHM) -> NavStateView(SHM) bridge daemon
//
// Pipeline:
//   NavStateSubscriberShm.poll() -> NavViewBuilder.build() -> NavViewPublisherShm.publish()
//
// Responsibilities (mature pipeline):
//   - Builder: mapping + health mapping + finite checks -> produces a candidate view (valid reflects data quality)
//   - Daemon : publish semantics (mono_ns/age_ms), staleness policy, degrade/hold policy, SHM I/O scheduling
//
// Defaults:
//   NavState shm: /rovctrl_nav_state_v1
//   NavView  shm: /rovctrl_nav_view_v1

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <algorithm>

#include "gateway/IPC/nav/nav_state_subscriber_shm.hpp"
#include "gateway/IPC/nav/nav_view_builder.hpp"
#include "gateway/IPC/nav/nav_view_publisher_shm.hpp"

#include "shared/msg/nav_state.hpp"
#include "shared/msg/nav_state_view.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

std::atomic_bool g_stop{false};
void on_sigint(int) { g_stop.store(true); }

inline std::uint64_t now_mono_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            SteadyClock::now().time_since_epoch())
            .count());
}

struct Args {
    // SHM names
    std::string nav_state_shm = "/rovctrl_nav_state_v1";
    std::string nav_view_shm  = "/rovctrl_nav_view_v1";

    // Rates
    double poll_hz = 50.0;   // poll at 50Hz
    double pub_hz  = 20.0;   // publish at 20Hz (0 disables publish)

    // Health / staleness
    std::uint32_t max_age_ms = 250;   // stale if (now - last_nav_pub_mono_ns) > max_age_ms (after warmup)
    std::uint32_t warmup_ms  = 1500;  // startup grace period

    // Degrade policy
    bool publish_when_stale = true;   // still publish when stale, but valid=0
    bool hold_last_good     = true;   // in degrade, keep last_good payload but valid=0
    double degrade_pub_hz   = 5.0;    // degrade publish throttle (0 => publish every pub slot)

    // Logging
    double print_hz = 2.0;
};

static void usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "Options:\n"
        << "  --nav-state-shm <name>     default: /rovctrl_nav_state_v1\n"
        << "  --nav-view-shm  <name>     default: /rovctrl_nav_view_v1\n"
        << "  --poll-hz <hz>             default: 50\n"
        << "  --pub-hz  <hz>             default: 20 (0 disables publish)\n"
        << "  --max-age-ms <ms>          default: 250\n"
        << "  --warmup-ms <ms>           default: 1500\n"
        << "  --publish-when-stale 0|1   default: 1\n"
        << "  --hold-last-good     0|1   default: 1\n"
        << "  --degrade-pub-hz <hz>      default: 5 (0 disables degrade throttle)\n"
        << "  --print-hz <hz>            default: 2\n";
}

static bool parse_bool(const std::string& s, bool& out)
{
    if (s == "1" || s == "true" || s == "TRUE")  { out = true;  return true; }
    if (s == "0" || s == "false" || s == "FALSE"){ out = false; return true; }
    return false;
}

static bool parse_args(int argc, char** argv, Args& a)
{
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];

        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[ERR] missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (k == "--help" || k == "-h") {
            usage(argv[0]);
            return false;
        } else if (k == "--nav-state-shm") {
            const char* v = need("--nav-state-shm"); if (!v) return false;
            a.nav_state_shm = v;
        } else if (k == "--nav-view-shm") {
            const char* v = need("--nav-view-shm"); if (!v) return false;
            a.nav_view_shm = v;
        } else if (k == "--poll-hz") {
            const char* v = need("--poll-hz"); if (!v) return false;
            a.poll_hz = std::atof(v);
        } else if (k == "--pub-hz") {
            const char* v = need("--pub-hz"); if (!v) return false;
            a.pub_hz = std::atof(v);
        } else if (k == "--max-age-ms") {
            const char* v = need("--max-age-ms"); if (!v) return false;
            a.max_age_ms = static_cast<std::uint32_t>(std::strtoul(v, nullptr, 10));
        } else if (k == "--warmup-ms") {
            const char* v = need("--warmup-ms"); if (!v) return false;
            a.warmup_ms = static_cast<std::uint32_t>(std::strtoul(v, nullptr, 10));
        } else if (k == "--publish-when-stale") {
            const char* v = need("--publish-when-stale"); if (!v) return false;
            bool b=false; if (!parse_bool(v, b)) { std::cerr << "[ERR] invalid bool: " << v << "\n"; return false; }
            a.publish_when_stale = b;
        } else if (k == "--hold-last-good") {
            const char* v = need("--hold-last-good"); if (!v) return false;
            bool b=false; if (!parse_bool(v, b)) { std::cerr << "[ERR] invalid bool: " << v << "\n"; return false; }
            a.hold_last_good = b;
        } else if (k == "--degrade-pub-hz") {
            const char* v = need("--degrade-pub-hz"); if (!v) return false;
            a.degrade_pub_hz = std::atof(v);
        } else if (k == "--print-hz") {
            const char* v = need("--print-hz"); if (!v) return false;
            a.print_hz = std::atof(v);
        } else {
            std::cerr << "[ERR] unknown arg: " << k << "\n";
            usage(argv[0]);
            return false;
        }
    }

    auto check_shm = [](const std::string& s, const char* name) -> bool {
        if (s.empty() || s.front() != '/') {
            std::cerr << "[ERR] " << name << " must start with '/': " << s << "\n";
            return false;
        }
        return true;
    };
    if (!check_shm(a.nav_state_shm, "nav_state_shm")) return false;
    if (!check_shm(a.nav_view_shm,  "nav_view_shm"))  return false;

    if (a.poll_hz <= 0.0)  { std::cerr << "[ERR] poll_hz must be > 0\n"; return false; }
    if (a.pub_hz < 0.0)    { std::cerr << "[ERR] pub_hz must be >= 0\n"; return false; }
    if (a.degrade_pub_hz < 0.0) { std::cerr << "[ERR] degrade_pub_hz must be >= 0\n"; return false; }
    if (a.print_hz <= 0.0) { std::cerr << "[ERR] print_hz must be > 0\n"; return false; }

    return true;
}

// period helpers
inline std::chrono::nanoseconds hz_to_period_ns(double hz)
{
    if (hz <= 0.0) return std::chrono::nanoseconds{0};
    const double sec = 1.0 / hz;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sec));
}

// bounded catch-up to avoid drift and huge catch-up loops after stalls
inline void advance_next(SteadyClock::time_point& next,
                         SteadyClock::time_point now,
                         std::chrono::nanoseconds period,
                         int max_catchup = 3)
{
    if (period.count() <= 0) { next = now; return; }
    if (now < next) return;

    int n = 0;
    while (now >= next && n < max_catchup) {
        next += period;
        ++n;
    }
    if (now >= next) next = now + period;
}

inline void fill_publish_fields(shared::msg::NavStateView& out,
                                std::uint64_t pub_mono_ns) noexcept
{
    out.mono_ns = pub_mono_ns;

    // age_ms: prefer using stamp_ns (nav fusion timestamp). stamp_ns is steady-clock ns in your NavState.
    if (out.stamp_ns != 0 && pub_mono_ns >= out.stamp_ns) {
        const std::uint64_t age_ms64 = (pub_mono_ns - out.stamp_ns) / 1000000ull;
        out.age_ms = (age_ms64 > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<std::uint32_t>(age_ms64);
    } else {
        out.age_ms = 0xFFFFFFFFu; // unknown
    }
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    Args args;
    if (!parse_args(argc, argv, args)) return 2;

    std::cerr
        << "[nav_viewd] starting...\n"
        << "  nav_state_shm=" << args.nav_state_shm << "\n"
        << "  nav_view_shm =" << args.nav_view_shm  << "\n"
        << "  poll_hz=" << args.poll_hz
        << " pub_hz=" << args.pub_hz
        << " degrade_pub_hz=" << args.degrade_pub_hz << "\n"
        << "  max_age_ms=" << args.max_age_ms
        << " warmup_ms=" << args.warmup_ms << "\n"
        << "  publish_when_stale=" << (args.publish_when_stale ? 1 : 0)
        << " hold_last_good=" << (args.hold_last_good ? 1 : 0)
        << " print_hz=" << args.print_hz << "\n";

    // -------------------------------------------------------------------------
    // Init subscriber (NavState shm)
    // -------------------------------------------------------------------------
    comm_gcs::ipc::nav::NavStateSubscriberShm nav_sub;
    {
        comm_gcs::ipc::nav::NavStateSubscriberShm::Config cfg;
        cfg.enable    = true;
        cfg.shm_name  = args.nav_state_shm;
        cfg.shm_size  = 0;
        cfg.lazy_init = true;
        if (!nav_sub.init(cfg)) {
            std::cerr << "[nav_viewd][ERR] NavStateSubscriberShm.init failed\n";
            return 10;
        }
    }

    // -------------------------------------------------------------------------
    // Init publisher (NavView shm)
    // -------------------------------------------------------------------------
    comm_gcs::ipc::nav::NavViewPublisherShm nav_pub;
    {
        comm_gcs::ipc::nav::NavViewPublisherShm::Config cfg;
        cfg.enable   = true;
        cfg.shm_name = args.nav_view_shm;
        cfg.shm_size = 0;
        if (!nav_pub.init(cfg)) {
            std::cerr << "[nav_viewd][ERR] NavViewPublisherShm.init failed\n";
            return 11;
        }
    }

    comm_gcs::ipc::nav::NavViewBuilder builder;

    // -------------------------------------------------------------------------
    // Timers
    // -------------------------------------------------------------------------
    const auto poll_period    = hz_to_period_ns(args.poll_hz);
    const auto pub_period     = hz_to_period_ns(args.pub_hz);
    const auto print_period   = hz_to_period_ns(args.print_hz);
    const auto degrade_period = hz_to_period_ns(args.degrade_pub_hz);

    const auto start_tp = SteadyClock::now();
    auto next_poll      = start_tp;
    auto next_pub       = start_tp;
    auto next_print     = start_tp;
    auto next_degrade   = start_tp;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    shared::msg::NavStateView last_good_view{};
    bool has_last_good = false;

    // from subscriber header (publisher timestamps in nav shm)
    std::uint64_t last_nav_pub_mono_ns = 0;
    // std::uint64_t last_nav_pub_wall_ns = 0;  // 当前未使用，保留注释说明

    // stats
    std::uint64_t cnt_poll = 0, cnt_poll_hit = 0, cnt_no_nav = 0;
    std::uint64_t cnt_pub  = 0, cnt_pub_degrade = 0, cnt_stale = 0;

    while (!g_stop.load()) {
        const auto now_tp = SteadyClock::now();

        // ---------------- Poll phase ----------------
        if (poll_period.count() > 0 && now_tp >= next_poll) {
            advance_next(next_poll, now_tp, poll_period);
            ++cnt_poll;

            std::uint64_t mono_ns = 0, wall_ns = 0;
            auto nav_opt = nav_sub.poll(&mono_ns, nullptr);

            if (nav_opt.has_value()) {
                ++cnt_poll_hit;
                last_nav_pub_mono_ns = mono_ns;
                last_nav_pub_wall_ns = wall_ns;

                // Builder defines "data quality" validity (finite + health mapping policy)
                shared::msg::NavStateView cand = builder.build(*nav_opt);

                // last_good only updates on valid==1 (mature: do not "freeze in" bad states)
                if (cand.valid) {
                    last_good_view = cand;
                    has_last_good  = true;
                }
            } else {
                ++cnt_no_nav;
            }
        }

        // ---------------- Publish phase ----------------
        const bool pub_enabled = (pub_period.count() > 0);
        if (pub_enabled && now_tp >= next_pub) {
            advance_next(next_pub, now_tp, pub_period);

            // warmup window
            const auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - start_tp).count();
            const bool in_warmup = (elapsed_ms < static_cast<long long>(args.warmup_ms));

            // staleness based on nav shm publisher mono timestamp
            const std::uint64_t now_ns = now_mono_ns();
            const std::uint64_t age_ms_from_nav_pub = (last_nav_pub_mono_ns == 0)
                ? UINT64_MAX
                : (now_ns - last_nav_pub_mono_ns) / 1000000ull;

            const bool no_nav_yet = (last_nav_pub_mono_ns == 0);
            const bool stale = (!in_warmup) &&
                               (age_ms_from_nav_pub != UINT64_MAX) &&
                               (age_ms_from_nav_pub > args.max_age_ms);

            // decide output
            bool do_publish = true;
            bool is_degrade = false;
            shared::msg::NavStateView out{};

            if (!stale && !no_nav_yet && has_last_good) {
                out = last_good_view;
                is_degrade = false;
            } else {
                is_degrade = true;
                if (stale) ++cnt_stale;

                if (!args.publish_when_stale) {
                    do_publish = false;
                } else {
                    // degrade throttle
                    if (degrade_period.count() > 0) {
                        if (now_tp < next_degrade) {
                            do_publish = false;
                        } else {
                            advance_next(next_degrade, now_tp, degrade_period);
                        }
                    }

                    if (do_publish) {
                        if (args.hold_last_good && has_last_good) {
                            out = last_good_view;
                        } else {
                            out = shared::msg::NavStateView{};
                            out.version = shared::msg::kNavStateViewWireVersion; // keep ABI header sane
                        }
                        // Degrade semantics: always invalid
                        out.valid = 0;
                    }
                }
            }

            if (do_publish) {
                // === publish timestamp (single source of truth) ===
                const std::uint64_t pub_mono_ns = now_mono_ns();

                out.mono_ns = pub_mono_ns;
                out.age_ms  = 0;              // age 由控制侧 / GCS 计算

                // If stale/degrade, valid must be 0
                if (is_degrade) {
                    out.valid = 0;
                }

                if (nav_pub.publish(out)) {
                   ++cnt_pub;
                    if (is_degrade) ++cnt_pub_degrade;
                } else {
                    std::cerr << "[nav_viewd][WARN] publish failed\n";
                }
            }

        }

        // ---------------- Diagnostics ----------------
        if (print_period.count() > 0 && now_tp >= next_print) {
            advance_next(next_print, now_tp, print_period);

            const std::uint64_t now_ns = now_mono_ns();
            const std::uint64_t age_ms_from_nav_pub = (last_nav_pub_mono_ns == 0)
                ? UINT64_MAX
                : (now_ns - last_nav_pub_mono_ns) / 1000000ull;

            std::cerr
                << "[nav_viewd] poll=" << cnt_poll
                << " hit=" << cnt_poll_hit
                << " no_nav=" << cnt_no_nav
                << " pub=" << cnt_pub
                << " degrade_pub=" << cnt_pub_degrade
                << " stale_cnt=" << cnt_stale
                << " last_age_ms=" << (age_ms_from_nav_pub == UINT64_MAX ? -1LL
                                  : static_cast<long long>(age_ms_from_nav_pub))
                << " sub_init=" << (nav_sub.initialized() ? 1 : 0)
                << "\n";
        }

        // ---------------- Sleep until next event ----------------
        SteadyClock::time_point next_wake = now_tp + std::chrono::milliseconds(50);

        if (poll_period.count() > 0)  next_wake = std::min(next_wake, next_poll);
        if (pub_period.count() > 0)   next_wake = std::min(next_wake, next_pub);
        if (print_period.count() > 0) next_wake = std::min(next_wake, next_print);
        if (degrade_period.count() > 0) next_wake = std::min(next_wake, next_degrade);

        auto sleep_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(next_wake - SteadyClock::now());
        if (sleep_ns.count() < 0) sleep_ns = std::chrono::nanoseconds(0);

        auto sleep_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sleep_ns);
        if (sleep_ms < std::chrono::milliseconds(1))  sleep_ms = std::chrono::milliseconds(1);
        if (sleep_ms > std::chrono::milliseconds(50)) sleep_ms = std::chrono::milliseconds(50);

        std::this_thread::sleep_for(sleep_ms);
    }

    std::cerr << "[nav_viewd] stopping...\n";
    nav_pub.shutdown();
    nav_sub.shutdown();
    return 0;
}
