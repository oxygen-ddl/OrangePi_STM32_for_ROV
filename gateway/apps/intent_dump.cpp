// gateway/apps/intent_dump.cpp
//
// intent_dump: dump ControlIntent snapshots from SHM for diagnostics.
//
// Typical usage:
//   ./intent_dump
//   ./intent_dump --only final
//   ./intent_dump --hz 10
//   ./intent_dump --once --only remote
//
// Default shm names (designed for pipeline-style gateway):
//   remote: /rovctrl_intent_remote_v1   (from GCS/UDP/server side)
//   local : /rovctrl_intent_local_v1    (from local keyboard/ssh operator)
//   auto  : /rovctrl_intent_auto_v1     (from autonomy / controller / test generator)
//   final : /rovctrl_intent_final_v1    (arbiter/mux output; consumed by control program)
//
// Notes:
// - This tool is read-only, safe to run anytime.
// - Uses lazy_init so it can start before publishers.
// - "stale" is decided by (now_mono_ns - publisher_mono_ns) > max_age_ms.
//
// Build integration example:
//   add_executable(intent_dump apps/intent_dump.cpp)
//   target_link_libraries(intent_dump PRIVATE gateway_core)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>   // std::isfinite, std::round, std::fabs
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "gateway/IPC/intent/intent_subscriber_shm.hpp"
#include "shared/msg/control_intent.hpp"

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

static const char* mode_str(shared::msg::ControlMode m)
{
    switch (m) {
    case shared::msg::ControlMode::kNone:   return "None";
    case shared::msg::ControlMode::kManual: return "Manual";
    case shared::msg::ControlMode::kAuto:   return "Auto";
    case shared::msg::ControlMode::kHold:   return "Hold";
    default: return "Unknown";
    }
}

static void print_flags(std::uint32_t flags, std::ostream& os)
{
    bool first = true;
    auto emit = [&](const char* s) {
        if (!first) os << "|";
        os << s;
        first = false;
    };

    if (flags == 0) { os << "NONE"; return; }

    if (flags & shared::msg::kHasEStopCmd)    emit("EStop");
    if (flags & shared::msg::kHasArmCmd)      emit("Arm");
    if (flags & shared::msg::kHasModeRequest) emit("Mode");
    if (flags & shared::msg::kHasTeleopDof)   emit("Dof");
    if (flags & shared::msg::kHasRef)         emit("Ref");
    if (flags & shared::msg::kHasRefDelta)    emit("RefDelta");
}

struct Args {
    std::string remote_shm = "/rovctrl_intent_remote_v1";
    std::string local_shm  = "/rovctrl_intent_local_v1";
    std::string auto_shm   = "/rovctrl_intent_auto_v1";
    std::string final_shm  = "/rovctrl_intent_final_v1";

    double hz = 5.0;                   // print/poll rate
    std::uint32_t max_age_ms = 300;     // stale threshold
    bool once = false;

    // only: "", "remote", "local", "auto", "final"
    std::string only;
};

static void usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "Options:\n"
        << "  --only <remote|local|auto|final>   default: (all)\n"
        << "  --hz <hz>                          default: 5\n"
        << "  --max-age-ms <ms>                  default: 300\n"
        << "  --once                             default: off\n"
        << "  --remote-shm <name>                default: /rovctrl_intent_remote_v1\n"
        << "  --local-shm  <name>                default: /rovctrl_intent_local_v1\n"
        << "  --auto-shm   <name>                default: /rovctrl_intent_auto_v1\n"
        << "  --final-shm  <name>                default: /rovctrl_intent_final_v1\n"
        << "  -h, --help\n";
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

        if (k == "-h" || k == "--help") {
            usage(argv[0]);
            return false;
        } else if (k == "--only") {
            const char* v = need("--only"); if (!v) return false;
            a.only = v;
            if (!(a.only == "remote" || a.only == "local" || a.only == "auto" || a.only == "final")) {
                std::cerr << "[ERR] invalid --only: " << a.only << "\n";
                return false;
            }
        } else if (k == "--hz") {
            const char* v = need("--hz"); if (!v) return false;
            a.hz = std::atof(v);
        } else if (k == "--max-age-ms") {
            const char* v = need("--max-age-ms"); if (!v) return false;
            a.max_age_ms = static_cast<std::uint32_t>(std::strtoul(v, nullptr, 10));
        } else if (k == "--once") {
            a.once = true;
        } else if (k == "--remote-shm") {
            const char* v = need("--remote-shm"); if (!v) return false;
            a.remote_shm = v;
        } else if (k == "--local-shm") {
            const char* v = need("--local-shm"); if (!v) return false;
            a.local_shm = v;
        } else if (k == "--auto-shm") {
            const char* v = need("--auto-shm"); if (!v) return false;
            a.auto_shm = v;
        } else if (k == "--final-shm") {
            const char* v = need("--final-shm"); if (!v) return false;
            a.final_shm = v;
        } else {
            std::cerr << "[ERR] unknown arg: " << k << "\n";
            usage(argv[0]);
            return false;
        }
    }

    if (a.hz <= 0.0) {
        std::cerr << "[ERR] --hz must be > 0\n";
        return false;
    }

    auto check = [&](const std::string& n, const char* label) {
        if (!n.empty() && n.front() != '/') {
            std::cerr << "[ERR] " << label << " must start with '/': " << n << "\n";
            return false;
        }
        return true;
    };

    if (!check(a.remote_shm, "--remote-shm")) return false;
    if (!check(a.local_shm,  "--local-shm"))  return false;
    if (!check(a.auto_shm,   "--auto-shm"))   return false;
    if (!check(a.final_shm,  "--final-shm"))  return false;

    return true;
}

static inline int sanitize_hz(const char* name, double hz,
                              int hz_min, int hz_max, int hz_def) noexcept
{
    // NaN/Inf or non-positive
    if (!std::isfinite(hz) || hz <= 0.0) {
        std::cerr << "[intent_dump][ERR] " << name << "=" << hz
                  << " invalid (must be finite and >0). Fallback to " << hz_def << " Hz.\n";
        return hz_def;
    }

    // Must be integer Hz
    const double r = std::round(hz);
    if (std::fabs(hz - r) > 1e-9) {
        std::cerr << "[intent_dump][ERR] " << name << "=" << hz
                  << " invalid (must be integer Hz). Fallback to " << hz_def << " Hz.\n";
        return hz_def;
    }

    // Range check via fallback (as requested: “报错 + 采用默认值”)
    const int ihz = static_cast<int>(r);
    if (ihz < hz_min || ihz > hz_max) {
        std::cerr << "[intent_dump][ERR] " << name << "=" << ihz
                  << " out of range [" << hz_min << "," << hz_max << "]. "
                  << "Fallback to " << hz_def << " Hz.\n";
        return hz_def;
    }

    return ihz;
}


// Pretty print one intent
static void dump_one(const char* tag,
                     const std::string& shm_name,
                     bool inited,
                     std::uint64_t pub_mono_ns,
                     const std::optional<shared::msg::ControlIntent>& opt,
                     std::uint32_t max_age_ms)
{
    const std::uint64_t now_ns = now_mono_ns();
    const std::uint64_t age_ms = (pub_mono_ns == 0) ? UINT64_MAX : (now_ns - pub_mono_ns) / 1000000ull;
    const bool stale = (pub_mono_ns != 0) && (age_ms > max_age_ms);

    std::cerr << "[" << tag << "] "
              << "shm=" << shm_name
              << " init=" << (inited ? 1 : 0)
              << " age_ms=" << (age_ms == UINT64_MAX ? -1LL : static_cast<long long>(age_ms))
              << " stale=" << (stale ? 1 : 0);

    if (!opt.has_value()) {
        std::cerr << " <no-snapshot>\n";
        return;
    }

    const auto& it = *opt;

    std::cerr
        << " seq=" << it.cmd_seq
        << " ttl_ms=" << it.ttl_ms
        << " flags=";
    print_flags(it.flags, std::cerr);

    std::cerr
        << " exit=" << static_cast<int>(it.request_exit)
        << " estop=" << static_cast<int>(it.estop)
        << " clr=" << static_cast<int>(it.clear_estop)
        << " arm=" << static_cast<int>(it.arm)
        << " disarm=" << static_cast<int>(it.disarm)
        << " mode=" << mode_str(it.mode_request);

    // Teleop DOF only meaningful if flag set; still print for convenience
    const auto& d = it.teleop_dof_cmd;
    std::cerr << std::fixed << std::setprecision(2)
              << " dof=["
              << d.surge << "," << d.sway << "," << d.heave << ","
              << d.roll  << "," << d.pitch<< "," << d.yaw
              << "]";

    std::cerr << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    Args args;
    if (!parse_args(argc, argv, args)) {
        return 2;
    }
    // hz: must be integer in [1,200], else fallback to default 5 (with error log)
    const int hz_i = sanitize_hz("--hz", args.hz, /*min*/1, /*max*/200, /*def*/5);


    std::cerr << "[intent_dump] starting...\n"
              << "  only=" << (args.only.empty() ? "<all>" : args.only) << "\n"
              << "  hz=" << args.hz << " max_age_ms=" << args.max_age_ms << " once=" << (args.once?1:0) << "\n"
              << "  remote=" << args.remote_shm << "\n"
              << "  local =" << args.local_shm  << "\n"
              << "  auto  =" << args.auto_shm   << "\n"
              << "  final =" << args.final_shm  << "\n";

    // Adjust this alias if your header uses a different namespace path.
    using Sub = comm_gcs::ipc::intent::GcsIntentSubscriberShm;

    auto make_sub = [&](const std::string& shm_name) {
        Sub s;
        Sub::Config cfg;
        cfg.enable = true;
        cfg.shm_name = shm_name;
        cfg.shm_size = 0;
        cfg.lazy_init = true; // allow starting before publisher exists
        (void)s.init(cfg);
        return s;
    };

    Sub sub_remote = make_sub(args.remote_shm);
    Sub sub_local  = make_sub(args.local_shm);
    Sub sub_auto   = make_sub(args.auto_shm);
    Sub sub_final  = make_sub(args.final_shm);

    // Use steady_clock integral duration to avoid time_point<duration<double>> assignment issue
    const auto period = std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::nanoseconds(static_cast<std::int64_t>(1000000000LL / hz_i)));

    auto next_tick = SteadyClock::now();

    while (!g_stop.load()) {
        const auto now = SteadyClock::now();
        if (now < next_tick) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        next_tick = now + period;

        if (args.only.empty() || args.only == "remote") {
            std::uint64_t mono_ns = 0, wall_ns = 0;
            auto opt = sub_remote.poll_wire(&mono_ns, &wall_ns);
            dump_one("REMOTE", args.remote_shm, sub_remote.initialized(), mono_ns, opt, args.max_age_ms);
        }
        if (args.only.empty() || args.only == "local") {
            std::uint64_t mono_ns = 0, wall_ns = 0;
            auto opt = sub_local.poll_wire(&mono_ns, &wall_ns);
            dump_one("LOCAL ", args.local_shm,  sub_local.initialized(),  mono_ns, opt, args.max_age_ms);
        }
        if (args.only.empty() || args.only == "auto") {
            std::uint64_t mono_ns = 0, wall_ns = 0;
            auto opt = sub_auto.poll_wire(&mono_ns, &wall_ns);
            dump_one("AUTO  ", args.auto_shm,   sub_auto.initialized(),   mono_ns, opt, args.max_age_ms);
        }
        if (args.only.empty() || args.only == "final") {
            std::uint64_t mono_ns = 0, wall_ns = 0;
            auto opt = sub_final.poll_wire(&mono_ns, &wall_ns);
            dump_one("FINAL ", args.final_shm,  sub_final.initialized(),  mono_ns, opt, args.max_age_ms);
        }

        if (args.once) break;
    }

    std::cerr << "[intent_dump] exit\n";
    return 0;
}
