// gateway/apps/teleop_local.cpp
//
// teleop_local: Local keyboard (TTY) -> KeyEvent ring SHM publisher
//
// Purpose:
//   - Read local keyboard events from stdin (TTY raw mode, non-blocking).
//   - Publish shared::msg::KeyEvent into shared::shm KeyEventRing (publisher owns SHM).
//
// Notes:
//   - This program does NOT encode control semantics (WASD -> surge, etc.).
//     It only publishes raw key events.
//   - Control semantics remain in pwm_control_program (teleop_keyboard).
//
// Default SHM name:
//   /rovctrl_key_event_local_v1
//
// Build integration (gateway/CMakeLists.txt):
//   add_executable(teleop_local apps/teleop_local.cpp)
//   target_link_libraries(teleop_local PRIVATE gateway_core)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#ifndef _WIN32
#include <cerrno>
#include <termios.h>
#include <unistd.h>
#endif

#include "gateway/IPC/keys/key_event_publisher_shm.hpp"
#include "shared/msg/key_event.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

std::atomic_bool g_stop{false};

void on_sig(int) { g_stop.store(true); }

// -----------------------------------------------------------------------------
// Time helpers (single clock policy: SteadyClock)
// -----------------------------------------------------------------------------
inline std::uint64_t now_mono_ns() noexcept
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            SteadyClock::now().time_since_epoch())
            .count());
}

inline std::chrono::nanoseconds hz_to_period_ns(double hz) noexcept
{
    if (hz <= 0.0) return std::chrono::nanoseconds{0};
    const double sec = 1.0 / hz;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(sec));
}

// advance timer with bounded catch-up, avoid drift and huge while loops after stalls.
inline void advance_next(SteadyClock::time_point& next,
                         SteadyClock::time_point now,
                         std::chrono::nanoseconds period,
                         int max_catchup = 3) noexcept
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

// -----------------------------------------------------------------------------
// Terminal raw mode + non-blocking key read (POSIX)
// -----------------------------------------------------------------------------
#ifndef _WIN32
class TerminalRawGuard final {
public:
    TerminalRawGuard() = default;
    ~TerminalRawGuard() { disable(); }

    bool enable()
    {
        if (enabled_) return true;

        if (!::isatty(STDIN_FILENO)) {
            std::cerr << "[teleop_local][WARN] stdin is not a TTY; cannot enable raw mode.\n";
            return false;
        }

        if (::tcgetattr(STDIN_FILENO, &orig_) != 0) {
            std::cerr << "[teleop_local][ERR] tcgetattr failed: " << std::strerror(errno) << "\n";
            return false;
        }

        ::termios raw = orig_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO)); // no canonical, no echo
        raw.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));  // no special processing
        raw.c_oflag &= static_cast<tcflag_t>(~(OPOST));
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            std::cerr << "[teleop_local][ERR] tcsetattr(raw) failed: " << std::strerror(errno) << "\n";
            return false;
        }

        enabled_ = true;
        return true;
    }

    void disable()
    {
        if (!enabled_) return;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &orig_) != 0) {
            std::cerr << "[teleop_local][WARN] tcsetattr(restore) failed: " << std::strerror(errno) << "\n";
        }
        enabled_ = false;
    }

    bool enabled() const { return enabled_; }

private:
    bool enabled_{false};
    ::termios orig_{};
};

inline int read_key_nonblock()
{
    unsigned char ch = 0;
    const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    if (n == 1) return static_cast<int>(ch);
    if (n == 0) return -1;
    if (errno == EAGAIN || errno == EWOULDBLOCK) return -1;

    std::cerr << "[teleop_local][WARN] read(STDIN) failed: " << std::strerror(errno) << "\n";
    return -1;
}
#endif

// -----------------------------------------------------------------------------
// Args / CLI
// -----------------------------------------------------------------------------
struct Args {
    std::string shm_name = "/rovctrl_key_event_local_v1";
    std::uint32_t capacity = 256;

    double poll_hz  = 200.0;
    double print_hz = 2.0;

    bool exit_if_not_tty = true;
    bool verbose = false;
};

static void usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "Options:\n"
        << "  --shm-name <name>          default: /rovctrl_key_event_local_v1\n"
        << "  --capacity <n>             default: 256\n"
        << "  --poll-hz <hz>             default: 200\n"
        << "  --print-hz <hz>            default: 2\n"
        << "  --exit-if-not-tty 0|1      default: 1\n"
        << "  --verbose 0|1              default: 0\n"
        << "  -h, --help\n";
}

static bool parse_bool(const std::string& s, bool& out)
{
    if (s == "1" || s == "true" || s == "TRUE") { out = true; return true; }
    if (s == "0" || s == "false" || s == "FALSE") { out = false; return true; }
    return false;
}

static bool parse_args(int argc, char** argv, Args& a)
{
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];

        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[teleop_local][ERR] missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (k == "--help" || k == "-h") {
            usage(argv[0]);
            return false;
        } else if (k == "--shm-name") {
            const char* v = need("--shm-name"); if (!v) return false;
            a.shm_name = v;
        } else if (k == "--capacity") {
            const char* v = need("--capacity"); if (!v) return false;
            a.capacity = static_cast<std::uint32_t>(std::strtoul(v, nullptr, 10));
        } else if (k == "--poll-hz") {
            const char* v = need("--poll-hz"); if (!v) return false;
            a.poll_hz = std::atof(v);
        } else if (k == "--print-hz") {
            const char* v = need("--print-hz"); if (!v) return false;
            a.print_hz = std::atof(v);
        } else if (k == "--exit-if-not-tty") {
            const char* v = need("--exit-if-not-tty"); if (!v) return false;
            bool b=false; if (!parse_bool(v, b)) { std::cerr << "[teleop_local][ERR] invalid bool: " << v << "\n"; return false; }
            a.exit_if_not_tty = b;
        } else if (k == "--verbose") {
            const char* v = need("--verbose"); if (!v) return false;
            bool b=false; if (!parse_bool(v, b)) { std::cerr << "[teleop_local][ERR] invalid bool: " << v << "\n"; return false; }
            a.verbose = b;
        } else {
            std::cerr << "[teleop_local][ERR] unknown arg: " << k << "\n";
            usage(argv[0]);
            return false;
        }
    }

    if (a.shm_name.empty() || a.shm_name.front() != '/') {
        std::cerr << "[teleop_local][ERR] shm-name must start with '/': " << a.shm_name << "\n";
        return false;
    }
    if (a.capacity < 16) {
        std::cerr << "[teleop_local][ERR] capacity too small: " << a.capacity << " (min 16)\n";
        return false;
    }
    if (a.poll_hz <= 0.0) {
        std::cerr << "[teleop_local][ERR] poll_hz must be > 0\n";
        return false;
    }
    if (a.print_hz <= 0.0) {
        std::cerr << "[teleop_local][ERR] print_hz must be > 0\n";
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// Key modifiers inference (best-effort; terminal raw mode limitation)
// -----------------------------------------------------------------------------
static std::uint16_t infer_mods_from_key(int key)
{
    std::uint16_t mods = shared::msg::kModNone;

    // Ctrl: ASCII control codes 1..26 typically correspond to Ctrl+A..Ctrl+Z.
    if (key >= 1 && key <= 26) {
        mods = static_cast<std::uint16_t>(mods | shared::msg::kModCtrl);
    }

    // Shift: guess from uppercase letters (caps-lock ambiguous).
    if (key >= 'A' && key <= 'Z') {
        mods = static_cast<std::uint16_t>(mods | shared::msg::kModShift);
    }

    return mods;
}

static void print_help()
{
    std::cerr
        << "\n[teleop_local] Keyboard -> KeyEvent SHM publisher\n"
        << "  - Publishes raw key codes only (no control semantics).\n"
        << "  - ESC (27) will exit teleop_local.\n\n";
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    Args args;
    if (!parse_args(argc, argv, args)) return 2;

    std::cerr << "[teleop_local] starting...\n"
              << "  shm_name=" << args.shm_name << "\n"
              << "  capacity=" << args.capacity << "\n"
              << "  poll_hz=" << args.poll_hz << " print_hz=" << args.print_hz << "\n"
              << "  exit_if_not_tty=" << (args.exit_if_not_tty ? 1 : 0)
              << " verbose=" << (args.verbose ? 1 : 0) << "\n";

    print_help();

#ifndef _WIN32
    TerminalRawGuard raw;
    const bool raw_ok = raw.enable();
    if (!raw_ok) {
        if (args.exit_if_not_tty) {
            std::cerr << "[teleop_local][ERR] raw mode not available; exiting.\n";
            return 10;
        }
        std::cerr << "[teleop_local][WARN] raw mode not available; continuing (will publish nothing).\n";
    }
#else
    std::cerr << "[teleop_local][ERR] Windows is not supported for TTY teleop.\n";
    return 10;
#endif

    // Init publisher (publisher owns creation & initialization).
    comm_gcs::ipc::keys::KeyEventPublisherShm pub;
    {
        comm_gcs::ipc::keys::KeyEventPublisherShm::Config cfg;
        cfg.enable    = true;
        cfg.shm_name  = args.shm_name;
        cfg.capacity  = args.capacity;

        cfg.truncate  = true; // clean ring layout after crash
        cfg.create_mode = 0666;

        if (!pub.init(cfg)) {
            std::cerr << "[teleop_local][ERR] KeyEventPublisherShm init failed.\n";
            return 11;
        }
    }

    const auto poll_period  = hz_to_period_ns(args.poll_hz);
    const auto print_period = hz_to_period_ns(args.print_hz);

    auto next_poll  = SteadyClock::now();
    auto next_print = SteadyClock::now();

    std::uint32_t seq = 0;

    std::uint64_t cnt_loops = 0;
    std::uint64_t cnt_read  = 0;
    std::uint64_t cnt_pub   = 0;

    static constexpr int kMaxBatch = 64;
    shared::msg::KeyEvent batch[kMaxBatch];

    while (!g_stop.load()) {
        const auto now = SteadyClock::now();

        if (now >= next_poll) {
            advance_next(next_poll, now, poll_period);
            ++cnt_loops;

#ifndef _WIN32
            if (raw_ok) {
                int n_batch = 0;

                for (; n_batch < kMaxBatch; ++n_batch) {
                    const int key = read_key_nonblock();
                    if (key < 0) break;

                    ++cnt_read;

                    if (key == 27) {
                        if (args.verbose) std::cerr << "[teleop_local] ESC received -> stopping\n";
                        g_stop.store(true);
                    }

                    shared::msg::KeyEvent ev{};
                    ev.version        = shared::msg::kKeyEventWireVersion;
                    ev.seq            = ++seq;
                    ev.stamp_mono_ns  = now_mono_ns();
                    ev.key            = static_cast<std::int32_t>(key);
                    ev.action         = shared::msg::KeyAction::kPress;
                    ev.mods           = infer_mods_from_key(key);

                    batch[n_batch] = ev;

                    if (args.verbose) {
                        std::cerr << "[teleop_local] key=" << key
                                  << " seq=" << ev.seq
                                  << " mods=" << ev.mods
                                  << "\n";
                    }
                }

                if (n_batch > 0) {
                    const std::size_t ok = pub.publish_many(batch, static_cast<std::size_t>(n_batch));
                    cnt_pub += ok;

                    if (ok != static_cast<std::size_t>(n_batch)) {
                        std::cerr << "[teleop_local][WARN] publish_many partial: ok=" << ok
                                  << " want=" << n_batch
                                  << " dropped=" << pub.dropped()
                                  << "\n";
                    }
                }
            }
#endif
        }

        if (now >= next_print) {
            advance_next(next_print, now, print_period);
            std::cerr << "[teleop_local] loops=" << cnt_loops
                      << " read=" << cnt_read
                      << " pub=" << cnt_pub
                      << " dropped=" << pub.dropped()
                      << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cerr << "[teleop_local] stopping...\n";
    pub.shutdown();
    return 0;
}
