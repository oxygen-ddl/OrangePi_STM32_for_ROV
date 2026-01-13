// gateway/apps/nav_view_dump.cpp
//
// nav_view_dump: dump NavStateView(SHM) for debugging/verification.
//
// Typical use:
//   ./nav_view_dump --nav-view-shm /rovctrl_nav_view_v1 --hz 5
//   ./nav_view_dump --hz 20 --csv logs/nav_view.csv
//
// Notes:
// - This tool does NOT feed control; it only observes the NavView SHM.
// - SHM contract must match: shared/shm/nav_state_view_shm.hpp
//
// Build (example):
//   add_executable(nav_view_dump apps/nav_view_dump.cpp)
//   target_link_libraries(nav_view_dump PRIVATE comm_gcs_lib)
//
// Namespace: keep comm_gcs as requested.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "shared/msg/nav_state_view.hpp"
#include "shared/shm/nav_state_view_shm.hpp"

namespace {

using ShmLayout = shared::shm::ShmLayout;

using SteadyClock = std::chrono::steady_clock;
std::atomic_bool g_stop{false};

void on_sigint(int) { g_stop.store(true); }

inline std::uint64_t now_mono_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            SteadyClock::now().time_since_epoch()).count());
}

struct Args {
    std::string nav_view_shm = "/rovctrl_nav_view_v1";

    double hz = 5.0;                // print/poll rate
    std::uint32_t max_age_ms = 500;  // stale threshold for diagnostics only

    // Output controls
    bool quiet = false;             // no console printing
    std::string csv_path;           // if non-empty, write CSV
    bool csv_header = true;

    // Behavior
    bool strict_header = true;      // if true, reject when header mismatch
};

static void usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "Options:\n"
        << "  --nav-view-shm <name>     default: /rovctrl_nav_view_v1\n"
        << "  --hz <hz>                 default: 5\n"
        << "  --max-age-ms <ms>         default: 500\n"
        << "  --csv <path>              write CSV (optional)\n"
        << "  --csv-header 0|1          default: 1\n"
        << "  --quiet 0|1               default: 0\n"
        << "  --strict-header 0|1       default: 1\n"
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
            if (i + 1 >= argc) { std::cerr << "[ERR] missing value for " << name << "\n"; return nullptr; }
            return argv[++i];
        };

        if (k == "-h" || k == "--help") {
            usage(argv[0]);
            return false;
        } else if (k == "--nav-view-shm") {
            const char* v = need("--nav-view-shm"); if (!v) return false;
            a.nav_view_shm = v;
        } else if (k == "--hz") {
            const char* v = need("--hz"); if (!v) return false;
            a.hz = std::atof(v);
        } else if (k == "--max-age-ms") {
            const char* v = need("--max-age-ms"); if (!v) return false;
            a.max_age_ms = static_cast<std::uint32_t>(std::strtoul(v, nullptr, 10));
        } else if (k == "--csv") {
            const char* v = need("--csv"); if (!v) return false;
            a.csv_path = v;
        } else if (k == "--csv-header") {
            const char* v = need("--csv-header"); if (!v) return false;
            bool b=false; if (!parse_bool(v,b)) { std::cerr << "[ERR] invalid bool: " << v << "\n"; return false; }
            a.csv_header = b;
        } else if (k == "--quiet") {
            const char* v = need("--quiet"); if (!v) return false;
            bool b=false; if (!parse_bool(v,b)) { std::cerr << "[ERR] invalid bool: " << v << "\n"; return false; }
            a.quiet = b;
        } else if (k == "--strict-header") {
            const char* v = need("--strict-header"); if (!v) return false;
            bool b=false; if (!parse_bool(v,b)) { std::cerr << "[ERR] invalid bool: " << v << "\n"; return false; }
            a.strict_header = b;
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
    if (a.nav_view_shm.empty() || a.nav_view_shm.front() != '/') {
        std::cerr << "[ERR] nav_view_shm must start with '/': " << a.nav_view_shm << "\n";
        return false;
    }
    return true;
}

static inline int sanitize_hz(const char* name, double hz,
                              int hz_min, int hz_max, int hz_def) noexcept
{
    if (!std::isfinite(hz) || hz <= 0.0) {
        std::cerr << "[nav_view_dump][ERR] " << name << "=" << hz
                  << " invalid (must be finite and >0). Fallback to " << hz_def << " Hz.\n";
        return hz_def;
    }

    const double r = std::round(hz);
    if (std::fabs(hz - r) > 1e-9) {
        std::cerr << "[nav_view_dump][ERR] " << name << "=" << hz
                  << " invalid (must be integer Hz). Fallback to " << hz_def << " Hz.\n";
        return hz_def;
    }

    const int ihz = static_cast<int>(r);
    if (ihz < hz_min || ihz > hz_max) {
        std::cerr << "[nav_view_dump][ERR] " << name << "=" << ihz
                  << " out of range [" << hz_min << "," << hz_max << "]. "
                  << "Fallback to " << hz_def << " Hz.\n";
        return hz_def;
    }

    return ihz;
}

static void csv_write_header(std::ofstream& os)
{
    os << "t_read_mono_ns"
       << ",shm_pub_mono_ns"
       << ",shm_pub_wall_ns"
       << ",age_ms"
       << ",valid"
       << ",health"
       << ",flags"
       << ",pos_x,pos_y,pos_z"
       << ",vel_x,vel_y,vel_z"
       << ",rpy_r,rpy_p,rpy_y"
       << ",depth_m"
       << ",omega_x,omega_y,omega_z"
       << ",acc_x,acc_y,acc_z"
       << "\n";
}

} // namespace

// -----------------------------------------------------------------------------
// Minimal SHM subscriber (tool-local) for shared::shm::ShmLayout
// -----------------------------------------------------------------------------
class NavViewShmReader final {
public:
    struct Config {
        std::string shm_name = "/rovctrl_nav_view_v1";
        bool strict_header   = true;
    };

    ~NavViewShmReader() noexcept { shutdown(); }

    bool init(const Config& cfg)
    {
        shutdown();
        cfg_ = cfg;

#ifdef _WIN32
        std::cerr << "[nav_view_dump] shm not supported on Windows.\n";
        return false;
#else
        shm_name_ = cfg_.shm_name;

        shm_fd_ = ::shm_open(shm_name_.c_str(), O_RDONLY, 0666);
        if (shm_fd_ < 0) {
            // Common when publisher not running
            return false;
        }

        struct ::stat st {};
        if (::fstat(shm_fd_, &st) != 0) {
            ::close(shm_fd_);
            shm_fd_ = -1;
            return false;
        }

        const std::size_t min_size = sizeof(shared::shm::ShmLayout);
        const std::size_t real_size = static_cast<std::size_t>(st.st_size);
        if (real_size < min_size) {
            ::close(shm_fd_);
            shm_fd_ = -1;
            return false;
        }

        shm_size_ = real_size;

        void* addr = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
        if (addr == MAP_FAILED) {
            ::close(shm_fd_);
            shm_fd_ = -1;
            shm_size_ = 0;
            return false;
        }

        shm_ptr_ = static_cast<const shared::shm::ShmLayout*>(addr);

        // Header sanity check (optional)
        if (!header_ok(*shm_ptr_, cfg_.strict_header)) {
            shutdown();
            return false;
        }

        initialized_ = true;
        return true;
#endif
    }

    void shutdown() noexcept
    {
#ifndef _WIN32
        if (shm_ptr_) {
            ::munmap(const_cast<shared::shm::ShmLayout*>(shm_ptr_), shm_size_);
            shm_ptr_ = nullptr;
        }
        if (shm_fd_ >= 0) {
            ::close(shm_fd_);
            shm_fd_ = -1;
        }
        shm_size_ = 0;
#endif
        initialized_ = false;
    }

    bool initialized() const noexcept { return initialized_; }

    struct Snapshot {
        shared::msg::NavStateView view{};
        std::uint64_t pub_mono_ns = 0;
        std::uint64_t pub_wall_ns = 0;
    };

    std::optional<Snapshot> poll_once() const noexcept
    {
#ifndef _WIN32
        if (!shm_ptr_) return std::nullopt;

        const auto* layout = shm_ptr_;
        const auto& hdr = layout->hdr;

        // Bounded seqlock retry
        for (int retry = 0; retry < 8; ++retry) {
            const std::uint64_t s1 = hdr.seq.load(std::memory_order_acquire);
            if (s1 & 1u) continue; // writer in progress

            // Read metadata first
            const std::uint64_t mono_ns = hdr.mono_ns;
            const std::uint64_t wall_ns = hdr.wall_ns;

            // Contract/ABI checks (fast fail)
            if (!header_ok(*layout, cfg_.strict_header)) {
                return std::nullopt;
            }

            shared::msg::NavStateView v{};
            std::memcpy(&v, &layout->payload, sizeof(v));

            const std::uint64_t s2 = hdr.seq.load(std::memory_order_acquire);
            if (s1 == s2 && ((s2 & 1u) == 0)) {
                Snapshot snap{};
                snap.view = v;
                snap.pub_mono_ns = mono_ns;
                snap.pub_wall_ns = wall_ns;
                return snap;
            }
        }
        return std::nullopt;
#else
        return std::nullopt;
#endif
    }

private:
    static bool header_ok(const shared::shm::ShmLayout& layout, bool strict) noexcept
    {
        const auto& h = layout.hdr;

        const bool ok =
            (h.magic == shared::shm::kNavViewMagic) &&
            (h.layout_ver == shared::shm::kNavViewLayoutVersion) &&
            (h.payload_ver == shared::msg::kNavStateViewWireVersion) &&
            (h.payload_size == sizeof(shared::shm::Payload)) &&
            (h.payload_align == alignof(shared::shm::Payload));

        if (strict) return ok;

        // Non-strict: require at least the magic+layout version. ABI mismatch still suspicious.
        return (h.magic == shared::shm::kNavViewMagic) &&
               (h.layout_ver == shared::shm::kNavViewLayoutVersion);
    }

private:
    Config cfg_{};
    bool initialized_ = false;

#ifndef _WIN32
    std::string shm_name_;
    int shm_fd_ = -1;
    std::size_t shm_size_ = 0;
    const shared::shm::ShmLayout* shm_ptr_ = nullptr;
#endif
};

int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    Args args;
    if (!parse_args(argc, argv, args)) return 2;

    // 强制整数 Hz + 安全范围 + 非法回退默认值（默认 5）
    const int hz = sanitize_hz("--hz", args.hz, /*min*/1, /*max*/200, /*def*/5);

    // 用整数 duration，避免 time_point 类型漂移
    using namespace std::chrono;
    const auto period = nanoseconds(static_cast<long long>(1000000000LL / hz));


    if (!args.quiet) {
        std::cerr << "[nav_view_dump] start\n"
                  << "  nav_view_shm=" << args.nav_view_shm << "\n"
                  << "  hz=" << hz << " max_age_ms=" << args.max_age_ms << "\n"
                  << "  csv=" << (args.csv_path.empty() ? "<none>" : args.csv_path) << "\n"
                  << "  strict_header=" << (args.strict_header ? 1 : 0) << "\n";
    }

    std::ofstream csv;
    if (!args.csv_path.empty()) {
        csv.open(args.csv_path, std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            std::cerr << "[nav_view_dump][ERR] cannot open csv: " << args.csv_path << "\n";
            return 3;
        }
        if (args.csv_header) csv_write_header(csv);
    }

    NavViewShmReader reader;
    NavViewShmReader::Config rcfg;
    rcfg.shm_name = args.nav_view_shm;
    rcfg.strict_header = args.strict_header;

    std::uint64_t cnt = 0, cnt_hit = 0, cnt_miss = 0, cnt_stale = 0;

    auto next_tick = SteadyClock::now();
    while (!g_stop.load()) {
        const auto now = SteadyClock::now();
        if (now < next_tick) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // 固定节拍推进，避免因为执行耗时导致节拍漂移
        next_tick += period;

        /// 若落后太多（例如程序卡顿），可以重置到 now + period 防止长期追赶
        if (now > next_tick + period * 5) { // 阈值可选
            next_tick = now + period;
        }

        ++cnt;

        if (!reader.initialized()) {
            // Retry init (publisher may start later)
            if (!reader.init(rcfg)) {
                ++cnt_miss;
                if (!args.quiet && (cnt % static_cast<std::uint64_t>(hz) == 0)) {
                    std::cerr << "[nav_view_dump] waiting shm " << args.nav_view_shm << "\n";
                }
                continue;
            }
        }

        auto snap_opt = reader.poll_once();
        if (!snap_opt.has_value()) {
            ++cnt_miss;
            continue;
        }

        ++cnt_hit;

        const auto& snap = *snap_opt;
        const std::uint64_t t_read_ns = now_mono_ns();

        const std::uint64_t age_ms = (snap.pub_mono_ns == 0)
            ? UINT64_MAX
            : (t_read_ns - snap.pub_mono_ns) / 1000000ull;

        const bool stale = (age_ms != UINT64_MAX) && (age_ms > args.max_age_ms);
        if (stale) ++cnt_stale;

        // Console
        if (!args.quiet) {
            std::cerr
                << "[nav_view_dump]"
                << " age_ms=" << (age_ms == UINT64_MAX ? -1LL : static_cast<long long>(age_ms))
                << " stale=" << (stale ? 1 : 0)
                << " valid=" << (snap.view.valid ? 1 : 0)
                << " health=" << static_cast<int>(snap.view.health)
                << " flags=0x" << std::hex << snap.view.flags << std::dec
                << " pos=(" << snap.view.pos[0] << "," << snap.view.pos[1] << "," << snap.view.pos[2] << ")"
                << " vel=(" << snap.view.vel[0] << "," << snap.view.vel[1] << "," << snap.view.vel[2] << ")"
                << " rpy=(" << snap.view.rpy[0] << "," << snap.view.rpy[1] << "," << snap.view.rpy[2] << ")"
                << " depth=" << snap.view.depth_m
                << "\n";
        }

        // CSV
        if (csv.is_open()) {
            csv << t_read_ns
                << "," << snap.pub_mono_ns
                << "," << snap.pub_wall_ns
                << "," << (age_ms == UINT64_MAX ? -1LL : static_cast<long long>(age_ms))
                << "," << (snap.view.valid ? 1 : 0)
                << "," << static_cast<int>(snap.view.health)
                << "," << snap.view.flags
                << "," << snap.view.pos[0] << "," << snap.view.pos[1] << "," << snap.view.pos[2]
                << "," << snap.view.vel[0] << "," << snap.view.vel[1] << "," << snap.view.vel[2]
                << "," << snap.view.rpy[0] << "," << snap.view.rpy[1] << "," << snap.view.rpy[2]
                << "," << snap.view.depth_m
                << "," << snap.view.omega_b[0] << "," << snap.view.omega_b[1] << "," << snap.view.omega_b[2]
                << "," << snap.view.acc_b[0]   << "," << snap.view.acc_b[1]   << "," << snap.view.acc_b[2]
                << "\n";
            csv.flush();
        }
    }

    if (!args.quiet) {
        std::cerr << "[nav_view_dump] stop"
                  << " cnt=" << cnt
                  << " hit=" << cnt_hit
                  << " miss=" << cnt_miss
                  << " stale=" << cnt_stale
                  << "\n";
    }

    return 0;
}
