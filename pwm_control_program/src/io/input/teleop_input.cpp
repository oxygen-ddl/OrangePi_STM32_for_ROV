// src/io/teleop_input.cpp

#include "io/input/teleop_input.hpp"
#include "platform/timebase.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <unistd.h>

extern "C" {
#include "io/teleop/teleop_keyboard.h"
}

namespace {

constexpr int kNoKey = -1;
constexpr int kMaxKeysPerPoll = 16;

// ----------------------------------------------------------------------------
// Terminal raw mode guard (process-wide)
// ----------------------------------------------------------------------------
class TerminalRawGuard {
public:
    TerminalRawGuard()  = default;
    ~TerminalRawGuard() { disable(); }

    bool enable() {
        if (enabled_) return true;

        if (!::isatty(STDIN_FILENO)) {
            std::cerr << "[Teleop] [WARN] stdin is not a TTY; raw mode disabled.\n";
            return false;
        }

        if (::tcgetattr(STDIN_FILENO, &orig_) != 0) {
            std::cerr << "[Teleop] [ERR] tcgetattr failed: " << std::strerror(errno) << "\n";
            return false;
        }

        ::termios raw = orig_;
        tcflag_t mask = static_cast<tcflag_t>(ICANON) | static_cast<tcflag_t>(ECHO);
        raw.c_lflag &= ~mask;     // no canonical, no echo
        raw.c_cc[VMIN]  = 0;      // non-blocking
        raw.c_cc[VTIME] = 0;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            std::cerr << "[Teleop] [ERR] tcsetattr(raw) failed: " << std::strerror(errno) << "\n";
            return false;
        }

        enabled_ = true;
        return true;
    }

    void disable() {
        if (!enabled_) return;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &orig_) != 0) {
            std::cerr << "[Teleop] [WARN] tcsetattr(restore) failed: " << std::strerror(errno) << "\n";
        }
        enabled_ = false;
    }

    bool enabled() const { return enabled_; }

private:
    bool      enabled_{false};
    ::termios orig_{};
};

// Process-wide singleton
TerminalRawGuard g_raw_guard;

// ----------------------------------------------------------------------------
// Map C teleop mode -> control_core::ControlMode
// ----------------------------------------------------------------------------
static rovctrl::control_core::ControlMode map_mode(pwm_teleop_mode_t m)
{
    using CM = rovctrl::control_core::ControlMode;
    switch (m) {
    case PWM_TELEOP_MODE_MANUAL:
        return CM::kManual;

    case PWM_TELEOP_MODE_HOLD_ATT:
    case PWM_TELEOP_MODE_HOLD_DEPTH:
    case PWM_TELEOP_MODE_VELOCITY:
    case PWM_TELEOP_MODE_POSITION:
    case PWM_TELEOP_MODE_MPC:
        return CM::kAuto;

    case PWM_TELEOP_MODE_NONE:
    default:
        return CM::kNone;
    }
}

} // anonymous namespace

namespace rovctrl::io {

namespace cc = rovctrl::control_core;

// 注意：头文件中 TeleopInputProvider() / ~TeleopInputProvider() 已经 = default
// 这里不能再写定义，否则会触发 “definition of explicitly-defaulted ...”

bool TeleopInputProvider::init()
{
    if (initialized_) {
        return true;
    }

    // If raw mode cannot be enabled, do NOT hard-fail the whole control process.
    if (!g_raw_guard.enable()) {
        std::cerr << "[Teleop] [WARN] Terminal raw mode not available; teleop will output empty intent.\n";
        initialized_ = true;
        raw_mode_    = false;
        return true;
    }

    pwm_teleop_reset();
    pwm_teleop_print_help();

    raw_mode_    = true;   // legacy (kept for now)
    initialized_ = true;

    std::cout << "[Teleop] Keyboard teleop ready. Press ESC to quit.\n";
    return true;
}

bool TeleopInputProvider::poll(cc::ControlState& state, cc::ControlIntent& intent)
{
    (void)state;

    intent = cc::ControlIntent{};
    intent.clear_payload();

    // Ensure initialized (non-fatal even if not a TTY)
    if (!initialized_) {
        (void)init();
    }

    // seq: one increment per poll cycle (metadata only once)
    intent.seq = ++seq_;
    finalize_intent(intent);

    // If raw mode is not enabled (not a TTY), we cannot read keys.
    if (!g_raw_guard.enabled()) {
        return true;
    }

    // Drain up to N keys per cycle
    for (int i = 0; i < kMaxKeysPerPoll; ++i) {
        const int key = read_key_nonblock();
        if (key == kNoKey) break;

        pwm_teleop_event_t evt{};
        std::memset(&evt, 0, sizeof(evt));

        const int rc = pwm_teleop_handle_key(key);

        if (rc == PWM_TELEOP_EXIT_REQUEST) {
            intent.request_exit  = true;
            exit_requested_      = true; // legacy
        } else if (rc < 0) {
            std::cerr << "[Teleop] [WARN] pwm_teleop_handle_key_ex error rc=" << rc << "\n";
        }

        // ---- Extended events -> ControlIntent ----
        if (evt.evt_mask & PWM_TELEOP_EVT_MODE) {
            intent.has_mode_request = true;
            intent.mode_request     = map_mode(evt.mode);
        }

        if (evt.evt_mask & PWM_TELEOP_EVT_ARM) {
            intent.has_arm_cmd = true;
            intent.arm         = true;
        }
        if (evt.evt_mask & PWM_TELEOP_EVT_DISARM) {
            intent.has_arm_cmd = true;
            intent.disarm      = true;
        }

        if (evt.evt_mask & PWM_TELEOP_EVT_ESTOP) {
            intent.has_estop_cmd = true;
            intent.estop         = true;
        }

        if (evt.evt_mask & PWM_TELEOP_EVT_REF_DELTA) {
            intent.has_ref_delta = true;

            // TODO: 将 evt 的 ref_delta 写入 intent.ref_delta（需结合你们 ControlReference 字段定义）
            (void)evt;
        }
    }

    // Teleop DOF state is authoritative each cycle
    pwm_teleop_state_t kstate{};
    pwm_teleop_get_state(&kstate);

    intent.teleop_dof_cmd.surge = kstate.cmd_surge;
    intent.teleop_dof_cmd.sway  = kstate.cmd_sway;
    intent.teleop_dof_cmd.heave = kstate.cmd_heave;
    intent.teleop_dof_cmd.yaw   = kstate.cmd_yaw;
    intent.teleop_dof_cmd.roll  = kstate.cmd_roll;
    intent.teleop_dof_cmd.pitch = kstate.cmd_pitch;
    intent.has_teleop_dof       = true;

    // refresh timestamp at end of poll (optional but often desired)
    finalize_intent(intent);
    return true;
}

void TeleopInputProvider::reset()
{
    pwm_teleop_reset();
    exit_requested_ = false;
    // raw mode lifecycle is owned by TerminalRawGuard (process-wide)
}

int TeleopInputProvider::read_key_nonblock()
{
    unsigned char ch = 0;
    const ssize_t n = ::read(STDIN_FILENO, &ch, 1);

    if (n == 1) {
        return static_cast<int>(ch);
    }

    if (n == 0) {
        return kNoKey;
    }

    // n < 0
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return kNoKey;
    }

    std::cerr << "[Teleop] [WARN] read(STDIN) failed: " << std::strerror(errno) << "\n";
    return kNoKey;
}

void TeleopInputProvider::finalize_intent(cc::ControlIntent& intent)
{
    // Unified timebase: monotonic ns
    intent.stamp_ns = static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());

    // Teleop input TTL: conservative default
    if (intent.ttl_ms == 0) {
        intent.ttl_ms = 200;
    }

    last_event_ns_ = static_cast<std::int64_t>(intent.stamp_ns);
}

} // namespace rovctrl::io
