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
        raw.c_lflag &= ~mask;
        raw.c_cc[VMIN]  = 0;
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

TerminalRawGuard g_raw_guard;

} // namespace

namespace rovctrl::io {

namespace cc = rovctrl::control_core;

bool TeleopInputProvider::init()
{
    if (initialized_) return true;

    if (!g_raw_guard.enable()) {
        std::cerr << "[Teleop] [WARN] Terminal raw mode not available; teleop will output empty intent.\n";
        initialized_ = true;
        raw_mode_    = false;
        return true;
    }

    pwm_teleop_reset();
    pwm_teleop_print_help();

    raw_mode_    = true;   // legacy
    initialized_ = true;

    std::cout << "[Teleop] Keyboard teleop ready. Press ESC to quit.\n";
    return true;
}

bool TeleopInputProvider::poll(cc::ControlState& state, cc::ControlIntent& intent)
{
    (void)state;

    intent = cc::ControlIntent{};
    intent.clear_payload();

    if (!initialized_) (void)init();

    intent.seq = ++seq_;

    // 1) 先给一个“当前时间戳”，用于本周期判断
    finalize_intent(intent); // 仅设置 stamp/ttl

    // 非 TTY：无法读键，明确输出“无遥控输入”
    if (!g_raw_guard.enabled()) {
        intent.has_teleop_dof = false;
        return true;
    }

    bool got_key_this_cycle = false;

    for (int i = 0; i < kMaxKeysPerPoll; ++i) {
        const int key = read_key_nonblock();
        if (key == kNoKey) break;

        got_key_this_cycle = true;

        const int rc = pwm_teleop_handle_key(key);

        if (rc == PWM_TELEOP_EXIT_REQUEST) {
            intent.request_exit = true;
            exit_requested_     = true;

            // 关键：立刻恢复终端，避免用户回到 shell 后“看起来卡死”
            g_raw_guard.disable();
            break;
        }

        if (rc < 0) {
            std::cerr << "[Teleop] [WARN] pwm_teleop_handle_key error rc=" << rc << "\n";
        }
    }

    // 2) 再刷新一次时间戳，让 now_ns 更接近“处理完按键后的时刻”
    finalize_intent(intent);
    const std::uint64_t now_ns = intent.stamp_ns;

    // 只有“确实读到按键”才更新 last_event_ns_
    if (got_key_this_cycle) {
        last_event_ns_ = static_cast<std::int64_t>(now_ns);
    }

    const std::uint64_t ttl_ns =
        static_cast<std::uint64_t>(intent.ttl_ms) * 1000000ULL;

    const bool teleop_alive =
        (last_event_ns_ > 0) &&
        (now_ns - static_cast<std::uint64_t>(last_event_ns_) <= ttl_ns);

    if (teleop_alive) {
        pwm_teleop_state_t kstate{};
        pwm_teleop_get_state(&kstate);

        intent.teleop_dof_cmd.surge = kstate.cmd_surge;
        intent.teleop_dof_cmd.sway  = kstate.cmd_sway;
        intent.teleop_dof_cmd.heave = kstate.cmd_heave;
        intent.teleop_dof_cmd.yaw   = kstate.cmd_yaw;
        intent.teleop_dof_cmd.roll  = kstate.cmd_roll;
        intent.teleop_dof_cmd.pitch = kstate.cmd_pitch;
        intent.has_teleop_dof       = true;
    } else {
        intent.teleop_dof_cmd = cc::DofCommand{};
        intent.has_teleop_dof = false;
    }

    return true;
}

void TeleopInputProvider::reset()
{
    pwm_teleop_reset();
    exit_requested_ = false;

    // 双保险：恢复终端
    g_raw_guard.disable();
}

int TeleopInputProvider::read_key_nonblock()
{
    unsigned char ch = 0;
    const ssize_t n = ::read(STDIN_FILENO, &ch, 1);

    if (n == 1) return static_cast<int>(ch);
    if (n == 0) return kNoKey;

    if (errno == EAGAIN || errno == EWOULDBLOCK) return kNoKey;

    std::cerr << "[Teleop] [WARN] read(STDIN) failed: " << std::strerror(errno) << "\n";
    return kNoKey;
}

void TeleopInputProvider::finalize_intent(cc::ControlIntent& intent)
{
    intent.stamp_ns = static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());

    if (intent.ttl_ms == 0) {
        intent.ttl_ms = 200;
    }

    // 关键：不要在这里更新 last_event_ns_
}

} // namespace rovctrl::io
