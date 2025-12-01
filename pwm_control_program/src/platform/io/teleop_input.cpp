// src/platform/io/teleop_input.cpp

#include "platform/io/teleop_input.hpp"

#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "platform/io/teleop_keyboard.h"


namespace {

// 终端 raw 模式 Guard，负责进入/退出 raw 模式
class TerminalRawGuard {
public:
    TerminalRawGuard()  = default;
    ~TerminalRawGuard() { disable(); }

    bool enable() {
        if (enabled_) return true;
        if (!isatty(STDIN_FILENO)) {
            std::cerr << "[Teleop] stdin is not a TTY, raw mode disabled\n";
            return false;
        }
        if (tcgetattr(STDIN_FILENO, &orig_) != 0) {
            perror("[Teleop] tcgetattr");
            return false;
        }
        termios raw = orig_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;  // 非阻塞读：立即返回
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            perror("[Teleop] tcsetattr");
            return false;
        }
        enabled_ = true;
        return true;
    }

    void disable() {
        if (!enabled_) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
        enabled_ = false;
    }

private:
    bool     enabled_ = false;
    termios  orig_{};
};

TerminalRawGuard g_raw_guard;

} // anonymous namespace

namespace rovctrl::io {

TeleopInputProvider::TeleopInputProvider()
    : initialized_(false),
      raw_mode_(false),
      mode_(rovctrl::control_core::ControlMode::MANUAL)
{}

TeleopInputProvider::~TeleopInputProvider() {
    if (raw_mode_) {
        g_raw_guard.disable();
    }
}

bool TeleopInputProvider::init() {
    if (!g_raw_guard.enable()) {
        std::cerr << "[Teleop] Failed to enable terminal raw mode\n";
        return false;
    }

    // 重置 Teleop 内部状态（清零 DOF + PWM 回中位）
    pwm_teleop_reset();
    // 打印一次帮助，提醒操作员键位含义
    pwm_teleop_print_help();

    raw_mode_    = true;
    initialized_ = true;

    std::cout << "[Teleop] Keyboard teleop ready. Press ESC to quit.\n";
    return true;
}

bool TeleopInputProvider::poll(rovctrl::control_core::ControlState& /*state*/,
                               rovctrl::control_core::ControlReference& ref,
                               bool& request_exit) {
    request_exit = false;

    if (!initialized_) {
        if (!init()) {
            return false;
        }
    }

    // 非阻塞读取一个键
    unsigned char ch = 0;
    ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    if (n > 0) {
        int rc = pwm_teleop_handle_key(static_cast<int>(ch));
        if (rc == PWM_TELEOP_EXIT_REQUEST) {
            // Teleop 请求退出
            request_exit = true;
        } else if (rc < 0) {
            std::cerr << "[Teleop] pwm_teleop_handle_key error rc=" << rc << "\n";
        }
        // rc == PWM_TELEOP_HANDLED / PWM_TELEOP_IGNORED 都不影响主循环继续
    }

    // 从 C 层读取当前 DOF 状态
    pwm_teleop_state_t kstate{};
    pwm_teleop_get_state(&kstate);

    // 映射到 ControlReference（按你当前的字段命名）
    // 这里假定 ControlReference 中有 surge/sway/heave/roll/pitch/yaw 六个字段，
    // 和之前你在 teleop_input.cpp 中用的一致。
    ref.surge = kstate.cmd_surge;
    ref.sway  = kstate.cmd_sway;
    ref.heave = kstate.cmd_heave;
    ref.roll  = kstate.cmd_roll;
    ref.pitch = kstate.cmd_pitch;
    ref.yaw   = kstate.cmd_yaw;

    // 如果将来你希望 emergency_stop 向上冒泡，可以在 pwm_teleop_state_t 或单独接口中加标志位，
    // 然后这里设置某个 ref 或 state 的字段，由控制循环决定是否调用 pwm_client.emergencyStop()

    return true;
}

void TeleopInputProvider::reset() {
    pwm_teleop_reset();
}

} // namespace rovctrl::io


