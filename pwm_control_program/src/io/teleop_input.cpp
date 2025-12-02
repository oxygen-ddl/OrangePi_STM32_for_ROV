// src/io/teleop_input.cpp

#include "io/teleop_input.hpp"

#include <iostream>
#include <termios.h>
#include <unistd.h>

extern "C" {
#include "io/teleop_keyboard.h"
}

namespace {

// 终端 raw 模式 Guard，负责进入/退出 raw 模式
class TerminalRawGuard {
public:
    TerminalRawGuard()  = default;
    ~TerminalRawGuard() { disable(); }

    bool enable() {
        if (enabled_) return true;

        if (!::isatty(STDIN_FILENO)) {
            std::cerr << "[Teleop] stdin is not a TTY, raw mode disabled\n";
            return false;
        }
        if (::tcgetattr(STDIN_FILENO, &orig_) != 0) {
            std::perror("[Teleop] tcgetattr");
            return false;
        }

        ::termios raw = orig_;
        raw.c_lflag &= ~(ICANON | ECHO);  // 关闭行缓冲与回显
        raw.c_cc[VMIN]  = 0;              // 非阻塞读：立即返回
        raw.c_cc[VTIME] = 0;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            std::perror("[Teleop] tcsetattr");
            return false;
        }

        enabled_ = true;
        return true;
    }

    void disable() {
        if (!enabled_) return;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &orig_) != 0) {
            std::perror("[Teleop] tcsetattr(restore)");
        }
        enabled_ = false;
    }

private:
    bool     enabled_{false};
    ::termios orig_{};
};

// 单例 guard，整个进程共享一份终端 raw 状态
TerminalRawGuard g_raw_guard;

// 非阻塞读取一个按键，没有按键时返回 EOF
int read_key_nonblock()
{
    unsigned char ch = 0;
    ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    if (n == 1) {
        return static_cast<int>(ch);
    }
    return EOF;
}

} // anonymous namespace

namespace rovctrl::io {

using namespace rovctrl::control_core;

TeleopInputProvider::TeleopInputProvider()
    : initialized_(false)
    , raw_mode_(false)
    , mode_(ControlMode::MANUAL)  // 目前仅用于标记“这是手动输入源”
{}

TeleopInputProvider::~TeleopInputProvider()
{
    if (raw_mode_) {
        g_raw_guard.disable();
        raw_mode_ = false;
    }
}

bool TeleopInputProvider::init()
{
    if (initialized_) {
        return true;
    }

    if (!g_raw_guard.enable()) {
        std::cerr << "[Teleop] Failed to enable terminal raw mode\n";
        return false;
    }

    // 重置 Teleop 内部 DOF 状态（只清 DOF，不再操作 PWM）
    pwm_teleop_reset();
    // 打印一次帮助，提醒操作员键位含义
    pwm_teleop_print_help();

    raw_mode_    = true;
    initialized_ = true;

    std::cout << "[Teleop] Keyboard teleop ready. Press ESC to quit.\n";
    return true;
}

bool TeleopInputProvider::poll(ControlState&     state,
                               ControlReference& ref,
                               bool&             request_exit)
{
    (void)state;  // 当前 Teleop 不使用导航状态，先显式标记未用
    request_exit = false;

    if (!initialized_) {
        if (!init()) {
            return false;
        }
    }

    // 非阻塞读取一个键
    int key = read_key_nonblock();
    if (key != EOF) {
        int rc = pwm_teleop_handle_key(key);
        if (rc == PWM_TELEOP_EXIT_REQUEST) {
            // Teleop 请求退出，由控制循环决定停机流程
            request_exit = true;
        } else if (rc < 0) {
            std::cerr << "[Teleop] pwm_teleop_handle_key error rc=" << rc << "\n";
            // 这里先不把错误视为致命错误，保持返回 true，交由上层决定是否退出
        }
        // rc == PWM_TELEOP_HANDLED / PWM_TELEOP_IGNORED 均正常继续
    }

    // 从 C 层读取当前 DOF 状态
    pwm_teleop_state_t kstate{};
    pwm_teleop_get_state(&kstate);

    // 映射到 ControlReference
    //
    // 情况 A：如果你已经采用了 DofCommand 结构：
    //
    //   ref.dof_cmd.surge = kstate.cmd_surge;
    //   ref.dof_cmd.sway  = kstate.cmd_sway;
    //   ref.dof_cmd.heave = kstate.cmd_heave;
    //   ref.dof_cmd.yaw   = kstate.cmd_yaw;
    //   ref.dof_cmd.roll  = kstate.cmd_roll;
    //   ref.dof_cmd.pitch = kstate.cmd_pitch;
    //   ref.use_dof_cmd   = true;
    //
    // 情况 B：如果当前 ControlReference 仍是平铺字段（surge/sway/...），
    // 先直接按旧字段名赋值，等后续你把 control_types.hpp 升级后再统一改用 A。
    //
    ref.surge = kstate.cmd_surge;
    ref.sway  = kstate.cmd_sway;
    ref.heave = kstate.cmd_heave;
    ref.yaw   = kstate.cmd_yaw;
    ref.roll  = kstate.cmd_roll;
    ref.pitch = kstate.cmd_pitch;

    return true;
}

void TeleopInputProvider::reset()
{
    pwm_teleop_reset();
}

} // namespace rovctrl::io
