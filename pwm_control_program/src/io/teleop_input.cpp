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
        // 避免 -Wsign-conversion：在 tcflag_t 域内构造 mask
        tcflag_t mask = static_cast<tcflag_t>(ICANON) | static_cast<tcflag_t>(ECHO);
        raw.c_lflag &= ~mask;     // 关闭行缓冲与回显
        raw.c_cc[VMIN]  = 0;      // 非阻塞读：立即返回
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
    bool      enabled_{false};
    ::termios orig_{};
};

// 单例 guard，整个进程共享一份终端 raw 状态
TerminalRawGuard g_raw_guard;

} // anonymous namespace

namespace rovctrl::io {

TeleopInputProvider::TeleopInputProvider()
    : initialized_(false)
    , raw_mode_(false)
    , exit_requested_(false)
    , last_t_ns_{0}
    , mode_(rovctrl::control_core::ControlMode::Manual)  // 这里只是个标签
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

bool TeleopInputProvider::poll(rovctrl::control_core::ControlState&     state,
                               rovctrl::control_core::ControlReference& ref,
                               bool&                                     request_exit)
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
            request_exit    = true;
            exit_requested_ = true;
        } else if (rc < 0) {
            std::cerr << "[Teleop] pwm_teleop_handle_key error rc=" << rc << "\n";
            // 暂不视为致命错误，保持返回 true，由上层决定是否退出
        }
        // rc == PWM_TELEOP_HANDLED / PWM_TELEOP_IGNORED 均正常继续
    }

    // 从 C 层读取当前 DOF 状态
    pwm_teleop_state_t kstate{};
    pwm_teleop_get_state(&kstate);

    // 映射到 ControlReference::dof_cmd
    ref.dof_cmd.surge = kstate.cmd_surge;
    ref.dof_cmd.sway  = kstate.cmd_sway;
    ref.dof_cmd.heave = kstate.cmd_heave;
    ref.dof_cmd.yaw   = kstate.cmd_yaw;
    ref.dof_cmd.roll  = kstate.cmd_roll;
    ref.dof_cmd.pitch = kstate.cmd_pitch;
    ref.use_dof_cmd   = true;

    // 如果将来引入统一时间基，可以在此记录最后一次按键时间
    // last_t_ns_ = rovctrl::platform::timebase::now_ns();

    return true;
}

void TeleopInputProvider::reset()
{
    pwm_teleop_reset();
    exit_requested_ = false;
    // 不修改终端 raw 模式，由 init()/析构负责
}

// 成员函数版本的非阻塞读键，与头文件声明一致
int TeleopInputProvider::read_key_nonblock()
{
    unsigned char ch = 0;
    ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    if (n == 1) {
        return static_cast<int>(ch);
    }
    return EOF;
}

} // namespace rovctrl::io
