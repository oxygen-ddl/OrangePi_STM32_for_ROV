#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>

#include <termios.h>
#include <unistd.h>

#include "platform/pwm_client.hpp"
#include "platform/io/teleop_keyboard.h"

// ============ 匿名命名空间：工具与局部类型 ============

namespace {

std::atomic<bool> g_stop{false};

void signal_handler(int)
{
    g_stop.store(true);
}

// 终端 raw 模式 RAII 封装
struct TerminalRawGuard {
    bool          active{false};
    struct termios old_tio{};

    TerminalRawGuard() = default;

    bool enter() {
        if (!::isatty(STDIN_FILENO)) {
            std::cerr << "[WARN] stdin is not a TTY, teleop may not work as expected.\n";
            return false;
        }
        if (::tcgetattr(STDIN_FILENO, &old_tio) != 0) {
            std::perror("tcgetattr");
            return false;
        }

        struct termios new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO); // 关闭行缓冲和回显
        new_tio.c_cc[VMIN]  = 0;             // 非阻塞读取
        new_tio.c_cc[VTIME] = 0;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) != 0) {
            std::perror("tcsetattr");
            return false;
        }
        active = true;
        return true;
    }

    ~TerminalRawGuard() {
        if (active) {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        }
    }
};

// 非阻塞读 1 字节按键，返回 int（EOF 表示无按键）
int read_key_nonblock()
{
    unsigned char ch = 0;
    ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    if (n == 1) {
        return static_cast<int>(ch);
    }
    return EOF;
}

// 简单 PID 占位（以后可以独立成 controllers/pid_controller.hpp）
struct SimplePID {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    double integral{0.0};
    double prev_error{0.0};
    bool   has_prev{false};

    void reset() {
        integral   = 0.0;
        prev_error = 0.0;
        has_prev   = false;
    }

    double step(double setpoint, double measured, double dt)
    {
        if (dt <= 0.0) return 0.0;

        double error = setpoint - measured;
        integral += error * dt;

        double derivative = 0.0;
        if (has_prev) {
            derivative = (error - prev_error) / dt;
        } else {
            has_prev = true;
        }

        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

enum class ControlMode {
    TELEOP_MANUAL,
    PID_DEMO
};

const char* mode_to_cstr(ControlMode m)
{
    return (m == ControlMode::TELEOP_MANUAL) ? "TELEOP_MANUAL" : "PID_DEMO";
}

// 专门处理键盘输入的函数
void handle_key(int key, ControlMode& mode)
{
    if (key == EOF) {
        return;
    }

    // ESC: 直接请求退出
    if (key == 27) {
        std::cout << "[INFO] ESC pressed, exiting...\n";
        g_stop.store(true);
        return;
    }

    // 大小写统一
    int key_upper = key;
    if (key_upper >= 'a' && key_upper <= 'z') {
        key_upper = key_upper - 'a' + 'A';
    }

    // P: 模式切换
    if (key_upper == 'P') {
        mode = (mode == ControlMode::TELEOP_MANUAL)
             ? ControlMode::PID_DEMO
             : ControlMode::TELEOP_MANUAL;
        std::cout << "[MODE] switched to " << mode_to_cstr(mode) << "\n";
        return;
    }

    // Z: 打印帮助
    if (key_upper == 'Z') {
        pwm_teleop_print_help();
        return;
    }

    // TELEOP 模式下，把按键交给底层 C teleop 模块
    if (mode == ControlMode::TELEOP_MANUAL) {
        int r = pwm_teleop_handle_key(key);
        if (r < 0) {
            std::cerr << "[ERR] pwm_teleop_handle_key rc=" << r << "\n";
        }
    }
    // PID_DEMO 模式下，暂不使用键盘控制（未来可以扩展为调整 setpoint 等）
}

} // namespace

// ============ 主程序 ============

int main(int argc, char** argv)
{
    // 信号处理（Ctrl+C 安全退出）
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 命令行参数：--mode pid / --mode teleop
    ControlMode mode = ControlMode::TELEOP_MANUAL;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--mode" && i + 1 < argc) {
            std::string m = argv[++i];
            if (m == "pid") {
                mode = ControlMode::PID_DEMO;
            } else {
                mode = ControlMode::TELEOP_MANUAL;
            }
        }
    }

    std::cout << "[INFO] pwm_control_program starting...\n";
    std::cout << "       initial mode: " << mode_to_cstr(mode) << "\n";

    // ========== 初始化 PWM 客户端（libpwm_host + pwm_control） ==========

    rovctrl::platform::PwmClientConfig cfg;
    cfg.ctrl_hz      = 100.0f;   // 主循环频率
    cfg.max_step_pct = 0.2f;     // 每步最大占空比变化

    rovctrl::platform::PwmClient pwm_client;
    if (!pwm_client.init(cfg)) {
        std::cerr << "[ERR] PwmClient init failed: "
                  << pwm_client.status().last_error_msg << "\n";
        return 1;
    }

    // 上电后先全部回中位
    if (int rc = pwm_client.setAllMid(); rc < 0) {
        std::cerr << "[ERR] setAllMid failed: "
                  << pwm_client.status().last_error_msg << "\n";
        return 1;
    }

    // ========== 终端 raw 模式（用于 teleop） ==========

    TerminalRawGuard term_guard;
    term_guard.enter();   // 失败也不致命，只是 teleop 不好用

    pwm_teleop_print_help();
    std::cout << "[INFO] 按 Z 可随时重新打印帮助；按 ESC 退出程序。\n";
    std::cout << "[INFO] 运行中按 P 键在 TELEOP 和 PID_DEMO 模式之间切换。\n";

    // ========== PID DEMO 占位控制器（暂不接传感器） ==========

    SimplePID pid_demo;
    pid_demo.kp = 0.5;
    pid_demo.ki = 0.0;
    pid_demo.kd = 0.0;

    double pid_setpoint = 0.0;  // 将来可绑定到目标深度/姿态等
    double pid_measured = 0.0;  // 将来接 IMU/深度/速度反馈
    double pid_output   = 0.0;  // 将来映射为某个 DOF thrust

    // ========== 主控制循环：固定周期 100 Hz ==========

    const double loop_hz     = cfg.ctrl_hz;
    const auto   loop_period = std::chrono::duration<double>(1.0 / loop_hz);

    auto last_tick = std::chrono::steady_clock::now();
    auto next_tick = last_tick + loop_period;

    int step_err_count = 0;

    while (!g_stop.load()) {
        // 固定周期
        auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
            now = std::chrono::steady_clock::now();
        }

        // 计算 dt（用于 PID，将来也可以给控制器用）
        double dt = std::chrono::duration<double>(now - last_tick).count();
        if (dt <= 0.0) {
            dt = 1.0 / loop_hz;  // 防守型兜底
        }
        last_tick = now;
        next_tick = now + loop_period;

        // ---------- 键盘输入处理 ----------
        int key = read_key_nonblock();
        handle_key(key, mode);
        if (g_stop.load()) {
            break;  // 在 handle_key 里按 ESC 会置位 g_stop
        }

        // ---------- 控制逻辑 ----------
        switch (mode) {
        case ControlMode::TELEOP_MANUAL:
            // 手动模式：具体 DOF→占空比 映射由 teleop_keyboard + pwm_control 完成
            break;

        case ControlMode::PID_DEMO:
            // 将来：从传感器更新 pid_measured
            pid_output = pid_demo.step(pid_setpoint, pid_measured, dt);

            // TODO: 把 pid_output 映射到某个 DOF 的 thrust，再映射为 8 路占空比并调用：
            // std::array<float, rovctrl::platform::kNumPwmChannels> pct{};
            // ... 计算 pct ...
            // pwm_client.setTargets(pct);
            break;
        }

        // ---------- 安全层 step（限斜率 + 分组 + 下发） ----------
        int step_rc = pwm_client.step();
        if (step_rc < 0) {
            ++step_err_count;
            if (step_err_count <= 3 || (step_err_count % 100 == 0)) {
                std::cerr << "[ERR] pwm_client.step rc=" << step_rc
                          << " msg=" << pwm_client.status().last_error_msg << "\n";
            }
            // 如果连续大量报错，可以考虑直接退出
            if (step_err_count > 1000) {
                std::cerr << "[FATAL] pwm_client.step errors too many, aborting.\n";
                break;
            }
        } else {
            step_err_count = 0;
        }
    }

    // 退出前平滑归中
    std::cout << "[INFO] emergencyStop(1.0s) before shutdown...\n";
    pwm_client.emergencyStop(1.0f);
    pwm_client.shutdown();

    std::cout << "[INFO] pwm_control_program exited.\n";
    return 0;
}
