#include "libpwm_host.h"
#include "pwm_control.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

static std::atomic<bool> g_running{true};
static void on_sigint(int){ g_running = false; }

/* ----------- 终端 raw 模式工具 ----------- */

static struct termios g_old_tio;
static bool g_term_inited = false;

static void term_set_raw()
{
    if (g_term_inited) return;
    struct termios new_tio;
    tcgetattr(STDIN_FILENO, &g_old_tio);
    new_tio = g_old_tio;
    cfmakeraw(&new_tio);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    // 非阻塞 stdin
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    g_term_inited = true;
}

static void term_restore()
{
    if (!g_term_inited) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
    g_term_inited = false;
}

/* ----------- 简单统计输出 ----------- */

static void print_stats(const char* tag)
{
    pwm_host_stats_t st{};
    pwm_host_get_stats(&st);
    double rtt = pwm_host_last_rtt_ms();
    std::cout << "[STAT][" << tag << "] tx_pwm=" << st.tx_pwm
              << " tx_hb=" << st.tx_hb
              << " rx_hb_ack=" << st.rx_hb_ack
              << " tx_err=" << st.tx_err
              << " rx_err=" << st.rx_err
              << " rtt=" << (rtt >= 0 ? rtt : -1.0) << " ms\n";
}

/* ----------- Teleop 状态与映射 ----------- */

// 虚拟操纵：-1..1
static float g_surge = 0.0f; // W/S
static float g_yaw   = 0.0f; // A/D
static float g_heave = 0.0f; // R/F

static float clamp01(float v)
{
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}

// 把 g_surge/g_yaw/g_heave 映射成 8 通道占空比目标并下发到 pwm_control
static int update_targets_from_command()
{
    float pct[PWM_HOST_CH_NUM];

    const float base = PWM_HOST_PCT_MID;  // 7.5%
    const float gain = 1.0f;              // 每个指令单位对应 1% 的变化（可调）

    // CH1..4: surge + yaw
    pct[0] = base + gain * ( g_surge + g_yaw); // CH1
    pct[1] = base + gain * ( g_surge - g_yaw); // CH2
    pct[2] = base + gain * ( g_surge + g_yaw); // CH3
    pct[3] = base + gain * ( g_surge - g_yaw); // CH4

    // CH5..8: heave
    for (int i = 4; i < 8; ++i) {
        pct[i] = base + gain * g_heave;
    }

    // 裁剪到 5~10%
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        if (pct[i] < PWM_HOST_PCT_MIN) pct[i] = PWM_HOST_PCT_MIN;
        if (pct[i] > PWM_HOST_PCT_MAX) pct[i] = PWM_HOST_PCT_MAX;
    }

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

/* ----------- 键盘处理：根据按键调节命令 ----------- */

static void print_help()
{
    std::cout <<
        "\n===== Teleop 键位说明 =====\n"
        "  W / S : surge 前进 / 后退\n"
        "  A / D : yaw 左转 / 右转\n"
        "  R / F : heave 上升 / 下降\n"
        "  M     : 所有通道回中位 (7.5%)\n"
        "  SPACE : 紧急平滑归中位 (1.0s)\n"
        "  Q     : 退出程序\n"
        "  H     : 显示本帮助\n"
        "当前 cmd 映射: base=7.5%, gain=1.0% * cmd [-1..1]\n"
        "注意：初次测试请拆螺旋桨 / 仅接示波器！\n"
        "============================\n\n";
}

// 每次循环调用，非阻塞读取键盘，有按键就更新 g_surge 等
static void teleop_handle_key()
{
    char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) return; // 无键

    const float step = 0.1f;

    switch (c) {
    case 'w': case 'W':
        g_surge = clamp01(g_surge + step);
        std::cout << "[KEY] W surge=" << g_surge << "\n";
        break;
    case 's': case 'S':
        g_surge = clamp01(g_surge - step);
        std::cout << "[KEY] S surge=" << g_surge << "\n";
        break;
    case 'a': case 'A':
        g_yaw = clamp01(g_yaw + step);
        std::cout << "[KEY] A yaw=" << g_yaw << "\n";
        break;
    case 'd': case 'D':
        g_yaw = clamp01(g_yaw - step);
        std::cout << "[KEY] D yaw=" << g_yaw << "\n";
        break;
    case 'r': case 'R':
        g_heave = clamp01(g_heave + step);
        std::cout << "[KEY] R heave=" << g_heave << "\n";
        break;
    case 'f': case 'F':
        g_heave = clamp01(g_heave - step);
        std::cout << "[KEY] F heave=" << g_heave << "\n";
        break;

    case 'm': case 'M':
        g_surge = g_yaw = g_heave = 0.0f;
        std::cout << "[KEY] M -> all command reset to 0 (中位)\n";
        pwm_ctrl_set_all_target_mid();
        break;

    case ' ':
        std::cout << "[KEY] SPACE -> emergency_stop(1.0s)\n";
        pwm_ctrl_emergency_stop(1.0f);
        g_surge = g_yaw = g_heave = 0.0f;
        break;

    case 'h': case 'H':
        print_help();
        break;

    case 'q': case 'Q':
        std::cout << "[KEY] Q -> exit\n";
        g_running = false;
        break;

    default:
        // 其他按键忽略
        break;
    }

    // 有可能更改了命令，刷新目标
    (void)update_targets_from_command();
}

/* ----------- 主程序：键盘 teleop 循环 ----------- */

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    const char* ip   = (argc > 1) ? argv[1] : "192.168.2.16";
    const int   port = (argc > 2) ? std::stoi(argv[2]) : 8000;
    const float ctrl_hz = (argc > 3) ? std::stof(argv[3]) : 51.0f;
    const int   hb_hz   = (argc > 4) ? std::stoi(argv[4]) : 1;

    std::cout << "[INFO] Teleop target=" << ip << ":" << port
              << " ctrl=" << ctrl_hz << "Hz hb=" << hb_hz << "Hz\n";

    // 底层 UDP
    pwm_host_config_t host_cfg{};
    host_cfg.stm32_ip      = ip;
    host_cfg.stm32_port    = static_cast<uint16_t>(port);
    host_cfg.send_hz       = static_cast<int>(ctrl_hz);
    host_cfg.socket_sndbuf = 0;
    host_cfg.nonblock_send = 0;

    pwmh_result_t rc_host = pwm_host_init(&host_cfg);
    if (rc_host != PWMH_OK) {
        std::cerr << "[ERR] pwm_host_init: " << pwm_host_strerror(rc_host) << "\n";
        return 1;
    }
    std::cout << "[INFO] libpwm_host version=" << pwm_host_version() << "\n";

    // 控制层
    pwm_ctrl_config_t ctrl_cfg{};
    ctrl_cfg.ctrl_hz      = ctrl_hz;
    ctrl_cfg.max_step_pct = 0.2f;                // 每步 0.2%
    ctrl_cfg.groupA_mask  = PWM_CH_MASK_1_4;     // 1-4
    ctrl_cfg.groupB_mask  = PWM_CH_MASK_5_8;     // 5-8
    ctrl_cfg.group_mode   = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;

    int rc = pwm_ctrl_init(&ctrl_cfg);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_init rc=" << rc << "\n";
        pwm_host_close();
        return 1;   
    }

    // 初始：全通道中位
    (void)pwm_ctrl_set_all_target_mid();
    g_surge = g_yaw = g_heave = 0.0f;

    term_set_raw();
    print_help();

    const double period_ms     = 1000.0 / (ctrl_hz > 0.0f ? ctrl_hz : 51.0f);
    const double hb_period_ms  = 1000.0 / (hb_hz > 0 ? hb_hz : 1);
    auto t_next_pwm  = Clock::now();
    auto t_next_hb   = Clock::now();
    auto t_next_stat = Clock::now() + Ms(1000);

    while (g_running.load()) {
        auto now = Clock::now();

        // 处理键盘，更新 command & target
        teleop_handle_key();

        // 控制步：平滑逼近目标
        if( now >= t_next_pwm) 
        {
            t_next_pwm += Ms((int)period_ms);
            int rc_step = pwm_ctrl_step();
            if (rc_step < 0) 
            {
                std::cerr << "[ERR] pwm_ctrl_step rc=" << rc_step << "\n";
                break;
            }
        }

        // 心跳
        if (now >= t_next_hb) {
            t_next_hb += Ms((int)hb_period_ms);
            pwmh_result_t rc_hb = pwm_host_send_heartbeat();
            if (rc_hb != PWMH_OK) {
                std::cerr << "[WARN] send_heartbeat: "
                          << pwm_host_strerror(rc_hb) << "\n";
            }
        }
        //！ 收 ACK
        (void)pwm_host_poll(1);
        // 统计
        if (now >= t_next_stat) {
            t_next_stat += Ms(1000);
            print_stats("teleop");
        }
    }

    term_restore();
    pwm_ctrl_emergency_stop(1.0f);  // 离开前平滑归中
    pwm_ctrl_deinit();
    pwm_host_close();
    std::cout << "[INFO] pwm_teleop exit.\n";
    return 0;
}
