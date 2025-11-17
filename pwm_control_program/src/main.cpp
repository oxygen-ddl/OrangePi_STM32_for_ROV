#include "libpwm_host.h"
#include "pwm_control.h"
#include "pwm_teleop_keys.h"

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
#include <cerrno>
#include <string>

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

static std::atomic<bool> g_running{true};
static void on_sigint(int){ g_running = false; }

/* ----------- 终端 raw 模式工具（加安全检查） ----------- */

static struct termios g_old_tio;
static bool g_term_inited = false;

/**
 * @return
 *   0   成功进入 raw 模式
 *  >0   非致命错误 / 条件不满足（跳过 raw 模式，但程序继续运行）
 *  <0   严重错误（一般不会出现）
 */
static int term_set_raw()
{
    if (g_term_inited) return 0;

    // 1) stdin 必须是 TTY
    if (!isatty(STDIN_FILENO)) {
        std::cerr << "[WARN] stdin is not a TTY, skip raw mode.\n";
        return 1;
    }

#ifdef TIOCGPGRP
    // 2) 必须是前台进程组，否则 tcsetattr 可能触发 SIGTTOU 挂起进程
    {
        pid_t tpgrp = tcgetpgrp(STDIN_FILENO);
        if (tpgrp == -1) {
            std::cerr << "[WARN] tcgetpgrp failed: " << std::strerror(errno)
                      << " ; skip raw mode.\n";
            return 2;
        }
        pid_t mypgrp = getpgrp();
        if (tpgrp != mypgrp) {
            std::cerr << "[WARN] process is NOT in foreground of this TTY, "
                         "skip raw mode to avoid SIGTTOU.\n";
            return 3;
        }
    }
#endif

    // 3) 获取当前属性
    if (tcgetattr(STDIN_FILENO, &g_old_tio) < 0) {
        std::cerr << "[WARN] tcgetattr failed: " << std::strerror(errno)
                  << " ; skip raw mode.\n";
        return -1;
    }

    struct termios new_tio = g_old_tio;
    cfmakeraw(&new_tio);

    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) < 0) {
        std::cerr << "[WARN] tcsetattr(raw) failed: " << std::strerror(errno)
                  << " ; skip raw mode.\n";
        return -2;
    }

    // 4) 非阻塞 stdin（失败也不致命）
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
            std::cerr << "[WARN] fcntl(O_NONBLOCK) failed: "
                      << std::strerror(errno) << "\n";
        }
    } else {
        std::cerr << "[WARN] fcntl(F_GETFL) failed: "
                  << std::strerror(errno) << "\n";
    }

    g_term_inited = true;
    std::cout << "[INFO] terminal set to RAW + non-blocking.\n";
    return 0;
}

static void term_restore()
{
    if (!g_term_inited) return;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio) < 0) {
        std::cerr << "[WARN] tcsetattr(restore) failed: "
                  << std::strerror(errno) << "\n";
    }
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

/* ----------- 非阻塞读键工具 ----------- */

static int read_key_nonblock()
{
    // 如果没成功进入 raw+nonblock，就直接不读键盘，避免意外阻塞
    if (!g_term_inited) return EOF;

    unsigned char c;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0) {
        return EOF;    // 没有按键 / EAGAIN
    }
    return (int)c;     // 返回 ASCII
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

    // 1) 初始化底层 UDP 驱动（libpwm_host）
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

    // 2) 初始化控制层（pwm_control）
    pwm_ctrl_config_t ctrl_cfg{};
    ctrl_cfg.ctrl_hz      = ctrl_hz;
    ctrl_cfg.max_step_pct = 0.2f;                // 每步 0.2%
    ctrl_cfg.groupA_mask  = PWM_CH_MASK_1_4;     // CH1-4
    ctrl_cfg.groupB_mask  = PWM_CH_MASK_5_8;     // CH5-8
    ctrl_cfg.group_mode   = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;

    int rc = pwm_ctrl_init(&ctrl_cfg);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_init rc=" << rc << "\n";
        pwm_host_close();
        return 1;
    }

    // 初始：全通道中位
    (void)pwm_ctrl_set_all_target_mid();

    // 3) 终端 raw 模式 + 打印一次帮助
    int trc = term_set_raw();
    if (trc != 0) {
        std::cerr << "[WARN] term_set_raw failed/skip (rc=" << trc
                  << "), keyboard teleop disabled (Ctrl+C to quit).\n";
    }
    pwm_teleop_print_help();

    const double pwm_period_ms = 1000.0 / (ctrl_hz > 0.0f ? ctrl_hz : 51.0f);
    const double hb_period_ms  = 1000.0 / (hb_hz > 0 ? hb_hz : 1);
    auto t_next_pwm   = Clock::now();
    auto t_next_hb    = Clock::now();
    auto t_next_stat  = Clock::now() + Ms(1000);

    while (g_running.load()) {

        auto now = Clock::now();

        // 1) 读键（非阻塞），交给 teleop 模块解释
        int ch = read_key_nonblock();
        if (ch != EOF) 
        {
            int r = pwm_teleop_handle_key(ch);
            if (r < 0) 
            {
                std::cerr << "[ERR] pwm_teleop_handle_key rc=" << r << "\n";
            }
            if (r == 2)  // ESC 请求退出
            {  
                g_running = false;
                continue;
            }
            // r>0 表示已处理该按键，r==0 表示按键未被本模块使用
        }

        // 2) 控制步：根据目标占空比 + 限斜率 + 分组策略，下发一帧 PWM
        if (now >= t_next_pwm) 
        {
            t_next_pwm += Ms((int)pwm_period_ms);
            int rc_step = pwm_ctrl_step();
            if (rc_step < 0) 
            {
            std::cerr << "[ERR] pwm_ctrl_step rc=" << rc_step << "\n";
            break;
            }
        }
        // 3) 心跳发送
        if (now >= t_next_hb) {
            t_next_hb += Ms((int)hb_period_ms);
            pwmh_result_t rc_hb = pwm_host_send_heartbeat();
            if (rc_hb != PWMH_OK) {
                std::cerr << "[WARN] send_heartbeat: "<< pwm_host_strerror(rc_hb) << "\n";
            }
        }

        // 4) 轮询接收心跳 ACK / RTT（完全非阻塞，更安全）
        (void)pwm_host_poll(1);

        // 5) 统计输出（每秒一次）
        if (now >= t_next_stat) {
            t_next_stat += Ms(1000);
            print_stats("teleop");
        }


        std::this_thread::sleep_for(Ms(1));
    }

    // 收尾
    term_restore();
    pwm_ctrl_emergency_stop(1.0f);  // 离开前平滑归中
    pwm_ctrl_deinit();
    pwm_host_close();
    std::cout << "[INFO] pwm_teleop exit.\n";
    return 0;
}
