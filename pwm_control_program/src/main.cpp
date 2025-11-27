#include "libpwm_host.h"
#include "pwm_control.h"
#include "pwm_teleop_keys.h"

// 统一时间基（与导航项目共享）
#include "timebase.h"

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

// 日志 + 目录创建 + 时间
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <cstdint>

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

static std::atomic<bool> g_running{true};
static void on_sigint(int){ g_running = false; }

/* ======================================================================
 *                          PWM 日志相关
 * ====================================================================== */

static std::ofstream g_pwm_log;

// 系统时间字符串，用于生成文件名：YYYY-MM-DD_HH-MM-SS
static std::string make_datetime_string()
{
    using std::chrono::system_clock;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm{};

    // OrangePi / Linux 使用 localtime_r
    localtime_r(&t, &tm);

    char buf[32];
    std::snprintf(buf, sizeof(buf),
                  "%04d-%02d-%02d_%02d-%02d-%02d",
                  tm.tm_year + 1900,
                  tm.tm_mon  + 1,
                  tm.tm_mday,
                  tm.tm_hour,
                  tm.tm_min,
                  tm.tm_sec);
    return std::string(buf);
}

static bool init_pwm_log_file()
{
    namespace fs = std::filesystem;

    try {
        fs::create_directories("logs");
    } catch (const std::exception& e) {
        std::cerr << "[WARN] create_directories(\"logs\") failed: "
                  << e.what() << "\n";
        return false;
    }

    std::string fname = "logs/pwm_teleop_" + make_datetime_string() + ".csv";
    g_pwm_log.open(fname, std::ios::out | std::ios::trunc);
    if (!g_pwm_log.is_open()) {
        std::cerr << "[WARN] open log file failed: " << fname << "\n";
        return false;
    }

    std::cout << "[INFO] PWM log file: " << fname << "\n";

    // 写表头：和 IMU/DVL 对齐，统一时间基协议
    g_pwm_log << "t_epoch_s"   // Unix 时间戳（秒，double）
              << ",t_mono_ns"  // 单调时间（纳秒，int64，用于排序/插值）
              << ",ch1_pct"
              << ",ch2_pct"
              << ",ch3_pct"
              << ",ch4_pct"
              << ",ch5_pct"
              << ",ch6_pct"
              << ",ch7_pct"
              << ",ch8_pct"
              << "\n";
    g_pwm_log.flush();
    return true;
}

static void close_pwm_log_file()
{
    if (g_pwm_log.is_open()) {
        g_pwm_log.flush();
        g_pwm_log.close();
    }
}

/* ======================================================================
 *                      终端 raw 模式工具（安全版）
 * ====================================================================== */

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

/* ======================================================================
 *                          简单统计输出
 * ====================================================================== */

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

/* ======================================================================
 *                        非阻塞读键工具
 * ====================================================================== */

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

/* ======================================================================
 *                    主程序：键盘 teleop 循环
 * ====================================================================== */

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    const char* ip   = (argc > 1) ? argv[1] : "192.168.2.16";
    const int   port = (argc > 2) ? std::stoi(argv[2]) : 8000;
    const float ctrl_hz = (argc > 3) ? std::stof(argv[3]) : 100.0f;  // 100Hz → AB 后每路约 50Hz
    const int   hb_hz   = (argc > 4) ? std::stoi(argv[4]) : 1;

    std::cout << "[INFO] Teleop target=" << ip << ":" << port
              << " ctrl=" << ctrl_hz << "Hz hb=" << hb_hz << "Hz\n";

    /* 0) 初始化统一时间基 */
    // 如果你的 timebase_init 有返回值，可以像这样检查：
    int tb_rc = timebase_init();
    if (tb_rc != 0) {
        std::cerr << "[WARN] timebase_init failed rc=" << tb_rc
                  << " ; timestamps may be invalid.\n";
        // 这里不强制退出，让你在现场也能应急用一下 PWM
    }

    /* 1) 初始化 libpwm_host（UDP 通信） */
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

    /* 2) 初始化 pwm_control 控制层 */
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

    /* 3) 终端 raw 模式 + 帮助信息 */
    int trc = term_set_raw();
    if (trc != 0) {
        std::cerr << "[WARN] term_set_raw failed/skip (rc=" << trc
                  << "), keyboard teleop disabled (Ctrl+C to quit).\n";
    }
    pwm_teleop_print_help();

    /* 4) 初始化 PWM 日志 */
    bool log_ok = init_pwm_log_file();
    if (!log_ok) {
        std::cerr << "[WARN] PWM logging disabled due to file/dir error.\n";
    }

    const double pwm_period_ms = 1000.0 / (ctrl_hz > 0.0f ? ctrl_hz : 100.0f);
    const double hb_period_ms  = 1000.0 / (hb_hz > 0 ? hb_hz : 1);
    auto t_next_pwm   = Clock::now();
    auto t_next_hb    = Clock::now();
    auto t_next_stat  = Clock::now() + Ms(1000);

    while (g_running.load()) {

        auto now = Clock::now();

        /* 1) 非阻塞读键，交给 teleop 模块 */
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

        /* 2) 控制步：根据目标占空比 + 限斜率 + 分组策略，下发一帧 PWM */
        if (now >= t_next_pwm)
        {
            t_next_pwm += Ms(static_cast<int>(pwm_period_ms));

            int rc_step = pwm_ctrl_step();
            if (rc_step < 0)
            {
                std::cerr << "[ERR] pwm_ctrl_step rc=" << rc_step << "\n";
                break;
            }

            // 记录“已经下发”的 PWM 状态（统一时间基）
            if (log_ok && g_pwm_log.is_open()) {
                pwm_ctrl_state_t st{};
                pwm_ctrl_get_state(&st);

                double  t_epoch_s = 0.0;
                int64_t t_mono_ns = 0;
                timebase_now(&t_epoch_s, &t_mono_ns);

                g_pwm_log << std::fixed << std::setprecision(6)
                          << t_epoch_s
                          << ',' << t_mono_ns
                          << ',' << st.current_pct[0]
                          << ',' << st.current_pct[1]
                          << ',' << st.current_pct[2]
                          << ',' << st.current_pct[3]
                          << ',' << st.current_pct[4]
                          << ',' << st.current_pct[5]
                          << ',' << st.current_pct[6]
                          << ',' << st.current_pct[7]
                          << '\n';
                // 每秒会在统计时 flush 一次，这里不强制
            }
        }

        /* 3) 心跳发送 */
        if (now >= t_next_hb) {
            t_next_hb += Ms(static_cast<int>(hb_period_ms));
            pwmh_result_t rc_hb = pwm_host_send_heartbeat();
            if (rc_hb != PWMH_OK) {
                std::cerr << "[WARN] send_heartbeat: "
                          << pwm_host_strerror(rc_hb) << "\n";
            }
        }

        /* 4) 轮询接收心跳 ACK / RTT（完全非阻塞） */
        (void)pwm_host_poll(1);

        /* 5) 统计输出（每秒一次） */
        if (now >= t_next_stat) {
            t_next_stat += Ms(1000);
            print_stats("teleop");

            // 顺便做一次日志 flush，降低数据丢失风险
            if (log_ok && g_pwm_log.is_open()) {
                g_pwm_log.flush();
            }
        }

        std::this_thread::sleep_for(Ms(1));
    }

    /* 收尾 */
    term_restore();
    pwm_ctrl_emergency_stop(1.0f);  // 离开前平滑归中
    pwm_ctrl_deinit();
    pwm_host_close();
    close_pwm_log_file();

    std::cout << "[INFO] pwm_teleop exit.\n";
    return 0;
}
