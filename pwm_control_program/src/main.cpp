#include "libpwm_host.h"
#include "pwm_control.h"
#include "pwm_teleop_keys.h"

// 统一时间基
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
// 给时间基起一个别名，方便调用
namespace tb = uwnav::timebase;
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

    // ----------------- 命令行参数解析（带默认值） -----------------
    const char* default_ip      = "192.168.2.16";
    const int   default_port    = 8000;
    const float default_ctrl_hz = 101.0f;  // 100 Hz → AB 后每路约 50 Hz
    const int   default_hb_hz   = 1;

    const char* ip   = (argc > 1) ? argv[1] : default_ip;
    int   port       = default_port;
    float ctrl_hz    = default_ctrl_hz;
    int   hb_hz      = default_hb_hz;

    if (argc > 2) {
        try { port = std::stoi(argv[2]); } catch (...) { port = default_port; }
    }
    if (argc > 3) {
        try { ctrl_hz = std::stof(argv[3]); } catch (...) { ctrl_hz = default_ctrl_hz; }
    }
    if (argc > 4) {
        try { hb_hz = std::stoi(argv[4]); } catch (...) { hb_hz = default_hb_hz; }
    }

    if (ctrl_hz <= 0.0f) ctrl_hz = default_ctrl_hz;
    if (hb_hz   <= 0)    hb_hz   = default_hb_hz;

    std::cout << "[INFO] Teleop target=" << ip << ":" << port
              << " ctrl=" << ctrl_hz << "Hz hb=" << hb_hz << "Hz\n";



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

    /* 2) 初始化 pwm_control 控制层（含电机映射与反向） */
    pwm_ctrl_config_t ctrl_cfg{};
    ctrl_cfg.ctrl_hz      = ctrl_hz;
    ctrl_cfg.max_step_pct = 0.2f;                // 每步最大 0.2 %
    ctrl_cfg.groupA_mask  = PWM_CH_MASK_1_4;     // 逻辑 CH1-4
    ctrl_cfg.groupB_mask  = PWM_CH_MASK_5_8;     // 逻辑 CH5-8
    ctrl_cfg.group_mode   = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;
    ctrl_cfg.enable_reverse_protection = 1;

    /* ------------------------------------------------------------------
     *           电机映射配置（新人重点修改区）
     *
     * 逻辑通道 CH1..CH8 = 程序中的“电机编号”，由键盘控制 / 自动控制使用
     * 物理通道 CH1..CH8 = STM32 上真实的 PWM 输出口编号（接线）
     *
     * motor_map[逻辑CH - 1] = 物理 PWM 通道：
     *   motor_map[0] = PWM_CH5;   // 逻辑 CH1 → 物理 PWM5
     *
     * 当前接线关系（可随时修改）：
     *   逻辑 CH1 → 物理 PWM5
     *   逻辑 CH2 → 物理 PWM6
     *   逻辑 CH3 → 物理 PWM7
     *   逻辑 CH4 → 物理 PWM8
     *   逻辑 CH5 → 物理 PWM1
     *   逻辑 CH6 → 物理 PWM2
     *   逻辑 CH7 → 物理 PWM3
     *   逻辑 CH8 → 物理 PWM4
     * ------------------------------------------------------------------ */
    {
        int motor_map[PWM_HOST_CH_NUM] = {
            PWM_CH5, PWM_CH6, PWM_CH7, PWM_CH8,  // 逻辑 CH1..CH4 → 物理 CH5..CH8
            PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4   // 逻辑 CH5..CH8 → 物理 CH1..CH4
        };
        std::memcpy(ctrl_cfg.motorch_to_pwmch, motor_map, sizeof(motor_map));
    }

    /* 电机反向配置：
     *   0 = 正向（默认）
     *   1 = 反向（会在 7.5% 对称反转）
     *
     * 修改示例：
     *   motor_rev[PWM_CH3 - 1] = 1;  // 逻辑 CH3 电机反向
     */
    {
        uint8_t motor_rev[PWM_HOST_CH_NUM] = {0,0,0,0,0,0,0,0};

        // 示例：如果以后需要让逻辑 CH3、CH7 反向，可以这样写：
        // motor_rev[PWM_CH3 - 1] = 1;
        // motor_rev[PWM_CH7 - 1] = 1;

        std::memcpy(ctrl_cfg.motor_reverse, motor_rev, sizeof(motor_rev));
    }

    // 启动时打印当前电机映射，便于接线核对
    std::cout << "\n[INFO] 当前电机映射 (逻辑CH -> 物理PWM, reverse标志):\n";
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        std::cout << "  CH" << (i + 1)
                  << " -> PWM" << ctrl_cfg.motorch_to_pwmch[i]
                  << " (reverse=" << static_cast<int>(ctrl_cfg.motor_reverse[i]) << ")\n";
    }
    std::cout << "------------------------------------------------------\n\n";

    int rc = pwm_ctrl_init(&ctrl_cfg);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_init rc=" << rc << "\n";
        pwm_host_close();
        return 1;
    }

    // 初始：全通道中位（逻辑空间），映射后由 pwm_control 下发到实际通道
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

    const double pwm_period_ms = 1000.0 / ctrl_hz;
    const double hb_period_ms  = 1000.0 / hb_hz;

    auto t_next_pwm  = Clock::now();
    auto t_next_hb   = Clock::now();
    auto t_next_stat = Clock::now() + Ms(1000);

    while (g_running.load()) {

        auto now = Clock::now();

        /* 1) 非阻塞读键，交给 teleop 模块（仅在 raw 模式下生效） */
        int key = read_key_nonblock();
        if (key != EOF)
        {
            int r = pwm_teleop_handle_key(key);
            if (r < 0) {
                std::cerr << "[ERR] pwm_teleop_handle_key rc=" << r << "\n";
            }
            if (r == 2) { // ESC 请求退出
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
            if (rc_step < 0) {
                std::cerr << "[ERR] pwm_ctrl_step rc=" << rc_step << "\n";
                break;
            }

           // 记录“已经下发”的 PWM 状态（统一时间基）
            if (log_ok && g_pwm_log.is_open()) {
                pwm_ctrl_state_t st{};
                pwm_ctrl_get_state(&st);

                // 绝对时间（系统时钟） → epoch 秒
                using SysClock = std::chrono::system_clock;
                auto sys_now   = SysClock::now();
                int64_t t_epoch_ns = std::chrono::duration_cast<
                        std::chrono::nanoseconds>(sys_now.time_since_epoch()).count();
                double  t_epoch_s  = static_cast<double>(t_epoch_ns) / 1e9;

                // 单调时间：由统一 timebase 提供
                int64_t t_mono_ns = tb::now_ns();

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
