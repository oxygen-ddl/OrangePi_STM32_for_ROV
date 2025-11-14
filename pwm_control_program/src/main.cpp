#include "libpwm_host.h"
#include "pwm_control.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

static std::atomic<bool> g_running{true};
static void on_sigint(int){ g_running = false; }

// 简单打印统计信息
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
              << " rtt_last=" << (rtt >= 0 ? rtt : -1.0) << " ms\n";
}

// 统一的“控制周期循环”：在 seconds 内按 ctrl_hz 调用 step+poll+心跳
static int run_for_seconds(float seconds, float ctrl_hz, int hb_hz, const char* phase_name)
{
    if (seconds <= 0.0f) return 0;

    std::cout << "\n========== PHASE: " << phase_name
              << " (" << seconds << " s) ==========\n";

    const double period_ms = 1000.0 / (ctrl_hz > 0.0f ? ctrl_hz : 51.0f);
    const double hb_period_ms = 1000.0 / (hb_hz > 0 ? hb_hz : 1);

    auto t_start      = Clock::now();
    auto t_next_hb    = t_start;
    auto t_next_stat  = t_start + Ms(1000);

    while (g_running.load()) {
        auto now = Clock::now();
        double elapsed_s =
            std::chrono::duration<double>(now - t_start).count();
        if (elapsed_s >= seconds) break;

        // 控制步：限斜率 + 分组更新 + 一帧 8CH PWM
        int rc_step = pwm_ctrl_step();
        if (rc_step < 0) {
            std::cerr << "[ERR] pwm_ctrl_step rc=" << rc_step << "\n";
            return rc_step;
        }

        // 心跳：hb_hz（比如 1Hz）
        if (now >= t_next_hb) {
            t_next_hb += Ms((int)hb_period_ms);
            pwmh_result_t rc_hb = pwm_host_send_heartbeat();
            if (rc_hb != PWMH_OK) {
                std::cerr << "[WARN] send_heartbeat: "
                          << pwm_host_strerror(rc_hb) << "\n";
            }
        }

        // 收包/解析 HB_ACK
        (void)pwm_host_poll(1);

        // 每秒打印统计信息
        if (now >= t_next_stat) {
            t_next_stat += Ms(1000);
            print_stats(phase_name);
        }

        std::this_thread::sleep_for(Ms((int)period_ms));
    }

    print_stats(phase_name);
    return 0;
}

// 辅助：单通道往返渐变（通过设置 target + run_for_seconds 实现）
static int test_single_channel_ramp(int ch, float ctrl_hz)
{
    std::cout << "\n--- Test: 单通道 CH" << ch
              << " 7.5% -> 9.5% -> 7.5% ---\n";

    // 先把所有目标拉到中位
    int rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) return rc;

    // 让系统有时间归中
    rc = run_for_seconds(1.0f, ctrl_hz, 1, "CH-mid-init");
    if (rc < 0) return rc;

    // 目标：CHch 设为 9.5%
    rc = pwm_ctrl_set_target_pct(ch, 9.5f);
    if (rc < 0) return rc;

    // 2 秒：渐变 + 保持
    rc = run_for_seconds(2.0f, ctrl_hz, 1, "CH-ramp-up");
    if (rc < 0) return rc;

    // 目标：CHch 回 7.5%
    rc = pwm_ctrl_set_target_pct(ch, PWM_HOST_PCT_MID);
    if (rc < 0) return rc;

    // 2 秒：渐变回中 + 稳定
    rc = run_for_seconds(2.0f, ctrl_hz, 1, "CH-ramp-down");
    return rc;
}

// 辅助：一组通道小幅前推（例如 8.5%），再回中
static int test_group_forward(const char* name,
                              pwm_channel_mask_t mask,
                              float fwd_pct,
                              float ctrl_hz)
{
    std::cout << "\n--- Test: 分组前推 [" << name << "] to "
              << fwd_pct << "% ---\n";

    // 先全中位
    int rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) return rc;
    rc = run_for_seconds(1.0f, ctrl_hz, 1, "grp-mid-init");
    if (rc < 0) return rc;

    // 按掩码设置目标
    float pct[ PWM_HOST_CH_NUM ];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        int ch = i + 1;
        if ((mask & (1u << (ch-1))) != 0) {
            pct[i] = fwd_pct;           // 该通道前推
        } else {
            pct[i] = PWM_HOST_PCT_MID;  // 其他保持中位
        }
    }
    rc = pwm_ctrl_set_targets_mask(mask, pct);
    if (rc < 0) return rc;

    // 2 秒：渐变到位 + 保持
    rc = run_for_seconds(2.0f, ctrl_hz, 1, "grp-forward");
    if (rc < 0) return rc;

    // 目标：全部回中位
    rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) return rc;
    rc = run_for_seconds(2.0f, ctrl_hz, 1, "grp-back-mid");
    return rc;
}

// 辅助：温和反向测试（先全中，再到 6.5%）
static int test_soft_reverse(float ctrl_hz)
{
    std::cout << "\n--- Test: 温和反向（全通道 7.5% -> 6.5% -> 7.5%）---\n";

    int rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) return rc;
    rc = run_for_seconds(1.0f, ctrl_hz, 1, "rev-mid-init");
    if (rc < 0) return rc;

    // 设置所有通道目标到 6.5%（轻微反向，避免直接打满 5% 极限）
    float pct[ PWM_HOST_CH_NUM ];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) pct[i] = 6.5f;
    rc = pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
    if (rc < 0) return rc;

    // 2 秒：渐变到 6.5% + 保持
    rc = run_for_seconds(2.0f, ctrl_hz, 1, "rev-to-6.5");
    if (rc < 0) return rc;

    // 回中位
    rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) return rc;
    rc = run_for_seconds(2.0f, ctrl_hz, 1, "rev-back-mid");
    return rc;
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    // ===== 参数：ip port ctrl_hz hb_hz =====
    const char* ip   = (argc > 1) ? argv[1] : "192.168.2.16";
    const int   port = (argc > 2) ? std::stoi(argv[2]) : 8000;
    const float ctrl_hz = (argc > 3) ? std::stof(argv[3]) : 51.0f;
    const int   hb_hz   = (argc > 4) ? std::stoi(argv[4]) : 1;

    std::cout << "[INFO] target=" << ip << ":" << port
              << " ctrl=" << ctrl_hz << "Hz hb=" << hb_hz << "Hz\n";

    // ===== 初始化底层 UDP 驱动（libpwm_host）=====
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

    // ===== 初始化控制层（pwm_control）=====
    pwm_ctrl_config_t ctrl_cfg{};
    ctrl_cfg.ctrl_hz      = ctrl_hz;
    ctrl_cfg.max_step_pct = 0.1f;  // 每步最大 0.1%，50Hz 时每秒最多 5%
    ctrl_cfg.groupA_mask  = PWM_CH_MASK_1_4;   // CH1-4
    ctrl_cfg.groupB_mask  = PWM_CH_MASK_5_8;   // CH5-8
    ctrl_cfg.group_mode   = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;

    int rc = pwm_ctrl_init(&ctrl_cfg);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_init rc=" << rc << "\n";
        pwm_host_close();
        return 1;
    }

    // ========== Phase 1: 全通道中位保持 3 秒 ==========
    rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) goto EXIT;
    rc = run_for_seconds(3.0f, ctrl_hz, hb_hz, "all-mid-3s");
    if (rc < 0) goto EXIT;

    // ========== Phase 2: CH1 单通道往返渐变 ==========
    rc = test_single_channel_ramp(/*ch=*/1, ctrl_hz);
    if (rc < 0) goto EXIT;

    // ========== Phase 3: Group A (CH1-4) 小幅前推 ==========
    rc = test_group_forward("GroupA_CH1-4", PWM_CH_MASK_1_4, 8.5f, ctrl_hz);
    if (rc < 0) goto EXIT;

    // ========== Phase 4: Group B (CH5-8) 小幅前推 ==========
    rc = test_group_forward("GroupB_CH5-8", PWM_CH_MASK_5_8, 8.5f, ctrl_hz);
    if (rc < 0) goto EXIT;

    // ========== Phase 5: 温和全通道反向 7.5% -> 6.5% -> 7.5% ==========
    rc = test_soft_reverse(ctrl_hz);
    if (rc < 0) goto EXIT;

    // ========== Phase 6: 紧急平滑归中位（1秒） ==========
    std::cout << "\n--- Phase 6: emergency_stop(1.0s) ---\n";
    rc = pwm_ctrl_emergency_stop(1.0f);
    if (rc < 0) goto EXIT;

EXIT:
    if (rc < 0) {
        std::cerr << "[ERR] test sequence aborted, rc=" << rc << "\n";
    }

    // 保险：最后再把目标设为中位并运行 1 秒
    (void)pwm_ctrl_set_all_target_mid();
    (void)run_for_seconds(1.0f, ctrl_hz, hb_hz, "final-mid");

    pwm_ctrl_deinit();
    pwm_host_close();
    std::cout << "[INFO] pwm_control_test exit.\n";
    return (rc < 0) ? 1 : 0;
}
