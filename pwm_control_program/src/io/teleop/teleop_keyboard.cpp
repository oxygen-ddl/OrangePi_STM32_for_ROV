// src/io/teleop_keyboard.cpp
//
// Teleop keyboard business logic (Hold-to-run + global throttle + gated motor test)
//
// Goals:
//  1) Hold-to-run for DOF keys (WASD/QE/GH/RT/FV): press/auto-repeat keeps active; release -> timeout -> zero.
//  2) Global throttle (0..1) scales all 6DOF: '-' decreases, '=' increases.
//  3) Blocking single-motor test (1..8) is allowed ONLY when idle (no motion keys recently + DOF=0).
//     Otherwise ignore and warn operator (rate-limited).
//  4) During motor test, lock out other keys (optionally allow ESC).
//

#include <algorithm> // std::max/min/clamp
#include <cmath>     // std::fabs
#include <cstdint>
#include <iostream>
#include <time.h>

extern "C" {
#include "io/teleop/teleop_keyboard.h"
#include "libpwm_host.h"
#include "pwm_control.h"
}

/* =========================================================
 *                工程参数
 * ========================================================= */

// 单电机测试用保持时间（秒）
static constexpr float TEST_HOLD_SEC = 2.0f;

// 全局 throttle 每次按键的步长
static constexpr float THROTTLE_STEP    = 0.10f;
// throttle 默认值（建议偏保守，方便首次下水）
static constexpr float THROTTLE_DEFAULT = 0.30f;

// throttle 变化限速（slew-rate），单位：每秒最大变化
// 例如 0.60 -> 从 0 到 1 大约 1.7s
static constexpr float THROTTLE_SLEW_PER_S = 0.60f;

// Hold-to-run 释放超时（依赖终端 auto-repeat），超过该时间未收到重复键 -> 视为松手
static constexpr std::uint64_t HOLD_TIMEOUT_NS = 150ULL * 1000ULL * 1000ULL; // 150ms

// 单电机测试：要求“空闲”窗口（最后一次运动相关按键之后的冷却时间）
static constexpr std::uint64_t TEST_IDLE_COOLDOWN_NS = 500ULL * 1000ULL * 1000ULL; // 0.5s

// 警告节流（避免刷屏）
static constexpr std::uint64_t TEST_WARN_INTERVAL_NS = 1000ULL * 1000ULL * 1000ULL; // 1s

/* =========================================================
 *                     时间工具
 * ========================================================= */

static std::uint64_t now_ns()
{
    ::timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL
         + static_cast<std::uint64_t>(ts.tv_nsec);
}

/* =========================================================
 *                     状态：方向 + throttle
 * ========================================================= */

// direction: -1/0/+1  (hold-to-run)
static std::int8_t  g_dir_surge = 0;
static std::int8_t  g_dir_sway  = 0;
static std::int8_t  g_dir_heave = 0;
static std::int8_t  g_dir_yaw   = 0;
static std::int8_t  g_dir_roll  = 0;
static std::int8_t  g_dir_pitch = 0;

// last key time per DOF
static std::uint64_t g_last_surge_ns = 0;
static std::uint64_t g_last_sway_ns  = 0;
static std::uint64_t g_last_heave_ns = 0;
static std::uint64_t g_last_yaw_ns   = 0;
static std::uint64_t g_last_roll_ns  = 0;
static std::uint64_t g_last_pitch_ns = 0;

// throttle: command & applied (slewed)
static float         g_throttle_cmd = THROTTLE_DEFAULT;
static float         g_throttle     = THROTTLE_DEFAULT;
static std::uint64_t g_last_throttle_ns = 0;

// motion & motor test gating
static bool          g_motor_test_in_progress = false;
static std::uint64_t g_last_motion_event_ns   = 0;  // any motion-related key touch
static std::uint64_t g_last_test_warn_ns      = 0;

// helpers
static inline float clamp01(float v) { return std::max(0.0f, std::min(1.0f, v)); }
static inline std::int8_t clamp_dir(int v) { return (v > 0) ? 1 : (v < 0 ? -1 : 0); }

static void reset_dirs()
{
    g_dir_surge = g_dir_sway = g_dir_heave = g_dir_yaw = g_dir_roll = g_dir_pitch = 0;
    g_last_surge_ns = g_last_sway_ns = g_last_heave_ns = g_last_yaw_ns = g_last_roll_ns = g_last_pitch_ns = 0;
}

static void log_dof_state(const char* key_tag)
{
    std::cout << "[KEY] " << key_tag
              << " throttle=" << g_throttle
              << " dir(surge=" << int(g_dir_surge)
              << ", sway="      << int(g_dir_sway)
              << ", heave="     << int(g_dir_heave)
              << ", yaw="       << int(g_dir_yaw)
              << ", roll="      << int(g_dir_roll)
              << ", pitch="     << int(g_dir_pitch)
              << ")\n";
}

/* =========================================================
 *                  hold-to-run 结算
 * ========================================================= */

static void apply_hold_timeout(std::uint64_t tns)
{
    auto expire = [&](std::int8_t dir, std::uint64_t last_ns) -> std::int8_t {
        if (dir == 0) return 0;
        if (last_ns == 0) return 0;
        return (tns - last_ns > HOLD_TIMEOUT_NS) ? 0 : dir;
    };

    g_dir_surge = expire(g_dir_surge, g_last_surge_ns);
    g_dir_sway  = expire(g_dir_sway,  g_last_sway_ns);
    g_dir_heave = expire(g_dir_heave, g_last_heave_ns);
    g_dir_yaw   = expire(g_dir_yaw,   g_last_yaw_ns);
    g_dir_roll  = expire(g_dir_roll,  g_last_roll_ns);
    g_dir_pitch = expire(g_dir_pitch, g_last_pitch_ns);
}

static void slew_throttle(std::uint64_t tns)
{
    if (g_last_throttle_ns == 0) {
        g_last_throttle_ns = tns;
        g_throttle = g_throttle_cmd;
        return;
    }

    const double dt =
    static_cast<double>(tns - g_last_throttle_ns) * 1e-9;
    g_last_throttle_ns = tns;

    const float max_step = static_cast<float>(THROTTLE_SLEW_PER_S * dt);
    const float err = g_throttle_cmd - g_throttle;

    if (std::fabs(err) <= max_step) {
        g_throttle = g_throttle_cmd;
    } else {
        g_throttle += (err > 0 ? max_step : -max_step);
    }
}

static void touch_dir(std::int8_t& dir, std::uint64_t& last_ns, int v)
{
    const std::uint64_t tns = now_ns();
    dir = clamp_dir(v);
    last_ns = tns;

    // 记录最近“运动按键”事件（用于禁止电机测试）
    g_last_motion_event_ns = tns;
}

/* =========================================================
 *           单电机测试（阻塞，直接 PWM）
 * ========================================================= */

static int handle_single_motor_test(int ch_num)
{
    if (ch_num < 1 || ch_num > PWM_HOST_CH_NUM) {
        return -PWMH_EINVAL;
    }

    std::cout << "[TEST] Channel " << ch_num
              << " : 7.5% -> 8.0% hold " << TEST_HOLD_SEC
              << "s -> emergencyStop(1.0s)\n";

    int rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_set_all_target_mid rc=" << rc << "\n";
        return rc;
    }

    // 阻塞式保持：内部会自己循环 step
    rc = pwm_ctrl_hold_pct_blocking(ch_num, 8.0f, TEST_HOLD_SEC);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_hold_pct_blocking ch=" << ch_num
                  << " rc=" << rc << "\n";
        return rc;
    }

    // 平滑回中位
    rc = pwm_ctrl_emergency_stop(1.0f);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_emergency_stop rc=" << rc << "\n";
        return rc;
    }

    std::cout << "[TEST] Channel " << ch_num << " 完成\n";
    return PWM_TELEOP_HANDLED;
}

/* =========================================================
 *           单电机测试门禁：必须 idle 才允许
 * ========================================================= */

static bool any_dir_nonzero_after_timeout(std::uint64_t now_ns)
{
    apply_hold_timeout(now_ns);
    return (g_dir_surge != 0) || (g_dir_sway != 0) || (g_dir_heave != 0) ||
           (g_dir_yaw   != 0) || (g_dir_roll != 0) || (g_dir_pitch != 0);
}

static void warn_test_blocked(std::uint64_t now_ns, const char* reason)
{
    if (now_ns - g_last_test_warn_ns < TEST_WARN_INTERVAL_NS) return;
    g_last_test_warn_ns = now_ns;

    std::cerr << "[TELEOP][WARN] Motor test blocked: " << reason << "\n"
              << "               Requirement: idle (DOF=0 and no motion keys recently).\n";
}

static bool safe_to_start_motor_test(std::uint64_t now_ns)
{
    if (g_motor_test_in_progress) return false;

    // 1) DOF 必须为 0（方向 hold 已归零）
    if (any_dir_nonzero_after_timeout(now_ns)) return false;

    // 2) 最近不能有运动事件（冷却窗口）
    if (g_last_motion_event_ns != 0 &&
        (now_ns - g_last_motion_event_ns) < TEST_IDLE_COOLDOWN_NS) {
        return false;
    }

    return true;
}

/* =========================================================
 *                       帮助信息
 * ========================================================= */

extern "C" void pwm_teleop_print_help(void)
{
    std::cout <<
        "\n===== Teleop 键位说明（Hold-to-run + Throttle + MotorTestGate） =====\n"
        "  [数字测试]（阻塞式，仅空闲时允许）\n"
        "    1..8 : 单电机测试（需要 DOF=0 且近期无运动键；否则忽略并报警）\n"
        "           7.5% -> 8.0% hold " << TEST_HOLD_SEC << "s -> 回中\n"
        "\n"
        "  [Hold-to-run 运动键：按住有效，松手自动归零]\n"
        "    W / S : surge + / -\n"
        "    A / D : sway  - / +\n"
        "    Q / E : yaw   + / -\n"
        "    H / G : heave + / -\n"
        "\n"
        "  [特殊姿态：纯动作（会清空其他 DOF），同样是按住有效]\n"
        "    R / T : roll  + / -\n"
        "    F / V : pitch + / -\n"
        "\n"
        "  [全局倍率 throttle（作用于全部 6DOF）]\n"
        "    = : throttle + " << THROTTLE_STEP << "\n"
        "    - : throttle - " << THROTTLE_STEP << "\n"
        "    当前 throttle_cmd=" << g_throttle_cmd << "（实际输出会做平滑限速）\n"
        "\n"
        "  [通用]\n"
        "    M : 清空 DOF（dir=0，不直接操作 PWM）\n"
        "    Z : 显示本帮助\n"
        "   ESC: 请求退出（由上层控制循环执行停机流程）\n"
        "====================================================================\n\n";
}

/* =========================================================
 *                  Teleop 状态操作（对外 API）
 * ========================================================= */

extern "C" void pwm_teleop_reset(void)
{
    reset_dirs();
    std::cout << "[TELEOP] reset: all DOF dir = 0, throttle_cmd=" << g_throttle_cmd << "\n";
}

extern "C" void pwm_teleop_get_state(pwm_teleop_state_t* out_state)
{
    if (!out_state) return;

    const std::uint64_t tns = now_ns();

    apply_hold_timeout(tns);
    slew_throttle(tns);

    out_state->cmd_surge = static_cast<float>(g_dir_surge) * g_throttle;
    out_state->cmd_sway  = static_cast<float>(g_dir_sway)  * g_throttle;
    out_state->cmd_heave = static_cast<float>(g_dir_heave) * g_throttle;
    out_state->cmd_yaw   = static_cast<float>(g_dir_yaw)   * g_throttle;
    out_state->cmd_roll  = static_cast<float>(g_dir_roll)  * g_throttle;
    out_state->cmd_pitch = static_cast<float>(g_dir_pitch) * g_throttle;
}

/* =========================================================
 *                   外部调用入口：按键处理
 * ========================================================= */

extern "C" int pwm_teleop_handle_key(int key)
{
    if (key == EOF) {
        return PWM_TELEOP_IGNORED;
    }

    // ESC 优先：允许随时退出（即使测试中）
    if (key == 27) {
        std::cout << "[KEY] ESC -> exit requested\n";
        return PWM_TELEOP_EXIT_REQUEST;
    }

    // 如果正在执行单电机测试：锁住其它按键，避免污染状态
    if (g_motor_test_in_progress) {
        // 这里选择“吞掉”所有按键（除 ESC），避免刷屏/误动作
        return PWM_TELEOP_HANDLED;
    }

    // 数字键：单电机测试（阻塞）——必须 idle 才允许
    if (key >= '1' && key <= '8') {
        const std::uint64_t tns = now_ns();

        if (!safe_to_start_motor_test(tns)) {
            warn_test_blocked(tns, "ROV is not idle / recent motion detected");
            return PWM_TELEOP_HANDLED;
        }

        g_motor_test_in_progress = true;

        // 双保险：测试前清零方向，避免残留
        reset_dirs();

        const int ch_num = key - '0';
        const int rc = handle_single_motor_test(ch_num);

        g_motor_test_in_progress = false;

        // 测试结束后标记“近期有动作”，避免立刻再次触发
        g_last_motion_event_ns = tns;

        return (rc < 0) ? rc : PWM_TELEOP_HANDLED;
    }

    // 统一转为大写，简化判断
    char c = static_cast<char>(key);
    if (c >= 'a' && c <= 'z') {
        c = static_cast<char>(c - 'a' + 'A');
    }

    // throttle keys
    if (c == '-') {
        g_throttle_cmd = clamp01(g_throttle_cmd - THROTTLE_STEP);
        std::cout << "[KEY] '-' throttle_cmd=" << g_throttle_cmd << "\n";
        return PWM_TELEOP_HANDLED;
    }
    if (c == '=') {
        g_throttle_cmd = clamp01(g_throttle_cmd + THROTTLE_STEP);
        std::cout << "[KEY] '=' throttle_cmd=" << g_throttle_cmd << "\n";
        return PWM_TELEOP_HANDLED;
    }

    switch (c) {
    case 'Z':
        pwm_teleop_print_help();
        return PWM_TELEOP_HANDLED;

    case 'M':
        reset_dirs();
        std::cout << "[KEY] M -> clear DOF dirs\n";
        return PWM_TELEOP_HANDLED;

    // -------- Hold-to-run 平面 DOF --------
    case 'W': touch_dir(g_dir_surge, g_last_surge_ns, +1); log_dof_state("W"); return PWM_TELEOP_HANDLED;
    case 'S': touch_dir(g_dir_surge, g_last_surge_ns, -1); log_dof_state("S"); return PWM_TELEOP_HANDLED;

    case 'A': touch_dir(g_dir_sway,  g_last_sway_ns,  -1); log_dof_state("A"); return PWM_TELEOP_HANDLED;
    case 'D': touch_dir(g_dir_sway,  g_last_sway_ns,  +1); log_dof_state("D"); return PWM_TELEOP_HANDLED;

    case 'Q': touch_dir(g_dir_yaw,   g_last_yaw_ns,   +1); log_dof_state("Q"); return PWM_TELEOP_HANDLED;
    case 'E': touch_dir(g_dir_yaw,   g_last_yaw_ns,   -1); log_dof_state("E"); return PWM_TELEOP_HANDLED;

    // -------- Hold-to-run 垂向 DOF --------
    case 'H': touch_dir(g_dir_heave, g_last_heave_ns, +1); log_dof_state("H"); return PWM_TELEOP_HANDLED;
    case 'G': touch_dir(g_dir_heave, g_last_heave_ns, -1); log_dof_state("G"); return PWM_TELEOP_HANDLED;

    // -------- 特殊姿态：纯动作（清空其他 DOF）--------
    case 'R':
        reset_dirs();
        touch_dir(g_dir_roll,  g_last_roll_ns,  +1);
        log_dof_state("R");
        return PWM_TELEOP_HANDLED;

    case 'T':
        reset_dirs();
        touch_dir(g_dir_roll,  g_last_roll_ns,  -1);
        log_dof_state("T");
        return PWM_TELEOP_HANDLED;

    case 'F':
        reset_dirs();
        touch_dir(g_dir_pitch, g_last_pitch_ns, +1);
        log_dof_state("F");
        return PWM_TELEOP_HANDLED;

    case 'V':
        reset_dirs();
        touch_dir(g_dir_pitch, g_last_pitch_ns, -1);
        log_dof_state("V");
        return PWM_TELEOP_HANDLED;

    default:
        return PWM_TELEOP_IGNORED;
    }
}
