// src/io/teleop_keyboard.cpp

#include <algorithm> // std::clamp
#include <iostream>

extern "C" {
#include "io/teleop_keyboard.h"
#include "libpwm_host.h"
#include "pwm_control.h"
}

/* =========================================================
 *                工程参数：命令档位 & 测试时间
 * ========================================================= */

// 键盘每次按下时，在命令空间内增加/减少的步长（类似档位）
static constexpr float CMD_STEP       = 0.25f;  // 建议 0.25：四档可打满 [-1,1]
// 单电机测试用保持时间（秒）
static constexpr float TEST_HOLD_SEC  = 2.0f;

/* =========================================================
 *            DOF 命令：键盘 → “虚拟 DOF 档位”
 * =========================================================
 * 约定：范围 [-1, 1]，后面由控制栈映射到推力 / PWM。
 */

static float g_cmd_surge = 0.0f;   // [-1, 1]  前进(+)/后退(-)
static float g_cmd_sway  = 0.0f;   // [-1, 1]  右移(+)/左移(-)
static float g_cmd_yaw   = 0.0f;   // [-1, 1]  航向
static float g_cmd_heave = 0.0f;   // [-1, 1]  上浮(+)/下潜(-)

// 纯姿态通道（主要供状态查询使用；在“纯 roll/pitch 模式”下使用）
static float g_cmd_roll  = 0.0f;   // [-1, 1]
static float g_cmd_pitch = 0.0f;   // [-1, 1]

/* =========================================================
 *                     小工具函数
 * ========================================================= */

static float clamp_unit(float v)
{
    return std::clamp(v, -1.0f, 1.0f);
}

static void reset_dof_cmds()
{
    g_cmd_surge = 0.0f;
    g_cmd_sway  = 0.0f;
    g_cmd_yaw   = 0.0f;
    g_cmd_heave = 0.0f;
    g_cmd_roll  = 0.0f;
    g_cmd_pitch = 0.0f;
}

// 统一打印当前 DOF 状态，避免重复代码
static void log_dof_state(const char* key_tag)
{
    std::cout << "[KEY] " << key_tag
              << " -> surge=" << g_cmd_surge
              << " sway="    << g_cmd_sway
              << " yaw="     << g_cmd_yaw
              << " heave="   << g_cmd_heave
              << " roll="    << g_cmd_roll
              << " pitch="   << g_cmd_pitch
              << "\n";
}

// 通用模板：修改某个 DOF 档位
static int step_cmd(char /*key_char*/,
                    float& cmd,
                    float delta_step,
                    const char* tag)
{
    cmd = clamp_unit(cmd + delta_step);
    log_dof_state(tag);
    return PWM_TELEOP_HANDLED;
}

/* =========================================================
 *                       帮助信息
 * ========================================================= */

extern "C" void pwm_teleop_print_help(void)
{
    std::cout <<
        "\n===== Teleop 键位说明（多自由度叠加 + 特殊姿态） =====\n"
        "  [数字测试]（阻塞式，适合示波器/单电机调试）\n"
        "    1..8 : 对应 PWM 通道单电机测试：7.5% → 8.0% 保持 "
        << TEST_HOLD_SEC <<
        "s → 回中位\n"
        "\n"
        "  [水平平面 DOF 叠加控制]\n"
        "    W / S : 前进 / 后退       （调整 surge 档位，可与 A/D/Q/E/G/H 叠加）\n"
        "    A / D : 左移 / 右移       （调整 sway 档位）\n"
        "    Q / E : 左转 / 右转       （调整 yaw 档位）\n"
        "    -> 多次按键会叠加成不同组合：例如 W+A+Q = 前进+左移+左转\n"
        "\n"
        "  [垂向 DOF 控制]\n"
        "    H : 上浮（heave 档位增加）\n"
        "    G : 下潜（heave 档位减小）\n"
        "\n"
        "  [特殊姿态（纯动作，不叠加其他 DOF）]\n"
        "    R / T : 纯横滚（Roll Right / Roll Left），只修改 roll 档位\n"
        "    F / V : 纯俯仰（Pitch Forward / Pitch Backward），只修改 pitch 档位\n"
        "      → 触发这些按键时，会自动清空 surge/sway/yaw/heave 命令，避免叠加导致意外姿态。\n"
        "\n"
        "  [通用]\n"
        "    M : 所有 DOF 清零（不直接操作 PWM）\n"
        "    Z : 显示本帮助\n"
        "   ESC: 请求退出（由上层控制循环执行停机流程）\n"
        "\n"
        "说明：\n"
        "  - 本模块主要维护 6-DOF 归一化命令状态，不负责自动控制算法；\n"
        "  - DOF→推进器→PWM 的链路由 ManualController + ControlLoop + PwmClient 完成；\n"
        "  - 数字键 1..8 为低层测试通道，会直接通过 pwm_control 阻塞式操作单个通道。\n"
        "========================================\n\n";
}

/* =========================================================
 *                  Teleop 状态操作（对外 API）
 * ========================================================= */

extern "C" void pwm_teleop_reset(void)
{
    reset_dof_cmds();
    std::cout << "[TELEOP] reset: all DOF = 0 (no PWM operation here)\n";
}

extern "C" void pwm_teleop_get_state(pwm_teleop_state_t* out_state)
{
    if (!out_state) return;

    out_state->cmd_surge = g_cmd_surge;
    out_state->cmd_sway  = g_cmd_sway;
    out_state->cmd_heave = g_cmd_heave;
    out_state->cmd_yaw   = g_cmd_yaw;
    out_state->cmd_roll  = g_cmd_roll;
    out_state->cmd_pitch = g_cmd_pitch;
}

/* =========================================================
 *             数字键：单电机测试（阻塞，直接 PWM）
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
 *                   外部调用入口：按键处理
 * ========================================================= */

extern "C" int pwm_teleop_handle_key(int key)
{
    if (key == EOF) {
        return PWM_TELEOP_IGNORED;
    }

    // ---------- 退出键（ESC）优先处理 ----------
    if (key == 27) {
        std::cout << "[KEY] ESC -> exit requested\n";
        return PWM_TELEOP_EXIT_REQUEST;
    }

    // ---------- 数字键：单电机测试（阻塞） ----------
    if (key >= '1' && key <= '8') {
        int ch_num = key - '0';
        return handle_single_motor_test(ch_num);
    }

    // 统一转为大写，简化判断
    char c = static_cast<char>(key);
    if (c >= 'a' && c <= 'z') {
        c = static_cast<char>(c - 'a' + 'A');
    }

    switch (c) {
    case 'Z':
        pwm_teleop_print_help();
        return PWM_TELEOP_HANDLED;

    case 'M':
        pwm_teleop_reset();
        return PWM_TELEOP_HANDLED;

    // ---------- 水平平面 DOF：W/S/A/D + Yaw(Q/E) ----------
    case 'W':  // 前进：surge 增加
        return step_cmd('W', g_cmd_surge, +CMD_STEP, "W");

    case 'S':  // 后退：surge 减少
        return step_cmd('S', g_cmd_surge, -CMD_STEP, "S");

    case 'A':  // 左移：sway 减少
        return step_cmd('A', g_cmd_sway, -CMD_STEP, "A");

    case 'D':  // 右移：sway 增加
        return step_cmd('D', g_cmd_sway, +CMD_STEP, "D");

    case 'Q':  // 左转 / 右转：具体正负由控制栈统一约定
        return step_cmd('Q', g_cmd_yaw, +CMD_STEP, "Q");

    case 'E':
        return step_cmd('E', g_cmd_yaw, -CMD_STEP, "E");

    // ---------- 垂向 DOF：G/H ----------
    case 'H':  // 上浮：heave 增加
        return step_cmd('H', g_cmd_heave, +CMD_STEP, "H");

    case 'G':  // 下潜：heave 减少
        return step_cmd('G', g_cmd_heave, -CMD_STEP, "G");

    // ---------- 特殊姿态：纯 roll / 纯 pitch（不叠加其他 DOF） ----------
    case 'R':   // 横滚右：roll = +1，清空其他 DOF
        reset_dof_cmds();
        g_cmd_roll  = +1.0f;
        log_dof_state("R");
        return PWM_TELEOP_HANDLED;

    case 'T':   // 横滚左：roll = -1
        reset_dof_cmds();
        g_cmd_roll  = -1.0f;
        log_dof_state("T");
        return PWM_TELEOP_HANDLED;

    case 'F':   // 俯仰：头下尾上：pitch = +1
        reset_dof_cmds();
        g_cmd_pitch = +1.0f;
        log_dof_state("F");
        return PWM_TELEOP_HANDLED;

    case 'V':   // 俯仰：尾下头上：pitch = -1
        reset_dof_cmds();
        g_cmd_pitch = -1.0f;
        log_dof_state("V");
        return PWM_TELEOP_HANDLED;

    default:
        // 其他按键：本模块不处理，上层可以做其它功能
        return PWM_TELEOP_IGNORED;
    }
}
