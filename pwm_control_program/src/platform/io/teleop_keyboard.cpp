#include "platform/io/teleop_keyboard.h"
#include "libpwm_host.h"
#include "pwm_control.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <algorithm> // std::clamp

/* =========================================================
 *                工程参数：占空比与动作增量
 * ========================================================= */

// 7.5% 中位（1500us），5% 最小（1000us），10% 最大（2000us）
static constexpr float PCT_MID = PWM_HOST_PCT_MID; // 7.5
static constexpr float PCT_MIN = PWM_HOST_PCT_MIN; // 5.0
static constexpr float PCT_MAX = PWM_HOST_PCT_MAX; // 10.0

// 运动指令的“动作幅度”：在中位基础上偏移多少百分比
// 水平与垂向可分别调整
static constexpr float MOVE_DELTA_H = 1.5f;   // 水平面每个 DOF 最大偏移（％）
static constexpr float MOVE_DELTA_V = 1.5f;   // 垂向 DOF 最大偏移（％）

// 键盘每次按下时，在命令空间内增加/减少的步长（类似档位）
static constexpr float CMD_STEP = 0.25f;      // 建议 0.25：四挡可打满

// 单电机测试用保持时间（秒）
static constexpr float TEST_HOLD_SEC = 2.0f;

// =========================================================
//            DOF 命令：键盘 → “虚拟 DOF 档位”
// =========================================================
// 约定：范围 [-1, 1]，后面会映射到占空比。
// 这组语义要和 control_core::ControlOutput 对齐：
//   - surge_cmd: 前(+)/后(-)
//   - sway_cmd : 右(+)/左(-)
//   - heave_cmd: 上(+)/下(-)
//   - yaw_cmd  : 左(+)/右(-)

static float g_cmd_surge = 0.0f;   // [-1, 1]  前进(+)/后退(-)
static float g_cmd_sway  = 0.0f;   // [-1, 1]  右移(+)/左移(-)
static float g_cmd_yaw   = 0.0f;   // [-1, 1]  左转(+)/右转(-)
static float g_cmd_heave = 0.0f;   // [-1, 1]  上浮(+)/下潜(-)

// 纯姿态通道（主要供状态查询使用；当前仅在“纯 roll/pitch 模式”下使用）
static float g_cmd_roll  = 0.0f;   // [-1, 1]
static float g_cmd_pitch = 0.0f;   // [-1, 1]


/* =========================================================
 *                     小工具函数
 * ========================================================= */

static float clamp_pct(float pct)
{
    return std::clamp(pct, PCT_MIN, PCT_MAX);
}

// 把命令空间限制在 [-1, 1]
static float clamp_unit(float v)
{
    return std::clamp(v, -1.0f, 1.0f);
}

static void fill_mid_array(float pct[PWM_HOST_CH_NUM])
{
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        pct[i] = PCT_MID;
    }
}

static int set_all_mid()
{
    return pwm_ctrl_set_all_target_mid();
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

// 统一打印当前四个 DOF 的档位，避免重复代码
static void log_dof_state(const char* key_tag)
{
    std::cout << "[KEY] " << key_tag
              << " -> surge=" << g_cmd_surge
              << " sway="    << g_cmd_sway
              << " yaw="     << g_cmd_yaw
              << " heave="   << g_cmd_heave
              << "\n";
}

// 通用模板：修改某个 DOF 档位并下发 PWM
static int step_and_apply(char key_char,
                          float& cmd,
                          float delta_step,
                          const char* tag)
{
    cmd = clamp_unit(cmd + delta_step);
    int rc = 0;

    rc = pwm_ctrl_set_targets_from_dof(
        g_cmd_surge,
        g_cmd_sway,
        g_cmd_heave,
        g_cmd_yaw
    );

    if (rc < 0) {
        std::cerr << "[ERR] apply_cmds_to_pwm(" << key_char << ") rc=" << rc << "\n";
        return rc;
    }

    log_dof_state(tag);
    return PWM_TELEOP_HANDLED;
}


/* =========================================================
 *                       帮助信息
 * ========================================================= */

void pwm_teleop_print_help(void)
{
    std::cout <<
        "\n===== Teleop 键位说明（多自由度叠加 + 特殊姿态） =====\n"
        "  [数字测试]（阻塞式，适合示波器/单电机调试）\n"
        "    1..8 : 对应通道单电机测试：7.5% → 8.0% (平滑) 保持 "
        << TEST_HOLD_SEC <<
        "s → 平滑回 7.5%\n"
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
        "    R / T : 纯横滚（Roll Right / Roll Left），只使用垂向电机做横滚\n"
        "    F / V : 纯俯仰（Pitch Forward / Pitch Backward），只使用垂向电机做俯仰\n"
        "      → 触发这些按键时，会自动清空 surge/sway/yaw/heave 命令，避免叠加导致意外姿态。\n"
        "\n"
        "  [通用]\n"
        "    M : 所有 DOF 清零，所有通道回中位 7.5%\n"
        "    Z : 显示本帮助\n"
        "   ESC: 退出该程序\n"
        "\n"
        "说明：\n"
        "  - 键盘不再“一键一个姿态”，而是叠加 4 个 DOF：surge / sway / yaw / heave；\n"
        "  - 所有占空比变化仍经由 pwm_control 的限斜率 / 反向保护 / AB 分组平滑执行；\n"
        "  - 数字键测试是阻塞式（期间主循环不再响应按键）；\n"
        "  - 初次带电测试强烈建议拆下螺旋桨或在空载环境下进行。\n"
        "========================================\n\n";
}


/* =========================================================
 *                  Teleop 状态操作（对外 API）
 * ========================================================= */

void pwm_teleop_reset(void)
{
    reset_dof_cmds();
    int rc = set_all_mid();
    if (rc < 0) {
        std::cerr << "[ERR] pwm_teleop_reset: set_all_mid rc=" << rc << "\n";
    } else {
        std::cout << "[TELEOP] reset: all DOF=0, all channels to mid\n";
    }
}

void pwm_teleop_get_state(pwm_teleop_state_t* out_state)
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
 *             数字键：单电机测试（阻塞）
 * ========================================================= */

static int handle_single_motor_test(int ch_num)
{
    if (ch_num < 1 || ch_num > PWM_HOST_CH_NUM) {
        return -PWMH_EINVAL;
    }

    std::cout << "[TEST] Channel " << ch_num
              << " : 7.5% -> 8.0% hold " << TEST_HOLD_SEC
              << "s -> 7.5%\n";

    int rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_set_all_target_mid rc=" << rc << "\n";
        return rc;
    }

    rc = pwm_ctrl_hold_pct_blocking(ch_num, 8.0f, TEST_HOLD_SEC);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_hold_pct_blocking ch=" << ch_num
                  << " rc=" << rc << "\n";
        return rc;
    }

    rc = pwm_ctrl_emergency_stop(1.0f);
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_emergency_stop rc=" << rc << "\n";
        return rc;
    }

    std::cout << "[TEST] Channel " << ch_num << " 完成\n";
    return PWM_TELEOP_HANDLED;
}


/* =========================================================
 *         通用 DOF → 8 通道 PWM 映射（集中封装）
 * ========================================================= */

/**
 * 当前版本：通过一个“中间层”接口把 DOF 映射交给 pwm_control，
 * 便于后续在同一接口下接入：
 *   - 直接 teleop → PWM（当前路径）
 *   - 或 control_core::ControlOutput → PWM（自动控制路径）
 *
 * 映射规则示意：
 *   - surge_cmd: 前(+)/后(-)
 *   - sway_cmd : 右(+)/左(-)
 *   - heave_cmd: 上(+)/下(-)
 *   - yaw_cmd  : 左(+)/右(-)
 *
 * 具体电机分配逻辑藏在 pwm_control 中，便于统一维护。
 */
static int apply_cmds_to_pwm()
{
    return pwm_ctrl_set_targets_from_dof(
        g_cmd_surge,
        g_cmd_sway,
        g_cmd_heave,
        g_cmd_yaw
    );
}


/* =========================================================
 *    特殊姿态：纯横滚 / 纯俯仰（不叠加其他 DOF，直接写模式）
 * ========================================================= */

// 纯横滚（roll）：左右两侧垂向推进器产生反向推力
static int handle_pure_roll_right()   // 向右滚
{
    reset_dof_cmds();
    g_cmd_roll  = +1.0f;
    g_cmd_pitch = 0.0f;

    // 这里直接调用“姿态模式”接口，由 pwm_control 负责实际通道分配
    std::cout << "[KEY] R -> PURE ROLL RIGHT pattern\n";
    int rc = pwm_ctrl_apply_pure_roll(+MOVE_DELTA_V);
    return (rc < 0) ? rc : PWM_TELEOP_HANDLED;
}

static int handle_pure_roll_left()    // 向左滚
{
    reset_dof_cmds();
    g_cmd_roll  = -1.0f;
    g_cmd_pitch = 0.0f;

    std::cout << "[KEY] T -> PURE ROLL LEFT pattern\n";
    int rc = pwm_ctrl_apply_pure_roll(-MOVE_DELTA_V);
    return (rc < 0) ? rc : PWM_TELEOP_HANDLED;
}

// 纯俯仰（pitch）：前后两侧垂向推进器产生反向推力
static int handle_pure_pitch_forward()  // 头下尾上
{
    reset_dof_cmds();
    g_cmd_roll  = 0.0f;
    g_cmd_pitch = +1.0f;

    std::cout << "[KEY] F -> PURE PITCH FORWARD pattern\n";
    int rc = pwm_ctrl_apply_pure_pitch(+MOVE_DELTA_V);
    return (rc < 0) ? rc : PWM_TELEOP_HANDLED;
}

static int handle_pure_pitch_backward() // 尾下头上
{
    reset_dof_cmds();
    g_cmd_roll  = 0.0f;
    g_cmd_pitch = -1.0f;

    std::cout << "[KEY] V -> PURE PITCH BACKWARD pattern\n";
    int rc = pwm_ctrl_apply_pure_pitch(-MOVE_DELTA_V);
    return (rc < 0) ? rc : PWM_TELEOP_HANDLED;
}


/* =========================================================
 *                   外部调用入口：按键处理
 * ========================================================= */

int pwm_teleop_handle_key(int key)
{
    if (key == EOF) {
        return PWM_TELEOP_IGNORED;
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

    int rc = 0;

    switch (c) {
    case 'Z':
        pwm_teleop_print_help();
        return PWM_TELEOP_HANDLED;

    case 'M':
        pwm_teleop_reset();
        return PWM_TELEOP_HANDLED;

    // ---------- 水平平面 DOF：W/S/A/D + Yaw(Q/E) ----------
    case 'W':  // 前进：surge 增加
        return step_and_apply('W', g_cmd_surge, +CMD_STEP, "W");

    case 'S':  // 后退：surge 减少
        return step_and_apply('S', g_cmd_surge, -CMD_STEP, "S");

    case 'A':  // 左移：sway 减少
        return step_and_apply('A', g_cmd_sway, -CMD_STEP, "A");

    case 'D':  // 右移：sway 增加
        return step_and_apply('D', g_cmd_sway, +CMD_STEP, "D");

    case 'Q':  // 左转：yaw 增加
        return step_and_apply('Q', g_cmd_yaw, +CMD_STEP, "Q");

    case 'E':  // 右转：yaw 减少
        return step_and_apply('E', g_cmd_yaw, -CMD_STEP, "E");

    // ---------- 垂向 DOF：G/H ----------
    case 'H':  // 上浮：heave 增加
        return step_and_apply('H', g_cmd_heave, +CMD_STEP, "H");

    case 'G':  // 下潜：heave 减少
        return step_and_apply('G', g_cmd_heave, -CMD_STEP, "G");

    // ---------- 特殊姿态：纯 roll / 纯 pitch（不叠加其他 DOF） ----------
    case 'R':   // 横滚右
        return handle_pure_roll_right();

    case 'T':   // 横滚左
        return handle_pure_roll_left();

    case 'F':   // 俯仰：头下尾上
        return handle_pure_pitch_forward();

    case 'V':   // 俯仰：尾下头上
        return handle_pure_pitch_backward();

    // ---------- 退出键 ----------
    case 27: // ESC 键
        std::cout << "[KEY] ESC -> exit requested\n";
        return PWM_TELEOP_EXIT_REQUEST;

    default:
        // 其他按键：本模块不处理，上层可以用来做其它功能
        return PWM_TELEOP_IGNORED;
    }
}
