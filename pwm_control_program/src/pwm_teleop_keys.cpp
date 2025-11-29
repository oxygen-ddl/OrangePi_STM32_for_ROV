#include "pwm_teleop_keys.h"
#include "libpwm_host.h"
#include "pwm_control.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>

/* ====== 工程参数：占空比与动作增量 ====== */

// 7.5% 中位（1500us），5% 最小（1000us），10% 最大（2000us）
static constexpr float PCT_MID  = PWM_HOST_PCT_MID; // 7.5
static constexpr float PCT_MIN  = PWM_HOST_PCT_MIN; // 5.0
static constexpr float PCT_MAX  = PWM_HOST_PCT_MAX; // 10.0

// 运动指令的“动作幅度”：在中位基础上偏移多少百分比
// 水平与垂向可以分开调
static constexpr float MOVE_DELTA_H = 1.5f;   // 水平面每个 DOF 最大偏移（％）
static constexpr float MOVE_DELTA_V = 1.5f;   // 垂向 DOF 最大偏移（％）

// 键盘每次按下时，在命令空间内增加/减少的步长（类似档位）
static constexpr float CMD_STEP = 0.25f;      // 建议 0.25：四档可打满

// 单电机测试用保持时间（秒）
static constexpr float TEST_HOLD_SEC = 1.0f;

// 水平面 3 个自由度命令：surge(前后)、sway(左右)、yaw(转向)
static float g_cmd_surge = 0.0f;   // [-1, 1]  前进(+)/后退(-)
static float g_cmd_sway  = 0.0f;   // [-1, 1]  右移(+)/左移(-)
static float g_cmd_yaw   = 0.0f;   // [-1, 1]  左转(+)/右转(-)

// 垂向 heave：上浮(+)/下潜(-)
static float g_cmd_heave = 0.0f;   // [-1, 1]

/* ====== 小工具函数 ====== */

static float clamp_pct(float pct)
{
    if (pct < PCT_MIN) pct = PCT_MIN;
    if (pct > PCT_MAX) pct = PCT_MAX;
    return pct;
}

// 把命令空间限制在 [-1, 1]
static float clamp_unit(float v)
{
    if (v < -1.0f) v = -1.0f;
    if (v >  1.0f) v =  1.0f;
    return v;
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

/* ====== 帮助信息 ====== */

void pwm_teleop_print_help(void)
{
    std::cout <<
        "\n===== Teleop 键位说明（多自由度叠加 + 特殊姿态） =====\n"
        "  [数字测试]（阻塞式，适合示波器/单电机调试）\n"
        "    1..8 : 对应通道单电机测试：7.5% → 9% (平滑) 保持 3s → 平滑回 7.5%\n"
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

/* ====== 数字键：单电机测试（阻塞） ====== */

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
    return 1;   // >0 表示已处理
}

/* ====== 多 DOF 命令 -> 8 通道 PWM 映射 ====== */

static int apply_cmds_to_pwm()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);  // 先全部 7.5%

    const float su = g_cmd_surge;  // surge: 前(+)/后(-)
    const float sw = g_cmd_sway;   // sway : 右(+)/左(-)
    const float yw = g_cmd_yaw;    // yaw  : 左(+)/右(-)
    const float hv = g_cmd_heave;  // heave: 上(+)/下(-)

    // ----------------------------
    // 水平面 4 通道 CH1~CH4
    // ----------------------------
    //
    // 保持你原来键位直觉一致：
    //   - 原设计 A：右移 → CH1, CH3 正向
    //   - 原设计 W：前进 → CH3, CH4 正向
    //   - 原设计 Q：左转 → CH1, CH4 正向
    //
    // 现在按键含义调整为：
    //   A：左移（sway -），D：右移（sway +），但正方向几何分配不变。
    //
    float d1 = 0.0f; // CH1
    float d2 = 0.0f; // CH2
    float d3 = 0.0f; // CH3
    float d4 = 0.0f; // CH4

    // surge：+1 时前进：CH3、CH4 正向
    d3 += su;
    d4 += su;

    // sway：+1 时等价于“右移几何分配”：CH1、CH3 正向
    d1 += sw;
    d3 += sw;

    // yaw：+1 时等价于“左转”：CH1、CH4 正向
    d1 += yw;
    d4 += yw;

    pct[0] = clamp_pct(PCT_MID + d1 * MOVE_DELTA_H); // CH1
    pct[1] = clamp_pct(PCT_MID + d2 * MOVE_DELTA_H); // CH2
    pct[2] = clamp_pct(PCT_MID + d3 * MOVE_DELTA_H); // CH3
    pct[3] = clamp_pct(PCT_MID + d4 * MOVE_DELTA_H); // CH4

    // ----------------------------
    // 垂向 4 通道 CH5~CH8（十字/对角布置）
    // ----------------------------
    //
    // heave up (上浮)：
    //   CH5: 7.5 - Δ   (反转)
    //   CH6: 7.5 + Δ   (正转)
    //   CH7: 7.5 + Δ   (正转)
    //   CH8: 7.5 - Δ   (反转)
    //
    pct[4] = clamp_pct(PCT_MID + hv * (-MOVE_DELTA_V)); // CH5
    pct[5] = clamp_pct(PCT_MID + hv * (+MOVE_DELTA_V)); // CH6
    pct[6] = clamp_pct(PCT_MID + hv * (+MOVE_DELTA_V)); // CH7
    pct[7] = clamp_pct(PCT_MID + hv * (-MOVE_DELTA_V)); // CH8

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

/* ====== 特殊姿态：纯横滚 / 纯俯仰（不叠加其他 DOF） ====== */

// 纯横滚（roll）：左右两侧垂向推进器产生反向推力
static int handle_pure_roll_right()   // 向右滚
{
    reset_dof_cmds();

    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);  // 所有通道先回中位

    // 右侧（6,7）推力略增，左侧（5,8）略减
    pct[4] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH5 左侧
    pct[5] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH6 右侧
    pct[6] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH7 右侧
    pct[7] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH8 左侧

    std::cout << "[KEY] R -> PURE ROLL RIGHT pattern\n";
    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

static int handle_pure_roll_left()    // 向左滚
{
    reset_dof_cmds();

    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    // 与右滚相反
    pct[4] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH5 左侧
    pct[5] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH6 右侧
    pct[6] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH7 右侧
    pct[7] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH8 左侧

    std::cout << "[KEY] T -> PURE ROLL LEFT pattern\n";
    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

// 纯俯仰（pitch）：前后两侧垂向推进器产生反向推力
static int handle_pure_pitch_forward()  // 头下尾上
{
    reset_dof_cmds();

    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    // 假设 CH5/CH6 在前，CH7/CH8 在后：
    pct[4] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH5 前左
    pct[5] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH6 前右
    pct[6] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH7 后右
    pct[7] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH8 后左

    std::cout << "[KEY] F -> PURE PITCH FORWARD pattern\n";
    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

static int handle_pure_pitch_backward() // 尾下头上
{
    reset_dof_cmds();

    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    pct[4] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH5 前左
    pct[5] = clamp_pct(PCT_MID + MOVE_DELTA_V); // CH6 前右
    pct[6] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH7 后右
    pct[7] = clamp_pct(PCT_MID - MOVE_DELTA_V); // CH8 后左

    std::cout << "[KEY] V -> PURE PITCH BACKWARD pattern\n";
    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

/* ====== 外部调用入口 ====== */

int pwm_teleop_handle_key(int key)
{
    if (key == EOF) return 0;

    // ---------- 数字键：单电机测试（阻塞） ----------
    if (key >= '1' && key <= '8') {
        int ch_num = key - '0';
        return handle_single_motor_test(ch_num);  // >0 处理成功；<0 错误
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
        return 1;

    case 'M':
        reset_dof_cmds();
        rc = set_all_mid();
        if (rc < 0) {
            std::cerr << "[ERR] set_all_mid rc=" << rc << "\n";
            return rc;
        }
        std::cout << "[KEY] M -> all mid & cmds reset\n";
        return 1;

    // ---------- 水平平面 DOF：W/S/A/D + Yaw(Q/E) ----------
    case 'W':  // 前进：surge 增加
        g_cmd_surge = clamp_unit(g_cmd_surge + CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (W) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("W");
        return 1;

    case 'S':  // 后退：surge 减少
        g_cmd_surge = clamp_unit(g_cmd_surge - CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (S) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("S");
        return 1;

    case 'A':  // 左移：sway 减少
        g_cmd_sway = clamp_unit(g_cmd_sway - CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (A) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("A");
        return 1;

    case 'D':  // 右移：sway 增加
        g_cmd_sway = clamp_unit(g_cmd_sway + CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (D) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("D");
        return 1;

    case 'Q':  // 左转：yaw 增加
        g_cmd_yaw = clamp_unit(g_cmd_yaw + CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (Q) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("Q");
        return 1;

    case 'E':  // 右转：yaw 减少
        g_cmd_yaw = clamp_unit(g_cmd_yaw - CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (E) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("E");
        return 1;

    // ---------- 垂向 DOF：G/H ----------
    case 'H':  // 上浮：heave 增加
        g_cmd_heave = clamp_unit(g_cmd_heave + CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (H) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("H");
        return 1;

    case 'G':  // 下潜：heave 减少
        g_cmd_heave = clamp_unit(g_cmd_heave - CMD_STEP);
        rc = apply_cmds_to_pwm();
        if (rc < 0) {
            std::cerr << "[ERR] apply_cmds_to_pwm (G) rc=" << rc << "\n";
            return rc;
        }
        log_dof_state("G");
        return 1;

    // ---------- 特殊姿态：纯 roll / 纯 pitch（不叠加其他 DOF） ----------
    case 'R':   // 横滚右
        rc = handle_pure_roll_right();
        return (rc < 0) ? rc : 1;

    case 'T':   // 横滚左
        rc = handle_pure_roll_left();
        return (rc < 0) ? rc : 1;

    case 'F':   // 俯仰：头下尾上
        rc = handle_pure_pitch_forward();
        return (rc < 0) ? rc : 1;

    case 'V':   // 俯仰：尾下头上
        rc = handle_pure_pitch_backward();
        return (rc < 0) ? rc : 1;

    // ---------- 退出键 ----------
    case 27: // ESC 键
        std::cout << "[KEY] ESC -> exit requested\n";
        return 2;

    default:
        // 其他按键：本模块不处理，上层可以用来做其它功能
        return 0;
    }
}
