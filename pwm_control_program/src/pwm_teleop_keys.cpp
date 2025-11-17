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
static constexpr float MOVE_DELTA = 1.5f;   // +-0.5%：比较温和、安全

// 单电机测试用保持时间（秒）
static constexpr float TEST_HOLD_SEC = 3.0f;

static float clamp_pct(float pct)
{
    if (pct < PCT_MIN) pct = PCT_MIN;
    if (pct > PCT_MAX) pct = PCT_MAX;
    return pct;
}

void pwm_teleop_print_help(void)
{
    std::cout <<
        "\n===== Teleop 键位说明 =====\n"
        "  [数字测试]（阻塞式，适合示波器/单电机调试）\n"
        "    1..8 : 对应通道单电机测试：7.5% → 9% (平滑) 保持 3s → 平滑回 7.5%\n"
        "\n"
        "  [水平平面运动]\n"
        "    W : 前进  （CH3,CH4 正向，其他中位）\n"
        "    S : 后退  （CH1,CH2 正向，其他中位）\n"
        "    A : 右移  （CH1,CH3 正向，其他中位）\n"
        "    D : 左移  （CH2,CH4 正向，其他中位）\n"
        "\n"
        "  [航向角（Yaw）]\n"
        "    Q : 左转  （CH1,CH4 正转）\n"
        "    E : 右转  （CH2,CH3 正转）\n"
        "\n"
        "  [垂向运动（Heave）]\n"
        "    G : 下潜  （CH5,CH8 正转；CH6,CH7 反转）\n"
        "    H : 上浮  （CH6,CH7 正转；CH5,CH8 反转）\n"
        "\n"
        "  [通用]\n"
        "    M : 所有通道回中位 7.5%\n"
        "    Z : 显示本帮助\n"
        "   ESC: 退出该程序\n"
        "\n"
        "说明：\n"
        "  - 所有占空比变化都经由 pwm_control 的限斜率平滑执行；\n"
        "  - 数字键测试是阻塞式（期间主循环不再响应按键）；\n"
        "  - 初次带电测试强烈建议拆下螺旋桨或在空载环境下进行。\n"
        "========================================\n\n";
}

/* ====== 数字键：单电机测试 ======
 * ch_num: 1..8
 * 流程：
 *   1) 所有通道目标设为中位
 *   2) 对指定通道持有 9%，持续 3s（内部 step + poll，阻塞）
 *   3) emergency_stop(1.0s) 平滑归中
 */
static int handle_single_motor_test(int ch_num)
{
    if (ch_num < 1 || ch_num > PWM_HOST_CH_NUM) {
        return -PWMH_EINVAL;
    }

    std::cout << "[TEST] Channel " << ch_num
              << " : 7.5% -> 9.0% hold " << TEST_HOLD_SEC
              << "s -> 7.5%\n";

    int rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) {
        std::cerr << "[ERR] pwm_ctrl_set_all_target_mid rc=" << rc << "\n";
        return rc;
    }

    rc = pwm_ctrl_hold_pct_blocking(ch_num, 9.0f, TEST_HOLD_SEC);
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

/* ====== 运动键：构造占空比模式并设置目标 ====== */

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

/* -- 水平运动 -- */

static int handle_forward()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    pct[2] = clamp_pct(PCT_MID + MOVE_DELTA); // CH3
    pct[3] = clamp_pct(PCT_MID + MOVE_DELTA); // CH4

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

static int handle_backward()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    pct[0] = clamp_pct(PCT_MID + MOVE_DELTA); // CH1
    pct[1] = clamp_pct(PCT_MID + MOVE_DELTA); // CH2

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

static int handle_move_right()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    pct[0] = clamp_pct(PCT_MID + MOVE_DELTA); // CH1
    pct[2] = clamp_pct(PCT_MID + MOVE_DELTA); // CH3

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

static int handle_move_left()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    pct[1] = clamp_pct(PCT_MID + MOVE_DELTA); // CH2
    pct[3] = clamp_pct(PCT_MID + MOVE_DELTA); // CH4

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

/* -- 航向角（yaw） -- */

static int handle_yaw_left()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    pct[0] = clamp_pct(PCT_MID + MOVE_DELTA); // CH1
    pct[3] = clamp_pct(PCT_MID + MOVE_DELTA); // CH4

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

static int handle_yaw_right()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    pct[1] = clamp_pct(PCT_MID + MOVE_DELTA); // CH2
    pct[2] = clamp_pct(PCT_MID + MOVE_DELTA); // CH3

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

/* -- 垂向 (heave) -- */

static int handle_heave_up()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    // 6,7 正转；5,8 反转
    pct[4] = clamp_pct(PCT_MID - MOVE_DELTA); // CH5 反转
    pct[5] = clamp_pct(PCT_MID + MOVE_DELTA); // CH6 正转
    pct[6] = clamp_pct(PCT_MID + MOVE_DELTA); // CH7 正转
    pct[7] = clamp_pct(PCT_MID - MOVE_DELTA); // CH8 反转

    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, pct);
}

static int handle_heave_down()
{
    float pct[PWM_HOST_CH_NUM];
    fill_mid_array(pct);

    // 5,8 正转；6,7 反转
    pct[4] = clamp_pct(PCT_MID + MOVE_DELTA); // CH5 正转
    pct[5] = clamp_pct(PCT_MID - MOVE_DELTA); // CH6 反转
    pct[6] = clamp_pct(PCT_MID - MOVE_DELTA); // CH7 反转
    pct[7] = clamp_pct(PCT_MID + MOVE_DELTA); // CH8 正转

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
        rc = set_all_mid();
        if (rc < 0) {
            std::cerr << "[ERR] set_all_mid rc=" << rc << "\n";
            return rc;
        }
        std::cout << "[KEY] M -> all mid\n";
        return 1;

    // 水平运动
    case 'W':
        rc = handle_forward();
        if (rc < 0) std::cerr << "[ERR] handle_forward rc=" << rc << "\n";
        else        std::cout << "[KEY] W -> forward pattern\n";
        return (rc < 0) ? rc : 1;

    case 'S':
        rc = handle_backward();
        if (rc < 0) std::cerr << "[ERR] handle_backward rc=" << rc << "\n";
        else        std::cout << "[KEY] S -> backward pattern\n";
        return (rc < 0) ? rc : 1;

    case 'A':
        rc = handle_move_right();
        if (rc < 0) std::cerr << "[ERR] handle_move_right rc=" << rc << "\n";
        else        std::cout << "[KEY] A -> move right pattern\n";
        return (rc < 0) ? rc : 1;

    case 'D':
        rc = handle_move_left();
        if (rc < 0) std::cerr << "[ERR] handle_move_left rc=" << rc << "\n";
        else        std::cout << "[KEY] D -> move left pattern\n";
        return (rc < 0) ? rc : 1;

    // 航向
    case 'Q':
        rc = handle_yaw_left();
        if (rc < 0) std::cerr << "[ERR] handle_yaw_left rc=" << rc << "\n";
        else        std::cout << "[KEY] Q -> yaw LEFT pattern\n";
        return (rc < 0) ? rc : 1;

    case 'E':
        rc = handle_yaw_right();
        if (rc < 0) std::cerr << "[ERR] handle_yaw_right rc=" << rc << "\n";
        else        std::cout << "[KEY] E -> yaw RIGHT pattern\n";
        return (rc < 0) ? rc : 1;

    // 垂向
    case 'G':
        rc = handle_heave_down();
        if (rc < 0) std::cerr << "[ERR] handle_heave_down rc=" << rc << "\n";
        else        std::cout << "[KEY] G -> heave DOWN pattern\n";
        return (rc < 0) ? rc : 1;

    case 'H':
        rc = handle_heave_up();
        if (rc < 0) std::cerr << "[ERR] handle_heave_up rc=" << rc << "\n";
        else        std::cout << "[KEY] H -> heave UP pattern\n";
        return (rc < 0) ? rc : 1;
        // ===== 新增：退出键 =====
    case 27: // ESC 键
        std::cout << "[KEY] ESC -> exit requested\n";
        return 2;
    default:
        // 其他按键：本模块不处理，上层可以用来做退出等
        return 0;
    }
}

