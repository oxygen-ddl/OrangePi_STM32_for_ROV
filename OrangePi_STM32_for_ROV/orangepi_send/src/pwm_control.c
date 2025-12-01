#include "pwm_control.h"
#include <string.h>
#include <math.h>
#include <time.h>
#include <errno.h>

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

/* ====================================================================== */
/*                         内部状态                                      */
/* ====================================================================== */

static int               s_inited      = 0;
static pwm_ctrl_config_t s_cfg;                         /* 当前配置（含原始映射配置） */
static float             s_current_pct[PWM_HOST_CH_NUM];/* 已下发占空比（逻辑电机） */
static float             s_target_pct [PWM_HOST_CH_NUM];/* 目标占空比（逻辑电机）  */
static uint64_t          s_step_count  = 0;

static int               s_group_toggle = 0;            /* AB 交替：0->A,1->B */

/* 收敛后的映射与反向标志（运行时使用） */
static int               s_motor_to_pwm[PWM_HOST_CH_NUM];   /* 逻辑电机 -> 物理 PWM 通道 (1-based) */
static uint8_t           s_motor_reverse[PWM_HOST_CH_NUM];  /* 每个逻辑电机是否反向 */

/* ====================================================================== */
/*                         内部工具函数                                  */
/* ====================================================================== */

static void sleep_ms(double ms)
{
    if (ms <= 0) return;
    struct timespec req, rem;
    req.tv_sec  = (time_t)(ms / 1000.0);
    req.tv_nsec = (long)((ms - (double)req.tv_sec * 1000.0) * 1e6);
    if (req.tv_nsec < 0) req.tv_nsec = 0;

    while (nanosleep(&req, &rem) != 0 && errno == EINTR) {
        req = rem;
    }
}

/* 推荐周期 ms（用于阻塞函数 sleep） */
static double ctrl_period_ms(void)
{
    float hz = (s_cfg.ctrl_hz > 0.0f) ? s_cfg.ctrl_hz : 100.0f;
    return 1000.0 / (double)hz;
}

/* 当前有效的 max_step_pct（每步最大变化量，单位：%） */
static float max_step_pct_effective(void)
{
    float step = s_cfg.max_step_pct;
    if (step <= 0.0f) {
        step = 0.2f;              /* 工程默认值 */
    }
    if (step > 20.0f) {
        step = 20.0f;             /* 工程上限，避免误配置 */
    }
    return step;
}

/* 把 DOF 命令限制在 [-1,1]，用于 4-DOF 接口 */
static float clamp_unit(float v)
{
    if (v < -1.0f) v = -1.0f;
    if (v >  1.0f) v =  1.0f;
    return v;
}

/* 判断某通道是否在 mask 中（ch 是逻辑电机号 1..N） */
static int ch_in_mask(int ch, pwm_channel_mask_t mask)
{
    if (ch < 1 || ch > PWM_HOST_CH_NUM) return 0;
    uint8_t bit = (uint8_t)(1u << (ch - 1));
    return (mask & bit) ? 1 : 0;
}

/* 当前 step 应该更新哪些逻辑通道（根据 group_mode / toggle） */
static pwm_channel_mask_t active_mask_for_this_step(void)
{
    switch (s_cfg.group_mode) {
    case PWM_CTRL_GROUP_MODE_ALL:
        return PWM_CH_MASK_ALL;

    case PWM_CTRL_GROUP_MODE_AB_ALTERNATE:
    default:
        /* AB 交替模式：一次 A，一次 B，循环往复 */
        if (s_group_toggle == 0) {
            s_group_toggle = 1;
            return s_cfg.groupA_mask;
        } else {
            s_group_toggle = 0;
            return s_cfg.groupB_mask;
        }
    }
}

/* 按当前配置的范围裁剪占空比（min_pct ~ max_pct） */
static float clamp_pct(float pct)
{
    float minp = (s_cfg.min_pct > 0.0f) ? s_cfg.min_pct : PWM_HOST_PCT_MIN;
    float maxp = (s_cfg.max_pct > 0.0f) ? s_cfg.max_pct : PWM_HOST_PCT_MAX;

    if (pct < minp) pct = minp;
    if (pct > maxp) pct = maxp;
    return pct;
}

/* 初始化 current/target：全部设为中位（逻辑电机空间） */
static void init_state_to_mid(void)
{
    float mid = (s_cfg.mid_pct > 0.0f) ? s_cfg.mid_pct : PWM_HOST_PCT_MID;

    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        s_current_pct[i] = mid;
        s_target_pct [i] = mid;
    }
    s_step_count   = 0;
    s_group_toggle = 0;
}

/* 配置默认值与安全收敛（不处理映射部分） */
static void sanitize_config(void)
{
    /* ctrl_hz 默认值 */
    if (s_cfg.ctrl_hz <= 0.0f) {
        s_cfg.ctrl_hz = 50.0f;
    }

    /* max_step_pct 默认值 & 工程上限 */
    if (s_cfg.max_step_pct <= 0.0f) {
        s_cfg.max_step_pct = 0.2f;
    } else if (s_cfg.max_step_pct > 20.0f) {
        s_cfg.max_step_pct = 20.0f;
    }

    /* 占空比范围：min/mid/max，缺省则回落到 5/7.5/10 */
    if (s_cfg.min_pct <= 0.0f)
        s_cfg.min_pct = PWM_HOST_PCT_MIN;  /* 5% */
    if (s_cfg.mid_pct <= 0.0f)
        s_cfg.mid_pct = PWM_HOST_PCT_MID;  /* 7.5% */
    if (s_cfg.max_pct <= 0.0f)
        s_cfg.max_pct = PWM_HOST_PCT_MAX;  /* 10% */

    /* 保护：如果 min/mid/max 关系不正确，强制为默认 */
    if (!(s_cfg.min_pct < s_cfg.mid_pct && s_cfg.mid_pct < s_cfg.max_pct)) {
        s_cfg.min_pct = PWM_HOST_PCT_MIN;
        s_cfg.mid_pct = PWM_HOST_PCT_MID;
        s_cfg.max_pct = PWM_HOST_PCT_MAX;
    }

    /* 反向保护：默认开启；否则非 0 视为 1 */
    s_cfg.enable_reverse_protection =
        (s_cfg.enable_reverse_protection != 0) ? 1 : 0;

    /* 分组默认值 */
    if (s_cfg.groupA_mask == 0 && s_cfg.groupB_mask == 0) {
        s_cfg.groupA_mask = PWM_CH_MASK_1_4;
        s_cfg.groupB_mask = PWM_CH_MASK_5_8;
    }
    if (s_cfg.group_mode != PWM_CTRL_GROUP_MODE_ALL &&
        s_cfg.group_mode != PWM_CTRL_GROUP_MODE_AB_ALTERNATE) {
        s_cfg.group_mode = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;
    }

    /* motor_reverse 归一化为 0/1 */
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        s_cfg.motor_reverse[i] =
            (s_cfg.motor_reverse[i] != 0) ? 1 : 0;
    }
}

/* 初始化电机映射与反向：从配置到内部运行态 */
static void init_motor_mapping_from_cfg(void)
{
    /* 默认：逻辑 1..N 对应物理 PWM 1..N，全部不反向 */
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        s_motor_to_pwm[i]  = i + 1;
        s_motor_reverse[i] = 0;
    }

    /* 检测是否配置了非 0 映射 */
    int any_nonzero = 0;
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        if (s_cfg.motorch_to_pwmch[i] != 0) {
            any_nonzero = 1;
            break;
        }
    }

    if (!any_nonzero) {
        /* 未显式配置映射，使用默认 1..N，同时根据 motor_reverse 填充反向 */
        for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
            s_motor_reverse[i] = s_cfg.motor_reverse[i];
        }
        return;
    }

    /* 校验映射是否为合法的 1..N 置换（无越界、无重复） */
    int used[PWM_HOST_CH_NUM] = {0};
    int ok = 1;

    for (int motor = 0; motor < PWM_HOST_CH_NUM; ++motor) {
        int pwmch = s_cfg.motorch_to_pwmch[motor];

        if (pwmch < 1 || pwmch > PWM_HOST_CH_NUM) {
            ok = 0;
            break;
        }
        if (used[pwmch - 1]) {
            ok = 0;
            break;
        }
        used[pwmch - 1] = 1;
    }

    if (!ok) {
        /* 非法配置：回退到默认 1..N 映射 */
        for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
            s_motor_to_pwm[i]  = i + 1;
            s_motor_reverse[i] = s_cfg.motor_reverse[i];
        }
        return;
    }

    /* 映射合法：拷贝到内部结构，并同步反向标志 */
    for (int motor = 0; motor < PWM_HOST_CH_NUM; ++motor) {
        s_motor_to_pwm[motor]  = s_cfg.motorch_to_pwmch[motor];
        s_motor_reverse[motor] = s_cfg.motor_reverse[motor];
    }
}

/* 将“按逻辑电机排列”的占空比，映射成“按物理 PWM 通道排列”的占空比 */
static void build_hw_frame_from_motor_pct(const float motor_pct[PWM_HOST_CH_NUM],
                                          float hw_pct_out[PWM_HOST_CH_NUM])
{
    float minp = (s_cfg.min_pct > 0.0f) ? s_cfg.min_pct : PWM_HOST_PCT_MIN;
    float midp = (s_cfg.mid_pct > 0.0f) ? s_cfg.mid_pct : PWM_HOST_PCT_MID;
    float maxp = (s_cfg.max_pct > 0.0f) ? s_cfg.max_pct : PWM_HOST_PCT_MAX;

    /* 默认先全部中位，防止未覆盖通道 */
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        hw_pct_out[i] = midp;
    }

    for (int motor = 0; motor < PWM_HOST_CH_NUM; ++motor) {
        int pwmch = s_motor_to_pwm[motor];    /* 1..N */
        int idx   = pwmch - 1;
        if (idx < 0 || idx >= PWM_HOST_CH_NUM) {
            continue; /* 理论上不会发生，防御性保护 */
        }

        float p = motor_pct[motor];

        /* 反向逻辑：p_rev = min + max - p */
        if (s_motor_reverse[motor]) {
            p = minp + maxp - p;
        }

        /* 再次裁剪到合法范围内 */
        if (p < minp) p = minp;
        if (p > maxp) p = maxp;

        hw_pct_out[idx] = p;
    }
}

/* ====================================================================== */
/*                         接口实现：生命周期                             */
/* ====================================================================== */

int pwm_ctrl_init(const pwm_ctrl_config_t* cfg)
{
    /* 由上层保证：libpwm_host_init 已经完成；此处只配置控制逻辑 */

    memset(&s_cfg, 0, sizeof(s_cfg));
    if (cfg) {
        s_cfg = *cfg; /* 浅拷贝配置 */
    }

    sanitize_config();
    init_state_to_mid();
    init_motor_mapping_from_cfg();

    /* 下发一次“全部中位（逻辑电机空间）”到 STM32 */
    float hw_mid[PWM_HOST_CH_NUM];
    build_hw_frame_from_motor_pct(s_current_pct, hw_mid);

    pwmh_result_t rc = pwm_host_set_all_pct(hw_mid);
    if (rc != PWMH_OK) {
        return PWM_CTRL_ERR_INTERNAL;
    }

    s_inited = 1;
    return PWM_CTRL_OK;
}

void pwm_ctrl_deinit(void)
{
    s_inited = 0;
    /* 不关闭 libpwm_host，由上层统一管理 */
}

void pwm_ctrl_get_state(pwm_ctrl_state_t* out_state)
{
    if (!out_state) return;
    memset(out_state, 0, sizeof(*out_state));
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        out_state->current_pct[i] = s_current_pct[i];
        out_state->target_pct [i] = s_target_pct [i];
    }
    out_state->step_count = s_step_count;
}

/* ====================================================================== */
/*                         目标设置接口                                   */
/* ====================================================================== */

int pwm_ctrl_set_target_pct(int ch, float pct)
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;
    if (ch < 1 || ch > PWM_HOST_CH_NUM) return PWM_CTRL_ERR_INVALID_ARG;

    if (pct < 0.0f) pct = s_cfg.mid_pct;   /* 负值视为中位 */
    pct = clamp_pct(pct);

    s_target_pct[ch - 1] = pct;
    return PWM_CTRL_OK;
}

int pwm_ctrl_set_targets_mask(pwm_channel_mask_t mask,
                              const float pct[PWM_HOST_CH_NUM])
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;
    if (!pct)      return PWM_CTRL_ERR_INVALID_ARG;

    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        int ch = i + 1;
        if (!ch_in_mask(ch, mask)) continue;

        float p = pct[i];
        if (p < 0.0f) p = s_cfg.mid_pct;
        s_target_pct[i] = clamp_pct(p);
    }
    return PWM_CTRL_OK;
}

int pwm_ctrl_set_all_target_mid(void)
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        s_target_pct[i] = s_cfg.mid_pct;
    }
    return PWM_CTRL_OK;
}

/**
 * 4-DOF → 8 通道逻辑电机目标占空比
 *
 * 这里内部固定了一套与现有 Teleop 风格一致的混控：
 *   - CH1~CH4 水平面：surge/sway/yaw 的线性组合
 *   - CH5~CH8 垂向：heave 十字/对角布置
 */
int pwm_ctrl_set_targets_from_dof4(const pwm_ctrl_dof4_cmd_t* cmd)
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;
    if (!cmd)      return PWM_CTRL_ERR_INVALID_ARG;

    float mid = (s_cfg.mid_pct > 0.0f) ? s_cfg.mid_pct : PWM_HOST_PCT_MID;
    float minp = (s_cfg.min_pct > 0.0f) ? s_cfg.min_pct : PWM_HOST_PCT_MIN;
    float maxp = (s_cfg.max_pct > 0.0f) ? s_cfg.max_pct : PWM_HOST_PCT_MAX;

    /* 取上下方向的最小裕度，防止一侧顶满一侧不满 */
    float span_up   = maxp - mid;
    float span_down = mid  - minp;
    float span      = (span_up < span_down) ? span_up : span_down;
    if (span <= 0.0f) {
        span = (maxp - minp) * 0.5f;
    }

    /* 经验系数：当前 Teleop 水平增量约 1.5%，对应 0.6 * 2.5% */
    const float H_GAIN = 0.6f;   /* 水平 DOF 利用 60% 的可用行程 */
    const float V_GAIN = 0.6f;   /* 垂向同理，可独立调整 */

    float delta_h = H_GAIN * span;
    float delta_v = V_GAIN * span;

    /* 归一化并限幅到 [-1,1] */
    float su = clamp_unit(cmd->surge);
    float sw = clamp_unit(cmd->sway);
    float hv = clamp_unit(cmd->heave);
    float yw = clamp_unit(cmd->yaw);

    /* 先全部中位（逻辑电机空间） */
    float motor_pct[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        motor_pct[i] = mid;
    }

    /* 水平面 4 通道 CH1~CH4：与 teleop 映射一致的线性组合 */
    float d1 = 0.0f; // CH1
    float d2 = 0.0f; // CH2
    float d3 = 0.0f; // CH3
    float d4 = 0.0f; // CH4

    // surge：+1 前进 → CH3、CH4 正向
    d3 += su;
    d4 += su;

    // sway：+1 右移 → CH1、CH3 正向
    d1 += sw;
    d3 += sw;

    // yaw：+1 左转 → CH1、CH4 正向
    d1 += yw;
    d4 += yw;

    motor_pct[0] = clamp_pct(mid + d1 * delta_h); // CH1
    motor_pct[1] = clamp_pct(mid + d2 * delta_h); // CH2
    motor_pct[2] = clamp_pct(mid + d3 * delta_h); // CH3
    motor_pct[3] = clamp_pct(mid + d4 * delta_h); // CH4

    /* 垂向 4 通道 CH5~CH8，保持十字/对角模式：
     *   hv > 0  上升：
     *     CH5/CH8 稍减，CH6/CH7 稍增
     */
    motor_pct[4] = clamp_pct(mid + hv * (-delta_v)); // CH5
    motor_pct[5] = clamp_pct(mid + hv * (+delta_v)); // CH6
    motor_pct[6] = clamp_pct(mid + hv * (+delta_v)); // CH7
    motor_pct[7] = clamp_pct(mid + hv * (-delta_v)); // CH8

    /* 仅更新目标，不立即下发；实际下发仍由 pwm_ctrl_step 完成 */
    return pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, motor_pct);
}

/* ====================================================================== */
/*                         核心 step 下发                                 */
/* ====================================================================== */

int pwm_ctrl_step(void)
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;

    pwm_channel_mask_t mask     = active_mask_for_this_step();
    const float        max_step = max_step_pct_effective();
    const float        mid      = s_cfg.mid_pct;

    float next_pct[PWM_HOST_CH_NUM];

    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        int   ch   = i + 1;          /* 逻辑电机号 */
        float cur  = s_current_pct[i];
        float tgt  = s_target_pct [i];

        /* 默认：本次以逻辑目标值为目标 */
        float eff_target = tgt;

        /* 若启用“禁止突然反向”，且目标与当前在中值两侧，则本步目标先夹到中值 */
        if (s_cfg.enable_reverse_protection) {
            int cur_above_mid = (cur > mid + 1e-6f);
            int cur_below_mid = (cur < mid - 1e-6f);
            int tgt_above_mid = (tgt > mid + 1e-6f);
            int tgt_below_mid = (tgt < mid - 1e-6f);

            if ((cur_above_mid && tgt_below_mid) ||
                (cur_below_mid && tgt_above_mid)) {
                eff_target = mid; /* 这一步只往中值方向走，不跨越中值 */
            }
        }

        float delta = eff_target - cur;

        /* 不在本轮更新的通道：保持 current 不变 */
        if (!ch_in_mask(ch, mask)) {
            next_pct[i] = cur;
            continue;
        }

        /* 限斜率：每步变化不能超过 max_step */
        if (fabsf(delta) > max_step) {
            if (delta > 0.0f) delta =  max_step;
            else              delta = -max_step;
        }

        float np = cur + delta;
        next_pct[i] = clamp_pct(np);
    }

    /* 逻辑电机空间 → 物理 PWM 通道空间，并下发完整 8 通道一帧 */
    float hw_pct[PWM_HOST_CH_NUM];
    build_hw_frame_from_motor_pct(next_pct, hw_pct);

    pwmh_result_t rc = pwm_host_set_all_pct(hw_pct);
    if (rc != PWMH_OK) {
        return PWM_CTRL_ERR_INTERNAL;
    }

    /* 更新 current_pct / 计数（仍在逻辑电机空间） */
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        s_current_pct[i] = next_pct[i];
    }
    ++s_step_count;

    return PWM_CTRL_OK;
}

/* ====================================================================== */
/*                         工程便利函数                                   */
/* ====================================================================== */

int pwm_ctrl_hold_pct_blocking(int ch, float pct, float seconds)
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;
    if (ch < 1 || ch > PWM_HOST_CH_NUM) return PWM_CTRL_ERR_INVALID_ARG;
    if (seconds <= 0.0f) return PWM_CTRL_ERR_INVALID_ARG;

    int rc = pwm_ctrl_set_target_pct(ch, pct);
    if (rc < 0) return rc;

    float  hz     = (s_cfg.ctrl_hz > 0.0f) ? s_cfg.ctrl_hz : 50.0f;
    int    steps  = (int)(seconds * hz + 0.5f);
    if (steps < 1) steps = 1;
    double period = ctrl_period_ms();

    for (int i = 0; i < steps; ++i) {
        int rc_step = pwm_ctrl_step();
        if (rc_step < 0) return rc_step;

        (void)pwm_host_poll(1);   /* 顺便收一下心跳 ACK 等 */

        sleep_ms(period);
    }

    return PWM_CTRL_OK;
}

int pwm_ctrl_emergency_stop(float seconds)
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;

    int rc = pwm_ctrl_set_all_target_mid();
    if (rc < 0) return rc;

    float  hz     = (s_cfg.ctrl_hz > 0.0f) ? s_cfg.ctrl_hz : 50.0f;
    double period = ctrl_period_ms();

    /* 估算需要的步数：
     * - 若 seconds > 0：按 seconds * hz
     * - 若 seconds <= 0：按 “最大偏差 / 单步步长” 估算上界 */
    float max_dev = 0.0f;
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        float dev = fabsf(s_current_pct[i] - s_cfg.mid_pct);
        if (dev > max_dev) max_dev = dev;
    }

    float step         = max_step_pct_effective();
    int   steps_by_dev = (step > 0.0f) ? (int)(max_dev / step + 1.0f) : 1;
    if (steps_by_dev < 1) steps_by_dev = 1;

    int steps;
    if (seconds > 0.0f) {
        int steps_by_time = (int)(seconds * hz + 0.5f);
        if (steps_by_time < 1) steps_by_time = 1;
        steps = (steps_by_time > steps_by_dev) ? steps_by_time : steps_by_dev;
    } else {
        steps = steps_by_dev;
    }

    for (int i = 0; i < steps; ++i) {
        int rc_step = pwm_ctrl_step();
        if (rc_step < 0) return rc_step;

        (void)pwm_host_poll(1);
        if (period > 0.0) sleep_ms(period);
    }

    return PWM_CTRL_OK;
}

/* ====================================================================== */
/*                         电机反向运行时开关                             */
/* ====================================================================== */

int pwm_ctrl_set_motor_reverse(int motor_id, int enable)
{
    if (!s_inited) return PWM_CTRL_ERR_NOT_INIT;
    if (motor_id < 1 || motor_id > PWM_HOST_CH_NUM) {
        return PWM_CTRL_ERR_INVALID_ARG;
    }

    int idx = motor_id - 1;
    s_motor_reverse[idx]       = (enable != 0) ? 1 : 0;
    s_cfg.motor_reverse[idx]   = s_motor_reverse[idx]; /* 同步回配置，便于查询 */

    return PWM_CTRL_OK;
}
