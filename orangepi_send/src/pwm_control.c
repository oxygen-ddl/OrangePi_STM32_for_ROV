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
static pwm_ctrl_config_t s_cfg;                         /* 当前配置 */
static float             s_current_pct[PWM_HOST_CH_NUM];/* 已下发占空比 */
static float             s_target_pct [PWM_HOST_CH_NUM];/* 目标占空比  */
static uint64_t          s_step_count  = 0;

static int               s_group_toggle = 0;            /* AB 交替：0->A,1->B */

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
    float hz = (s_cfg.ctrl_hz > 0.0f) ? s_cfg.ctrl_hz : 50.0f;
    return 1000.0 / (double)hz;
}

/* 当前有效的 max_step_pct（每步最大变化量，单位：%） */
static float max_step_pct_effective(void)
{
    if (s_cfg.max_step_pct <= 0.0f) {
        return 100.0f; /* 不限制：给一个大值，相当于单步到位 */
    }
    return s_cfg.max_step_pct;
}

/* 判断某通道是否在 mask 中 */
static int ch_in_mask(int ch, pwm_channel_mask_t mask)
{
    if (ch < 1 || ch > PWM_HOST_CH_NUM) return 0;
    uint8_t bit = (uint8_t)(1u << (ch - 1));
    return (mask & bit) ? 1 : 0;
}

/* 当前 step 应该更新哪些通道（根据 group_mode / toggle） */
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

/* 初始化内部状态：全部设为中位 */
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

    /* ctrl_hz / max_step_pct 默认值 */
    if (s_cfg.ctrl_hz <= 0.0f)
        s_cfg.ctrl_hz = 50.0f;
    if (s_cfg.max_step_pct <= 0.0f)
        s_cfg.max_step_pct = 0.2f;

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

    /* 反向保护：cfg 未填时默认为开启；否则非 0 视为 1 */
    if (!cfg) {
        s_cfg.enable_reverse_protection = 1;
    } else {
        s_cfg.enable_reverse_protection = (cfg->enable_reverse_protection != 0) ? 1 : 0;
    }

    /* 分组默认值 */
    if (s_cfg.groupA_mask == 0 && s_cfg.groupB_mask == 0) {
        s_cfg.groupA_mask = PWM_CH_MASK_1_4;
        s_cfg.groupB_mask = PWM_CH_MASK_5_8;
    }
    if (s_cfg.group_mode != PWM_CTRL_GROUP_MODE_ALL &&
        s_cfg.group_mode != PWM_CTRL_GROUP_MODE_AB_ALTERNATE) {
        s_cfg.group_mode = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;
    }

    init_state_to_mid();

    /* 下发一次“全部中位”到 STM32 */
    float mid_arr[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) mid_arr[i] = s_cfg.mid_pct;

    pwmh_result_t rc = pwm_host_set_all_pct(mid_arr);
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
        int   ch   = i + 1;
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

    /* 下发完整 8 通道一帧 */
    pwmh_result_t rc = pwm_host_set_all_pct(next_pct);
    if (rc != PWMH_OK) {
        return PWM_CTRL_ERR_INTERNAL;
    }

    /* 更新 current_pct / 计数 */
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

    float hz = (s_cfg.ctrl_hz > 0.0f) ? s_cfg.ctrl_hz : 50.0f;
    int   steps = (int)(seconds * hz + 0.5f);
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

    float hz = (s_cfg.ctrl_hz > 0.0f) ? s_cfg.ctrl_hz : 50.0f;
    double period = ctrl_period_ms();

    /* 估算需要的步数：  
     * - 若 seconds > 0：按 seconds * hz  
     * - 若 seconds <= 0：按 “最大偏差 / 单步步长” 估算上界 */
    float max_dev = 0.0f;
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        float dev = fabsf(s_current_pct[i] - s_cfg.mid_pct);
        if (dev > max_dev) max_dev = dev;
    }

    float step = max_step_pct_effective();
    int steps_by_dev = (step > 0.0f) ? (int)(max_dev / step + 1.0f) : 1;
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

