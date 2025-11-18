#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

/**
 * @file    pwm_control.h
 * @brief   基于 libpwm_host 的工程化 PWM 控制层（8 通道 ROV 电机）
 *
 * 分层设计：
 *  - STM32 端：解析协议 + 输出 8 路 PWM（1000~2000us）
 *  - libpwm_host：打包协议 + UDP 发送（“会说话”）
 *  - pwm_control：面向“电机控制场景”的安全封装（“怎么说话”）
 *
 * 主要功能：
 *  - 管理 8 路推进器的“当前占空比 / 目标占空比”（单位：%）
 *  - 限斜率：每步最大变化量 max_step_pct，避免突变
 *  - 禁止突然反向：正推→反推必须经过中位 7.5%
 *  - 分组更新：一次只动部分电机（降低瞬时电流）
 *  - 提供工程接口：保持占空比、渐变、分组测试、紧急归中位等
 *
 * 物理对应关系（与 STM32 工程约定一致）：
 *  - 5.0%   → 1000 us（反向最大）
 *  - 7.5%   → 1500 us（中位）
 *  - 10.0%  → 2000 us（正向最大）
 *
 * 典型使用方式（伪代码）：
 *
 *    // 1. 初始化 libpwm_host 和 pwm_control
 *    pwm_host_init(&host_cfg);
 *    pwm_ctrl_config_t ctrl_cfg = {0};
 *    ctrl_cfg.ctrl_hz      = 100.0f;                     // 上层循环 100Hz 调用 step()
 *    ctrl_cfg.max_step_pct = 0.2f;                       // 每步最多 0.2%
 *    ctrl_cfg.group_mode   = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;
 *    ctrl_cfg.groupA_mask  = PWM_CH_MASK_1_4;            // CH1-4
 *    ctrl_cfg.groupB_mask  = PWM_CH_MASK_5_8;            // CH5-8
 *    pwm_ctrl_init(&ctrl_cfg);
 *
 *    // 2. 目标设置：例如把 1 号电机从中位拉到 9.0%
 *    pwm_ctrl_set_target_pct(PWM_CH1, 9.0f);
 *
 *    // 3. 主循环（100Hz）
 *    while (running) {
 *        pwm_host_poll(0);                               // 可选：接收心跳 ACK 等
 *        pwm_ctrl_step();                                // 内部按限斜率 + 分组平滑逼近目标
 *        usleep(10000);                                  // 10ms ~ 100Hz
 *    }
 *
 *    // 4. 退出前安全归中位
 *    pwm_ctrl_emergency_stop(1.0f);                      // 1 秒平滑拉回 7.5%
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "libpwm_host.h"

/* ====================================================================== */
/*                         通道与分组约定                                 */
/* ====================================================================== */

/**
 * @brief 通道索引（1..8），与 STM32 / 物理推进器一一对应
 *
 * 建议在项目文档中明确每个通道对应的物理解算：
 *   CH1: 前左水平推进器
 *   CH2: 前右水平推进器
 *   CH3: 后左水平推进器
 *   CH4: 后右水平推进器
 *   CH5~CH8: 垂向 / 对角等
 */
typedef enum {
    PWM_CH1 = 1,
    PWM_CH2 = 2,
    PWM_CH3 = 3,
    PWM_CH4 = 4,
    PWM_CH5 = 5,
    PWM_CH6 = 6,
    PWM_CH7 = 7,
    PWM_CH8 = 8,
} pwm_channel_t;

/**
 * @brief 通道掩码（bit0 → CH1，bit7 → CH8）
 *
 * 例如：
 *   0x0F = 0000 1111b = CH1~CH4
 *   0xF0 = 1111 0000b = CH5~CH8
 */
typedef uint8_t pwm_channel_mask_t;

/* 常用分组宏（可按 ROV 构型在文档中细化说明） */
#define PWM_CH_MASK_ALL      ((pwm_channel_mask_t)0xFFu)  /* CH1-8 全部 */
#define PWM_CH_MASK_1_4      ((pwm_channel_mask_t)0x0Fu)  /* CH1-4 */
#define PWM_CH_MASK_5_8      ((pwm_channel_mask_t)0xF0u)  /* CH5-8 */

/* ====================================================================== */
/*                         分组更新模式                                   */
/* ====================================================================== */

/**
 * @brief 分组更新模式
 *
 * 分组的目的：
 *   - 避免“一口气更新 8 路电机”带来的瞬时电流冲击
 *   - 工程上常用：先动一半，再动另一半；或按对角线分组
 */
typedef enum {
    /**
     * @brief 不分组：每次 step() 更新所有通道
     *
     * 优点：逻辑最简单；缺点：瞬时电流冲击较大（仅建议仿真或空载测试）。
     */
    PWM_CTRL_GROUP_MODE_ALL = 0,

    /**
     * @brief A/B 分组交替更新
     *
     * - 使用 config.groupA_mask / groupB_mask 指定各组通道；
     * - 第一次 step() 更新组 A，第二次更新组 B，如此往复；
     *
     * 示例：
     *   ctrl_hz = 100 Hz，groupA=CH1-4，groupB=CH5-8：
     *   → 每个通道实际更新频率约为 50 Hz，贴合 STM32 当前 50 Hz 控制节奏。
     */
    PWM_CTRL_GROUP_MODE_AB_ALTERNATE = 1,
} pwm_ctrl_group_mode_t;

/* ====================================================================== */
/*                         错误码定义                                     */
/* ====================================================================== */

/* 统一的返回值约定：0 成功，<0 表示错误类型 */
#define PWM_CTRL_OK                0
#define PWM_CTRL_ERR_NOT_INIT     -1  /**< 未调用 pwm_ctrl_init */
#define PWM_CTRL_ERR_INVALID_ARG  -2  /**< 参数非法（通道号/指针等） */
#define PWM_CTRL_ERR_INTERNAL     -3  /**< 内部错误（如 libpwm_host 发送失败） */

/* ====================================================================== */
/*                         配置结构体                                     */
/* ====================================================================== */

/**
 * @brief pwm_control 初始化配置
 *
 * 所有字段若为 0/NULL，将使用“安全的工程默认值”：
 *   - ctrl_hz           → 50.0f
 *   - max_step_pct      → 0.2f
 *   - min_pct           → 5.0f
 *   - mid_pct           → 7.5f
 *   - max_pct           → 10.0f
 *   - enable_reverse_protection → 1
 *   - groupA_mask       → 0x0F (CH1-4)
 *   - groupB_mask       → 0xF0 (CH5-8)
 *   - group_mode        → PWM_CTRL_GROUP_MODE_AB_ALTERNATE
 */
typedef struct {
    /**
     * @brief 理论控制频率（Hz）
     *
     * - 建议设置为上层调用 pwm_ctrl_step() 的实际频率；
     * - 仅用于“估算每步最大允许变化量”的参考，不直接控制 sleep。
     */
    float ctrl_hz;

    /**
     * @brief 单步最大占空比变化量（百分比）
     *
     * - 例如 ctrl_hz=50, max_step_pct=0.2：
     *     → 每秒最大变化约 0.2 * 50 = 10%；
     * - 若为 0 或负数，则使用默认 0.2f。
     */
    float max_step_pct;

    /**
     * @brief 允许的占空比范围（百分比）
     *
     * - min_pct：一般 5.0f，对应 1000us；
     * - mid_pct：一般 7.5f，对应 1500us（中位/停止）；
     * - max_pct：一般 10.0f，对应 2000us；
     * - 为 0.0f 时使用默认值（5/7.5/10）。
     */
    float min_pct;
    float mid_pct;
    float max_pct;

    /**
     * @brief 是否启用“禁止突然反向”保护
     *
     * - enable_reverse_protection = 1（默认）：
     *     若当前在中值上方而目标在中值下方（或反之），
     *     本次 step 只允许向中值方向移动，不跨过中值；
     *     实际效果是：正向 → 中值 → 反向（每步连续）。
     */
    int   enable_reverse_protection;

    /**
     * @brief 分组掩码：仅在 AB 模式下有效
     */
    pwm_channel_mask_t groupA_mask;
    pwm_channel_mask_t groupB_mask;

    /**
     * @brief 分组更新模式
     */
    pwm_ctrl_group_mode_t group_mode;

} pwm_ctrl_config_t;

/* ====================================================================== */
/*                         状态结构体                                     */
/* ====================================================================== */

/**
 * @brief 控制层状态快照（调试 / 记录用）
 */
typedef struct {
    float current_pct[ PWM_HOST_CH_NUM ];  /**< 当前已下发的占空比（软件影子值） */
    float target_pct[  PWM_HOST_CH_NUM ];  /**< 当前目标占空比（最近一次设定） */
    uint64_t step_count;                   /**< 已执行的 step() 次数 */
} pwm_ctrl_state_t;

/* ====================================================================== */
/*                         基础生命周期                                   */
/* ====================================================================== */

/**
 * @brief 初始化 pwm_control 层（基于全局状态）
 *
 * 要求：
 *  - 调用前必须已经成功初始化 libpwm_host（pwm_host_init）；
 *  - 本函数不会创建 socket，只使用 libpwm_host 的发送接口；
 *  - 初始化时会将 current/target 全部设置为中位，并立即下发一帧中位 PWM。
 *
 * @param cfg 若为 NULL 则使用默认配置（工程安全）
 * @return PWM_CTRL_OK 或负错误码
 */
int pwm_ctrl_init(const pwm_ctrl_config_t* cfg);

/**
 * @brief 释放控制层资源（重置内部状态，不关闭 libpwm_host）
 */
void pwm_ctrl_deinit(void);

/**
 * @brief 获取当前控制配置和状态快照
 *
 * @param out_state 若为 NULL，则忽略
 */
void pwm_ctrl_get_state(pwm_ctrl_state_t* out_state);

/* ====================================================================== */
/*                         目标设置接口                                   */
/* ====================================================================== */

/**
 * @brief 设置单通道目标占空比（％）
 *
 * @param ch   通道号（1..8）
 * @param pct  目标占空比：
 *               - pct < 0 → 使用中位；
 *               - pct 在 [min_pct, max_pct] 之外会被裁剪。
 *
 * 说明：
 *  - 只更新“目标值”，不会立刻下发；
 *  - 实际下发由后续的 pwm_ctrl_step() 负责。
 */
int pwm_ctrl_set_target_pct(int ch, float pct);

/**
 * @brief 一次性设置多个通道的目标占空比
 *
 * @param mask 通道掩码，bit0→CH1 ... bit7→CH8；
 * @param pct  长度为 8 的数组，对应 CH1..CH8；pct < 0 表示中位。
 */
int pwm_ctrl_set_targets_mask(pwm_channel_mask_t mask,
                              const float pct[PWM_HOST_CH_NUM]);

/**
 * @brief 将所有通道的目标占空比设置为中位（7.5%）
 */
int pwm_ctrl_set_all_target_mid(void);

/* ====================================================================== */
/*                         调度与下发（核心）                             */
/* ====================================================================== */

/**
 * @brief 控制层主循环入口：执行一次“逼近目标 + 下发 PWM”的 step
 *
 * 调用频率：
 *  - 推荐由上层以固定频率调用，例如 50Hz 或 100Hz；
 *  - cfg.ctrl_hz 仅用于估算每步最大变化量（max_step_pct）。
 *
 * 内部逻辑（在实现中完成）：
 *  1. 根据 current_pct[] / target_pct[] + max_step_pct 计算“本步变化值”；
 *  2. 根据 group_mode / groupA_mask / groupB_mask 选择本次更新通道；
 *  3. 对所选通道应用“限斜率 + 禁止突然反向”策略；
 *  4. 调用 pwm_host_set_all_pct() 下发 8 通道 PWM；
 *  5. 更新 current_pct[] 与 step_count。
 *
 * @return PWM_CTRL_OK 或负错误码
 */
int pwm_ctrl_step(void);

/* ====================================================================== */
/*                         工程便利函数                                   */
/* ====================================================================== */

/**
 * @brief 单通道“保持某占空比一段时间”（阻塞式），便于示波器测试
 *
 * @param ch      通道号（1..8）
 * @param pct     目标占空比（pct < 0 → 中位）
 * @param seconds 持续时间（秒），>0
 *
 * 内部做法：
 *   - 修改目标占空比；
 *   - 在给定时间内循环调用 pwm_ctrl_step()（频率取 cfg.ctrl_hz）；
 *   - 期间可以在外层同时调用 pwm_host_poll() 观察 RTT 等；
 *   - 时间到后函数返回，当前占空比保持在最后值。
 *
 * 注意：阻塞函数，适合离线测试，不建议在主控制循环中直接调用。
 */
int pwm_ctrl_hold_pct_blocking(int ch, float pct, float seconds);

/**
 * @brief 紧急归中位：在指定时间内，将所有通道平滑拉回 7.5%
 *
 * @param seconds 归中总时间（秒），<=0 表示“尽快”（仍受 max_step_pct 限制）
 *
 * 典型用法：
 *   - ROV 停机 / 上岸前；
 *   - 检测到通讯异常 / 传感器失效时。
 */
int pwm_ctrl_emergency_stop(float seconds);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PWM_CONTROL_H */
