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
 *  - 禁止突然反向：正推→反推必须经过中位（7.5%）
 *  - 分组更新：一次只动部分电机（降低瞬时电流）
 *  - 电机通道映射：逻辑电机 1..8 映射到物理 PWM 通道 1..8
 *  - 电机方向反转：按配置或运行时开关做对称反向（min+max-pct）
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
 *
 *    pwm_ctrl_config_t ctrl_cfg = {0};
 *    ctrl_cfg.ctrl_hz         = 100.0f;                     // 上层循环 100Hz 调用 step()
 *    ctrl_cfg.max_step_pct    = 0.2f;                       // 每步最多 0.2%
 *    ctrl_cfg.group_mode      = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;
 *    ctrl_cfg.groupA_mask     = PWM_CH_MASK_1_4;            // 逻辑电机 1-4
 *    ctrl_cfg.groupB_mask     = PWM_CH_MASK_5_8;            // 逻辑电机 5-8
 *
 *    // 逻辑电机 1..8 → 物理 PWM 通道 5..8,1..4（举例）
 *    int motor_map[8] = {5,6,7,8,1,2,3,4};
 *    memcpy(ctrl_cfg.motorch_to_pwmch, motor_map, sizeof(motor_map));
 *    // 如需反向的电机置 1，未反向置 0
 *    // ctrl_cfg.motor_reverse[2] = 1; // 逻辑 3 号电机反向
 *
 *    pwm_ctrl_init(&ctrl_cfg);
 *
 *    // 2. 目标设置：例如把“逻辑 1 号电机”从中位拉到 9.0%
 *    pwm_ctrl_set_target_pct(PWM_CH1, 9.0f);
 *
 *    // 3. 主循环（100Hz）
 *    while (running) {
 *        pwm_host_poll(0);                               // 可选：接收心跳 ACK 等
 *        pwm_ctrl_step();                                // 内部限斜率 + 分组 + 映射 + 反向
 *        usleep(10000);                                  // 10ms ~ 100Hz
 *    }
 *
 *    // 4. 退出前安全归中位（在“逻辑电机空间”归中，底层自动映射到物理通道）
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
 * @brief 逻辑电机通道索引（1..8）
 *
 * 注意：
 *  - 这里的 PWM_CH1..PWM_CH8 表示“逻辑电机编号”，
 *    真正的物理 PWM 通道由配置中的 motorch_to_pwmch[] 映射决定；
 *  - 建议在项目文档中明确逻辑电机 1..8 的物理含义，例如：
 *      CH1: 前左水平推进器（逻辑）
 *      CH2: 前右水平推进器（逻辑）
 *      CH3: 后左水平推进器（逻辑）
 *      CH4: 后右水平推进器（逻辑）
 *      CH5~CH8: 垂向 / 对角等（逻辑）
 *    然后通过 motorch_to_pwmch[] 把这些逻辑电机对到 STM32 的实际 PWM 通道。
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
 *   0x0F = 0000 1111b = 逻辑 CH1~CH4
 *   0xF0 = 1111 0000b = 逻辑 CH5~CH8
 */
typedef uint8_t pwm_channel_mask_t;

/* 常用分组宏（按逻辑电机编号） */
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
 *   - 避免“一口气更新 8 路电机”带来的瞬时电流冲击；
 *   - 工程上常用：先动一半，再动另一半；或按对角线分组。
 */
typedef enum {
    /**
     * @brief 不分组：每次 step() 更新所有逻辑通道
     *
     * 优点：逻辑最简单；
     * 缺点：瞬时电流冲击较大（仅建议仿真或空载测试）。
     */
    PWM_CTRL_GROUP_MODE_ALL = 0,

    /**
     * @brief A/B 分组交替更新
     *
     * - 使用 config.groupA_mask / groupB_mask 指定各组逻辑通道；
     * - 第一次 step() 更新组 A，第二次更新组 B，如此往复；
     *
     * 示例：
     *   ctrl_hz = 100 Hz，groupA=CH1-4，groupB=CH5-8：
     *   → 每个逻辑通道实际更新频率约为 50 Hz，可贴合 STM32 侧 50 Hz 节奏。
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
 *   - ctrl_hz                → 50.0f
 *   - max_step_pct           → 0.2f
 *   - min_pct                → 5.0f
 *   - mid_pct                → 7.5f
 *   - max_pct                → 10.0f
 *   - enable_reverse_protection → 1
 *   - groupA_mask            → 0x0F (CH1-4)
 *   - groupB_mask            → 0xF0 (CH5-8)
 *   - group_mode             → PWM_CTRL_GROUP_MODE_AB_ALTERNATE
 *   - motorch_to_pwmch[]     → [1,2,3,4,5,6,7,8]（逻辑=物理）
 *   - motor_reverse[]        → 全 0（均不反向）
 */
typedef struct {
    /**
     * @brief 理论控制频率（Hz）
     *
     * - 建议设置为上层调用 pwm_ctrl_step() 的实际频率；
     * - 内部用于估算每步最大允许变化量，不直接负责 sleep。
     */
    float ctrl_hz;

    /**
     * @brief 单步最大占空比变化量（百分比）
     *
     * - 例如 ctrl_hz=50, max_step_pct=0.2：
     *     → 在连续 step 下，每秒最大变化约 0.2 * 50 = 10%；
     * - 若为 0 或负数，则使用默认 0.2f；
     * - 实现中会对该值设置一个工程上限（如 20%）以防误填。
     */
    float max_step_pct;

    /**
     * @brief 允许的占空比范围（百分比）
     *
     * - min_pct：一般 5.0f，对应 1000us；
     * - mid_pct：一般 7.5f，对应 1500us（中位/停止）；
     * - max_pct：一般 10.0f，对应 2000us；
     * - 为 0.0f 时使用默认值（5/7.5/10）；
     * - 若 min/mid/max 关系不合法，将强制回退到默认三点。
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
     *     实际效果是：正向 → 中值 → 反向（连续多步完成）。
     * - enable_reverse_protection = 0：
     *     允许直接跨过中值（仍然受 max_step_pct 限制）。
     */
    int   enable_reverse_protection;

    /**
     * @brief 分组掩码：仅在 AB 模式下有效（逻辑通道）
     */
    pwm_channel_mask_t groupA_mask;
    pwm_channel_mask_t groupB_mask;

    /**
     * @brief 分组更新模式
     */
    pwm_ctrl_group_mode_t group_mode;

    /**
     * @brief 逻辑电机 → 物理 PWM 通道映射表（1-based）
     *
     * - motorch_to_pwmch[i] 表示“逻辑 (i+1) 号电机接在第几路物理 PWM 通道上”；
     * - 合法取值范围：1..PWM_HOST_CH_NUM；
     * - 若数组整体为 0，或某个元素无效/冲突，将回退到默认 1..N 映射；
     * - 该映射仅在内部用于构造 pwm_host_set_all_pct() 的实际下发帧，
     *   上层全部工作在“逻辑电机编号 1..8”空间。
     */
    int motorch_to_pwmch[PWM_HOST_CH_NUM];

    /**
     * @brief 每个逻辑电机是否反向（0: 正常, 非 0: 反向）
     *
     * - 若 motor_reverse[i] != 0，则输出占空比将按：
     *        pct_rev = min_pct + max_pct - pct
     *   做对称反向（以中值为对称轴）；
     * - 适用于电机安装方向相反、线序问题导致“正 PWM 结果反转”的场景；
     * - 默认全 0（均不反向），可通过配置或运行时接口修改。
     */
    uint8_t motor_reverse[PWM_HOST_CH_NUM];

} pwm_ctrl_config_t;

/* ====================================================================== */
/*                         状态结构体                                     */
/* ====================================================================== */

/**
 * @brief 控制层状态快照（调试 / 记录用）
 *
 * 注意：current_pct[] / target_pct[] 的索引均为“逻辑电机编号 1..8”，
 * 物理 PWM 通道的顺序与 motorch_to_pwmch[] 映射有关。
 */
typedef struct {
    float current_pct[ PWM_HOST_CH_NUM ];  /**< 当前已下发的占空比（逻辑电机视角的软件影子值） */
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
 *  - 初始化时会将 current/target 全部设置为中位，并立即下发一帧“中位 PWM”；
 *  - motorch_to_pwmch[] / motor_reverse[] 会在内部做合法性检查与收敛。
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
 * @brief 获取当前控制状态快照（逻辑电机空间）
 *
 * @param out_state 若为 NULL，则忽略
 */
void pwm_ctrl_get_state(pwm_ctrl_state_t* out_state);

/* ====================================================================== */
/*                         目标设置接口                                   */
/* ====================================================================== */

/**
 * @brief 设置单通道目标占空比（％，逻辑电机）
 *
 * @param ch   逻辑通道号（1..8）
 * @param pct  目标占空比：
 *               - pct < 0 → 使用中位；
 *               - pct 在 [min_pct, max_pct] 之外会被裁剪到该范围。
 *
 * 说明：
 *  - 只更新“目标值”，不会立刻下发；
 *  - 实际下发由后续的 pwm_ctrl_step() 统一负责（包含限斜率/分组/映射/反向）。
 */
int pwm_ctrl_set_target_pct(int ch, float pct);

/**
 * @brief 一次性设置多个通道的目标占空比（逻辑电机空间）
 *
 * @param mask 通道掩码，bit0→CH1 ... bit7→CH8；
 * @param pct  长度为 PWM_HOST_CH_NUM 的数组，对应逻辑 CH1..CH8；pct < 0 表示中位。
 */
int pwm_ctrl_set_targets_mask(pwm_channel_mask_t mask,
                              const float pct[PWM_HOST_CH_NUM]);

/**
 * @brief 将所有逻辑通道的目标占空比设置为中位（7.5%）
 */
int pwm_ctrl_set_all_target_mid(void);

/* ====================================================================== */
/*                         调度与下发（核心）                             */
/* ====================================================================== */

/**
 * @brief 控制层主循环入口：执行一次“逼近目标 + 映射 + 下发 PWM”的 step
 *
 * 调用频率：
 *  - 推荐由上层以固定频率调用，例如 50Hz 或 100Hz；
 *  - cfg.ctrl_hz 仅用于估算每步最大变化量（max_step_pct）。
 *
 * 内部逻辑（在实现中完成）：
 *  1. 在“逻辑电机空间”中，根据 current_pct[] / target_pct[] + max_step_pct
 *     计算本步变化值；
 *  2. 根据 group_mode / groupA_mask / groupB_mask 选择本次更新逻辑通道；
 *  3. 对所选通道应用“限斜率 + 禁止突然反向”策略；
 *  4. 将逻辑电机占空比通过 motorch_to_pwmch[] + motor_reverse[] 映射到
 *     物理 PWM 通道顺序，构造完整 8 路帧；
 *  5. 调用 pwm_host_set_all_pct() 下发 8 通道 PWM；
 *  6. 更新 current_pct[] 与 step_count。
 *
 * @return PWM_CTRL_OK 或负错误码
 */
int pwm_ctrl_step(void);

/* ====================================================================== */
/*                         工程便利函数                                   */
/* ====================================================================== */

/**
 * @brief 单通道“保持某占空比一段时间”（阻塞式），便于示波器 / 单电机测试
 *
 * @param ch      逻辑通道号（1..8）
 * @param pct     目标占空比（pct < 0 → 中位，逻辑电机空间）
 * @param seconds 持续时间（秒），>0
 *
 * 内部做法：
 *   - 修改“逻辑电机”的目标占空比；
 *   - 在给定时间内循环调用 pwm_ctrl_step()（频率取 cfg.ctrl_hz）；
 *   - 期间可在外层调用 pwm_host_poll() 观察 RTT 等；
 *   - 时间到后函数返回，当前占空比保持在最后值。
 *
 * 注意：阻塞函数，适合离线/台架测试，不建议在主控制循环中直接调用。
 */
int pwm_ctrl_hold_pct_blocking(int ch, float pct, float seconds);

/**
 * @brief 紧急归中位：在指定时间内，将所有逻辑通道平滑拉回中位（7.5%）
 *
 * @param seconds 归中总时间（秒），<=0 表示“尽快”（仍受 max_step_pct 限制）
 *
 * 典型用法：
 *   - ROV 停机 / 上岸前；
 *   - 检测到通讯异常 / 传感器失效 / 上层控制退出时。
 */
int pwm_ctrl_emergency_stop(float seconds);

/* ====================================================================== */
/*                         电机反向运行时开关                             */
/* ====================================================================== */

/**
 * @brief 运行时设置某个逻辑电机的反向标志
 *
 * @param motor_id 逻辑电机编号（1..PWM_HOST_CH_NUM）
 * @param enable   0: 正常方向；非 0: 反向（min+max-pct）
 *
 * 说明：
 *  - 典型用于调试阶段：发现某个电机正反装错、线序颠倒时，
 *    可以通过该接口快速做软件级补偿；
 *  - 若希望开机即生效，应优先通过 pwm_ctrl_config_t.motor_reverse[] 配置；
 *  - 若 pwm_control 未初始化，返回 PWM_CTRL_ERR_NOT_INIT。
 */
int pwm_ctrl_set_motor_reverse(int motor_id, int enable);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PWM_CONTROL_H */
