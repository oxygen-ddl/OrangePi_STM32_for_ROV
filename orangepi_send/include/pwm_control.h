#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

/**
 * @file    pwm_control.h
 * @brief   基于 libpwm_host 的工程化 PWM 控制层（8 通道 ROV 推进器）
 *
 * 分层设计：
 *  - STM32 端：解析协议 + 输出 8 路 PWM（1000~2000 us）
 *  - libpwm_host：打包协议 + UDP 发送（“会说话”）
 *  - pwm_control：面向“电机控制场景”的安全封装（“怎么说话”）
 *
 * 主要功能：
 *  - 管理 8 路推进器的“当前占空比 / 目标占空比”（单位：%）
 *  - 限斜率：每步最大变化量 max_step_pct，避免占空比突变
 *  - 禁止突然反向：正推→反推必须经过中位（7.5%）
 *  - 分组更新：一次只更新部分电机（降低瞬时电流）
 *  - 电机通道映射：逻辑电机 1..8 → 物理 PWM 通道 1..8
 *  - 电机方向反转：按配置或运行时开关做对称反向（min + max - pct）
 *  - 提供工程接口：保持占空比、渐变、分组测试、紧急归中位等
 *
 * 物理对应关系（与 STM32 工程约定一致）：
 *  -  5.0% → 1000 us（反向最大）
 *  -  7.5% → 1500 us（中位）
 *  - 10.0% → 2000 us（正向最大）
 *
 * 典型使用方式（伪代码）：
 *
 *    // 1. 初始化 libpwm_host 和 pwm_control
 *    pwm_host_init(&host_cfg);
 *
 *    pwm_ctrl_config_t ctrl_cfg = {0};
 *    ctrl_cfg.ctrl_hz         = 100.0f;   // 上层循环 100 Hz 调用 pwm_ctrl_step()
 *    ctrl_cfg.max_step_pct    = 0.2f;     // 每步最多 0.2% 的占空比变化
 *    ctrl_cfg.group_mode      = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;
 *    ctrl_cfg.groupA_mask     = PWM_CH_MASK_1_4;   // 逻辑电机 1-4
 *    ctrl_cfg.groupB_mask     = PWM_CH_MASK_5_8;   // 逻辑电机 5-8
 *
 *    int motor_map[8] = {5,6,7,8,1,2,3,4};        // 逻辑 → 物理 PWM 通道映射
 *    memcpy(ctrl_cfg.motorch_to_pwmch, motor_map, sizeof(motor_map));
 *
 *    pwm_ctrl_init(&ctrl_cfg);
 *
 *    // 2. 设置目标占空比：例如把逻辑 1 号电机从中位拉到 9.0%
 *    pwm_ctrl_set_target_pct(PWM_CH1, 9.0f);
 *
 *    // 3. 主循环（100 Hz）
 *    while (running) {
 *        pwm_host_poll(0);      // 可选：接收心跳 ACK 等
 *        pwm_ctrl_step();       // 限斜率 + 分组 + 映射 + 反向 + 下发
 *        usleep(10000);         // 10 ms ~ 100 Hz
 *    }
 *
 *    // 4. 退出前安全归中位
 *    pwm_ctrl_emergency_stop(1.0f);   // 1 秒平滑回到 7.5%
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
 *    真正的物理 PWM 通道由配置中的 motorch_to_pwmch[] 映射决定。
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
#define PWM_CH_MASK_ALL      ((pwm_channel_mask_t)0xFFu)  /**< CH1-8 全部   */
#define PWM_CH_MASK_1_4      ((pwm_channel_mask_t)0x0Fu)  /**< CH1-4        */
#define PWM_CH_MASK_5_8      ((pwm_channel_mask_t)0xF0u)  /**< CH5-8        */

/* ====================================================================== */
/*                         分组更新模式                                   */
/* ====================================================================== */

/**
 * @brief 分组更新模式
 *
 * 分组目的：
 *   - 避免“一口气更新 8 路电机”带来的瞬时电流冲击；
 *   - 工程上常用：先动一半，再动另一半；或按对角线/对称分组。
 */
typedef enum {
    /**
     * @brief 不分组：每次 pwm_ctrl_step() 更新所有逻辑通道
     */
    PWM_CTRL_GROUP_MODE_ALL = 0,

    /**
     * @brief A/B 分组交替更新
     *
     * - 使用 config.groupA_mask / groupB_mask 指定各组逻辑通道；
     * - 第一次 step() 更新组 A，第二次更新组 B，如此往复。
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

#define PWM_CTRL_OK                0   /**< 成功                       */
#define PWM_CTRL_ERR_NOT_INIT     -1   /**< 未调用 pwm_ctrl_init      */
#define PWM_CTRL_ERR_INVALID_ARG  -2   /**< 参数非法（通道号/指针等） */
#define PWM_CTRL_ERR_INTERNAL     -3   /**< 内部错误（如 libpwm_host）*/

/* ====================================================================== */
/*                         配置结构体                                     */
/* ====================================================================== */

/**
 * @brief pwm_control 初始化配置
 *
 * 所有字段若为 0/NULL，将使用“安全的工程默认值”（见实现）。
 */
typedef struct {
    /**
     * @brief 控制频率（Hz）
     *
     * 上层应以该频率调用 pwm_ctrl_step()。
     * 若为 0，则使用内部默认值。
     */
    float ctrl_hz;

    /**
     * @brief 单步最大占空比变化量（%）
     *
     * 若为 0 或负数，则使用内部默认值（例如 0.2）。
     */
    float max_step_pct;

    /**
     * @brief 允许的占空比范围（%）
     *
     * 若为 0，则使用内部默认值：
     *   - min_pct: 默认约 5.0
     *   - mid_pct: 默认约 7.5
     *   - max_pct: 默认约 10.0
     */
    float min_pct;
    float mid_pct;
    float max_pct;

    /**
     * @brief 是否启用“禁止突然反向”
     *
     * 启用时：正推→反推必须经过中位（mid_pct）过渡。
     * 0 关闭，非 0 打开。
     */
    int   enable_reverse_protection;

    /**
     * @brief AB 分组掩码及模式
     */
    pwm_channel_mask_t   groupA_mask;
    pwm_channel_mask_t   groupB_mask;
    pwm_ctrl_group_mode_t group_mode;

    /**
     * @brief 逻辑电机 → 物理 PWM 通道映射表（1-based）
     *
     * motorch_to_pwmch[i] 表示“逻辑 (i+1) 号电机接在第几路物理 PWM 通道上”。
     * 若为 0，则使用内部默认映射。
     */
    int motorch_to_pwmch[PWM_HOST_CH_NUM];

    /**
     * @brief 每个逻辑电机是否反向（0: 正常, 非 0: 反向）
     *
     * 反向的实现为占空比关于 (min_pct + max_pct)/2 对称。
     */
    uint8_t motor_reverse[PWM_HOST_CH_NUM];

} pwm_ctrl_config_t;

/* ====================================================================== */
/*                         状态结构体                                     */
/* ====================================================================== */

/**
 * @brief 控制层状态快照（调试 / 记录用）
 *
 * 说明：
 *   - current_pct[]：
 *       “逻辑电机空间”的当前占空比（单位：%），
 *       已经过限幅/限斜率/分组调度，是 **安全层当前持有的实际下发值**；
 *   - target_pct[]：
 *       上层最近设置的目标占空比（单位：%），
 *       尚未考虑限斜率和分组调度。
 */
typedef struct {
    float    current_pct[PWM_HOST_CH_NUM];  /**< 当前实际占空比（逻辑视角，已限斜率/分组） */
    float    target_pct [PWM_HOST_CH_NUM];  /**< 目标占空比（逻辑视角）                     */
    uint64_t step_count;                    /**< 已执行的 pwm_ctrl_step() 次数               */
} pwm_ctrl_state_t;

/* ====================================================================== */
/*                         DOF 级别命令（4-DOF）                          */
/* ====================================================================== */

/**
 * @brief 4-DOF 归一化命令（[-1,1]），用于“控制层 → PWM 控制层”的统一入口
 *
 * 语义需与控制栈 / Teleop 保持一致：
 *   - surge : X 轴 前进(+) / 后退(-)
 *   - sway  : Y 轴 右移(+) / 左移(-)
 *   - heave : Z 轴 上升(+) / 下潜(-)
 *   - yaw   : 航向右转(+) / 左转(-)
 *
 * 建议：
 *   - 控制器（Manual / PID / MPC）只输出该结构；
 *   - 具体“DOF → 8 电机”的混控矩阵和占空比范围，封装在 pwm_control 内部。
 */
typedef struct {
    float surge;   /**< X 轴前后，[-1,1] */
    float sway;    /**< Y 轴左右，[-1,1] */
    float heave;   /**< Z 轴上下，[-1,1] */
    float yaw;     /**< 航向右转(+)/左转(-)，[-1,1] */
} pwm_ctrl_dof4_cmd_t;

/* ====================================================================== */
/*                         基础生命周期                                   */
/* ====================================================================== */

/**
 * @brief 初始化 PWM 控制层
 *
 * @param cfg 若为 NULL，则使用全默认配置。
 *
 * @return PWM_CTRL_OK 或负错误码。
 */
int  pwm_ctrl_init(const pwm_ctrl_config_t* cfg);

/**
 * @brief 反初始化 PWM 控制层
 *
 * 将清理内部状态，使得后续调用除 init 外的接口返回错误。
 */
void pwm_ctrl_deinit(void);

/**
 * @brief 读取控制层状态快照
 *
 * @param out_state 若非 NULL，则填充为当前状态。
 *
 * 说明：
 *   - out_state->current_pct[] 即为“安全层当前持有的实际占空比”
 *    （逻辑电机空间，单位：%），适合作为 **日志记录 / 离线分析** 的信号；
 *   - out_state->target_pct[] 为上层近期设置的目标占空比。
 */
void pwm_ctrl_get_state(pwm_ctrl_state_t* out_state);

/* ====================================================================== */
/*                         目标设置接口                                   */
/* ====================================================================== */

/**
 * @brief 设置单个逻辑通道的目标占空比（单位：%）
 *
 * @param ch  逻辑通道（PWM_CH1..PWM_CH8）
 * @param pct 目标占空比（％）
 */
int pwm_ctrl_set_target_pct(int ch, float pct);

/**
 * @brief 使用掩码批量设置多个通道的目标占空比（单位：%）
 *
 * @param mask   逻辑通道掩码
 * @param pct    长度为 PWM_HOST_CH_NUM 的数组（按逻辑电机索引）
 */
int pwm_ctrl_set_targets_mask(pwm_channel_mask_t mask,
                              const float pct[PWM_HOST_CH_NUM]);

/**
 * @brief 将所有逻辑电机目标占空比重置为中位（mid_pct）
 */
int pwm_ctrl_set_all_target_mid(void);

/**
 * @brief 使用 4-DOF 归一化命令设置逻辑电机的目标占空比
 *
 * @param cmd  4-DOF 命令（若为 NULL 则返回 PWM_CTRL_ERR_INVALID_ARG）
 *
 * 行为：
 *   - 内部包含一套 DOF→8 电机 的 thrust allocation；
 *   - 按 cmd->surge/sway/heave/yaw 计算每个逻辑电机的目标占空比（单位：%），
 *     并写入 target_pct[]；
 *   - 不会立即下发，仍由后续 pwm_ctrl_step() 做限斜率 / 映射 / 分组。
 */
int pwm_ctrl_set_targets_from_dof4(const pwm_ctrl_dof4_cmd_t* cmd);

/* ====================================================================== */
/*                         调度与下发（核心）                             */
/* ====================================================================== */

/**
 * @brief 执行一轮“限斜率 + 分组调度 + 通道映射 + 方向反转 + 下发”
 *
 * 典型调用频率：ctrl_hz（如 50/100 Hz）。
 *
 * 返回值：
 *   - PWM_CTRL_OK            ：成功；
 *   - PWM_CTRL_ERR_NOT_INIT  ：尚未初始化；
 *   - PWM_CTRL_ERR_INTERNAL  ：内部错误（如底层发送失败）。
 */
int pwm_ctrl_step(void);

/* ====================================================================== */
/*                         工程便利函数                                   */
/* ====================================================================== */

/**
 * @brief 在给定时间内保持单通道占空比，并持续调用 step()
 *
 * @param ch       逻辑通道
 * @param pct      目标占空比（％）
 * @param seconds  持续时间（秒）
 */
int pwm_ctrl_hold_pct_blocking(int ch, float pct, float seconds);

/**
 * @brief 紧急停止：在给定时间内平滑回到中位
 *
 * @param seconds  平滑归中位的时间（秒）
 */
int pwm_ctrl_emergency_stop(float seconds);

/* ====================================================================== */
/*                         电机反向运行时开关                             */
/* ====================================================================== */

/**
 * @brief 设置某一逻辑电机是否反向
 *
 * @param motor_id 逻辑电机编号（1..PWM_HOST_CH_NUM）
 * @param enable   0: 关闭反向；非 0: 开启反向
 */
int pwm_ctrl_set_motor_reverse(int motor_id, int enable);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PWM_CONTROL_H */
