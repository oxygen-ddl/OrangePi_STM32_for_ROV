/**
 * @file board.h
 * @brief 硬件映射集中配置（定时器/通道/串口/时基等）
 *
 * 把“与具体板卡/引脚/外设实例相关”的定义集中在这里：
 * - PWM 定时器/通道映射（8 通道）
 * - 串口句柄（命令口/调试口）
 * - PWM 脉宽取值（μs）与时基刻度约定（TICK_PER_US）
 * 修改此文件即可完成移植，不应影响上层业务逻辑。
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ========================= 外部句柄声明 ========================= */
/* 这些句柄由 CubeMX 生成的 usart.c/tim.c 定义，这里仅引用 */
extern UART_HandleTypeDef huart1;   /* 调试串口（printf） */
extern UART_HandleTypeDef huart5;   /* 协议收发串口（经下级桥接为网口字节流） */

extern TIM_HandleTypeDef  htim1;    /* PWM 定时器 A（CH1~CH4） */
extern TIM_HandleTypeDef  htim4;    /* PWM 定时器 B（CH5~CH8） */

/* ========================= 串口角色定义 ========================= */
#define UART_DBG_HANDLE       (huart1)   /* printf/日志 */
#define UART_PROTO_HANDLE     (huart5)   /* 协议字节流（DMA + 空闲中断） */

/* ========================= PWM 脉宽约束（单位：μs） ========================= */
/* 与电调一致：1000（最小/反推），1500（中位/不推），2000（最大/正推） */
#define PWM_MIN_US            1000u
#define PWM_MID_US            1500u
#define PWM_MAX_US            2000u

/* 若使用 0..10000 表示量 → μs 的线性映射（供驱动层使用） */
#define PWM_SCALE_DEN         10000u

/* ========================= 定时器时基刻度约定 ========================= */
/**
 * @note 推荐将定时器配置为“1 tick = 1 μs”（典型做法：定时器输入时钟 84MHz，
 *       预分频 PSC=83 → 1MHz 时基；ARR ≥ 20000 以覆盖 20ms 周期）。
 *       若你的时基不是 1 μs，请把 TICK_PER_US 改为实际值，并在 Driver_pwm.c 做换算。
 */
#define TICK_PER_US           1u

/* ========================= 8 路 PWM 通道映射表 ========================= */
/* 你当前工程里使用 TIM1 CH1~CH4 + TIM4 CH1~CH4 输出 8 路 PWM */
#define PWM_CH1_TIM           (htim1)
#define PWM_CH1_CH            (TIM_CHANNEL_1)

#define PWM_CH2_TIM           (htim1)
#define PWM_CH2_CH            (TIM_CHANNEL_2)

#define PWM_CH3_TIM           (htim1)
#define PWM_CH3_CH            (TIM_CHANNEL_3)

#define PWM_CH4_TIM           (htim1)
#define PWM_CH4_CH            (TIM_CHANNEL_4)

#define PWM_CH5_TIM           (htim4)
#define PWM_CH5_CH            (TIM_CHANNEL_1)

#define PWM_CH6_TIM           (htim4)
#define PWM_CH6_CH            (TIM_CHANNEL_2)

#define PWM_CH7_TIM           (htim4)
#define PWM_CH7_CH            (TIM_CHANNEL_3)

#define PWM_CH8_TIM           (htim4)
#define PWM_CH8_CH            (TIM_CHANNEL_4)

/* ========================= 便捷宏（可在驱动层使用） ========================= */
/* 将“μs”写入 CCR（假设 1tick=1μs；若非 1μs，请在 Driver 中做换算） */
#define PWM_SET_US(htim, ch, us)   __HAL_TIM_SET_COMPARE(&(htim), (ch), (uint32_t)((us) / TICK_PER_US))

/* ========================= 编译期健壮性检查 ========================= */
#if (PWM_MIN_US >= PWM_MID_US) || (PWM_MID_US >= PWM_MAX_US)
#  error "PWM_MIN_US / PWM_MID_US / PWM_MAX_US 配置不合法（应满足 MIN < MID < MAX）"
#endif

#if (TICK_PER_US == 0u)
#  error "TICK_PER_US 不能为 0，请根据定时器 PSC/ARR 配置正确的时基刻度。"
#endif

#ifdef __cplusplus
}
#endif
