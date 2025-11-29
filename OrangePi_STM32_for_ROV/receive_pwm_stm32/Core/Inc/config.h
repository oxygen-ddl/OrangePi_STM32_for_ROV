/**
 * @file config.h
 * @brief 运行参数集中配置（与硬件无关的“策略/阈值/开关”）
 *
 * 放置所有可调的超时、频率、保护阈值、缓冲大小等配置。
 * - 修改这些值不应影响底层驱动或引脚映射（那些在 board.h）。
 * - 建议只在此处改数值，其他代码使用这些宏，便于统一管理与回溯。
 */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 基本版本信息（可上报） ========================= */
#define FW_VERSION_MAJOR        1
#define FW_VERSION_MINOR        0
#define FW_VERSION_PATCH        0
#define FW_VERSION_U16          ((FW_VERSION_MAJOR << 8) | (FW_VERSION_MINOR))  /* 0x0100 */

/* ========================= 协议/心跳/失联保护 ========================= */
/* 心跳 ACK 功能开关（STM32 收到 HB 后是否回 ACK） */
#define CFG_HB_ACK_ENABLE               1

/* 失联保护：超过该时间未收到“合法帧”（PWM/HB/HB_ACK）→ 全部回中位（ms） */
#define CFG_FAILSAFE_TIMEOUT_MS         300u

/* 软急停：收到 ESTOP 命令后，中位并锁定该时长禁止输出（ms） */
#define CFG_ESTOP_LOCK_MS               500u

/* 上电暖机：上电/复位后固定中位输出的时长（ms），用于电调自检与安全等待 */
#define CFG_STARTUP_WARMUP_MS           3000u

/* 状态上报频率（Hz）：建议 1~5Hz；0 表示不主动上报（仅应答型） */
#define CFG_STATUS_FEEDBACK_HZ          2u

/* ========================= PWM 输出保护与整形 ========================= */
/* 斜率限幅（μs/秒）：限制输出变化速度，保护电调/推进器。典型 1000~3000 */
#define CFG_PWM_SLEW_US_PER_S           1500u

/* 死区宽度（μs）：抑制小抖动，典型 20~50（对应 ~1~2.5% 占空） */
#define CFG_PWM_DEADBAND_US             30u

/* ========================= 缓冲大小与接收栈 ========================= */
/* 协议接收滑窗/环形缓冲容量（字节） */
#define CFG_PROTO_RX_BUF_CAP            512u

/* UART5 DMA 接收环形缓冲大小（应 ≥ 最大一帧长度 + 抖动余量） */
#define CFG_UART5_RX_DMA_BUF_SIZE       512u

/* ========================= 看门狗与调试开关 ========================= */
/* 独立看门狗使能（由 main.c 或系统初始化处拉起并喂狗） */
#define CFG_IWDG_ENABLE                 0

/* 打印/调试开关（0=关闭，1=最小日志，2=详细日志） */
#define CFG_LOG_VERBOSITY               1

/* ========================= 编译期健壮性检查 ========================= */
#if (CFG_STATUS_FEEDBACK_HZ > 10u)
#  error "CFG_STATUS_FEEDBACK_HZ 建议 <= 10Hz"
#endif

#if (CFG_FAILSAFE_TIMEOUT_MS < 100u)
#  error "CFG_FAILSAFE_TIMEOUT_MS 太小，建议 >= 100ms"
#endif

#if (CFG_PWM_SLEW_US_PER_S > 10000u)
#  error "CFG_PWM_SLEW_US_PER_S 过大，建议 <= 10000 us/s"
#endif

/* 便捷换算宏 */
#define CFG_MS_PER_TICK(ms)             (ms)
#define CFG_STATUS_PERIOD_MS            ((CFG_STATUS_FEEDBACK_HZ) ? (1000u / (CFG_STATUS_FEEDBACK_HZ)) : 0u)

#ifdef __cplusplus
}
#endif
