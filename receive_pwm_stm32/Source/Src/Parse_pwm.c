#include "Parse_pwm.h"
#include <string.h>
#include <stdint.h>
#include "Driver_pwm.h"
#include "Uart_service.h"
#include "usart.h" // huart5

/* ====================== 协议与常量定义（旧协议版，保留向下兼容） ====================== */
/*
 * 旧版 PWM 帧（UART 字节流）：
 *   [0]   0xAA
 *   [1]   0x55                       // 帧头
 *   [2]   FRAME_ID = 0x01            // 消息 ID
 *   [3]   PAYLOAD_LEN = 0x10         // 8×u16 = 16B
 *   [4..19]  payload: 8 × u16 (BE)   // 8 路 PWM（0..10000）
 *   [20]  sum8 over bytes [0..19]    // 低 8 位累加和校验
 *
 * 心跳帧（旧实现）：
 *   [0] 0x55
 *   [1] 0xAA                          // 帧头（与 PWM 反序，用于区分）
 *   ... 其余内容原样透传回上位机
 */

#define PWM_FRAME_ID 0x01u
#define PWM_PAYLOAD_LEN 0x10u
#define PWM_FRAME_LEN (2u + 1u + 1u + PWM_PAYLOAD_LEN + 1u) // 21B

#define PWM_SOF0 0xAAu
#define PWM_SOF1 0x55u
#define HB_SOF0 0x55u
#define HB_SOF1 0xAAu

/* UART DMA 接收缓冲约束（与 Uart_service.h 中的定义保持一致） */
#ifndef power_board_max_len
#define power_board_max_len 256
#endif

/* ====================== 类型与静态数据 ====================== */
typedef struct
{
    uint16_t pwm_raw[8]; // 0..10000（大端→主机端后）
    float duty[8];       // -1..+1
} PwmFrame;

/* 中断与主循环之间的“到达一坨数据”的信号与缓冲 */
static volatile uint8_t uart5_it_flag = 0; // 1=有新数据
Receive_Msg_t uart5_msg;                   // 串口接收数据缓冲
static uint8_t uart5_buf[power_board_max_len];

/* ====================== 工具函数 ====================== */
static uint8_t sum8(const uint8_t *p, size_t n)
{
    uint32_t s = 0;
    for (size_t i = 0; i < n; ++i)
        s += p[i];
    return (uint8_t)(s & 0xFF);
}

static float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

static uint16_t be16_read(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

/* ====================== 帧级解析（允许粘包/半包/噪声） ====================== */
/**
 * @brief 尝试在 buf[offset..len) 处解析一帧 PWM
 * @param consumed 输出：若成功=整帧长度，若数据不够=0，若非帧头=1（跳过一个字节）
 */
static bool try_parse_pwm_at(const uint8_t *buf, size_t len, size_t offset,
                             PwmFrame *out, size_t *consumed)
{
    *consumed = 0;
    if (offset + 2 > len)
        return false;

    // 帧头不匹配：跳过一个字节继续扫
    if (buf[offset] != PWM_SOF0 || buf[offset + 1] != PWM_SOF1)
    {
        *consumed = 1;
        return false;
    }
    // 需要至少完整帧长度
    if (offset + PWM_FRAME_LEN > len)
    {
        // 数据不够，等待下次
        *consumed = 0;
        return false;
    }

    const uint8_t *p = buf + offset;
    const uint8_t id = p[2];
    const uint8_t plen = p[3];

    if (id != PWM_FRAME_ID || plen != PWM_PAYLOAD_LEN)
    {
        // 虽是帧头但不是我们支持的帧，丢弃本字节，继续向后扫描
        *consumed = 1;
        return false;
    }

    // 校验和：覆盖 [0..(4+16-1)] 共 20 字节
    const uint8_t expect = sum8(p, 2 + 1 + 1 + PWM_PAYLOAD_LEN);
    const uint8_t got = p[4 + PWM_PAYLOAD_LEN];
    if (expect != got)
    {
        // 校验失败，跳过一个字节，避免卡死
        *consumed = 1;
        return false;
    }

    // 解析 8×u16（BE）
    const uint8_t *pay = p + 4;
    for (int i = 0; i < 8; ++i)
    {
        const uint16_t v = be16_read(pay + i * 2);
        out->pwm_raw[i] = v;
        // 0..10000 → -1..+1；超界裁剪，防异常值污染
        float duty = ((float)v - 5000.0f) / 5000.0f;
        out->duty[i] = clampf(duty, -1.0f, 1.0f);
    }

    *consumed = PWM_FRAME_LEN;
    return true;
}

/**
 * @brief 尝试在 buf[offset..len) 处解析心跳帧（仅原样回环心跳帧）
 * @param consumed 输出：同上
 */
static bool try_parse_heartbeat_at(const uint8_t *buf, size_t len, size_t offset, size_t *consumed)
{
    *consumed = 0;
    if (offset + 2 > len)
        return false;

    if (buf[offset] != HB_SOF0 || buf[offset + 1] != HB_SOF1)
    {
        *consumed = 1;
        return false;
    }

    // 简单策略：把“以 0x55,0xAA 开头的一整坨”回回去。
    // 为避免把“粘在后面的 PWM”也发回，这里只回“检测到的这帧长度范围”。
    // 旧心跳没有长度字段，这里约定：若整包就是一帧心跳，DMA 空闲给的 len 就是帧长度。
    // 因此仅当“此帧正好位于缓冲开头并占满整个缓冲”时才回环。
    if (offset == 0)
    {
        // 回环当前收到的整包
        HAL_UART_Transmit_DMA(&huart5, (uint8_t *)buf, (uint16_t)len);
    }

    // 由于旧心跳无明确长度，这里按“跳过 2 字节头”，继续向后扫描。
    *consumed = 2;
    return true;
}

/* ====================== 硬件执行层 ====================== */
static void apply_pwm_frame(const PwmFrame *f)
{
    // 保持你们原有通道编号（1..8）
    Driver_pwm_SetDuty(1, f->duty[0]);
    Driver_pwm_SetDuty(2, f->duty[1]);
    Driver_pwm_SetDuty(3, f->duty[2]);
    Driver_pwm_SetDuty(4, f->duty[3]);
    Driver_pwm_SetDuty(5, f->duty[4]);
    Driver_pwm_SetDuty(6, f->duty[5]);
    Driver_pwm_SetDuty(7, f->duty[6]);
    Driver_pwm_SetDuty(8, f->duty[7]);
}

/* ====================== 串口 DMA 初始化与中断钩子 ====================== */
void Uart5_Parse_Init(void)
{
    HAL_UART_Receive_DMA(&huart5, uart5_buf, power_board_max_len);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}

/**
 * @brief 在 UART5_IRQHandler 中调用
 */
void UART5_IT_TASK(void)
{
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);

        // 关 DMA 取本次收到字节数
        HAL_UART_DMAStop(&huart5);
        uint16_t len = (uint16_t)(power_board_max_len - __HAL_DMA_GET_COUNTER(huart5.hdmarx));
        if (len > power_board_max_len)
            len = power_board_max_len;

        if (len > 0 && uart5_it_flag == 0)
        {
            uart5_msg.len = len;
            memcpy(uart5_msg.data, uart5_buf, len);
            uart5_it_flag = 1; // 通知主循环有一坨新字节
        }
        // 重新启动 DMA
        HAL_UART_Receive_DMA(&huart5, uart5_buf, power_board_max_len);
    }
}

/* ====================== 主循环处理（多帧扫描） ====================== */
void process_uart5_message(void)
{
    if (uart5_it_flag == 0)
        return;

    const uint8_t *buf = uart5_msg.data;
    size_t len = uart5_msg.len;
    size_t off = 0;

    // 一次“吃干净”这一坨字节，支持：前缀噪声、多帧、半包（保留到下次）
    while (off < len)
    {
        size_t consumed = 0;

        // 优先尝试 PWM 帧
        PwmFrame f;
        if (try_parse_pwm_at(buf, len, off, &f, &consumed))
        {
            apply_pwm_frame(&f);
            off += consumed;
            continue;
        }
        if (consumed > 0)
        {
            off += consumed;
            continue;
        }

        // 再尝试心跳帧（仅当帧头匹配时原样回环/前进）
        if (try_parse_heartbeat_at(buf, len, off, &consumed))
        {
            off += consumed;
            continue;
        }
        if (consumed > 0)
        {
            off += consumed;
            continue;
        }

        // 都不是：向前滑动一个字节，继续扫描
        off += 1;
    }

    // 清标志（允许下一坨数据进来）
    uart5_it_flag = 0;
}
