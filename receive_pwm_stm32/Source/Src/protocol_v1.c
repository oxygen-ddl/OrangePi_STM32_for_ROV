#include "protocol_v1.h"
#include "config.h"
#include "board.h"
#include "Driver_pwm.h"
#include "crc16_ccitt.h" // 需要提供 uint16_t crc16_ccitt(const uint8_t* data, uint16_t len)
#include "protocol_v1.h"
#include <string.h> // memmove
#include <stdbool.h>
#include <stddef.h>

/* ========================= 协议常量与工具 ========================= */

/* 帧头标识 */
#define SOF_B0 0xAAu
#define SOF_B1 0x55u

/* 头部固定长度（不含 SOF 的 2 字节） */
#define HEADER_REST_LEN 10u // VER(1) + MSG(1) + SEQ(2) + TICKS(4) + LEN(2)
#define SOF_LEN 2u
#define CRC_LEN 2u
#define HEADER_TOTAL_LEN (SOF_LEN + HEADER_REST_LEN) // 12

#define MIN_FRAME_LEN (HEADER_TOTAL_LEN + CRC_LEN) // 14


/* 大端读写工具 */
static inline uint16_t be16_read(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}
static inline uint32_t be32_read(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static inline void be16_write(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}
static inline void be32_write(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v & 0xFF);
}

/* ========================= 接收缓存与统计 ========================= */

#ifndef CFG_PROTO_RX_BUF_CAP
#define CFG_PROTO_RX_BUF_CAP 512u
#endif

static uint8_t s_rxbuf[CFG_PROTO_RX_BUF_CAP];
static volatile uint16_t s_rxlen = 0; // 缓冲中“当前有效字节数”

static proto_stats_t s_stats = {0};

/* 运行时控制 */
static uint32_t s_last_ok_rx_ms = 0; // 最近一次收到“合法帧”的时刻（ms）
static uint32_t s_failsafe_timeout_ms = CFG_FAILSAFE_TIMEOUT_MS;

/* ========================= 内部函数声明 ========================= */

static void process_rx_buffer(void);
static bool try_parse_one_frame(uint16_t *consumed);
static void handle_msg_pwm(const uint8_t *payload, uint16_t len);
static void handle_msg_hb(uint16_t seq, uint32_t ticks);
static void enter_failsafe_mid_all(void);

/* ========================= 对外 API ========================= */
// 初始化
void protocol_init(void)
{
    s_rxlen = 0;
    memset((void *)&s_stats, 0, sizeof(s_stats));
    s_last_ok_rx_ms = HAL_GetTick(); // 单位为ms
    s_failsafe_timeout_ms = CFG_FAILSAFE_TIMEOUT_MS;

    /* 上电暖机阶段由 main.c 控制，这里不阻塞 */
}
// 数据传入缓冲区并进行解析数据
void protocol_feed_bytes(const uint8_t *data, uint16_t n)
{
    if (protocol_flag == 0) // 首先判断协议处理标志位，避免重复进入
    {
        if (!data || n == 0)
            return;

        /* 如果新数据 + 旧数据会溢出，尽量丢弃老数据保留最新（简单稳妥） */
        if (n > sizeof(s_rxbuf))
        {
            /* 输入太大，直接清空，只保留最后的一段数据 */
            data += (n - sizeof(s_rxbuf));
            n = sizeof(s_rxbuf);
            s_rxlen = 0;
        }
        if (s_rxlen + n > sizeof(s_rxbuf))
        {
            /* 左移腾位置（简单滑窗） */
            const uint16_t over = (uint16_t)((s_rxlen + n) - sizeof(s_rxbuf));
            memmove(s_rxbuf, s_rxbuf + over, s_rxlen - over);
            s_rxlen = (uint16_t)(s_rxlen - over);
        }
        memcpy(s_rxbuf + s_rxlen, data, n);
        s_rxlen = (uint16_t)(s_rxlen + n);

        protocol_flag = 1;
        // process_rx_buffer();//解析数据，解析数据转移到protocol_process()中，在主循环进行处理
    }
}

void protocol_poll(void)
{
    /* 失联保护：超时则回中位 */
    const uint32_t now = HAL_GetTick();
    if ((now - s_last_ok_rx_ms) > s_failsafe_timeout_ms)
    {
        enter_failsafe_mid_all();
        /* 防止重复刷 log，可选择这里刷新时戳 */
        s_last_ok_rx_ms = now;
    }
}

void protocol_set_failsafe_timeout_ms(uint32_t ms)
{
    if (ms < 50u)
        ms = 50u; /* 安全下限 */
    s_failsafe_timeout_ms = ms;
}

const proto_stats_t *protocol_stats(void)
{
    return &s_stats;
}

/* ========================= 内部实现 ========================= */

static void process_rx_buffer(void)
{
    /* 尝试从缓冲中解析尽可能多的完整帧 */
    for (;;)
    {
        uint16_t consumed = 0;
        if (!try_parse_one_frame(&consumed))
        {
            break; /* 需要更多数据或缓冲为空 */
        }
        /* 丢弃已消费的字节 */
        if (consumed > 0)
        {
            const uint16_t remain = (uint16_t)(s_rxlen - consumed);
            if (remain > 0)
            {
                memmove(s_rxbuf, s_rxbuf + consumed, remain);
            }
            s_rxlen = remain;
        }
    }
}

/**
 * @brief 试图从 s_rxbuf 解析出一帧
 * @param consumed 若成功/失败但移动了指针，返回本次应丢弃的字节数；若无事可做，返回 0
 * @return true=本次成功解析并处理了一帧（调用方应继续尝试）；false=本次无帧可解（等待更多数据）
 */
static bool try_parse_one_frame(uint16_t *consumed)
{
    *consumed = 0;
    if (s_rxlen < 1)
        return false;

    /* 寻找 SOF（简洁做法：线性扫描到第一个 0xAA 0x55） */
    uint16_t pos = 0;
    while (pos + 1 < s_rxlen)
    {
        if (s_rxbuf[pos] == SOF_B0 && s_rxbuf[pos + 1] == SOF_B1)
        {
            break;
        }
        ++pos;
    }
    if (pos > 0)
    {
        /* 丢弃垃圾字节 */
        *consumed = pos;
        return true;
    }

    /* pos==0：当前缓冲开头就是 SOF */
    if (s_rxlen < MIN_FRAME_LEN)
    {
        return false; /* 头都不够，等待更多数据 */
    }

    const uint8_t *p = s_rxbuf;

    /* 读取固定头：VER/MSG/SEQ/TICKS/LEN（不含 SOF） */
    const uint8_t ver = p[2];
    const uint8_t msg = p[3];
    const uint16_t seq = be16_read(p + 4);
    const uint32_t ticks = be32_read(p + 6);
    const uint16_t len = be16_read(p + 10);

    /* 版本校验 */
    if (ver != PROTO_VER_1)
    {
        /* 不支持的版本：丢弃一个字节（避免卡死），计数为 unsupported */
        s_stats.rx_unsupported++;
        *consumed = 1;
        return true;
    }

    /* 计算整帧长度与可用性 */
    const uint32_t frame_len = (uint32_t)HEADER_TOTAL_LEN + (uint32_t)len + (uint32_t)CRC_LEN;
    if (frame_len > sizeof(s_rxbuf))
    {
        /* 明显异常长度：丢掉一个字节继续 */
        s_stats.rx_len_err++;
        *consumed = 1;
        return true;
    }
    if (s_rxlen < frame_len)
    {
        /* 数据不完整，继续等 */
        return false;
    }

    /* 校验 CRC：覆盖 VER..LEN(+PAYLOAD)=10+len 字节 */
    const uint8_t *crc_base = p + 2; // 从 VER 开始
    const uint16_t crc_calc = crc16_ccitt(crc_base, (uint16_t)(10u + len));
    const uint16_t crc_rx = be16_read(p + 12 + len);
    if (crc_calc != crc_rx)
    {
        s_stats.rx_crc_err++;
        /* 丢弃一个字节，继续搜 SOF（更鲁棒） */
        *consumed = 1;
        return true;
    }

    /* 到这里是一帧完整合法帧 */
    const uint8_t *payload = p + HEADER_TOTAL_LEN;

    switch (msg)
    {
    case MSG_PWM:
        /* 只有 PWM 与 HB 收到后才更新“链路活跃”时间（防止噪声误刷新） */
        s_last_ok_rx_ms = HAL_GetTick();
        handle_msg_pwm(payload, len);
        s_stats.rx_ok++;
        break;

    case MSG_HB:
        s_last_ok_rx_ms = HAL_GetTick();
        handle_msg_hb(seq, ticks);
        s_stats.rx_ok++;
        break;

    case MSG_HB_ACK:
        /* 正常情况下主机不会给我们发 ACK，这里标记为 unsupported 但不算错 */
        s_stats.rx_unsupported++;
        break;

    default:
        s_stats.rx_unsupported++;
        break;
    }

    *consumed = (uint16_t)frame_len;
    return true;
}

/* ========== 业务处理：PWM ==========
 * 期望 LEN=16，内容为 8×uint16（大端），0..10000 对应占空 -1..1（5000->0）。
 */
static void handle_msg_pwm(const uint8_t *payload, uint16_t len)
{
    if (len != 16u)
    {
        s_stats.rx_len_err++;
        return;
    }

    float duty[8];
    for (int i = 0; i < 8; ++i)
    {
        const uint16_t v = be16_read(payload + i * 2);
        /* 裁剪到 0..10000 */
        uint16_t vv = v;
        if (vv > 10000u)
            vv = 10000u;
        /* 线性映射到 -1..1（5000 -> 0） */
        duty[i] = ((float)((int32_t)vv - 5000) / 5000.0f);
        if (duty[i] > 1.0f)
            duty[i] = 1.0f;
        if (duty[i] < -1.0f)
            duty[i] = -1.0f;
    }

    /* 下发到 8 路 PWM 输出（驱动层内部可做斜率限幅/死区/μs↔CCR 等） */
    Driver_pwm_SetDuty(1, duty[0]);
    Driver_pwm_SetDuty(2, duty[1]);
    Driver_pwm_SetDuty(3, duty[2]);
    Driver_pwm_SetDuty(4, duty[3]);
    Driver_pwm_SetDuty(5, duty[4]);
    Driver_pwm_SetDuty(6, duty[5]);
    Driver_pwm_SetDuty(7, duty[6]);
    Driver_pwm_SetDuty(8, duty[7]);
}

/* ========== 业务处理：HB（立即回 ACK） ==========
 * 我们回一帧：SOF AA55 / VER 01 / MSG 11 / SEQ=原样 / TICKS=本地HAL_GetTick() / LEN=0000 / CRC(VER..LEN)
 */
static void handle_msg_hb(uint16_t seq, uint32_t ticks)
{
#if (CFG_HB_ACK_ENABLE)
    uint8_t buf[MIN_FRAME_LEN]; // 14 bytes
    uint8_t *p = buf;

    /* SOF */
    *p++ = SOF_B0;
    *p++ = SOF_B1;

    /* VER / MSG */
    *p++ = PROTO_VER_1;
    *p++ = MSG_HB_ACK;

    /* SEQ / TICKS / LEN=0 */
    be16_write(p, seq);
    p += 2;
    be32_write(p, HAL_GetTick());
    p += 4;
    be16_write(p, 0);
    p += 2;

    /* CRC 覆盖 VER..LEN (10 字节) */
    const uint16_t crc = crc16_ccitt(buf + 2, 10u);
    be16_write(p, crc);
    p += 2;

    /* 通过协议串口发送（DMA 或阻塞均可；这里用阻塞更简单稳妥） */
    HAL_UART_Transmit(&UART_PROTO_HANDLE, buf, (uint16_t)sizeof(buf), 50);
#else
    (void)seq;
#endif
}

/* 将所有通道回中位（失联/急停） */
static void enter_failsafe_mid_all(void)
{
    /* 回中，即控制推进器0输出*/
    Driver_pwm_SetDuty(1, 0.0f);
    Driver_pwm_SetDuty(2, 0.0f);
    Driver_pwm_SetDuty(3, 0.0f);
    Driver_pwm_SetDuty(4, 0.0f);
    Driver_pwm_SetDuty(5, 0.0f);
    Driver_pwm_SetDuty(6, 0.0f);
    Driver_pwm_SetDuty(7, 0.0f);
    Driver_pwm_SetDuty(8, 0.0f);
}
void protocol_force_failsafe(void)
{
    /* 回中，即控制推进器0输出*/
    Driver_pwm_SetDuty(1, 0.0f);
    Driver_pwm_SetDuty(2, 0.0f);
    Driver_pwm_SetDuty(3, 0.0f);
    Driver_pwm_SetDuty(4, 0.0f);
    Driver_pwm_SetDuty(5, 0.0f);
    Driver_pwm_SetDuty(6, 0.0f);
    Driver_pwm_SetDuty(7, 0.0f);
    Driver_pwm_SetDuty(8, 0.0f);
}

void protocol_reset_stats(void)
{
    memset((void *)&s_stats, 0, sizeof(s_stats));
}
uint8_t protocol_buf[PROTOCOL_MSG_LEN];
uint8_t protocol_flag = 0;                         // 协议处理标志


void protocol_process_init(void)
{
    protocol_init();
    protocol_flag = 0;

    HAL_UART_Receive_DMA(&huart5, protocol_buf, PROTOCOL_MSG_LEN);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);//串口空闲中断使能
}

/**
 * @brief 协议处理
 * @attention 在主循环中调用
 */
void protocol_process(void)
{
    if (protocol_flag == 1)//
    {
        process_rx_buffer();

        protocol_flag = 0;//数据处理结束
    }
}
/**
 * @brief 在 UART5_IRQHandler 中调用
 */
void protocol_it_process(void)
{
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);

        // 关 DMA 取本次收到字节数
        HAL_UART_DMAStop(&huart5);
        uint16_t len = (uint16_t)(PROTOCOL_MSG_LEN - __HAL_DMA_GET_COUNTER(huart5.hdmarx));
        if (len > 0 && protocol_flag == 0)
        {
            protocol_feed_bytes(protocol_buf, len);
            protocol_flag = 1; // 通知主循环有一坨新字节
        }
        HAL_UART_Receive_DMA(&huart5, protocol_buf, PROTOCOL_MSG_LEN);
    }
}

