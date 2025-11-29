#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif

/* ========================= 协议版本与常量 ========================= */
/**
 * 端序：大端（big-endian）
 * 帧格式（总述）：
 *   SOF(2) = 0xAA 0x55
 *   VER(1) = 0x01
 *   MSG(1)
 *   SEQ(2)      big-endian
 *   TICKS(4)    big-endian, ms tick（主机/设备各自时基）
 *   LEN(2)      big-endian, payload 字节数
 *   PAYLOAD(n)
 *   CRC(2)      big-endian, CRC16-CCITT-FALSE 覆盖 VER..LEN(+PAYLOAD)
 */
#define PROTO_VER_1 0x01
#define PROTO_SOF_BE 0xAA55u

/* 固定长度（便于解析/调试） */
#define PROTO_SOF_LEN 2u
#define PROTO_HEAD_REST_LEN 10u                             /* VER(1)+MSG(1)+SEQ(2)+TICKS(4)+LEN(2) */
#define PROTO_HDR_LEN (PROTO_SOF_LEN + PROTO_HEAD_REST_LEN) /* 12 */
#define PROTO_CRC_LEN 2u
#define PROTO_MIN_FRAME_LEN (PROTO_HDR_LEN + PROTO_CRC_LEN) /* 14 */

/* ========================= 消息 ID ========================= */
/* 已实现 */
#define MSG_PWM 0x01    /* 主机→设备：8×u16(0..10000)，LEN=16 */
#define MSG_HB 0x10     /* 主机→设备：心跳，LEN=0 */
#define MSG_HB_ACK 0x11 /* 设备→主机：心跳应答，LEN=0 */

/* 预留扩展（建议后续实现） */
#define MSG_ESTOP 0x20  /* 主机→设备：软急停，LEN=0 */
#define MSG_STATUS 0x40 /* 设备→主机：状态上报（频率低），LEN=可变 */

    /* ========================= 语义类型（便于协作） ========================= */
    typedef uint16_t proto_seq_t;      /* 帧序号（回绕） */
    typedef uint32_t proto_ticks_ms_t; /* 毫秒时基（各端自洽，不做跨端比较） */

    /* ========================= 对外 API ========================= */
    /**
     * @brief 协议栈初始化：清空缓冲/统计，刷新 last_ok_rx
     */
    void protocol_init(void);

    /**
     * @brief 喂入原始字节流（DMA+空闲中断回调里调用）
     * @param data 新到数据指针
     * @param n    新到字节数
     */
    void protocol_feed_bytes(const uint8_t *data, uint16_t n);

    /**
     * @brief 轮询钩子：做失联判定与保护（建议 1~5ms 周期调用）
     *        - 超过 failsafe 超时时间未收到“合法帧”（PWM/HB/HB_ACK），则回中位
     */
    void protocol_poll(void);

    /**
     * @brief 运行时调整失联超时（ms）。实现可做下限保护（如 <50ms 则钳制）。
     */
    void protocol_set_failsafe_timeout_ms(uint32_t ms);

    /* ==== 可选的安全/调试辅助（不调用也不影响现有逻辑） ==== */

    /**
     * @brief 立即进入保护状态（全部中位），可用于上电自检/外部故障联动。
     *
     */
    void protocol_force_failsafe(void);

    /**
     * @brief 清空统计计数器（调试/自检时使用）。未实现可留空。
     */
    void protocol_reset_stats(void);

    /* ========================= 统计信息（可选扩展） ========================= */
    typedef struct
    {
        uint32_t rx_ok;          /* 成功解析并处理的帧数 */
        uint32_t rx_crc_err;     /* CRC 校验失败次数 */
        uint32_t rx_len_err;     /* 长度/结构异常次数 */
        uint32_t rx_unsupported; /* 不支持的版本/消息 */
        /* 可选扩展字段（实现里可不维护，默认为 0） */
        uint32_t bytes_rx;    /* 接收的原始字节计数 */
        proto_seq_t last_seq; /* 最近一次合法帧的 seq */
    } proto_stats_t;

    /**
     * @return 只读指针；若未实现扩展字段则返回 0 值
     */
    const proto_stats_t *protocol_stats(void);

    /****************************第二版解析数据协议********************************* */

    #define PROTOCOL_MSG_LEN 128

    void protocol_process_init(void);
    void protocol_process(void);    // 协议处理，在主循环中调用s
    void protocol_it_process(void); // 在 UART5_IRQHandler 中调用
    extern uint8_t protocol_flag;

#ifdef __cplusplus 
}
#endif
