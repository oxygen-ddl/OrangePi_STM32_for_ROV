#include "libpwm_host.h"

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <errno.h>

/* ============================ 内部常量/宏 ============================ */

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

/* protocol_v1 信息（与 STM32 侧一致） */
enum {
    MSG_PWM    = PWM_HOST_MSG_PWM,
    MSG_HB     = PWM_HOST_MSG_HB,      /* 0x10 心跳 */
    MSG_HB_ACK = PWM_HOST_MSG_HB_ACK   /* 0x11 心跳应答 */
};

/* 固定头长度（不含 SOF 两字节，不含 CRC）:
 *  VER(1)+MSG(1)+SEQ(2)+TICKS(4)+LEN(2) = 10
 */
#define V1_FIXED_HEADER_LEN 10

/* 整帧头长度（含 SOF 两字节，不含 CRC） */
#define V1_HEADER_TOTAL_LEN (2 + V1_FIXED_HEADER_LEN)

/* CRC 长度 */
#define V1_CRC_LEN 2

/* 最大负载（PWM payload = 16 字节（8×u16）） */
#define V1_MAX_PAYLOAD 16
#define V1_MAX_FRAME   (V1_HEADER_TOTAL_LEN + V1_MAX_PAYLOAD + V1_CRC_LEN)

/* 接收缓存（足够放下完整帧） */
#define RX_BUF_SIZE 256

/* ============================ 内部状态 ============================ */

static int                s_sock   = -1;
static struct sockaddr_in s_addr;
static uint16_t           s_seq    = 0;
static uint16_t           s_shadow[PWM_HOST_CH_NUM];  /* 当前影子值（0..10000） */
static int                s_send_hz       = 50;
static int                s_nonblock_send = 0;

/* 统计与 RTT */
static pwm_host_stats_t   s_stats = {0};
static double             s_last_rtt_ms = -1.0;

/* 记录最近一次发送心跳时的 seq→ticks（用于 RTT 计算） */
static uint16_t           s_last_hb_seq        = 0;
static uint32_t           s_last_hb_send_ticks = 0;

/* ============================ 工具函数 ============================ */

static inline uint16_t be16(uint16_t v) { return htons(v); }
static inline uint32_t be32(uint32_t v) { return htonl(v); }

static uint32_t ticks_ms(void)
{
#if defined(CLOCK_MONOTONIC)
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
#else
    /* 退化实现：精度差一些，但足够做 RTT 粗略估计 */
    return (uint32_t)time(NULL) * 1000u;
#endif
}

/* 睡眠：处理 EINTR，保证尽量睡满 */
static void sleep_ms(double ms)
{
    if (ms <= 0.0) return;

    struct timespec req, rem;
    req.tv_sec  = (time_t)(ms / 1000.0);
    req.tv_nsec = (long)((ms - (double)req.tv_sec * 1000.0) * 1e6);
    if (req.tv_nsec < 0) req.tv_nsec = 0;

    while (nanosleep(&req, &rem) != 0 && errno == EINTR) {
        req = rem;
    }
}

/* CRC16-CCITT(False): poly=0x1021, init=0xFFFF, no-reflect, xorout=0x0000 */
static uint16_t crc16_ccitt_false(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* 以 protocol_v1 组帧（VER..LEN+PAYLOAD 做 CRC；SOF=0xAA 0x55） */
static pwmh_result_t v1_pack(uint8_t msg_id,
                             const uint8_t* payload, uint16_t payload_len,
                             uint8_t* out, uint16_t out_cap, uint16_t* out_len)
{
    if (!out || !out_len) return PWMH_EINVAL;
    const uint16_t need = (uint16_t)(V1_HEADER_TOTAL_LEN + payload_len + V1_CRC_LEN);
    if (out_cap < need) return PWMH_EINVAL;

    uint8_t* p = out;

    /* SOF */
    *p++ = (uint8_t)(PWM_HOST_SOF_BE >> 8);
    *p++ = (uint8_t)(PWM_HOST_SOF_BE & 0xFF);

    /* VER..LEN */
    *p++ = (uint8_t)PWM_HOST_PROTO_VER;
    *p++ = (uint8_t)msg_id;

    uint16_t seq_be = be16(++s_seq);
    memcpy(p, &seq_be, 2); p += 2;

    uint32_t t_be = be32(ticks_ms());
    memcpy(p, &t_be, 4); p += 4;

    uint16_t len_be = be16(payload_len);
    memcpy(p, &len_be, 2); p += 2;

    /* PAYLOAD */
    if (payload_len && payload) {
        memcpy(p, payload, payload_len);
        p += payload_len;
    }

    /* CRC 覆盖 VER..LEN(+PAYLOAD)，即跳过 SOF 两字节 */
    const uint16_t crc    = crc16_ccitt_false(out + 2, (uint16_t)(V1_FIXED_HEADER_LEN + payload_len));
    const uint16_t crc_be = be16(crc);
    memcpy(p, &crc_be, 2); p += 2;

    *out_len = (uint16_t)(p - out);
    return PWMH_OK;
}

static pwmh_result_t v1_send_frame(uint8_t msg_id,
                                   const uint8_t* payload, uint16_t payload_len)
{
    if (s_sock < 0) return PWMH_ENOTINIT;

    uint8_t buf[V1_MAX_FRAME];
    uint16_t n = 0;
    pwmh_result_t r = v1_pack(msg_id, payload, payload_len, buf, sizeof(buf), &n);
    if (r != PWMH_OK) return r;

    int flags = s_nonblock_send ? MSG_DONTWAIT : 0;

    ssize_t sent;
    do {
        sent = sendto(s_sock, buf, n, flags, (struct sockaddr*)&s_addr, sizeof(s_addr));
    } while (sent < 0 && errno == EINTR);

    if (sent < 0 || (size_t)sent != n) {
        ++s_stats.tx_err;
        return PWMH_ESYS;
    }
    return PWMH_OK;
}

/* 解析 HB_ACK（最小校验：SOF/VER/MSG/CRC；LEN 允许为 0） */
static int v1_try_parse_hb_ack(const uint8_t* buf, int len, uint16_t* out_seq, uint32_t* out_ticks)
{
    if (!buf || len < (V1_HEADER_TOTAL_LEN + V1_CRC_LEN)) return 0;

    /* SOF */
    if (buf[0] != (uint8_t)(PWM_HOST_SOF_BE >> 8) ||
        buf[1] != (uint8_t)(PWM_HOST_SOF_BE & 0xFF)) {
        return 0;
    }

    const uint8_t* p = buf + 2;
    uint8_t ver = p[0];
    uint8_t msg = p[1];
    if (ver != PWM_HOST_PROTO_VER || msg != MSG_HB_ACK) return 0;

    uint16_t seq_be;   memcpy(&seq_be,   p + 2, 2);
    uint32_t ticks_be; memcpy(&ticks_be, p + 4, 4);
    uint16_t len_be;   memcpy(&len_be,   p + 8, 2);

    uint16_t payload_len = ntohs(len_be);
    const int frame_len  = V1_HEADER_TOTAL_LEN + payload_len + V1_CRC_LEN;
    if (len < frame_len) return 0;

    /* CRC 检验（覆盖 VER..LEN(+PAYLOAD)） */
    uint16_t crc_rx_be; memcpy(&crc_rx_be, buf + (frame_len - 2), 2);
    uint16_t crc_rx   = ntohs(crc_rx_be);
    uint16_t crc_calc = crc16_ccitt_false(buf + 2, (uint16_t)(V1_FIXED_HEADER_LEN + payload_len));
    if (crc_rx != crc_calc) return 0;

    if (out_seq)   *out_seq   = ntohs(seq_be);
    if (out_ticks) *out_ticks = ntohl(ticks_be);
    return frame_len; /* consumed bytes */
}

/* ============================ 错误字符串 ============================ */

PWMH_API const char* pwm_host_strerror(pwmh_result_t rc)
{
    switch (rc) {
    case PWMH_OK:        return "OK";
    case PWMH_EINVAL:    return "EINVAL";
    case PWMH_ENOTINIT:  return "ENOTINIT";
    case PWMH_ESYS:      return "ESYS";
    case PWMH_EBUSY:     return "EBUSY";
    case PWMH_EINTERNAL: return "EINTERNAL";
    default:             return "UNKNOWN";
    }
}

/* ============================ 默认配置 ============================ */

PWMH_API void pwm_host_default_config(pwm_host_config_t* cfg)
{
    if (!cfg) return;
    cfg->stm32_ip      = "192.168.2.16";
    cfg->stm32_port    = 8000;
    cfg->send_hz       = 50;
    cfg->socket_sndbuf = 0;
    cfg->nonblock_send = 0;
}

/* ============================ 映射工具实现 ============================ */

PWMH_API uint16_t pwm_host_percent_to_u16(float pct)
{
    if (pct < PWM_HOST_PCT_MIN) pct = PWM_HOST_PCT_MIN;
    if (pct > PWM_HOST_PCT_MAX) pct = PWM_HOST_PCT_MAX;
    /* 5.0% → 0, 10.0% → 10000 */
    float norm = (pct - PWM_HOST_PCT_MIN) / (PWM_HOST_PCT_MAX - PWM_HOST_PCT_MIN); /* 0..1 */
    int v = (int)(norm * (float)PWM_HOST_VAL_MAX + 0.5f);
    if (v < PWM_HOST_VAL_MIN) v = PWM_HOST_VAL_MIN;
    if (v > PWM_HOST_VAL_MAX) v = PWM_HOST_VAL_MAX;
    return (uint16_t)v;
}

PWMH_API float pwm_host_u16_to_percent(uint16_t v)
{
    if (v > PWM_HOST_VAL_MAX) v = PWM_HOST_VAL_MAX;
    /* 0 → 5.0%, 10000 → 10.0% */
    float pct = PWM_HOST_PCT_MIN
              + (PWM_HOST_PCT_MAX - PWM_HOST_PCT_MIN) * ((float)v / (float)PWM_HOST_VAL_MAX);
    return pct;
}

/* ============================ 生命周期 ============================ */

PWMH_API pwmh_result_t pwm_host_init(const pwm_host_config_t* cfg)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }

    pwm_host_config_t local_cfg;
    if (cfg == NULL) {
        pwm_host_default_config(&local_cfg);
        cfg = &local_cfg;
    }

    const char* ip   = (cfg->stm32_ip   != NULL) ? cfg->stm32_ip   : "192.168.2.16";
    uint16_t    port = (cfg->stm32_port != 0   ) ? cfg->stm32_port : 8000;
    s_send_hz         = (cfg->send_hz   > 0   ) ? cfg->send_hz     : 50;
    s_nonblock_send   = (cfg->nonblock_send != 0) ? 1 : 0;

    s_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (s_sock < 0) return PWMH_ESYS;

    memset(&s_addr, 0, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &s_addr.sin_addr) != 1) {
        close(s_sock); s_sock = -1;
        return PWMH_EINVAL;
    }

    if (cfg->socket_sndbuf > 0) {
        int sndbuf = cfg->socket_sndbuf;
        (void)setsockopt(s_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    }

    /* 影子值设为中位 */
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        s_shadow[i] = PWM_HOST_VAL_MID;
    }
    s_seq = 0;

    /* 重置统计与 RTT */
    memset(&s_stats, 0, sizeof(s_stats));
    s_last_rtt_ms        = -1.0;
    s_last_hb_seq        = 0;
    s_last_hb_send_ticks = 0;

    return PWMH_OK;
}

PWMH_API void pwm_host_close(void)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
}

PWMH_API const char* pwm_host_version(void)
{
    return PWM_HOST_SEMVER;
}

/* ============================ 发送接口 ============================ */

PWMH_API pwmh_result_t pwm_host_set_all_u16(const uint16_t v[PWM_HOST_CH_NUM])
{
    if (s_sock < 0) return PWMH_ENOTINIT;
    if (!v) return PWMH_EINVAL;

    /* clamp & 覆盖影子 */
    uint16_t payload16[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        uint16_t vi = v[i];
        if (vi > PWM_HOST_VAL_MAX) vi = PWM_HOST_VAL_MAX;
        payload16[i] = vi;
        s_shadow[i]  = vi;
    }

    /* payload 大端打包 */
    uint8_t payload[V1_MAX_PAYLOAD];
    uint8_t* p = payload;
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        uint16_t be = be16(payload16[i]);
        memcpy(p, &be, 2);
        p += 2;
    }

    pwmh_result_t rc = v1_send_frame(MSG_PWM, payload, sizeof(payload));
    if (rc == PWMH_OK) ++s_stats.tx_pwm;
    else               ++s_stats.tx_err;
    return rc;
}

PWMH_API pwmh_result_t pwm_host_set_all_pct(const float pct[PWM_HOST_CH_NUM])
{
    if (s_sock < 0) return PWMH_ENOTINIT;
    if (!pct) return PWMH_EINVAL;

    uint16_t vv[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        float pi = pct[i];
        if (pi < 0.0f) pi = PWM_HOST_PCT_MID;  /* 负值视为中位 */
        vv[i] = pwm_host_percent_to_u16(pi);
    }
    return pwm_host_set_all_u16(vv);
}

PWMH_API pwmh_result_t pwm_host_set_ch_pct(int ch, float pct)
{
    if (s_sock < 0) return PWMH_ENOTINIT;
    if (ch < 1 || ch > PWM_HOST_CH_NUM) return PWMH_EINVAL;

    uint16_t vv[PWM_HOST_CH_NUM];
    /* 从影子复制当前值 */
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        vv[i] = s_shadow[i];
    }

    float p = (pct < 0.0f) ? PWM_HOST_PCT_MID : pct;
    vv[ch - 1] = pwm_host_percent_to_u16(p);
    return pwm_host_set_all_u16(vv);
}

/* ============================ 心跳与轮询 ============================ */

PWMH_API pwmh_result_t pwm_host_send_heartbeat(void)
{
    if (s_sock < 0) return PWMH_ENOTINIT;

    /* 先“窥视”一下将要使用的 seq：v1_pack 会 ++s_seq */
    uint16_t next_seq = (uint16_t)(s_seq + 1);
    uint32_t now_ms   = ticks_ms();

    pwmh_result_t rc = v1_send_frame(MSG_HB, NULL, 0);
    if (rc == PWMH_OK) {
        ++s_stats.tx_hb;
        s_last_hb_seq        = next_seq;
        s_last_hb_send_ticks = now_ms;
    } else {
        ++s_stats.tx_err;
    }
    return rc;
}

/**
 * @brief 轮询收包/处理
 * @param timeout_ms 0=非阻塞；>0=阻塞等待至多 timeout_ms
 * @return
 *   - >= 0 : 本次处理的帧数
 *   - <  0 : 负的错误码（-PWMH_ENOTINIT, -PWMH_ESYS 等）
 */
PWMH_API int pwm_host_poll(int timeout_ms)
{
    if (s_sock < 0) return -PWMH_ENOTINIT;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s_sock, &rfds);

    struct timeval tv, *ptv = NULL;
    if (timeout_ms > 0) {
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        ptv = &tv;
    }

    int nsel;
    do {
        nsel = select(s_sock + 1, &rfds, NULL, NULL, ptv);
    } while (nsel < 0 && errno == EINTR);

    if (nsel < 0) {
        ++s_stats.rx_err;
        return -PWMH_ESYS;
    }
    if (nsel == 0) return 0; /* 超时，无数据 */

    if (!FD_ISSET(s_sock, &rfds)) return 0;

    /* 尽量消费掉内核缓冲区（避免积压） */
    int handled = 0;
    for (;;) {
        uint8_t buf[RX_BUF_SIZE];
        ssize_t rcv;
        do {
            rcv = recvfrom(s_sock, buf, sizeof(buf), MSG_DONTWAIT, NULL, NULL);
        } while (rcv < 0 && errno == EINTR);

        if (rcv < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            ++s_stats.rx_err;
            return (handled > 0) ? handled : -PWMH_ESYS;
        }
        if (rcv == 0) break; /* UDP: 理论上少见，这里直接退出循环 */

        /* 尝试解析 HB_ACK */
        uint16_t seq_rx   = 0;
        uint32_t ticks_rx = 0;
        int used = v1_try_parse_hb_ack(buf, (int)rcv, &seq_rx, &ticks_rx);
        if (used > 0) {
            ++handled;
            ++s_stats.rx_hb_ack;

            /* 匹配最近一次心跳，给出 RTT（粗略，以 host 单调时钟为准） */
            if (seq_rx == s_last_hb_seq && s_last_hb_send_ticks != 0) {
                uint32_t now = ticks_ms();
                double rtt = (double)(now - s_last_hb_send_ticks);
                s_last_rtt_ms = rtt;
            }
            continue;
        }

        /* 其他帧：暂不处理（后续可扩展 STATUS 等） */
        ++handled;
    }
    return handled;
}

PWMH_API double pwm_host_last_rtt_ms(void)
{
    return s_last_rtt_ms;
}

PWMH_API void pwm_host_get_stats(pwm_host_stats_t* out)
{
    if (!out) return;
    *out = s_stats;
}

/* ============================ 阻塞式线性渐变 ============================ */

PWMH_API pwmh_result_t pwm_host_ramp_pct(int ch, float start_pct, float end_pct, float seconds, int hz)
{
    if (s_sock < 0) return PWMH_ENOTINIT;
    if (ch < 1 || ch > PWM_HOST_CH_NUM) return PWMH_EINVAL;
    if (seconds <= 0.0f) return PWMH_EINVAL;
    if (hz <= 0) hz = (s_send_hz > 0) ? s_send_hz : 50;

    /* 计算步数与步进 */
    int steps = (int)(seconds * (float)hz + 0.5f);
    if (steps < 1) steps = 1;
    const double period_ms = 1000.0 / (double)hz;

    /* 基于影子生成初值 */
    uint16_t base[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        base[i] = s_shadow[i];
    }

    const uint16_t start_v = pwm_host_percent_to_u16(start_pct);
    const uint16_t end_v   = pwm_host_percent_to_u16(end_pct);

    for (int k = 0; k <= steps; ++k) {
        double t = (double)k / (double)steps; /* 0..1 */
        int iv = (int)((double)start_v + ((double)end_v - (double)start_v) * t + 0.5);
        if (iv < PWM_HOST_VAL_MIN) iv = PWM_HOST_VAL_MIN;
        if (iv > PWM_HOST_VAL_MAX) iv = PWM_HOST_VAL_MAX;

        uint16_t vv[PWM_HOST_CH_NUM];
        memcpy(vv, base, sizeof(vv));
        vv[ch - 1] = (uint16_t)iv;

        pwmh_result_t r = pwm_host_set_all_u16(vv);
        if (r != PWMH_OK) return r;

        if (k < steps) {
            sleep_ms(period_ms);
        }
    }
    return PWMH_OK;
}
