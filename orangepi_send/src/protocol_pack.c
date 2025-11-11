// protocol_pack.c (host/orangepi lightweight packer)
#include "protocol_pack.h"
#include <string.h>
#include <time.h>

/* 帧字段布局 */
#define SOF_B0           0xAAu
#define SOF_B1           0x55u
#define HEADER_FIXED_LEN 12u   // SOF2 + VER1 + MSG1 + SEQ2 + TICKS4 + LEN2
#define CRC_LEN          2u

/* 大端读写 */
static inline void be16_write(uint8_t* p, uint16_t v){ p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)(v&0xFF); }
static inline void be32_write(uint8_t* p, uint32_t v){
    p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)(v);
}

/* 用 MONOTONIC，避免系统时钟调整造成跳变 */
static uint32_t host_ticks_ms(void){
#if defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec*1000ull + ts.tv_nsec/1000000ull);
#else
    return (uint32_t)time(NULL)*1000u;
#endif
}

/* CRC16-CCITT-FALSE：poly 0x1021, init 0xFFFF, 无反射, xorout 0x0000 */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len){
    uint16_t crc = 0xFFFF;
    for(uint16_t i=0;i<len;i++){
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for(uint8_t j=0;j<8;j++){
            crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
        }
    }
    return crc;
}

/* 本地序列号 */
static uint16_t s_seq = 0;

void protocol_pack_init(void){ s_seq = 0; }
void protocol_pack_set_seq(uint16_t seq){ s_seq = seq; }
uint16_t protocol_pack_get_seq(void){ return s_seq; }

bool protocol_pack_pwm(const uint16_t pwm8[8], uint8_t* out_buf, uint16_t out_cap, uint16_t* out_len){
    if(!pwm8 || !out_buf || !out_len) return false;

    const uint16_t len   = 16u; // 8×u16
    const uint16_t total = (uint16_t)(HEADER_FIXED_LEN + len + CRC_LEN);
    if(out_cap < total) return false;

    uint8_t* p = out_buf;
    *p++ = SOF_B0; *p++ = SOF_B1;         // SOF
    *p++ = PROTO_VER_1;                   // VER
    *p++ = MSG_PWM;                       // MSG
    be16_write(p, s_seq++); p += 2;       // SEQ
    be32_write(p, host_ticks_ms()); p+=4; // TICKS
    be16_write(p, len); p += 2;           // LEN

    // PAYLOAD：8×u16 大端，超界裁剪
    for(int i=0;i<8;i++){
        uint16_t v = pwm8[i];
        if(v > 10000u) v = 10000u;
        be16_write(p, v); p += 2;
    }

    const uint16_t crc = crc16_ccitt(out_buf + 2, (uint16_t)(10u + len)); // VER..LEN+PAYLOAD
    be16_write(p, crc); p += 2;

    *out_len = (uint16_t)(p - out_buf);
    return true;
}

bool protocol_pack_heartbeat(uint8_t* out_buf, uint16_t out_cap, uint16_t* out_len){
    if(!out_buf || !out_len) return false;

    const uint16_t len   = 0u;
    const uint16_t total = (uint16_t)(HEADER_FIXED_LEN + CRC_LEN);
    if(out_cap < total) return false;

    uint8_t* p = out_buf;
    *p++ = SOF_B0; *p++ = SOF_B1;         // SOF
    *p++ = PROTO_VER_1;                   // VER
    *p++ = MSG_HB;                        // MSG
    be16_write(p, s_seq++); p += 2;       // SEQ
    be32_write(p, host_ticks_ms()); p+=4; // TICKS
    be16_write(p, len); p += 2;           // LEN

    const uint16_t crc = crc16_ccitt(out_buf + 2, 10u);      // VER..LEN (10字节)
    be16_write(p, crc); p += 2;

    *out_len = (uint16_t)(p - out_buf);
    return true;
}
