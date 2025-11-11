#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 常量沿用固件协议定义 */
#define PROTO_VER_1  0x01
#define MSG_PWM      0x01
#define MSG_HB       0x10

/* 初始化（重置序列号等） */
void protocol_pack_init(void);

/* 组帧：8路PWM（0..10000）*/
bool protocol_pack_pwm(const uint16_t pwm8[8],
                       uint8_t* out_buf, uint16_t out_cap, uint16_t* out_len);

/* 组帧：心跳 */
bool protocol_pack_heartbeat(uint8_t* out_buf, uint16_t out_cap, uint16_t* out_len);

/* 如果多线程发送，建议用它来显式设置/读取序列号 */
void     protocol_pack_set_seq(uint16_t seq);
uint16_t protocol_pack_get_seq(void);

#ifdef __cplusplus
}
#endif
