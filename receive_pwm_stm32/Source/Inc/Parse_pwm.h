#ifndef PROTOCOL_V1_H
#define PROTOCOL_V1_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


#define power_board_max_len 64
typedef struct
{
    uint8_t data[power_board_max_len];
    uint16_t len;
}Receive_Msg_t;

void Uart5_Parse_Init(void);                           // UART5 DMA 接收初始化
void UART5_IT_TASK(void);                                   // UART5 中断服务函数里调用
void process_uart5_message(void);                     // 主循环里调用，处理收到的数据

#ifdef __cplusplus
}
#endif
#endif
