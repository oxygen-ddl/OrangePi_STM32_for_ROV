#ifndef Uart_service_H
#define Uart_service_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>


bool UART_SendFloats_DMA(uint8_t count, ...);
bool UART_SendFloats_Blocking(uint8_t count, ...);

#endif
