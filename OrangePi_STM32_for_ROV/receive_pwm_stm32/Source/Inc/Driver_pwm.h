#ifndef Driver_pwm_H
#define Driver_pwm_H

#include "stm32f4xx_hal.h"

void Driver_PWM_Init(void);
void Driver_pwm_SetDuty(uint8_t channel,float duty);



#endif

