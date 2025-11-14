#include "Driver_pwm.h"
#include "tim.h"

#define ALL_PWM_OUT 0//选择是否配置全部通道输出PWM
/**
 * @brief 初始化所有PWM通道
 */
void Driver_PWM_Init(void)
{
#if ALL_PWM_OUT
    // 启动TIM1的所有PWM通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    // 启动TIM2的PWM通道
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    // 启动TIM4的所有PWM通道
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    // 启动TIM3的所有PWM通道
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

#else
    //仅输出8个通道pwm信号

    // 启动TIM1的前4个PWM通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    // 启动TIM4的前4个PWM通道
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

#endif

    //置于中值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);


    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1500);
    //其他通道均为0占空比

    HAL_Delay(3000); //等待3秒,确保电机初始化
}
//! pwm对应推进器的映射关系转移到上层应用（即香橙派）


/**
 * @brief 设置指定通道的PWM占空比
 * @param channel 通道号，范围1-8
 * @param duty 占空比，范围-1.0f-0.0f-1.0f
 */
void Driver_pwm_SetDuty(uint8_t channel,float duty)
{
    //5%-7.5%-10% ，对应1000-1500-2000us，ccr值为1000-1500-2000
    uint16_t ccr_value = 1500+500*duty;
    //计算CCR值
    switch (channel)
    {
    case 1:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_value);
        break;
    case 2:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_value);
        break;
    case 3:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_value);
        break;
    case 4:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr_value);
        break;
    case 5:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr_value);
        break;
    case 6:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr_value);
        break;
    case 7:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr_value);
        break;

    case 8:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, ccr_value);
        break;
    default:
        break;
    }
}


