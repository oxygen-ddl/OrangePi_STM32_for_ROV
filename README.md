# 项目概述
本项目实现了 **香橙派（上位机）与 STM32（下位机）之间的通信与上位机控制电机运行**

## 香橙派与STM32通信功能包
**Orangepi_STM32_for_ROV**
#### 主要实现功能：
- 香橙派通过 UDP 协议周期性发送 8 路 PWM 控制指令及心跳包；
- STM32 通过 UART5 接收数据帧，解析协议、校验 CRC；
- STM32 根据指令驱动 8 路 PWM 输出；
- 若通信超时（心跳丢失），自动进入失联保护模式，将 PWM 置中位。
#### 内容分类
`orangepi_send`: 香橙派端功能包
`receive_pwm_stm32`:stm32端代码
#### 外部调用接口
`Orangepi_STM32_for_ROV/orangepi_send/include/`
- `libpwm_host.h`: 上位机（香橙派）控制 STM32 PWM 的最小可复用 C 接口
- `pwm_control.h`: 基于 libpwm_host 的工程化 PWM 控制层（8 通道 ROV 电机）
  
## 上位机控制部分
**pwm_control_program**
#### 主要实现功能：
- 键盘控制ROV简单运动
