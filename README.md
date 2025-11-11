# 项目概述
本项目实现了 **香橙派（上位机）与 STM32（下位机）之间的通信与电机驱动控制**，核心功能包括：

* 香橙派通过 UDP 协议周期性发送 8 路 PWM 控制指令及心跳包；
* STM32 通过 UART5 接收数据帧，解析协议、校验 CRC；
* STM32 根据指令驱动 8 路 PWM 输出；
* 若通信超时（心跳丢失），自动进入失联保护模式，将 PWM 置中位。

# 相关文档请移步`docs/`文件夹
* `docs/handover_guide.md`：项目交接说明书（STM32 与香橙派 PWM 控制系统）
* `docs/protocol_v1.md`:通信协议说明书（Protocol V1）
* `docs/test_plan_en.md`:系统测试与联调指南
