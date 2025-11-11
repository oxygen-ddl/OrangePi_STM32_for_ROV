
---

# 项目交接说明书（STM32 与香橙派 PWM 控制系统）

**版本号：** v1.0
**最后修改日期：** 2025-11-11
**编写人：** 王雨舒
**交接对象：** 王立，孙之媛

---

## 1️⃣ 项目概述

本项目实现了 **香橙派（上位机）与 STM32（下位机）之间的通信与电机驱动控制**，核心功能包括：

* 香橙派通过 UDP 协议周期性发送 8 路 PWM 控制指令及心跳包；
* STM32 通过 UART5 接收数据帧，解析协议、校验 CRC；
* STM32 根据指令驱动 8 路 PWM 输出；
* 若通信超时（心跳丢失），自动进入失联保护模式，将 PWM 置中位。

该系统最终用于水下机器人推进器驱动闭环控制的硬件执行层。

---

## 2️⃣ 系统架构

### STM32 说明
* `Source/Src/Uart_service.c`:包含满足vofa+协议的数据传输函数
* `Source/Src/Driver_pwm.c`:pwm底层驱动函数
* `Source/Src/protocal_v1`:适用于v1版本协议PWM数据包，心跳包格式的解析
* `Source/Src/crc16_ccit.c`:crc16计算函数
* `Source/Src/Parse_pwm`：老版本数据解析函数


#### 上位机（香橙派）说明

上位机以 Linux C++ 代码运行，包括：

* `main.cpp`：主循环（发送 PWM 帧与心跳帧）
* `PwmFrameBuilder.cpp`：生成通信帧
* `UdpSender.cpp`：UDP 数据发送模块
* 通信协议版本统一为 `protocol_v1`

---

## 3️⃣ 模块功能说明

| 模块                     | 功能说明                      | 关键接口                                          | 注意事项                                            |
| ---------------------- | ------------------------- | --------------------------------------------- | ----------------------------------------------- |
| **protocol_v1**        | 解析通信帧、校验CRC、执行控制命令与ACK响应  | `protocol_feed_bytes()` / `protocol_poll()`   | 必须周期调用 `protocol_poll()` 以触发失联保护                |
| **Uart_service**       | 管理 UART5 DMA 接收与发送，触发协议解析 | `Uart5_OnRxToIdle()` / `Uart5_SendBlocking()` | 在 `HAL_UARTEx_RxEventCallback()` 中调用 OnRxToIdle |
| **Driver_pwm**         | PWM 驱动与保护逻辑（斜率限幅、死区、中位安全） | `Driver_pwm_Apply8_0_10000()`                 | 上电初始化中位；保护参数在 `config.h`                        |
| **config.h / board.h** | 参数配置与硬件映射                 | 宏定义                                           | 所有硬件通道、串口句柄修改只在此处调整                             |

---

## 4️⃣ 通信协议说明（protocol_v1）

| 字段      | 长度 | 描述                                 |
| ------- | -- | ---------------------------------- |
| SOF     | 2B | 固定帧头 0xAA55                        |
| Ver     | 1B | 协议版本号（0x01）                        |
| MsgID   | 1B | 消息类型（0x01=PWM，0x10=HB，0x11=HB_ACK） |
| Seq     | 2B | 序列号                                |
| Len     | 2B | 有效载荷长度                             |
| Payload | N  | 载荷内容（如8通道PWM值）                     |
| CRC16   | 2B | CRC16-CCITT 校验                     |

### 当前支持指令

| MsgID | 名称         | 方向          | 功能          |
| ----- | ---------- | ----------- | ----------- |
| 0x01  | MSG_PWM    | 上位机 → STM32 | 下发 PWM 控制指令 |
| 0x10  | MSG_HB     | 上位机 → STM32 | 心跳包         |
| 0x11  | MSG_HB_ACK | STM32 → 上位机 | 心跳应答        |

未来可扩展：

* 0x20：`MSG_ESTOP`（急停）
* 0x40：`MSG_STATUS`（状态上报）

---

## 5️⃣ 初始化流程

1. 系统启动后：

   ```c
  /* USER CODE BEGIN 2 */
  Driver_PWM_Init();  //初始化PWM通道
  protocol_force_failsafe(); //进入保护状态，所有通道回中位
  protocol_process_init(); //协议处理初始化

  /* USER CODE END 2 */
   ```

2. 主循环：

   ```c
  while (1)
  {
    //process_uart5_message();
    
    protocol_process(); //协议处理，在主循环中调用
    protocol_poll();    //协议轮询钩子
    HAL_Delay(1);
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
  }
   ```

3. 中断回调（在 `stm32f4xx_it.c`）：

   ```c
   /**
   * @brief This function handles UART5 global interrupt.
   */
   void UART5_IRQHandler(void)
   {
   /* USER CODE BEGIN UART5_IRQn 0 */
   //UART5_IT_TASK();
   protocol_it_process();
   /* USER CODE END UART5_IRQn 0 */
   HAL_UART_IRQHandler(&huart5);
   /* USER CODE BEGIN UART5_IRQn 1 */

   /* USER CODE END UART5_IRQn 1 */
   }
   ```

---

## 6️⃣ 运行参数建议（config.h）

| 参数名                       | 默认值  | 说明            |
| ------------------------- | ---- | ------------- |
| `CFG_FAILSAFE_TIMEOUT_MS` | 500  | 心跳丢失后触发中位输出时间 |
| `CFG_PWM_SLEW_US_PER_S`   | 1500 | 斜率限幅速率（μs/s）  |
| `CFG_PWM_DEADBAND_US`     | 20   | PWM 死区保护      |
| `PWM_MID_US`              | 1500 | PWM中位脉宽       |
| `PWM_MIN_US`              | 1100 | 最小脉宽          |
| `PWM_MAX_US`              | 1900 | 最大脉宽          |

---

## 7️⃣ 调试与验证建议

1. **初次联调**

   * 仅发送心跳包；
   * STM32 回传 `HB_ACK`；
   * 查看串口波形是否正常（周期1Hz）。

2. **PWM验证**

   * 固定发8路中位信号；
   * 逐步调整 ±10%、±30%、±50%；
   * 观测波形周期（20ms）与脉宽变化；
   * 确认 `Driver_pwm` 斜率限幅逻辑生效。

3. **异常验证**

   * 停止心跳包；
   * 500ms 后所有PWM应回到中位；
   * 重新上线后恢复正常。

---

## 8️⃣ 后续维护建议

* 若新增通信指令（如姿态反馈），统一修改：

  * `protocol_v1.h`（增加 `MSG_*` 宏）
  * `protocol_v1.c`（解析与执行逻辑）
  * 上位机对应构帧逻辑（`PwmFrameBuilder.cpp`）

* 若修改硬件引脚：

  * 在stm32cube中进行修改

* 若加入 RTOS：

  * 将 `protocol_poll()` 与 `Driver_pwm` 更新逻辑放入独立任务；
  * 保证任务调度周期 ≤ 10ms。

---

## 9️⃣ 验收清单（交付标准）

| 项目     | 验收条件                       |
| ------ | -------------------------- |
| 通信协议   | 香橙派与 STM32 心跳正常往返          |
| CRC 校验 | 错帧自动丢弃，不影响通信               |
| PWM 输出 | 范围准确（1100–1900μs），中位1500μs |
| 失联保护   | 心跳断开500ms内进入中位             |
| 重启恢复   | 系统复位后可重新握手通信               |
| 源码结构   | 三层架构清晰、注释完整                |

---

## 🔚 附录

  * `handover_guide.md`（本文件）
  * `protocol_v1.md`（协议说明）
  * `test_plan.md`（测试步骤）

---
