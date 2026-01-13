> 📌 阅读指引  
> 如果你是 **ROV 操作员 / 控制算法开发者**，  
> 而不是在做上位机通信或协议开发，  
> 请优先阅读项目总 README 与 `pwm_control_program/README.md`：
>
> - 项目总览：`../README.md`
> - 控制主程序：`../pwm_control_program/README.md`

---

# `comm_gcs` — Ground Control Station Communication Module

## 1. 模块定位（What & Why）

`comm_gcs` 是 **OrangePi_STM32_for_ROV 系统中的“上位机通信中枢”模块**，负责实现：

> **ROV ↔ 上位机（GCS）之间的可靠 UDP 通信、会话管理与协议编解码**

该模块**不直接参与控制算法、不直接操作 PWM、不依赖具体控制器实现**，其职责仅限于：

* 网络通信（UDP）
* 数据包封装 / 校验 / 解包
* 会话（Session）状态管理
* 为上层模块提供**干净、结构化的控制与遥测数据接口**

---

## 2. 在系统架构中的位置

整体系统中，`comm_gcs` 所处位置如下：

```
┌──────────────┐
│   上位机 GCS  │
│ (GUI / 手柄) │
└──────┬───────┘
       │ UDP
┌──────▼──────────────────────────┐
│            comm_gcs              │
│  - UDP Server / Client           │
│  - GCS Session                   │
│  - Packet Codec                  │
└──────┬──────────────────────────┘
       │ 结构化数据（Intent / Telemetry）
┌──────▼──────────────────────────┐
│      pwm_control_program         │
│  - GcsInputProvider              │
│  - ControlGuard                  │
│  - ControllerManager             │
└─────────────────────────────────┘
```

**重要原则：**

* `comm_gcs` **不知道** 推进器、PWM、PID、MPC
* `pwm_control_program` **不关心** UDP、CRC、Session 细节
* 二者通过 **清晰的数据结构与接口解耦**

---

## 3. 目录结构说明

```
comm_gcs/
├── include/
│   └── comm_gcs/
│       ├── udp_client.hpp        # UDP 客户端
│       ├── udp_server.hpp        # UDP 服务端
│       ├── udp_endpoint.hpp      # UDP endpoint 抽象
│       └── session/
│           └── gcs_session.hpp   # GCS 会话与状态机
│
├── src/
│   ├── udp_client.cpp
│   ├── udp_server.cpp
│   ├── udp_endpoint.cpp
│   └── session/
│       └── gcs_session.cpp
│
├── apps/
│   ├── gcs_server.cpp            # 示例：GCS 服务端
│   └── gcs_client.cpp            # 示例：GCS 客户端
│
├── tests/
│   ├── test_session.cpp          # Session 单元测试
│   └── test_codec.cpp            # 编解码测试
│
├── CMakeLists.txt
└── README.md   ←（本文件）
```

---

## 4. 核心概念说明

### 4.1 UDP 通信模型

* 使用 **UDP**（低延迟、易穿透、适合遥控）
* 不依赖 TCP 连接状态
* 通过 **Session + 心跳 + TTL** 弥补 UDP 的不可靠性

### 4.2 GCS Session（会话）

`GcsSession` 是本模块的核心，负责：

* 识别对端（IP / Port）
* 维护会话状态（未连接 / 已连接 / 超时）
* 校验数据包合法性（版本、长度、CRC）
* 提供：

  * 最新控制指令（Control Intent）
  * 会话心跳与超时检测

⚠️ **注意**：
Session 只关心“通信是否健康”，**不决定系统是否 armed / 是否输出 PWM**。

---

## 5. 提供给上层的能力

`comm_gcs` 向上层（如 `pwm_control_program`）提供：

* ✅ **结构化、已校验的数据**
* ✅ 明确的“是否新数据 / 是否过期”
* ❌ 不直接提供线程调度
* ❌ 不做任何控制逻辑判断

典型用法（概念示意）：

```cpp
if (session.has_valid_intent(now_ns)) {
    intent = session.latest_intent();
} else {
    // 交由 ControlGuard 处理失联 / 超时
}
```

---

## 6. 可执行程序说明（apps）

### 6.1 `gcs_server`

用途：

* 在 **ROV / OrangePi** 上运行
* 验证 UDP 接收、解包、Session 逻辑是否正确
* 常用于 **脱离真实上位机的通信联调**

运行示例：

```bash
./gcs_server
```

### 6.2 `gcs_client`

用途：

* 在 PC 或 OrangePi 上运行
* 模拟 GCS 向 server 发送控制数据
* 用于协议、CRC、Session 行为验证

---

## 7. 单元测试（tests）

* `test_session`

  * 测试会话状态机、超时逻辑

* `test_codec`

  * 测试协议打包 / 解包 / CRC

⚠️ 这些程序**不是现场运行程序**，仅用于开发与回归测试。

---

## 8. 与 `pwm_control_program` 的关系

在 `pwm_control_program` 中：

* `comm_gcs` **不会被直接调用**
* 而是通过：

  * `io/gcs/gcs_link_udp`
  * `io/input/gcs_input_provider`
  * `io/input/gcs_input_adapter`
    进行二次封装

这样做的目的：

* 允许未来替换通信方式（如串口 / CAN / ROS2）
* 保持控制层代码稳定

---

## 9. 设计原则总结

* **单一职责**：只做通信
* **强校验**：CRC / 长度 / 版本
* **可测试**：独立 apps + tests
* **可替换**：不侵入控制逻辑

---

## 10. 新人须知（必读）

* ❌ 不要在 `comm_gcs` 里写控制逻辑
* ❌ 不要在这里判断 armed / estop
* ✅ 所有“安全策略”应在 `control_guard`
* ✅ 所有“运动决策”应在 controller 层

---