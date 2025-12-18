

```md
# GCS ↔ ROV Control & Telemetry Protocol  
## ControlIntent / Telemetry UDP Protocol Specification

---

## 0. 文档定位（必读）

本文件定义 **上位机（GCS）与 ROV 控制系统（OrangePi）之间**  
**唯一权威的通信协议规范**，用于：

- 远程控制 ROV 推进器运动
- 传递系统状态与安全信息
- 支持多控制模式（Manual / Auto / Failsafe）
- 保证通信不可靠条件下的系统安全

> ⚠️ 本协议是 **控制级协议**，  
> 不是日志、不是导航数据、不是高带宽传感器流。

---

## 1. 通信总体设计

### 1.1 通信方式

| 项目 | 说明 |
|----|----|
| 传输层 | UDP |
| 方向 | 双向（GCS ⇄ ROV） |
| 可靠性 | 通过 Session / TTL / 序号保证 |
| 目标 | 低延迟、可失联、可快速恢复 |

---

### 1.2 协议分层

```

┌─────────────────────────────┐
│ ControlIntent / Telemetry   │  ← 本文档定义
├─────────────────────────────┤
│ Packet Header / CRC         │  ← proto_gcs
├─────────────────────────────┤
│ UDP Socket                  │
└─────────────────────────────┘

```

---

## 2. 会话模型（Session Model）

### 2.1 GCS Session 定义

- ROV **不维护 TCP 连接**
- 每个 GCS 通过 `(IP, Port)` 被识别
- 第一次合法数据包 → 创建 Session
- 超时未收到数据 → Session 进入 stale 状态

### 2.2 会话状态

| 状态 | 含义 |
|----|----|
| INIT | 尚未收到合法数据 |
| ACTIVE | 正常通信 |
| STALE | 超过 TTL，输入失效 |

> Session 状态 **不等价于 Armed / Disarmed**

---

## 3. 数据包通用结构

### 3.1 Packet Header（通用）

| 字段 | 类型 | 说明 |
|----|----|----|
| magic | u16 | 固定帧头 |
| version | u8 | 协议版本 |
| msg_type | u8 | 消息类型 |
| length | u16 | payload 长度 |
| seq | u32 | 序号（单调递增） |
| timestamp_ns | u64 | 发送方时间戳 |
| payload | bytes | 数据体 |
| crc | u16 / u32 | CRC 校验 |

> 所有数值均为 **Little Endian**（如代码中另有定义，以代码为准）

---

### 3.2 Message Type 定义

| msg_type | 名称 |
|----|----|
| 0x01 | ControlIntent |
| 0x02 | Telemetry |
| 0x03 | Heartbeat（可选） |

---

## 4. ControlIntent（GCS → ROV）

### 4.1 设计目标

ControlIntent 是 **“我希望 ROV 做什么”** 的唯一表达形式：

- 不描述“怎么做”
- 不描述“PWM”
- 不关心控制器类型

---

### 4.2 ControlIntent 结构

| 字段 | 类型 | 说明 |
|----|----|----|
| surge | float | 前进 / 后退 |
| sway | float | 横移 |
| heave | float | 升沉 |
| roll | float | 横滚 |
| pitch | float | 俯仰 |
| yaw | float | 偏航 |
| mode | u8 | 控制模式请求 |
| arm | bool | ARM / DISARM |
| estop | bool | 急停 |
| ttl_ms | u32 | 输入有效期 |
| intent_flags | u32 | 扩展标志 |

---

### 4.3 控制模式（mode）

| 值 | 含义 |
|----|----|
| 0 | Manual |
| 1 | Auto |
| 2 | Failsafe |

> ⚠️ mode 是**请求**，是否允许由 ROV 决定

---

### 4.4 TTL 与失效规则

- `ttl_ms = 0` → 使用系统默认 TTL
- `now_ns - timestamp_ns > ttl_ms` → Intent 失效
- 失效 Intent **不会进入控制器**

---

### 4.5 ESTOP 语义（重要）

| estop | 行为 |
|----|----|
| true | 立即进入急停锁存 |
| false | 仅表示“未请求急停”，不等于解除 |

解除急停必须通过 **明确的 CLEAR / DISARM 逻辑**（由 ControlGuard 定义）。

---

## 5. Telemetry（ROV → GCS）

### 5.1 设计目标

Telemetry 用于：

- 让操作员“知道发生了什么”
- 调试通信 / 控制 / 安全状态
- 不用于闭环控制

---

### 5.2 Telemetry 结构

| 字段 | 类型 | 说明 |
|----|----|----|
| system_state | u8 | 系统状态 |
| control_mode | u8 | 当前控制模式 |
| armed | bool | 是否 Armed |
| estop | bool | 是否急停 |
| active_controller | u8 | 当前控制器 |
| last_intent_age_ms | u32 | 最近 Intent 延迟 |
| fault_flags | u32 | 故障位 |
| session_state | u8 | Session 状态 |

---

### 5.3 system_state

| 值 | 含义 |
|----|----|
| 0 | INIT |
| 1 | READY |
| 2 | RUNNING |
| 3 | FAILSAFE |

---

### 5.4 fault_flags（示例）

| bit | 含义 |
|----|----|
| 0 | Intent timeout |
| 1 | ESTOP latched |
| 2 | Controller error |
| 3 | PWM link lost |

---

## 6. 安全设计原则（协议层）

- ❌ GCS **不能直接控制 PWM**
- ❌ GCS **不能强制解除 ESTOP**
- ❌ GCS **不能绕过 TTL**
- ✅ ROV 永远可以拒绝不安全请求

---

## 7. 典型时序示例

### 7.1 正常控制

```

GCS → ControlIntent (arm=true)
ROV → Telemetry (armed=true)

GCS → ControlIntent (motion)
ROV → Telemetry (RUNNING)

```

---

### 7.2 失联保护

```

GCS → ControlIntent (ttl=100ms)
[通信中断]

100ms 后：
ROV → FAILSAFE
PWM → 中位

```

---

## 8. 版本管理

- `version` 字段用于协议升级
- 新字段必须 **向后兼容**
- GCS 应拒绝未知 version

---

## 9. 与代码的对应关系（索引）

| 协议概念 | 代码位置 |
|----|----|
| ControlIntent | `io/input/control_intent.hpp` |
| GCS Session | `comm_gcs/session/gcs_session.*` |
| Telemetry | `io/gcs/gcs_telemetry_adapter.*` |
| TTL 检测 | `control_core/control_guard.*` |

---

## 10. 明确不在本协议范围内

- 导航数据（IMU / DVL / USBL）
- 轨迹规划
- 日志文件格式
- 图像 / 声呐数据

---

## 11. 总结

> 本协议的核心目标不是“功能多”，  
> 而是 **即使通信异常，也不会让推进器失控**。

这是一个 **为真实下水系统设计的控制协议**。

```

---

## ✅ 这份协议文档现在解决了什么问题

* 🔒 **ControlIntent 成为唯一控制入口**
* 🔒 GCS 权限边界被文档锁死
* 🔗 与 `comm_gcs` / `pwm_control_program` **一一对齐**
* 🧠 新人、上位机开发者 **不会再猜字段含义**

---

