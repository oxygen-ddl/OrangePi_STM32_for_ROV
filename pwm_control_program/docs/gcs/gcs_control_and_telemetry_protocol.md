
---

```md
# GCS Control & Telemetry Protocol Specification  
## OrangePi ROV Control Core – External Interface

**Version**: v1.0  
**Scope**: GCS ↔ OrangePi（pwm_control_program）  
**Transport**: UDP  
**Audience**: GCS / 上位机 / 算法客户端开发者  

---

## 1. 文档目的

本文件定义 **GCS 与 OrangePi 控制中枢之间的唯一对外接口规范**，包括：

- 控制指令（ControlIntent）
- 系统状态遥测（Telemetry）
- 会话与安全约束

> ⚠️ **GCS 不应依赖 OrangePi 内部实现细节**  
> 本文档是 GCS 侧的“唯一事实来源（Single Source of Truth）”。

---

## 2. 总体通信模型

```

## GCS                         OrangePi (pwm_control_program)

ControlIntent  ──────────▶  GcsInputProvider
↓
ControlGuard / ControllerManager
↓
Telemetry       ◀────────── GcsTelemetryAdapter

````

- **控制方向**：GCS → OrangePi  
- **遥测方向**：OrangePi → GCS  
- **控制与遥测完全解耦**（互不阻塞）

---

## 3. 时间与安全约束（必须遵守）

### 3.1 时间基准

- 所有时间戳均使用 **单调时间（monotonic clock）**
- 单位：`nanoseconds (ns)`
- GCS 不要求与 OrangePi 绝对对时，但需**连续递增**

---

### 3.2 TTL（Time-To-Live）

每个控制指令都带 `ttl_ms`：

- 超过 TTL 的指令将被 **ControlGuard 丢弃**
- TTL = 0 表示 **禁用超时检测（不推荐）**

> 推荐值：`100 ~ 300 ms`

---

### 3.3 安全优先级（硬规则）

| 行为 | 优先级 |
|----|----|
| Emergency Stop (ESTOP) | 最高 |
| Exit / Shutdown | 高 |
| ARM / DISARM | 高 |
| Mode Switch | 中 |
| DOF / Ref / RefDelta | 低 |

---

## 4. ControlIntent（GCS → OrangePi）

### 4.1 设计理念

- **一个包 = 一个完整意图**
- 不允许“隐式状态”
- 不允许“部分依赖历史包”

---

### 4.2 ControlIntent 结构（逻辑模型）

```cpp
struct ControlIntent {
    uint64_t seq;          // 单调递增序号
    uint64_t stamp_ns;     // 本地发送时间（monotonic）
    uint32_t ttl_ms;       // 超时时间

    bool request_exit;

    // ---- Emergency / Safety ----
    bool has_estop_cmd;
    bool estop;
    bool clear_estop;

    // ---- Arm / Disarm ----
    bool has_arm_cmd;
    bool arm;
    bool disarm;

    // ---- Mode request ----
    bool has_mode_request;
    ControlMode mode_request;

    // ---- Teleoperation ----
    bool has_teleop_dof;
    DofCommand teleop_dof_cmd;

    // ---- Reference control ----
    bool has_ref;
    ControlReference ref;

    bool has_ref_delta;
    ControlReference ref_delta;
};
````

---

### 4.3 DOF 控制（Teleoperation）

```cpp
struct DofCommand {
    double surge;   // forward/backward
    double sway;    // left/right
    double heave;   // up/down
    double roll;
    double pitch;
    double yaw;
};
```

约束：

* 每个分量范围：`[-1.0, +1.0]`
* 表示 **归一化控制量**
* 实际推力映射由 OrangePi 决定

---

### 4.4 ControlMode 枚举

```cpp
enum class ControlMode {
    kNone = 0,
    kManual,
    kAuto,
    kFailsafe,
    kUnknown
};
```

| 模式       | 说明                |
| -------- | ----------------- |
| Manual   | 人工 / 直接控制         |
| Auto     | 自动控制器（PID/MPC/RL） |
| Failsafe | 保护模式（零输出或受限输出）    |

---

### 4.5 常见控制示例

#### 示例 1：纯遥控（Manual）

```text
has_teleop_dof = true
teleop_dof_cmd = {surge=0.5, yaw=0.2}
ttl_ms = 200
```

---

#### 示例 2：请求切换自动模式

```text
has_mode_request = true
mode_request = kAuto
```

---

#### 示例 3：急停

```text
has_estop_cmd = true
estop = true
```

---

## 5. Telemetry（OrangePi → GCS）

### 5.1 设计理念

* **只读**
* **周期发送**
* **用于监控与可视化**
* 不作为控制反馈闭环的一部分

---

### 5.2 TelemetryFrame（逻辑模型）

```cpp
struct TelemetryFrameV1 {
    uint64_t t_ns;

    bool session_established;
    bool link_alive;
    bool estop;

    ControlMode mode;

    std::string active_controller;
    std::string desired_controller;

    uint32_t consecutive_failures;
    uint32_t auto_fail_limit;
};
```

---

### 5.3 字段说明

| 字段                   | 含义           |
| -------------------- | ------------ |
| session_established  | 是否已建立 GCS 会话 |
| link_alive           | GCS 链路是否存活   |
| estop                | 是否处于急停锁存     |
| mode                 | 当前生效控制模式     |
| active_controller    | 当前正在运行的控制器   |
| desired_controller   | 请求的控制器       |
| consecutive_failures | 连续计算失败次数     |

---

### 5.4 遥测频率

* 默认：`10 Hz`
* 可在 `GcsTelemetryAdapter::Config` 中配置
* GCS 应允许丢包（UDP）

---

## 6. 会话与连接管理

* OrangePi 维护 **GcsSession**
* 自动检测：

  * 首包建立
  * 超时断开
* 遥测允许在“未建立会话”状态下发送（用于调试）

---

## 7. 错误处理与容错建议（GCS 侧）

* 不要假设每个控制包都会被执行
* 不要假设遥测连续
* 始终：

  * 周期发送控制
  * 周期刷新 TTL
  * 提供 ESTOP UI

---

## 8. 非目标（明确不支持）

* ❌ 通过 Telemetry 控制机器人
* ❌ 依赖隐式状态（如“上一个包”）
* ❌ 通过 GCS 绕过 ControlGuard
* ❌ 在 GCS 侧实现推进器级控制

---

## 9. 版本演进策略

* 新字段只 **追加**
* 不移除既有字段
* 使用 `TelemetryFrameVx` 区分版本

---

## 10. 总结

> **GCS 是操作者，OrangePi 是裁决者。**

GCS 可以“请求”，
但是否执行，永远由 OrangePi 的 **ControlGuard + ControllerManager** 决定。

这保证了系统在水下环境中的**长期安全运行能力**。

---

```

---

