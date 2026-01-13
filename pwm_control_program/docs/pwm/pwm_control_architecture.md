
---

# PWM 控制系统总体架构说明（新版本）

面向水下机器人 8 通道推进器控制
（工程文档，支持手动控制、自动控制、MPC、PID 等扩展）

---

# 1. 总览：控制系统三层结构

本项目采用 **统一的三层控制架构**，覆盖从输入源（键盘、算法）到 PWM 下发的完整链路。

```
输入层（Input Layer）
     │  TeleopInput / 上位机 / 自动控制
     ▼
控制器层（Controller Layer）
     │  Manual / PID / MPC / RL 都实现 IController
     ▼
调度层（Control Loop Layer）
     │  固定频率循环、调度输入→控制器→PWM
     ▼
执行层（Execution Layer）
     │  PwmClient → pwm_control 安全层 → UDP → STM32
```

三层完全解耦，任何部分都可以独立替换或升级。

---

# 2. 输入层（Input Layer）

输入层由 `IInputProvider` 抽象定义。

当前实现：`TeleopInputProvider`
未来支持：

* 上位机 TCP 输入
* ROS 控制输入
* 航迹规划器输出
* 网络控制 API
* 自动化测试脚本

输入层的职责：

1. 产生 `ControlReference`
2. 选择是否退出程序（`request_exit`）
3. 写入控制模式（Manual, Auto…）
4. 与控制器完全解耦

示例 Teleop 输入到 DOF：

```
键盘按键 → DOF 指令（surge/sway/heave/roll/pitch/yaw）
DOF 指令写入 ControlReference.dof_cmd
```

---

# 3. 控制器层（Controller Layer）

控制器层由接口 `IController` 统一：

```cpp
bool compute(const ControlState& state,
             const ControlReference& ref,
             ControlOutput& out,
             double dt);
```

所有控制算法都实现这一个函数：

* ManualController（已实现）
* PIDController（待扩展）
* MPCController（未来扩展）
* SMPC / RL（未来扩展）

控制器只做**数学映射**，不接触 PWM 和 IO。

示例 Manual 控制器输出：

```
DOF 指令（surge/sway/...） → 推进器 8 维 thruster_command[]
```

控制器输出统一交给 ControlLoop 处理。

---

# 4. 调度层：ControlLoop（Control Loop Layer）

核心文件：`control_loop.cpp`

主要职责：

1. 固定频率执行控制循环（100 Hz）
2. 调用输入源 → 维护 ControlState / ControlReference
3. 调用控制器 → 生成 ControlOutput
4. 调用 PwmClient → 进行安全层控制
5. 统一记录 PWM 日志
6. 错误检测、限频、退出机制

调度层是整个程序的“心脏”。

时序示意图：

```
for each cycle (10 ms):
    │
    ├─ 1. input_->poll()       // 读取输入
    │
    ├─ 2. controller_->compute()
    │         输入：state + ref
    │         输出：thruster_command[8]
    │
    ├─ 3. pwm_.setTargets()    // 写入目标占空比
    │
    ├─ 4. pwm_.step()          // 调用底层安全层，平滑逼近
    │
    └─ 5. pwm_logger.log()     // 写入日志
```

ControlLoop 是所有模式（Manual/MPC/...）的统一调度器。

---

# 5. 执行层（Execution Layer）

执行层由 `PwmClient` 和底层 C 安全逻辑组成：

```
PwmClient C++
    ▼
pwm_control.c（安全层）
    ▼
libpwm_host（UDP 通信）
    ▼
STM32（PWM 真实输出）
```

## 5.1 PwmClient（C++ 封装）

职责：

1. 提供 C++ 友好的 API：`init() / setTargets() / step() / emergencyStop()`
2. 全部调用底层安全逻辑
3. 绝不允许 bypass 安全层（强约束）
4. 维护错误状态，向上层报告

## 5.2 pwm_control（安全层）

功能：

* **限斜率限制**（平滑逼近）
* **AB 分组更新**
* **占空比边界保护**
* **单通道测试**
* **紧急停止**
* **平滑归中**

示例（限斜率）：

```
current = 7.5%, target = 9.0%, max_step = 0.2%
7.5 → 7.7 → 7.9 → 8.1 → ... → 9.0
```

## 5.3 libpwm_host（UDP 通信）

负责：

* UDP 发送 PWM 帧给 STM32
* 心跳机制
* 超时与错误码

不做任何控制逻辑。

---

# 6. PWM 日志系统（自动启用）

由 ControlLoop 统一驱动：

```
log(t, thruster_command[8])
```

存储结构：

```
logs/
    2025-12-02-15-30-10/
        pwm_log.csv
```

日志包含：

* 时间戳（循环相对时间）
* 8 路归一化 PWM 指令

日志功能与控制器、输入源无关，无论 Manual/MPC 都会自动记录。

---

# 7. 各模块职责矩阵（重要）

| 模块          | 输入                  | 输出               | 职责         | 与谁通信                      |
| ----------- | ------------------- | ---------------- | ---------- | ------------------------- |
| TeleopInput | 键盘                  | ControlReference | 生成 DOF 指令  | ControlLoop               |
| IController | state + ref         | ControlOutput    | 算法层映射      | ControlLoop               |
| ControlLoop | Input/Controller 输出 | 占空比目标            | 调度、日志、错误控制 | PwmClient                 |
| PwmClient   | 目标占空比               | —                | 调用安全层和 UDP | pwm_control / libpwm_host |
| pwm_control | 目标占空比               | 平滑占空比            | 限斜率、安全策略   | libpwm_host               |
| libpwm_host | 平滑占空比               | —                | UDP 发送     | STM32                     |
| STM32       | 占空比                 | 推进器 PWM          | 最终执行       | —                         |

---

# 8. 新人快速理解路径（工程建议）

推荐阅读顺序：

1. `docs/pwm_control_architecture.md`（本文件）
2. `control_types.hpp`（控制系统的数据字典）
3. `controller_base.hpp`（控制器接口）
4. `manual_controller.cpp`（8 推进器布局与 DOF 组合）
5. `input_provider.hpp` + `teleop_input.cpp`
6. `pwm_client.cpp`
7. `pwm_control.c`（限斜率安全层）

掌握以上内容后即可开发：

* 新控制器（PID/MPC）
* 新输入源（TCP/ROS）
* 新分布式控制策略

---

# 9. 未来升级方向

* 自动控制 / MPC / SMPC 集成（直接写 IController）
* 基于导航融合状态的闭环控制
* 上位机远程操控输入
* 支持 BlueROV2/ArduSub 推进器矩阵
* 自适应增益 + 自动调参
* 统一 ROS2 接口

---
