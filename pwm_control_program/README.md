
---

# ROV 控制程序（pwm_control_program）

### 上位机实时控制系统架构说明与开发指南（OrangePi）

本仓库实现了**水下机器人 ROV 的上位机实时控制系统**，负责把操控输入、控制算法、推力分配、安全层下发串联成一个可靠的控制链路。

系统核心目标包括：

* 统一的控制架构（ControlLoop + Controller + Allocator）
* 可靠的 8 通道推进器控制（基于安全层 pwm_control 保护）
* 手动模式（Teleop）与自动控制模式（未来：PID / MPC / SMC）
* 与底层 PWM 执行层完全解耦
* 工程级可扩展架构，适合科研到实机部署

该架构已经通过在 OrangePi 上的实际编译与调试验证。

---

# 1. 整体控制链路概述

控制链路由三层组成：

```
【控制程序（本仓库）】
   Controller → ThrusterCommand[8]
        ↓
【PWM 安全层（pwm_control.c）】
   限斜率 / AB 分组 / 越零保护
        ↓
【驱动层 libpwm_host → UDP → STM32】
        ↓
【ESC → 推进器】
```

职责划分：

| 层级          | 作用                   |
| ----------- | -------------------- |
| 控制程序        | 决策（控制算法、模式、手动输入解析）   |
| 安全层（C 库）    | PWM 信号硬性保护，不允许任何失控指令 |
| 驱动层（UDP 协议） | 可靠下发至 STM32          |
| 执行层         | 最终驱动 8 路 PWM         |

控制程序**永远不直接生成物理 PWM**，只是给安全层提供归一化指令。

---

# 2. 项目目录结构（保持可扩展）

结构如下：

```
pwm_control_program/
├── CMakeLists.txt
├── README.md
│
├── config/
│   └── pwm_client.yaml           ← 底层通信参数（IP、端口等）
│
├── docs/
│   ├── pwm_control_architecture.md
│   ├── control_stack_safety.md
│   ├── pwm_test_procedures.md
│   └── teleop_usage.md
│
├── include/
│   ├── control_core/
│   │   ├── control_types.hpp     ← ControlState/Reference/Output
│   │   └── control_loop.hpp      ← 主循环调度器
│   │
│   ├── controllers/
│   │   ├── controller_base.hpp   ← 接口 IController
│   │   └── manual_controller.hpp ← 手动控制器
│   │
│   ├── io/
│   │   ├── input_provider.hpp    ← 抽象输入源
│   │   └── teleop_input.hpp      ← 键盘 teleop 实现
│   │
│   ├── platform/
│   │   ├── pwm_client.hpp        ← 对安全层+UDP的封装
│   │   └── timebase.hpp
│   │
│   └── utils/
│       └── config_loader.hpp     ← YAML 加载（可后续加入）
│
└── src/
    ├── main.cpp                  ← 程序入口
    ├── control_core/
    │   ├── app_main.cpp
    │   └── control_loop.cpp
    ├── controllers/manual_controller.cpp
    ├── io/teleop_input.cpp
    ├── io/teleop_keyboard.cpp
    ├── platform/pwm_client.cpp
    └── platform/timebase.cpp
```

该结构已完全兼容你当前的编译结果。

---

# 3. 控制系统架构

## 3.1 控制程序内部架构

```
+----------------------------+
|      InputProvider         |  (键盘 / 上位机指令 / 未来自动轨迹)
+----------------------------+
               |
               v
+----------------------------+
|      Controller            |  (Manual / PID / MPC / SMC)
+----------------------------+
               |
               v
+----------------------------+
|  ControlOutput (Thrusters) |
+----------------------------+
               |
               v
+----------------------------------------+
|      PwmClient  +  pwm_control.c       |
|   (限斜率 / AB 分组 / 越零保护 / UDP)   |
+----------------------------------------+
```

控制框架是**完全模块化的**：

| 模块            | 责任                                 |
| ------------- | ---------------------------------- |
| InputProvider | 读取输入（键盘、网络、轨迹）并写入 ControlReference |
| Controller    | 计算 ThrusterCommand[8]              |
| ControlLoop   | 管理周期、调度输入和输出                       |
| PwmClient     | 封装安全层 + UDP 下发                     |
| pwm_control.c | **强制性安全保障，不可绕过**                   |

---

# 4. 控制循环控制策略

主循环（ControlLoop）运行在固定频率（默认 100 Hz）：

```
while true:
    poll input           （TeleopInputProvider）
    controller.compute()  （ManualController）
    pwm_client.setTargets()
    pwm_client.step()     （AB分组 + 限斜率 + 越零保护）
```

流程保证：

* 输入 → 输出 → 安全层 → 下发 全流程可控
* 单次错误不会导致程序退出
* 连续错误超过阈值才退出

---

# 5. 控制模式体系（未来扩展）

虽然目前实现的是 **ManualController**，但架构已经支持：

| 控制模式             | 描述                |
| ---------------- | ----------------- |
| Manual           | 手动控制，6DOF 转 8 推进器 |
| PID DepthHold    | 深度保持              |
| PID AttitudeHold | 姿态保持              |
| MPC Track        | 轨迹跟踪              |
| SMC Robust       | 强鲁棒控制             |

每种模式对应新的 Controller 子类。

---

# 6. Teleop 输入系统

TeleopInputProvider 完成：

* 切换终端为 raw mode
* 非阻塞读取按键
* 调用 C 层 teleop_keyboard
* 写入 ControlReference.dof_cmd

键位（简化示例）：

| 按键  | 含义           |
| --- | ------------ |
| W/S | Surge ±      |
| A/D | Sway ±       |
| Q/E | Yaw ±        |
| H/G | Heave ±      |
| R/T | Roll ±（纯姿态）  |
| F/V | Pitch ±（纯姿态） |
| 1–8 | 单电机阻塞测试      |
| M   | 清零           |
| ESC | 请求退出         |

---

# 7. 手动控制器 ManualController

将 DOF 命令映射为 8 推进器：

```
surge → 1/2/3/4
sway  → 差动分配
yaw   → 正反交替分配
heave → 5/6/7/8 同向
roll  → 5~8 左右差动
pitch → 5~8 前后差动
```

ManualController 已与你当前 ROV 推进器布局一致。

---

# 8. PWM 客户端与安全层交互（关键工程价值）

PwmClient 是控制层与驱动层之间的唯一接口。

功能：

* setTargets(float[8])
* step() 调用安全层
* emergencyStop()
* 自动处理错误
* 自动清理资源（析构）

所有 PWM 信号必须经过安全层：

```
限斜率 → 越零保护 → A/B分组 → UDP打包
```

确保推进器硬件不会受损。

---

# 9. 日志与调试（后续加入）

未来会加入：

* CSV 控制日志
* 推进器命令日志
* 事件日志（模式切换、错误）

---

# 10. 开发路线（Roadmap）

| 阶段 | 内容                                |
| -- | --------------------------------- |
| 1  | ManualController + Teleop 完成（已完成） |
| 2  | 简易 PID 控制器加入                      |
| 3  | 推力分配矩阵（6DOF → 8 Thrusters）        |
| 4  | MPC 控制模块接入                        |
| 5  | 控制参数 YAML 支持                      |
| 6  | 高级模式系统（ModeManager）               |

当前代码结构完全支持以上路线。

---

# 11. 总结

你现在的控制程序已经具备：

✔ 专业工程结构
✔ 安全层全流程接入
✔ Teleop → Controller → PWM 的完整链路
✔ 完整 CMake 工程
✔ 适合未来拓展 MPC / PID / SMC

---

## 目录结构与模块职责

整个 `pwm_control_program` 只做一件事：
在 **控制算法** 和 **底层 PWM 安全层 + STM32** 之间，提供一个安全、可扩展的控制程序。

从上往下看，功能分层大致是：

```
控制算法层（controllers/*）
    ↓  thruster_command[8] 逻辑推力指令
控制主循环（control_core/*）
    ↓  motor_pct[8] 逻辑占空比
PWM 客户端（platform/pwm_client）
    ↓  调用 C 安全层 pwm_control.c
底层通信 orangepi_send + STM32
```

### 1. 顶层文件

#### `CMakeLists.txt`

* 定义整个 `pwm_control_program` 子项目的构建规则。
* 主要职责：

  * 指定 C++ 标准、警告等级。
  * 告诉编译器去哪儿找头文件（`include/` 等）。
  * 链接底层 `libpwm_host.a`、`yaml-cpp` 等依赖。
* 实际工作中：

  * 新增源文件 / 模块时，记得在这里的 `add_library` / `add_executable` 中补上。
  * 如果底层库路径有变动（例如 `orangepi_send_build` 目录），也在这里改。

#### `README.md`

* 面向“工程师”的总览文档：

  * 本控制程序在整个 ROV 系统中的角色。
  * 控制链路：控制算法 → 安全 PWM → UDP → STM32。
  * 适合新同事快速理解“这个子项目到底干什么”。

---

### 2. `config/` 配置层

#### `config/pwm_client.yaml`

* 用于配置 **底层 PWM 通信参数**：

  * UDP 目标 IP（STM32 所在板子的 IP）。
  * UDP 端口号。
  * 逻辑通道数量（一般 8）。
  * 心跳周期、超时等参数。
* 特点：

  * 这是你在换实验平台 / 换网段 / 换 STM32 时最常改的文件。
  * 控制算法不需要关心这些细节，只要它能看到“电机 0–7 的逻辑占空比”。

后续可以在 `config/` 下增加：

* `controller_manual.yaml`：手动模式配置（各 DOF 增益、限幅）。
* `controller_mpc.yaml`：MPC 参数。
* `thrust_allocator.yaml`：推力分配矩阵等。

---

### 3. `docs/` 文档层

这部分是给“人”看的，而不是给编译器看的。

* `pwm_control_architecture.md`
  描述整个 PWM 控制栈的架构，包括：

  * teleop → 控制程序 → pwm_control → UDP → STM32 的链路。
  * 安全措施位置（限斜率、越零保护、急停）。

* `control_stack_safety.md`
  专门讲安全策略：

  * 为什么有 AB 交替、限斜率等措施。
  * 出现异常（通信丢失、心跳超时）时如何保护推进器。

* `pwm_test_procedures.md`
  工程测试流程：

  * 第一次上电前要拆桨。
  * 每次实验前后的检查清单。
  * 如何逐通道验证、如何回归测试。

* `teleop_usage.md`
  针对操作者的“键盘操作说明书”：

  * 键位定义。
  * 多 DOF 叠加规则。
  * 退出流程和急停行为。

新人如果要“先用起来、再看代码”，建议从 `docs/` 开始看。

---

### 4. `include/` 头文件层（接口与抽象）

这一层定义了“模块边界”和“API 协议”。它们是各个 cpp 文件之间的契约。

#### 4.1 `control_core/` 核心数据结构与主循环

* `control_core/control_types.hpp`
  定义这一套控制栈的“语言”：

  * `ControlMode`：控制模式枚举（目前有 `MANUAL`，未来会扩展为 `MPC_TRACK` 等）。
  * `Pose / Twist / Accel`：位姿、速度、加速度。
  * `DofCommand`：归一化 6-DOF 指令（surge / sway / heave / roll / pitch / yaw ∈ [-1,1]）。
  * `ControlState`：当前状态（来自导航/估计系统）。
  * `ControlReference`：期望（来自 Teleop / 上位机 / 轨迹规划）。
  * `ControlOutput`：

    * `body_wrench`：物理量级别的 6-DOF 力/力矩。
    * `thruster_command[8]`：8 推进器归一化指令（本次我们用的就是这个）。

  新人可以把这个文件理解成：“控制程序的统一数据字典”。

* `control_core/control_loop.hpp`
  控制主循环 `ControlLoop` 的声明：

  * 定义 `Config`：主循环频率、错误阈值、是否打开 PWM 日志等。
  * 声明：

    * 构造函数：接收 `PwmClient`、`InputProvider`、`IController`。
    * `run()`：执行主循环。

  任何新的控制模式（MPC、SMC）都会复用这套循环框架，而不是重新写 while(true)。

---

#### 4.2 `controllers/` 控制算法层

* `controllers/controller_base.hpp`

  * 定义统一控制器接口 `class IController`：

    * `name()`：名字（用于日志）。
    * `mode()`：返回 `ControlMode`。
    * `reset()`：重置内部状态。
    * `compute(state, ref, output, dt)`：

      * 所有控制算法只实现这一个入口。
      * 不直接操作 PWM，不关心 UDP，不做 IO，纯算法。

  * 任何一个新算法模块（PID、MPC、RL）都应该 `class XxxController : public IController`。

* `controllers/manual_controller.hpp`

  * 一个最简单的控制器实现，主要是“线性 DOF → 推进器”的映射：

    * 从 `ControlReference.dof_cmd` 取 6 个 DOF 指令。
    * 乘以方向增益（surge/sway/yaw/roll/pitch/heave）。
    * 按当前推进器布局，把 6 个 DOF 合成成 8 路 `thruster_command`。
    * 限幅（保护 DOF 输出在 [-1,1] 或 cfg.max_cmd_abs 内）。

  * 新人理解推进器布局、DOF → 电机映射时可以重点读这个文件和它的 cpp。

---

#### 4.3 `io/` 输入输出抽象层

* `io/input_provider.hpp`

  * 定义 `class IInputProvider`：

    * `init()`：初始化输入源（比如打开终端 raw 模式）。
    * `poll(state, ref, request_exit)`：

      * 每个控制周期调用一次。
      * 填充当前的 `ControlState`（也可以先空着）。
      * 填充 `ControlReference`（比如键盘 Teleop 指令）。
      * 根据用户输入可以设置 `request_exit=true`（退出控制循环）。

  * 新的输入源（上位机 TCP 命令、autopilot 脚本）只要实现这个接口即可。

* `io/teleop_input.hpp`

  * `TeleopInputProvider` 的接口声明：

    * 一个具体版本的 `IInputProvider`，数据来源是键盘。
    * 内部持有一个 `ControlMode mode_{MANUAL}` 作为标签。
    * 在 cpp 中用 `pwm_teleop_*` 函数和 C 层键盘处理逻辑对接。

  * 新人要理解“键盘按键如何最终变成 DOF 指令”，可以看：

    * `teleop_input.cpp` + C 层的 `pwm_teleop_keys.c`。

  * 注意：**PWM 日志记录并不在这里做**，而是在 `ControlLoop` 中统一实现，所以更换控制器 / 输入源时日志功能不会丢。

---

#### 4.4 `platform/` 平台适配层

* `platform/pwm_client.hpp`

  * 封装底层 PWM 安全层和 UDP 发送逻辑。

  * 提供上层调用接口：

    * `init()`：初始化底层 `pwm_control` 与 `libpwm_host`。
    * `setTargets(thruster_command)`：设置本周期的逻辑目标占空比。
    * `step()`：

      * 调用安全层一步（限斜率、AB 分组、越零保护）。
      * 调用 `libpwm_host` 下发容量与通信。
    * `emergencyStop()` / `shutdown()`：安全收尾。

  * 控制算法不直接操作 C 函数，只要和 `PwmClient` 打交道。

* `platform/timebase.hpp`

  * 提供时间基工具（steady clock 等）。
  * 目前主要用于：

    * 控制循环的周期调度。
    * 未来可用于和导航、IMU/DVL 日志对齐。

---

#### 4.5 `utils/` 通用工具层

* `utils/config_loader.hpp`

  * 负责从 YAML 加载配置，比如：

    * PWM client 的 IP / 端口等参数。
    * 控制器增益。
  * 目前可以是简单封装，后面有需要再扩展。

---

### 5. `src/` 源码实现层

这一层是“真正跑起来的代码”。

#### 5.1 入口与控制主循环

* `src/main.cpp`

  * 进程入口：

    * 解析命令行（如后续需要）。
    * 调用 `control_core::app_main(argc, argv)`。
  * 新人如果只想“跑起来看效果”，看这个文件就够了。

* `src/control_core/app_main.cpp`

  * 负责把所有部件串起来：

    * 加载配置。
    * 创建 `PwmClient`。
    * 创建 `InputProvider`（当前是 `TeleopInputProvider`）。
    * 创建 `ManualController`。
    * 构造 `ControlLoop`，（把以上组件都交给它）。
    * 调用 `loop.run()`。
  * 后续如果增加其他模式：

    * 可以在这里决定用哪个 controller / input provider。

* `src/control_core/control_loop.cpp`

  * `ControlLoop::run()` 的具体实现：

    * 固定频率调度（`loop_hz`）。
    * 调 `input_->init()` 和每周期 `input_->poll(...)`。
    * 调 `controller_->compute(...)` 得到 `ControlOutput`。
    * 调 `pwm_.setTargets()` + `pwm_.step()`。
    * 错误统计（连续 `step()` 失败次数）。
    * 统一的 PWM 日志记录：

      * 只要 `cfg_.enable_pwm_log=true`，无论 controller 是 manual/MPC/SMC，都会记录。
      * 日志内容：时间戳 + 8 路 thruster_command。

---

#### 5.2 控制器实现

* `src/controllers/manual_controller.cpp`

  * 对应 `manual_controller.hpp` 的实现：

    * 把 `ControlReference.dof_cmd` 映射到 `ControlOutput.thruster_command[8]`。
    * 这里有详细注释说明当前推进器拓扑和 DOF 分解。
  * 将来如果控制布局变化，只需要调整这里的组合逻辑。

---

#### 5.3 IO 层实现

* `src/io/teleop_input.cpp`

  * 实现 `TeleopInputProvider`：

    * 启用/恢复终端 raw 模式。
    * 非阻塞读取键盘。
    * 调用 C 层 `pwm_teleop_handle_key()` 维护 DOF 状态。
    * 将当前键盘 DOF 写入 `ControlReference.dof_cmd`。

* `src/io/teleop_keyboard.cpp`

  * C 层或更底层的键盘处理实现（如果存在）：

    * 具体键位映射和组合逻辑实现。
    * 一般情况下，新人不用直接改这里，但可以当成参考。

---

#### 5.4 平台适配实现

* `src/platform/pwm_client.cpp`

  * `PwmClient` 的具体行为实现：

    * 调 C 接口 `pwm_ctrl_init(...)`、`pwm_ctrl_step()`。
    * 调 `pwm_host_set_all_pct()` 下发到 STM32。
    * 内部维护状态与错误信息。

* `src/platform/timebase.cpp`

  * 与 `timebase.hpp` 对应的实现。
  * 负责封装 `std::chrono` 并提供统一接口。

---

## 新同事如何上手阅读代码？

建议给新同事一个“推荐阅读路径”，大致如下：

1. 看 `README.md` 和 `docs/pwm_control_architecture.md`
   明白整个控制栈在 ROV 项目中的位置。

2. 看 `include/control_core/control_types.hpp`
   搞清楚项目里“状态 / 参考 / 输出”的统一定义。

3. 看 `include/controllers/controller_base.hpp` +
   `include/controllers/manual_controller.hpp` +
   `src/controllers/manual_controller.cpp`
   理解：

   * 控制器接口长什么样。
   * 手动控制是如何 DOF → 8 推进器。

4. 看 `include/io/input_provider.hpp` + `include/io/teleop_input.hpp` +
   `src/io/teleop_input.cpp`
   理解键盘如何变成 DOF 指令。

5. 看 `include/platform/pwm_client.hpp` + `src/platform/pwm_client.cpp`
   理解逻辑占空比如何走到安全层 + UDP + STM32。

6. 最后看 `src/control_core/control_loop.cpp` + `src/control_core/app_main.cpp`
   看完整的控制循环与模块串联逻辑。

这样一圈下来，新人基本能掌握每个模块的“生态位”：谁负责算法、谁负责 IO、谁负责安全与下发、日志在哪里统一做、要加新控制器应该怎么接入。

它已经不是简单的键盘控制程序，而是：

### **一个可扩展、模块化、工程化的 ROV 控制框架。**

你可以在此基础上继续发展复杂模式与控制算法，而不需要再改底层结构。

---
