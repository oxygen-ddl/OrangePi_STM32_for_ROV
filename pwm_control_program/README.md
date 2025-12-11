那我直接给你一份“合并整理 + 加上新特性（config_loader / 轨迹 / PWM 日志）”后的 README 草稿，你可以直接覆盖 `README.md` 用：

````markdown
# ROV 控制程序（pwm_control_program）

### 上位机实时控制系统架构说明与开发指南（OrangePi）

本仓库实现了**水下机器人 ROV 的上位机实时控制系统**，负责把操控输入、控制算法、推力分配、安全层下发串联成一个可靠的控制链路。

系统核心目标包括：

- 统一的控制架构（ControlLoop + Controller + Allocator）
- 可靠的 8 通道推进器控制（基于安全层 `pwm_control` 保护）
- 手动模式（Teleop）与自动控制模式（未来：PID / MPC / SMC）
- 与底层 PWM 执行层完全解耦
- 工程级可扩展架构，适合科研到实机部署

该架构已经在 OrangePi 上完成编译与调试验证。

---

## 1. 整体控制链路概述

控制链路由三层组成：

```text
【控制程序（本仓库）】
   Controller → ThrusterCommand[8]
        ↓
【PWM 安全层（pwm_control.c）】
   限斜率 / AB 分组 / 越零保护
        ↓
【驱动层 libpwm_host → UDP → STM32】
        ↓
【ESC → 推进器】
````

职责划分：

| 层级          | 作用                  |
| ----------- | ------------------- |
| 控制程序        | 决策（控制算法、模式、手动输入解析）  |
| 安全层（C 库）    | PWM 信号硬性保护，避免任何失控指令 |
| 驱动层（UDP 协议） | 可靠下发至 STM32         |
| 执行层         | 最终驱动 8 路 PWM/推进器    |

> 控制程序**永远不直接生成物理 PWM**，只是给安全层提供归一化指令。

---

## 2. 项目目录结构

当前目录结构如下：

```
pwm_control_program/
.
├── CMakeLists.txt                      ← 本子项目的 CMake 构建脚本（目标：pwm_control_program）
├── config                              ← 运行时配置文件目录（不需要重新编译）
│   ├── control_params.yaml             
│   ├── pwm_client.yaml                 
│   └── trajectory.yaml                 
├── docs                                ← 工程文档与操作说明（给人看的，不参与编译）
│   ├── architecture.svg                ← 控制架构总览图（控制程序 ↔ 安全层 ↔ STM32）
│   ├── control_loop_sequence.svg       ← 控制主循环时序图（Input → Controller → PwmClient）
│   ├── pwm_control_architecture.md     ← PWM 三层结构及安全设计说明（新成员优先阅读）
│   ├── PWM.md                          
│   ├── pwm_teleop_usage.md             
│   ├── pwm_teleop_user_manual.md       
│   ├── pwm_test_procedures.md          ← PWM 上电/测试流程与安全 checklist（实验前必读）
│   └── test                           
│       ├── control_algorithm_development_guide.md 
│       └── pid_test_guide.md           
├── include                             ← 头文件（对外接口与模块边界声明）
│   ├── control_core                    ← 控制核心：数据类型 + 主循环 + 推力分配 + 轨迹跟踪
│   │   ├── app_main.hpp                ← app_main 入口声明：组装各模块并启动 ControlLoop
│   │   ├── control_loop.hpp            ← ControlLoop 声明：单线程控制主循环框架
│   │   ├── control_types.hpp           
│   │   ├── mode.hpp                    ← 控制模式枚举与模式相关定义（Manual/MPC/...）
│   │   ├── thruster_allocation.hpp     
│   │   └── trajectory_tracking.hpp     
│   ├── controllers                     ← 控制器接口与各类控制器头文件
│   │   ├── controller_base.hpp         ← 抽象基类 IController（compute(state,ref,output,dt)）
│   │   ├── controller_manager.hpp      ← 控制器管理器（未来支持多模式切换/注册）
│   │   ├── manual_controller.hpp       
│   │   └── pid_controller.hpp          
│   ├── io                              ← 输入/输出适配层（键盘、导航状态、PWM 日志等）
│   │   ├── input_provider.hpp          
│   │   ├── nav_state_subscriber.hpp    ← 导航状态订阅接口（从导航进程/共享内存读取 NavState）
│   │   ├── pwm_logger.hpp              
│   │   ├── teleop_input.hpp            ← 键盘 Teleop 输入提供者声明（封装终端 + 键盘解析）
│   │   └── teleop_keyboard.h           ← C 风格键盘处理辅助接口（按键 → DOF 状态）
│   ├── platform                        ← 平台适配层（与底层 C 库、时间系统打交道）
│   │   ├── pwm_client.hpp              ← PwmClient 接口：对接 pwm_control.c + libpwm_host
│   │   └── timebase.hpp                ← 时间基工具：稳态时钟/时间戳/周期睡眠等封装
│   └── utils                           
│       └── config_loader.hpp           ← 配置加载/路径解析工具（pwm/control/trajectory 三类）
├── pwm_control_update.md               
├── README.md                           
├── src                                 ← 源码实现（与 include 中的头文件一一对应）
│   ├── control_core
│   │   ├── app_main.cpp                ← app_main 实现：加载配置、创建组件、启动 ControlLoop
│   │   ├── control_loop.cpp            ← ControlLoop::run 实现：主循环逻辑 + 错误统计 + 日志
│   │   ├── thruster_allocation.cpp     
│   │   └── trajectory_tracking.cpp     
│   ├── controllers
│   │   ├── controller_manager.cpp      ← 控制器管理器实现（预留多模式/动态切换）
│   │   ├── manual_controller.cpp       
│   │   └── pid_controller.cpp          
│   ├── io
│   │   ├── nav_state_subscriber.cpp    ← 从导航进程/共享内存读取 NavState 的具体实现
│   │   ├── pwm_logger.cpp              
│   │   ├── teleop_input.cpp            ← TeleopInputProvider 实现（终端 raw 模式 + 按键轮询）
│   │   └── teleop_keyboard.cpp         ← 键盘映射的 C 实现：键位 → DOF 增减/清零逻辑
│   ├── main.cpp                        
│   ├── platform
│   │   ├── pwm_client.cpp              ← PwmClient 实现：限斜率/AB 分组/越零保护 + UDP 下发
│   │   └── timebase.cpp                ← 时间工具实现：steady clock / sleep_until 封装
│   └── utils
│       └── config_loader.cpp           ← 解析 YAML 路径 & 读取 pwm/control/trajectory 配置
└── update_v2.md                        ← v2 版本的整体更新说明/迁移指南（与旧版差异摘要）

```

---

## 3. 控制程序内部架构

### 3.1 模块关系概览

```text
+----------------------------+
|      InputProvider         |  (键盘 / 上位机指令 / 未来自动轨迹)
+----------------------------+
               |
               v
+----------------------------+
|        Controller          |  (Manual / PID / MPC / SMC)
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
               |
               v
        libpwm_host → STM32 → ESC
```

核心模块职责：

| 模块            | 责任                                   |
| ------------- | ------------------------------------ |
| InputProvider | 读取输入（键盘、网络、轨迹）并填充 `ControlReference` |
| Controller    | 根据状态与参考，计算 `ThrusterCommand[8]`      |
| ControlLoop   | 管理周期、调用 Input/Controller、调度 PWM 下发   |
| PwmClient     | 封装安全层 + UDP，负责实际 PWM 下发与错误处理         |
| pwm_control.c | C 层安全逻辑：限斜率、越零保护、A/B 分组              |

---

## 4. 数据类型与主循环

### 4.1 核心数据结构（`control_types.hpp`）

在 `include/control_core/control_types.hpp` 中，定义了统一数据字典，包括：

* `Pose / Twist / Accel`：位姿、速度、加速度
* `DofCommand`：6-DOF 指令（surge/sway/heave/roll/pitch/yaw ∈ [-1,1]）
* `ControlState`：当前状态（来自导航或上位机）
* `ControlReference`：期望参考（Teleop / 轨迹 / 任务层）
* `ControlOutput`：

  * `body_wrench`：物理 6-DOF 力/力矩（未来可用于 MPC/动力学控制）
  * `thruster_command[8]`：8 推进器归一化指令（本程序当前主用）

### 4.2 控制主循环（`control_loop.hpp / .cpp`）

`ControlLoop` 完成：

* 固定频率循环（默认 `loop_hz = 100`）

* 每周期调用：

  1. `input_->poll(state, ref, request_exit)`
  2. `controller_->compute(state, ref, output, dt)`
  3. `pwm_.setTargets(output.thruster_command)`
  4. `pwm_.step()`
  5. 记录 PWM 日志（如果开启）

* 连续错误计数与退出条件

* 优雅退出时调用 `PwmClient::emergencyStop()` 和 `shutdown()`

---

## 5. 控制器体系（controllers）

### 5.1 控制器接口（`controller_base.hpp`）

统一接口 `IController`：

* `std::string name() const`
* `ControlMode mode() const`
* `void reset()`
* `void compute(const ControlState&, const ControlReference&, ControlOutput&, double dt)`

所有控制算法只在 `compute()` 中实现决策逻辑，不直接接触 PWM/UDP。

### 5.2 手动控制器（`manual_controller.hpp / .cpp`）

`ManualController` 负责：

* 从 `ControlReference.dof_cmd` 取出 6 个 DOF 指令
* 乘各向增益（`surge_gain/sway_gain/heave_gain/...`）
* 按 ROV 布局将 6 DOF 组合成 8 个 `thruster_command[i]`
* 对指令限幅到 `[-max_cmd_abs, max_cmd_abs]`

DOF → 电机的具体分配规则写在 `manual_controller.cpp` 中，是理解当前推进器布局的关键文件。

---

## 6. 输入系统（io）

### 6.1 输入抽象接口（`input_provider.hpp`）

`IInputProvider` 定义：

* `bool init()`
* `bool poll(ControlState& state, ControlReference& ref, bool& request_exit)`

每个控制周期由 `ControlLoop` 调用 `poll()`，完成：

* 采集控制状态（可从导航、共享内存或上位机获取）
* 更新参考 `ref`（键盘 Teleop / 轨迹参考 / 任务层指令）
* 根据退出指令（如 ESC）设置 `request_exit=true`

### 6.2 键盘 Teleop（`teleop_input.hpp / .cpp`）

`TeleopInputProvider`：

* 切换终端为 raw mode、非阻塞读取按键
* 调用 C 层键盘处理逻辑（`teleop_keyboard.*`）
* 将当前键盘 DOF 写入 `ControlReference.dof_cmd`

典型键位（可详见 `docs/pwm_teleop_usage.md`）：

| 按键  | DOF     |
| --- | ------- |
| W/S | Surge ± |
| A/D | Sway ±  |
| G/H | Heave ± |
| Q/E | Yaw ±   |
| R/T | Roll ±  |
| F/V | Pitch ± |
| M   | 清零      |
| ESC | 请求退出    |

---

## 7. PWM 客户端与安全层（platform）

### 7.1 PwmClient（`pwm_client.hpp / .cpp`）

职责：

* 初始化安全层 `pwm_control.c` 与 `libpwm_host`

* 接收 `thruster_command[8]`（归一化指令 `u ∈ [-1,1]`）

* 在 `step()` 中调用：

  * 限斜率
  * 越零保护
  * AB 分组输出
  * UDP 下发给 STM32

* 提供：

  * `setTargets(const std::array<float,8>&)`
  * `step()`
  * `setAllMid()`
  * `emergencyStop(float hold_s)`
  * `shutdown()`

> 所有 PWM 都必须经过 `PwmClient` 与安全层，控制器不允许直接下发物理 PWM。

---

## 8. 配置系统与 config_loader（utils）

配置文件统一放在 `config/` 目录，由 `utils/config_loader.*` 负责路径解析与加载。

### 8.1 路径解析策略

`config_loader` 提供三个解析函数：

* `resolve_pwm_client_config_path(...)`
* `resolve_control_config_path(...)`
* `resolve_trajectory_config_path(...)`

通用优先级：

1. 命令行参数（例如 `--config`, `--control-config`, `--traj-config`）

2. 环境变量：

   * `PWM_CLIENT_CONFIG`
   * `ROV_CONTROL_CONFIG`
   * `ROV_TRAJECTORY_CONFIG`

3. 相对可执行文件位置的若干候选路径，例如：

   * `./config/pwm_client.yaml`
   * `../../pwm_control_program/config/pwm_client.yaml`

这样可以兼容：

* 在 `pwm_control_program/` 下直接运行
* 在上层 `build/` 目录中运行可执行文件

### 8.2 配置文件说明

#### `config/pwm_client.yaml`

设置：

* 底层 UDP 参数（remote_ip/port 等）
* 安全层频率 `ctrl_hz`
* 限斜率参数 `max_step_pct`
* PWM 范围：`min_pct/mid_pct/max_pct`（典型 5/7.5/10）
* AB 分组掩码
* 逻辑电机 → 物理 PWM 通道映射 `motorch_to_pwmch`
* 每路电机反向标志 `motor_reverse`

#### `config/control_params.yaml`

设置推力分配/控制相关参数，例如：

* `vehicle.thrusters.order`：推进器命名顺序
* `vehicle.thrusters.allocation_matrix.data`：6×8 推力分配矩阵
* `vehicle.thrusters.active_rows`：启用的 DOF（Fx/Fy/Fz/Mx/My/Mz）
* `vehicle.thrusters.limits.norm_min/norm_max`
* `vehicle.thrusters.thrust_model.*`（如最大推力）

由 `load_thruster_allocation_config()` 解析到 `ThrusterAllocationConfig` 中。

#### `config/trajectory.yaml`

用于未来的轨迹跟踪模块，描述：

* 坐标系 `frame`（`NED` / `ENU`）
* 角度单位 `angle_unit`（`rad` / `deg`）
* 轨迹类型 `type`（当前主要为 `piecewise`）
* `waypoints[]`：

  * `t`、`x`、`y`、`z`
  * `yaw`
  * 可选 `vx/vy/vz/yaw_rate`

由上层解析为 `TrajectoryConfig`，再注入 `TrajectoryTracking`。

---

## 9. 轨迹跟踪（trajectory_tracking）

`include/control_core/trajectory_tracking.hpp` / `src/control_core/trajectory_tracking.cpp` 提供：

* 用 `TrajectoryPoint` / `TrajectoryConfig` 保存离散轨迹点

* 根据当前控制时间 `t_now_s` 在线性插值出：

  * `TrajectorySample.pose_ref`
  * `TrajectorySample.vel_ref`
  * `TrajectorySample.accel_ref`

* 提供：

  * `fill_reference()`：将采样结果写入 `ControlReference`
  * `compute_error()`：计算状态与轨迹参考之间的误差（含 yaw wrap 到 [-π, π]）

注意：**轨迹加载本身由 `config_loader` 完成**，`TrajectoryTracking` 只做插值与误差计算，两者是解耦的。

---

## 10. PWM 日志记录（pwm_logger）

PWM 日志由 `io/pwm_logger.*` 实现，并在 `ControlLoop` 中统一调用。

### 10.1 日志文件位置与命名

* 日志根目录由 `ControlLoop::Config` 指定（例如 `./logs/pwm`）
* 每次运行会生成一个新文件：

  ```text
  logs/pwm/pwm_YYYYMMDD_HHMMSS.csv
  ```

### 10.2 记录内容与单位

当前实现中，控制器和安全层内部使用**归一化指令**：

* `u ∈ [-1, 1]`

  * `u = 0`  → 中位
  * `u = +1` → 最大正向
  * `u = -1` → 最大反向

日志中不直接记 `u`，而是转换为**占空比百分数 `duty_pct`（单位：%）**：

> 假定 PWM 频率 ≈ 50 Hz，周期 T ≈ 20 ms：

* `min_pct = 5.0` → 1.0 ms 脉宽
* `mid_pct = 7.5` → 1.5 ms 脉宽
* `max_pct = 10.0` → 2.0 ms 脉宽

映射关系：

```text
u =  0   → duty_pct =  7.5 % → τ ≈ 1.5 ms（中位）
u = +1   → duty_pct = 10.0 % → τ ≈ 2.0 ms（最大正向）
u = -1   → duty_pct =  5.0 % → τ ≈ 1.0 ms（最大反向）

通用：τ_ms ≈ duty_pct / 100 * 20
```

### 10.3 CSV 格式

#### Mode::AppliedOnly

表头：

```csv
t_s,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8
```

* `t_s`：控制时间，单位秒
* `ch1`~`ch8`：各通道占空比百分数（例如 7.5 表示 7.5%）

示例行（四位小数）：

```csv
12.3400,7.5000,8.1250,8.7500,6.8750,7.5000,7.5000,7.5000,7.5000
```

含义：

* 通道 1 中位（1.5 ms）
* 通道 2 稍大于中位（约 1.625 ms）
* 通道 3 接近半油门（约 1.75 ms）
* 通道 4 稍小于中位（约 1.375 ms）
* 通道 5–8 在中位

#### Mode::CmdAndApplied

表头：

```csv
t_s,ch1_cmd,...,ch8_cmd,ch1_applied,...,ch8_applied
```

* `*_cmd`：期望占空比（由归一化指令映射）
* `*_applied`：安全层之后实际下发的占空比

方便分析：

* 安全层限斜率/越零保护是否触发
* 控制器指令与最终下发 PWM 的偏差

---

## 11. 开发路线（Roadmap）

| 阶段 | 内容                              | 状态     |
| -- | ------------------------------- | ------ |
| 1  | ManualController + Teleop       | 已完成    |
| 2  | 推力分配矩阵配置（`control_params.yaml`） | 已接入    |
| 3  | 配置解析工具 `config_loader`          | 已接入    |
| 4  | PWM 占空比日志记录（duty_pct）           | 已接入    |
| 5  | 轨迹配置 `trajectory.yaml` 与插值模块    | 已实现骨架  |
| 6  | PID 控制器（深度/姿态）                  | 进行中/预留 |
| 7  | MPC/SMC 控制模式                    | 预留     |

---

## 12. 新人阅读建议

给新同事的推荐阅读顺序：

1. `README.md` + `docs/pwm_control_architecture.md`
   了解 ROV 控制栈整体结构与控制链路。

2. `include/control_core/control_types.hpp`
   熟悉项目中的“统一控制语言”（状态 / 参考 / 输出）。

3. `include/controllers/controller_base.hpp`
   理解控制器接口约束。

4. `include/controllers/manual_controller.hpp` + `src/controllers/manual_controller.cpp`
   了解当前 ROV 布局下 DOF → 8 推进器的映射方式。

5. `include/io/input_provider.hpp` + `include/io/teleop_input.hpp` + `src/io/teleop_input.cpp`
   理解键盘 Teleop 到 DOF 指令的转换逻辑。

6. `include/platform/pwm_client.hpp` + `src/platform/pwm_client.cpp`
   理解逻辑占空比如何通过安全层与 UDP 下发到 STM32。

7. `src/control_core/control_loop.cpp` + `src/control_core/app_main.cpp`
   查看完整控制循环和模块串联方式。

阅读完这条路径，基本可以独立：

* 添加新的控制模式（PID/MPC）
* 调整推力分配
* 扩展输入通道（轨迹 / 上位机）
* 理解并分析实验中的 PWM 日志数据

---

整个 `pwm_control_program` 的唯一职责：

> 在 **控制算法** 和 **底层 PWM 安全层 + STM32** 之间，构建一个安全、可扩展、可调试的控制程序。

接下来，扩展控制器（PID / MPC / RL）、接轨导航状态、接轨轨迹规划，都可以在这一框架上自然演进。


```
