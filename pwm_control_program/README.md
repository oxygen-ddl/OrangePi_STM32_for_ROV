
---

# **ROV 控制程序：架构说明与开发指南**

本仓库实现了水下机器人（ROV）在 OrangePi 平台上的实时控制程序。
其核心目标是：

* 提供稳定、安全、可扩展的**推进器控制链路**
* 支持多种**控制算法**（PID / MPC / SMC）
* 支持多种**控制模式**（手动、姿态保持、深度保持、轨迹跟踪）
* 与底层 **PWM 安全层** 与 **导航系统** 无缝协作
* 可持续演进到商用级控制架构

本 README 文档将介绍当前系统架构、核心流程及未来升级路线。

---

# **1. 项目总览**

整个控制链路由三类组件协作：

```
控制程序（本仓库）
   ↓ motor_pct[8]（逻辑电机占空比）
PWM 安全层（pwm_control.c）
   ↓ 经斜率限制 / 越零保护后的 PWM
驱动层（libpwm_host → UDP → STM32）
   ↓ 物理 PWM 输出
ESC & Thrusters
```

其中本仓库主要承担：

* 控制模式管理（手动 / 深度保持 / MPC 轨迹跟踪等）
* 控制算法（PID、MPC、SMC）
* 推力分配与推力→PWM 映射
* 与导航系统通信，获取 ROV 状态（位置、姿态、速度）
* 记录控制数据用于实验与模型训练

本系统采用模块化设计，支持后续持续扩展。

---

# **2. 项目目录结构（持续演进版本）**

当前结构（会在未来逐步升级为标准商用架构）：

```
pwm_control_program/
├── CMakeLists.txt
├── README.md
├── pwm_control_update.md
│
├── config/                      ← 控制参数与模式配置（后续扩展）
│   ├── controller_pid.yaml
│   ├── controller_mpc.yaml
│   ├── controller_smc.yaml
│   ├── thrust_allocator.yaml
│   └── pwm_client.yaml
│
├── docs/                        ← 文档（设计说明 / 使用说明）
│   ├── pwm_control_architecture.md
│   ├── control_stack_integration_and_safety.md
│   ├── pwm_teleop_usage.md
│   ├── pwm_teleop_user_manual.md
│   └── pwm_test_procedures.md
│
├── include/
│   ├── control_core/
│   │   ├── control_types.hpp     ← ControlState / ControlRef / Output
│   │   ├── mode.hpp              ← Mode 抽象接口
│   │   ├── control_loop.hpp      ← 控制循环调度
│   │   └── app_main.hpp          ← 程序入口
│   │
│   ├── controllers/             ← 控制算法接口与实现
│   │   ├── controller_base.hpp   ← IController
│   │   ├── pid_controller.hpp
│   │   ├── mpc_controller.hpp
│   │   └── smc_controller.hpp
│   │
│   ├── allocation/              ← 推力分配与 PWM 映射
│   │   ├── thrust_allocator.hpp
│   │   └── pwm_mapper.hpp
│   │
│   ├── io/                      ← 输入输出抽象
│   │   ├── input_provider.hpp
│   │   ├── teleop_input.hpp
│   │   └── log_writer.hpp
│   │
│   ├── platform/
│   │   ├── timebase.hpp         ← 控制周期管理
│   │   └── pwm_client.hpp       ← 对底层安全层的封装
│   │
│   └── utils/
│       └── config_loader.hpp     ← YAML 配置加载
│
└── src/
    ├── main.cpp
    ├── control_core/
    ├── controllers/
    ├── allocation/
    ├── platform/
    ├── io/
    └── utils/
```

> 本项目采用“可演化架构”：
> 新增模块不会破坏旧功能，而是逐步向商用标准靠拢。

---

# **3. 控制系统总体架构**

### **3.1 三层控制链路**

```
[控制程序]
   Mode + Controller + Allocator + Mapper
       ↓ motor_pct[8]
[安全层 pwm_control.c]
       ↓ 安全 PWM（经斜率 / 越零保护）
[驱动层 libpwm_host]
       ↓ 物理 PWM 信号
[STM32 → ESC → Thrusters]
```

说明：

* **控制程序** 负责“算”，不直接接触硬件
* **安全层** 负责防止任何危险 PWM
* **驱动层** 负责把数据可靠地送到 STM32
* **执行层** 负责真正驱动推进器

---

### **3.2 控制程序内部架构（本仓库重点）**

```
+---------------------------+
|   Mode 层（控制模式）     |
|  Manual / DepthHold /     |
|  VelHold / MPCTrack ...   |
+-------------+-------------+
              |
              v
+---------------------------+
|   Controller 层           |
| PID / MPC / SMC           |
+-------------+-------------+
              |
              v
+---------------------------+
|   分配层 ThrustAllocator |
|   PWM 映射层 PwmMapper    |
+-------------+-------------+
              |
              v
+---------------------------+
|   PWM Client (封装安全层)|
+---------------------------+
```

---

# **4. 控制程序运行流程**

控制进程启动后，流程如下：

```
1. 读取配置文件（控制参数、分配矩阵、PWM 配置）
2. 初始化 PWM 客户端（pwm_control + libpwm_host）
3. 初始化日志模块
4. 创建 ModeManager（进入默认模式，如 MANUAL）
5. 进入实时控制循环：
   5.1 从导航系统读取 ROV 状态
   5.2 检查健康状态（安全监测）
   5.3 根据外部输入决定是否切换 Mode
   5.4 调用当前 Mode::update()
   5.5 记录控制日志
   5.6 sleep_until_next_tick()
```

控制循环频率：**30–50 Hz**

安全层内部频率：**100 Hz（每通道有效 50 Hz）**

---

# **5. 控制模式（Mode）系统**

为借鉴 ArduPilot 的成熟方案，本项目采用 Mode 机制。

每个模式：

* 拥有自己的控制器（PID / MPC / SMC）
* 拥有自己的参考源（遥控、轨迹、上位机命令）
* 拥有自己的内部状态（如 DepthHold 锁定的深度）

每个 Mode 至少包含：

```
on_enter()     ← 进入模式
update()       ← 每个控制周期执行
on_exit()      ← 离开模式
can_enter()    ← 校验是否允许进入模式
```

当前计划实现的模式（逐步推进）：

* `Mode_Manual`
* `Mode_DepthHold`
* `Mode_VelHold`
* `Mode_PosHold`
* `Mode_MPCTrack`
* 后续：`Mode_TestThrust`（推力标定模式）

---

# **6. 控制器（Controller）层**

所有控制器遵循统一接口：

```cpp
class IController {
public:
    virtual ~IController() = default;
    virtual void reset() = 0;
    virtual void update(
        const ControlState& state,
        const ControlRef&   ref,
        ControlOutput&      out) = 0;
};
```

### 支持的控制算法：

* **PID**：适合简单单轴控制、稳定性调试
* **MPC**：多轴耦合控制、轨迹跟踪
* **SMC**：高扰动场景、鲁棒控制

控制器不接触 PWM、安全层和硬件。

---

# **7. 推力分配与 PWM 映射**

控制器通常输出物理量（力/力矩），但推进器执行的是 PWM。

因此分两个步骤：

### **1）推力分配（τ → thrust[8]）**

考虑：

* 8 个推进器的位置和方向
* 机体动力学耦合
* 推力饱和约束

### **2）推力→PWM 映射**

基于实验标定：

* 推进器推力曲线
* 推力 → PWM 百分比
* 针对每个电机独立映射

这层完全独立，不影响控制器逻辑。

---

# **8. PWM 客户端（PwmClient）**

该模块负责：

* 初始化底层安全层 `pwm_control.c`
* 设置 motor_pct[8] 作为“目标占空比”
* 触发安全层 `pwm_ctrl_step()`
* 调用 `pwm_host_set_all_pct()` 下发

上层不直接调用底层驱动 API。

保证：

* 所有 PWM 信号都经过斜率/越零保护
* 所有 PWM 都经过安全层处理
* 驱动通信与控制逻辑完全解耦

---

# **9. 日志与可视化**

每个控制周期记录：

* 时间戳
* ControlState（来自导航）
* ControlRef（期望）
* ControlOutput（控制器输出）
* thrust[8]
* motor_pct[8]
* 当前 Mode
* PWM 下发成功/失败标识

支持：

* CSV / 二进制格式
* 便于 Python / MATLAB 分析

---

# **10. 状态估计接口（与导航系统集成）**

控制程序从导航系统读取状态，例如：

* 位置 / 姿态
* 线速度 / 角速度
* 深度
* IMU 健康参数

通信方式可为：

* UDP
* 共享内存
* TCP
* 文件流（实验阶段）

控制程序不自己计算姿态或速度，而是依赖导航进程（nav_core）。

---

# **11. 未来升级路线图**

本控制程序设计为可持续扩展结构，后续将加入：

### 1）完整 Mode 系统

* 手动
* 深度保持
* 速度保持
* 位置保持
* MPC 轨迹跟踪

### 2）3 套控制器

* PID：基准实现
* MPC：主研究方向
* SMC：强鲁棒性能

### 3）动态推力分配

* 在线更新阻尼/质量矩阵
* 适配故障推进器
* 强推力限制

### 4）上位机交互协议

* 更换模式
* 动态调整 PID/MPC 参数
* 实时发送轨迹点

### 5）实验特性

* 推力标定模式
* 自动扫频模式
* 安全测试模式

### 6）更成熟的参数系统

* 动态加载参数
* 类似 PX4 的参数表
* 支持运行时调整与保存

---

# **12. 总结**

本控制程序将成长为一套完整的 ROV 控制框架，目标是：

* **可维护**
* **可扩展**
* **可实验 / 可研究**
* **可靠（安全层保障）**
* **工程化（模块清晰、责任明确）**

随着项目迭代，本 README 将持续更新，反映架构演进与模块升级。

---
