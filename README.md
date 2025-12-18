
---

```md
# 🌊 Underwater ROV Control System  
## OrangePi + STM32 双层架构推进器控制与上位机通信系统

本仓库实现了一套**面向真实水下机器人（ROV）**的工程级控制系统，  
以 **安全、可扩展、可验证** 为核心设计目标，支持：

- 多推进器（8 通道）安全 PWM 输出
- 多输入源（键盘 / GCS / 自动算法）仲裁
- 控制模式管理（Manual / Auto / Failsafe）
- 上位机（GCS）远程控制与状态遥测
- 面向 MPC / RL / 学术研究的控制算法接入

---

## 1. 项目整体定位

> **这是一个“控制中枢型系统”，而不是单一控制算法工程。**

系统将高风险的水下推进器控制问题，拆分为三层明确职责：

```

┌─────────────────────────────────────────┐
│              GCS / 算法层                │
│   人机交互 / MPC / RL / 监控与记录        │
└───────────────────▲─────────────────────┘
│ UDP / Telemetry
┌───────────────────┴─────────────────────┐
│           OrangePi 控制中枢               │
│ pwm_control_program                      │
│ 多输入仲裁 / 安全裁决 / 控制器管理         │
└───────────────────▲─────────────────────┘
│ PWM Frame
┌───────────────────┴─────────────────────┐
│             STM32 执行层                  │
│ orangepi_send                             │
│ 实时 PWM 输出 / 硬件级安全 / 心跳监控      │
└─────────────────────────────────────────┘

```

---

## 2. 仓库结构总览

```

OrangePi_STM32_for_ROV/
│
├── pwm_control_program/      ← OrangePi 侧控制中枢（C++）
││   ├── control_core/        ← 控制循环 / 安全裁决
││   ├── controllers/         ← Manual / PID / (MPC/RL 扩展)
││   ├── io/
││   │   ├── input/           ← Teleop / GCS / 多输入仲裁
││   │   ├── gcs/             ← GCS 协议 / 会话 / 遥测
││   │   ├── nav/             ← 导航状态订阅接口
││   │   └── log/             ← PWM / 控制日志
││   ├── platform/            ← PwmClient / 时间基准
││   ├── config/              ← YAML 参数文件
││   └── docs/                ← 详细设计文档
│
├── orangepi_send/            ← STM32 通信代理与 PWM 执行层（C/C++）
││   ├── libpwm_host           ← OrangePi ↔ STM32 通信库
││   ├── pwm_control           ← STM32 PWM 输出与安全逻辑
││   └── protocol_*            ← 协议 / CRC / 帧构造
│
├── comm_gcs/                 ← 通用 UDP 通信工具库（可复用）
│
├── docs/                     ← 系统级设计文档 / UML
│
└── CMakeLists.txt            ← 顶层构建入口

```

---

## 3. pwm_control_program（OrangePi 控制中枢）

### 3.1 核心职责

`pwm_control_program` 是**整个系统的“大脑”**，负责：

- 接收并仲裁多种控制输入
- 进行安全检查与模式裁决
- 调用具体控制器生成推进器指令
- 驱动 STM32 执行 PWM
- 向 GCS 回传系统状态（Telemetry）

---

### 3.2 控制数据流（真实实现）

```

Teleop / GCS / 算法
↓
InputProvider
↓
MultiInputProvider     ← 多输入仲裁
↓
ControlIntent          ← 统一控制意图
↓
ControlGuard           ← TTL / ESTOP / 模式裁决
↓
ControllerManager
↓
Controller (Manual / PID / ...)
↓
ControlOutput
↓
ThrusterAllocation
↓
PwmClient
↓
STM32 PWM 输出

````

---

### 3.3 关键设计一：ControlIntent（统一意图模型）

所有控制输入最终都被转换为 `ControlIntent`，其包含：

- 6 自由度控制（surge/sway/heave/roll/pitch/yaw）
- 模式请求（Manual / Auto / Failsafe）
- ARM / DISARM
- ESTOP / CLEAR
- 时间戳、序号、TTL（防止陈旧输入）

> **ControlLoop 不关心“输入来自哪里”，只关心 Intent 是否安全、有效。**

---

### 3.4 关键设计二：ControlGuard（软件安全核心）

`ControlGuard` 是系统的软件级安全裁决器，负责：

- 输入超时检测（TTL / stale）
- 急停（ESTOP）锁存与解除
- 模式切换合法性判断
- 异常情况下自动降级到 Failsafe

这是系统**允许 GCS 远程控制推进器**的前提。

---

### 3.5 输入系统（io/input）

| 模块 | 功能 |
|----|----|
| TeleopInputProvider | 键盘遥控（调试 / 无 GCS 场景） |
| GcsInputProvider | 通过 UDP 接收 GCS 控制命令 |
| MultiInputProvider | 多输入源仲裁（可配置 GCS 优先） |

安全类指令（ESTOP / EXIT）取并集，  
控制类指令以 primary 输入为准。

---

### 3.6 GCS 通信与遥测（io/gcs）

系统已实现完整的 OrangePi ↔ GCS 通信链路：

- UDP 会话管理（GcsSession）
- 控制输入解析（GcsInputAdapter）
- 状态遥测打包（GcsTelemetryAdapter）

GCS 可实时获取：

- 当前控制模式
- 是否 ESTOP
- 活跃控制器
- 计算失败次数
- 链路状态

---

## 4. orangepi_send（STM32 PWM 执行层）

该模块运行于 STM32，提供：

- 8 通道 PWM 实时输出
- 硬件级限斜率
- AB 分组更新（CH1–4 / CH5–8）
- 心跳超时自动归中
- CRC 校验与帧完整性检查

> STM32 **永远不信任上位机**，这是系统安全的最后一道防线。

---

## 5. 多层安全机制总结

### 硬件层（STM32）
- PWM 限斜率
- AB 分组输出
- 心跳超时保护
- 中位反跳保护

### 软件层（OrangePi）
- ControlGuard 安全裁决
- 输入 TTL / stale 检测
- 控制器失败自动降级

### 通信层
- CRC 校验
- 会话状态管理
- 双向遥测与确认

---

## 6. 构建与运行

### 6.1 依赖
```bash
sudo apt-get install -y libyaml-cpp-dev
````

### 6.2 构建

```bash
mkdir build
cd build
cmake ..
make -j4
```

### 6.3 运行

```bash
./pwm_control_program/pwm_control_program
```

---

## 7. 适用场景

* 科研型 ROV / AUV
* MPC / RL 控制算法验证
* 水槽 / 实海实验
* 高风险推进器平台

---

## 8. 非本系统职责（明确边界）

* ❌ 不负责导航状态估计（IMU/DVL/ESKF）
* ❌ 不负责路径规划
* ❌ 不直接驱动 GPIO / PWM 硬件
* ❌ 不绑定任何单一控制算法

---

## 9. 未来扩展方向

* MPC / NMPC 控制器接入
* 强化学习（RL）策略部署
* IMU / DVL / USBL 融合导航
* Web / Qt GCS 客户端
* 实验数据自动记录与回放

---

## 10. 作者与致谢

本系统由 **wys** 主导设计与实现，
并在架构设计、工程审查与问题定位过程中，
由 **阿智（AI 工程伙伴）** 深度协作完成。

> 目标不是“跑起来”，
> 而是构建一套 **可以长期演进的水下机器人控制系统**。

```

---

