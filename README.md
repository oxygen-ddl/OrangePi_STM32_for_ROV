
---

# 🌊 Underwater ROV PWM Control System

### 双机架构（OrangePi + STM32）水下机器人推进器控制系统

为 MPC / SMC / RL 等高层控制器提供可靠、可验证的底层推进器驱动基础。

---

# 1. 项目简介

本系统面向**水下机器人（ROV/AUV）**的低层推进器控制，提供：

* 高可靠性的 **8 通道 PWM 输出链路**
* 覆盖全流程的 **安全机制**
* 可组合的 **手动/自动控制架构**
* 适配**高层控制器（PID / MPC / NMPC / RL）**使用

整个工程分为两个子系统：

```
OrangePi （上位机）           STM32 （下位机）
--------------------------------------------------------------
pwm_control_program       <-->      orangepi_send
控制逻辑 / 输入层 / 安全层           PWM 执行 / 心跳监控 / 协议解析
```

系统保证**不论上层控制算法是否成熟，推进器永远处于安全状态**。

---

# 2. 系统架构总览

## 2.1 目录结构（大项目层级）

```
OrangePi_STM32_for_ROV/
│
├── orangepi_send/                ← STM32 通信代理与 PWM 底层服务（C）
│     ├── include/libpwm_host.h
│     ├── include/pwm_control.h
│     └── build/libpwm_host.a     ← 输出静态库
│
├── pwm_control_program/          ← PWM 控制层（C++）
│     ├── include/
│     │     ├── control_core/     ← 控制循环、数据结构
│     │     ├── controllers/      ← ManualController/MPC Controller
│     │     ├── io/               ← 键盘输入 TeleopInput
│     │     ├── platform/         ← PwmClient、安全层封装
│     │     └── utils/            ← 配置/日志工具（可扩展）
│     ├── src/
│     └── config/                 ← YAML 参数（pwm_client.yaml 等）
│
└── CMakeLists.txt                ← 顶层构建入口（统一构建两个项目）
```

---

# 3. 运行链路

## 3.1 控制数据流（核心）

```
[ Teleop / 上位机控制算法 ]  →  InputProvider
          ↓
     ControlReference      （surge / sway / heave / yaw / roll / pitch）
          ↓
     ControllerBase        （Manual / MPC / RL / SMC）
          ↓
     ControlOutput         （8 通道归一化 thruster_cmd）
          ↓
     PwmClient             （限斜率 + AB 分组 + 安全策略）
          ↓
     libpwm_host           （UDP 通讯层）
          ↓
     STM32 pwm_control     （实际 PWM 输出）
```

---

# 4. 功能模块说明

## 4.1 上位机控制程序（pwm_control_program）

### 控制核心（control_core）

* 固定周期控制循环（ControlLoop）
* 管理输入 Provider、控制器、PWM 客户端
* 负责与安全层协同执行限斜率、组交替、心跳

### 控制器层（controllers）

* ManualController（键盘遥控）
* MPCController（未来扩展）
* RLController（未来扩展）
* 所有控制器统一输出 8 通道 thruster_cmd [-1,1]

### 输入层（io）

* TeleopInputProvider：键盘输入，映射到 DOF（6 个自由度）

### 平台层（platform）

* PwmClient:

  * setTargets()
  * step()（限斜率 + AB 分组 + 心跳前置检查）
  * emergencyStop()
  * status 管理

---

# 5. 安全机制

系统内建多重硬件保护策略，用于应对水下高风险环境。

### 1. 限斜率保护（slope limit）

* 每周期最大变化约 **0.2–0.5%**
* 避免电流瞬间拉高和推进器反向冲击

### 2. AB 分组输出

* CH1–4、CH5–8 分两组交替更新
* 防止 STM32 同时处理 8 路导致电源尖峰

### 3. 中位反跳保护（deadband crossing protection）

* 禁止 PWM 直接从“前进”跳到“后退”
* 必须先经过中位（7.5%）

### 4. 心跳与超时保活

* 上位机 >500 ms 无心跳
* 下位机自动回中位 + 上位机应激处理

### 5. CRC 校验和数据完整性

* 所有帧均带 CRC16，丢包/损坏数据自动丢弃

### 6. 急停（Emergency Stop）

* 平滑归中，避免水下推进器瞬停造成姿态失稳

---

# 6. 构建方式（CMake）

顶层统一构建：

```bash
mkdir build && cd build
cmake ..
make -j4
```

输出内容：

```
orangepi_send/libpwm_host.a
pwm_control_program/pwm_control_program
```

运行：

```bash
./pwm_control_program/pwm_control_program
```

---

# 7. 测试流程（实验室验证）

### 1. 单通道测试（阻塞式）

按键：`1~8`

验证：

* 占空比从 7.5% → 8% → 回中位
* 电流变化平滑无冲击

### 2. 多通道混合动作

按键组合：

* `WASD` 平面运动
* `QE` 航向旋转
* `GH` 垂直控制
* `RT / FV` 姿态测试

验证：

* AB 分组正常工作
* 限斜率限制生效
* 无反向跳跃

### 3. 通信错误模拟

* 拔掉网线
* 修改 IP/端口
* 观察“心跳超时→自动回中”是否正常

---

# 8. 工程价值

### 1. 可插拔架构

你可以轻松插入：

* 自己的 MPC 控制器
* LSTM 控制器
* RL 策略
* 外部定深/定姿算法

### 2. 工业级安全基础设施

适用于：

* 科研型 ROV
* 工业无人船/水下机器人
* 多推进器平台

### 3. 已验证的稳定性

在多个真实实验中通过验证，包括：

* 水槽实验

* 长时间运行稳定性测试

---

# 9. 未来扩展

* [ ] MPC 控制器接入（位置/姿态跟踪）
* [ ] ROV 运动学 + 动力学模型
* [ ] IMU/DVL/深度计融合
* [ ] USBL 定位与闭环控制
* [ ] 日志系统（状态机时间戳）
* [ ] WebUI 上位机控制界面
* [ ] AUV 自主控制模块

---

# 10. 致谢

本系统由 **wys + 阿智（AI 伙伴）**
在大量硬件调试、算法迭代和工程验证中共同打造。

---

