
---

# **机器人控制算法与底层安全层、驱动层的协同工作机制说明（含项目架构）**

## 1. 文档目的

本文件用于说明：

1. **上层控制算法（PID/MPC/SMC）** 如何与
2. **底层安全层（PWM Protection Layer）** 以及
3. **驱动层（libpwm_host / STM32）**

协作，以实现安全、可控、高鲁棒性的推进器控制链路。

同时，本文件包含本项目当前的文件架构，用于新成员理解整个系统的模块边界。

---

# 2. 项目文件架构（截至当前版本）

项目根目录：

```
D:\UnderwaterRobotSystem\OrangePi_STM32_for_ROV
```

整体结构如下：

```
.
|   README.md
|   命令行.txt
|
+---OrangePi_STM32_for_ROV
|   |   README.md
|   |
│   +---docs
│   |       handover_guide.md
│   |       protocol_v1.md
│   |       pwm_control_layer_safety_overview.md
│   |       test_plan_en.md
│   |
│   \---orangepi_send
│       |   CMakeLists.txt
│       |   README.md
│       |
│       +---include
│       |       crc16_ccitt.h
│       |       libpwm_host.h
│       |       protocol_pack.h
│       |       protocol_pack.hpp
│       |       PwmFrameBuilder.h
│       |       pwm_control.h              ← 安全层头文件
│       |       UdpSender.h
│       |
│       \---src
│               crc16_ccitt.cpp
│               libpwm_host.c             ← 驱动层（下发帧）
│               main.cpp
│               protocol_pack.c
│               PwmFrameBuilder.cpp
│               pwm_control.c             ← 安全层核心实现（本文件文档主要描述对象）
│               UdpSender.cpp
|
+---pwm_control_program
|   |   CMakeLists.txt
|   |   pwm_control_update.md
|   |   README.md
|   |
|   +---include
|   |       pwm_teleop_keys.h
|   |       timebase.h
|   |
|   \---src
|           main.cpp                      ← 控制算法层/Teleop 入口
|           pwm_teleop_keys.cpp
|           timebase.cpp
|
\---tools
        project_size_report.py
        project_size_report.txt
```

## 分层说明（关键点）

| 层级                        | 所在目录                              | 功能                                             |
| ------------------------- | --------------------------------- | ---------------------------------------------- |
| **控制算法层**                 | `pwm_control_program/`            | Teleop、未来 PID/MPC/SMC 都在此层运行（逻辑电机空间 motor_pct） |
| **安全层（Protection Layer）** | `orangepi_send/src/pwm_control.c` | 统一保护电机：斜率限制、反向保护、映射、限幅、分组更新                    |
| **驱动层（Driver Layer）**     | `orangepi_send/src/libpwm_host.c` | UDP/serial 下发占空比、心跳 ACK、数据帧校验                  |

---

# 3. 控制系统三层结构图

```
┌──────────────────────────────┐
│  上层控制算法层              │
│  (PID / MPC / SMC / Teleop)  │
│   输出：motor_pct[8]         │
└─────────────┬────────────────┘
              │ 逻辑电机空间（不考虑反向/通道映射）
              ▼
┌──────────────────────────────┐
│  PWM 安全层（pwm_control.c） │
│   - 100Hz 内环               │
│   - 斜率限制 max_step_pct    │
│   - 禁止突然反向             │
│   - min/mid/max 安全限幅     │
│   - 逻辑→物理通道映射        │
│   - motor_reverse 反向支持   │
│   - AB 分组 → ESC 实际 50Hz  │
│   输出：物理 PWM 通道占空比  │
└─────────────┬────────────────┘
              │ 硬件安全的 PWM 百分比帧
              ▼
┌──────────────────────────────┐
│   底层驱动层 libpwm_host     │
│   - UDP/串口发帧              │
│   - CRC 校验                  │
│   - 心跳/ACK                  │
└─────────────┬────────────────┘
              ▼
┌──────────────────────────────┐
│      STM32 PWM 输出层        │
│      ESC 驱动 → 电机         │
└──────────────────────────────┘
```

---

# 4. 安全层功能清单（核心逻辑）

安全层内部通过 `pwm_ctrl_step()` 实现推进器保护，这一层是整个控制栈最关键的防护环节。

## 4.1 控制频率与 ESC 匹配

* 安全层频率：**100 Hz**
* AB 分组交替更新：**每路电机 50 Hz**
* 精确匹配 ESC 推荐刷新频率（50 Hz）

## 4.2 八大安全机制

### 1）斜率限制（max_step_pct）

限制单步占空比变化幅度（例如 0.2%），避免：

* 电机突然加减速
* ESC 电流冲击
* 机械结构冲击

### 2）禁止突然反向（越零保护）

如果当前在 mid 上方，而目标在 mid 下方：

* 本步目标自动设为 mid
* 下一步才允许向反方向运动

可防止“瞬间反转”，避免损坏推进器与 ESC。

### 3）逻辑→物理通道映射（motorch_to_pwmch）

支持配置任意电机拓扑：

* 控制算法永远用“逻辑 motor 1..8”
* 安全层映射到真实 PWM 通道

### 4）电机方向反转（motor_reverse）

用于处理螺旋桨左右旋/安装角度特性。

### 5）min/mid/max 限幅保护

自动裁剪非法占空比，例如：

* 低于 5%
* 高于 10%
* mid 不在 min/max 中间

避免误配置导致 ESC 输出不可预测 PWM。

### 6）分组控制（group_mode）

为了控制电源负载、匹配动态：

* A 组更新（如 1–4）
* B 组更新（如 5–8）
* 循环往复 → 50 Hz 每通道

### 7）初始化与中位保护

系统启动时：

* 所有通道输出 mid（7.5%）
* 不会出现随机 PWM 杂声

### 8）安全急停（emergency_stop）

在规定时间内（如 0.5秒）平滑回中位。

---

# 5. 上层控制算法如何正确使用安全层？

## 5.1 原则：控制算法不直接面向硬件

控制算法必须工作在 **逻辑电机空间 motor_pct[8]**：

* 不关心逻辑电机→物理通道映射
* 不关心反向标志
* 不关心 PWM 限幅或安全规则
* 只需要输出“期望占空比 motor_pct[i]”

**以下内容禁止上层控制器实现：**

* PWM 斜率限制
* 越零保护
* 分组控制
* 物理通道映射
* 最终下发 `pwm_host_set_all_pct()`

这些规则已经由安全层强制保证，避免重复逻辑或冲突。

---

## 5.2 上层控制程序标准调用流程

典型的控制主循环：

```c
// 初始化驱动层
pwm_host_init(&host_cfg);

// 初始化安全层
pwm_ctrl_init(&ctrl_cfg);  // ctrl_hz = 100 Hz

while (1) {

    // 1. 读取状态
    ControlState state = read_state_from_ESKF();

    // 2. 读取参考（轨迹、指令）
    ControlRef ref = read_ref();

    // 3. 控制算法（PID / MPC / SMC）
    float motor_pct[8];
    controller_update(&state, &ref, motor_pct);

    // 4. 写入安全层目标
    pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, motor_pct);

    // 5. 安全层执行一步（AB 交替 → 每电机 50 Hz）
    pwm_ctrl_step();

    // 6. 使用驱动层处理心跳与 ACK
    pwm_host_poll(1);
}
```

> 控制器运行 30–50 Hz
>
> 安全层运行 100 Hz 内部循环
>
> ESC 实际见到 50 Hz 输出（AB 分组）

---

# 6. 控制算法适配方式（PID / MPC / SMC）

因为安全层已经做好所有 PWM 级别保护，三种控制器都可以遵循同一接口：

### 输入

* `ControlState`：由导航系统提供 IMU/DVL/ESKF 状态
* `ControlRef`：期望位置/速度/姿态/力等

### 输出

* **motor_pct[8]**（逻辑电机空间）

控制器只需专注于动力学或轨迹跟踪：

| 控制器 | 上层职责        | 下层职责（安全层）         |
| --- | ----------- | ----------------- |
| PID | 简单反馈调节      | 斜率限制、越零保护         |
| MPC | 最优控制（约束、预测） | min/mid/max 限幅、映射 |
| SMC | 强鲁棒控制       | 硬件保护，防反冲          |

**两者互不冲突。**

---

# 7. 异常处理与急停

### 7.1 急停

```c
pwm_ctrl_emergency_stop(0.5f);
```

安全层以受控斜率将所有通道平滑恢复到中位。

常见触发：

* IMU/DVL 状态跳变
* 上层控制算法崩溃
* 上位机人工按键
* 通信中断

### 7.2 控制器崩溃的系统行为

若上层不再调用 `pwm_ctrl_set_target_*()`：

* 安全层保持最近的安全占空比（不会随机下发）
* 必要时可触发自动中位

---

# 8. 为什么必须用这种三层架构？

### 主要工程优势：

1. **硬件保护强制化**
   无论上层控制器出现 bug、数据飘移、时间错乱，安全层保证 PWM 不会突然跳变。

2. **控制算法可自由替换**
   PID → MPC → SMC 只需替换上层模块，不影响底层安全。

3. **调试更简单**

   * 上层调性能
   * 中层保安全
   * 下层保通讯
     各层边界分明。

4. **可扩展性强**
   支持多模式控制，例如 Teleop → 自动控制，不担心硬件风险。

---

# 9. 最终总结

在本项目中：

* **控制算法层** 负责控制性能
* **安全层（pwm_control.c）** 负责电机保护、防反转、斜率限制、占空比安全保证
* **驱动层（libpwm_host）** 负责通讯可靠性
* 三者协同构成一条**可信、鲁棒、安全的推进器控制链路**

这种结构保证：

* 控制算法随时可换
* 安全策略不会被上层覆盖
* PWM 输出永远是合法、安全、可预测的
* 机器人整体系统达到工程级可靠性

---
