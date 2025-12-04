
---

## 1. 变更摘要（相较于原版 PWM 模块）

当前工程在原始 `orangepi_send + pwm_control.c` 的基础上，做了以下关键升级与重构：

1. **控制链路分层更清晰**

   * 底层仍由 **`pwm_control.c + libpwm_host`** 负责限斜率 / AB 分组 / 越零保护 / UDP 下发
   * 新增上层 C++ 控制程序 **`pwm_control_program`**，负责：

     * 控制主循环 `ControlLoop`
     * 键盘 Teleop 输入 `TeleopInputProvider`
     * 手动控制器 `ManualController`（6DOF → 8 推进器）

2. **统一时间基 (timebase)**

   * C 层和 C++ 控制层都围绕“单调时间戳”设计时间基接口：

     * 用于控制主循环定时
     * 为后续 IMU / DVL / 其他传感器数据对齐做准备

3. **Teleop 键盘控制逻辑重构**

   * 由纯 C 版本（直接操作 PWM）的 teleop，升级为：

     * C 层：`teleop_keyboard` 维护 **6DOF 归一化命令**
     * C++：`TeleopInputProvider` 负责键盘读取与状态映射
     * C++：`ManualController` 负责 DOF → 8 推进器布局映射
   * 支持 **多自由度叠加控制（overlay）**，操作体验更接近手柄
   * 大动作键（纯 roll / 纯 pitch）仍保持为**独占触发**，作为工程安全保护

4. **控制频率由主循环统一驱动**

   * 底层 PWM 安全层仍按配置的 `ctrl_hz` 计算最大斜率
   * 实际的 `step()` 调用频率由上层 `ControlLoop` 精确控制（默认目标 100 Hz）
   * AB 交替更新策略在高层频率统一后表现更稳定、可预期

5. **日志体系预留扩展点**

   * PWM 层、控制层都为后续日志记录（CSV/二进制）预留设计：

     * 可以记录：时间戳、DOF 指令、8 路推力指令、实际 PWM 等
     * 推荐保存路径：`logs/` 目录（如 `logs/pwm_teleop_*.csv`）

---

## 2. 具体改动项

### 2.1 统一时间基 (Timebase)

**C 层 PWM 模块侧**

* 位置（旧工程保持不变时）：`external/timebase/timebase.c/.h`（如仍在使用）
* 职责：

  * 提供稳定、单调的时间戳接口（避免受到系统时钟跳变影响）
  * 支撑：

    * 心跳超时判定
    * 内部超时保护逻辑

**C++ 控制层（`pwm_control_program`）侧**

* 位置：`pwm_control_program/src/platform/timebase.cpp`
* 头文件：`pwm_control_program/include/platform/timebase.hpp`
* 职责：

  * 提供 `now_ns()` 等接口，为 `ControlLoop` 提供精确周期控制
  * 后续可被 IMU / DVL / 导航系统共享，用于时间对齐

> 统一思想：**控制循环与传感器系统尽量基于同一时间参考（单调时间）工作**。

---

### 2.2 PWM 日志记录（设计与推荐实践）

> 说明：在当前版本中，日志接口是**架构预留**，可以按需要逐步落地，不强制。

推荐做法：

* 在 `pwm_control_program` 控制循环内，在每次调用 `pwm_.step()` 之后记录一行日志
* 推荐字段示例：

| 字段               | 说明                            |
| ---------------- | ----------------------------- |
| `t_s`            | 控制循环启动以来的时间（单位：秒）             |
| `dof_*`          | 当前 6DOF 归一化指令（surge/sway/...） |
| `thruster[0..7]` | 控制器输出的 8 路归一化推力指令             |
| `pwm_step_ok`    | `pwm_.step()` 是否成功            |

* 推荐路径与命名：

```text
pwm_control_program/logs/
  ├─ pwm_teleop_YYYY-mm-dd_HH-MM-SS.csv
  ├─ pwm_auto_YYYY-mm-dd_HH-MM-SS.csv
```

* 对原有 C 版 `pwm_teleop` 程序：

  * 如仍使用，可保持旧有的日志命名规则
  * 也可以统一改为上述格式，便于后续数据融合

---

### 2.3 键盘控制逻辑重构（Overlay + 安全保护）

**原逻辑**

* Teleop 直接对 PWM 通道占空比进行调整（叠加逻辑与安全逻辑部分耦合）

**新逻辑拆分**

1. C 层：`io/teleop_keyboard.c/.h`

   * 只维护一个内部的 `pwm_teleop_state_t`：

     * `cmd_surge / cmd_sway / cmd_heave / cmd_yaw / cmd_roll / cmd_pitch`
     * 所有值在 `[-1, 1]` 区间
   * 处理键盘按键事件：

     * 叠加类按键：W/S/A/D/Q/E/G/H
     * 大动作按键：R/T/F/V（纯 roll / 纯 pitch），触发时清空其它 DOF
     * 单电机测试：'1'..'8'（仍然走阻塞式测试逻辑）
     * 功能按键：M 清零，Z 打印帮助，ESC 请求退出

2. C++ 层：`TeleopInputProvider`（`src/io/teleop_input.cpp`）

   * 负责：

     * 设置/恢复终端 raw 模式
     * 非阻塞读取键盘按键
     * 调用 `pwm_teleop_handle_key(key)`
     * 调用 `pwm_teleop_get_state()` 获取 DOF 状态并写入 `ControlReference::dof_cmd`

3. C++ 层：`ManualController`

   * 负责：

     * 把 `dof_cmd` 中的 6DOF 命令映射到 8 路推进器指令 `ThrusterArray`
     * 结果写入 `ControlOutput`，由 `ControlLoop` 交给 `PwmClient`

**Overlay 叠加逻辑**

* 可叠加类按键：

  * `W` / `S` → surge ±
  * `A` / `D` → sway ±
  * `Q` / `E` → yaw ±
  * `H` / `G` → heave ±
* 典型组合示例：

  * `W + A + E` → 前进 + 左移 + 右转
  * `S + H` → 后退 + 上浮

**大动作键（独占模式）**

* `R/T`：纯 roll
* `F/V`：纯 pitch
* 行为：

  * 触发时先 `reset_dof_cmds()`，将 surge/sway/heave/yaw 清零
  * 再仅设置 `roll` 或 `pitch` = ±1
  * 这类动作**不与其他 DOF 叠加**，作为高风险动作的安全措施

---

### 2.4 控制频率改为由主循环统一驱动

**原逻辑（老版 C 程序）**

* `pwm_control` 内部有自己的 `sleep + step` 循环，通过估算周期设定最大斜率

**当前逻辑**

* `pwm_control.c` 仍根据配置 `ctrl_hz` 计算 `max_step_pct`

* 但真正的调用节奏改为：

  * C++ `ControlLoop` 按固定频率调度
  * 每个周期执行：

    * `input_->poll(...)`
    * `controller_->compute(...)`
    * `pwm_.setTargets(...)`
    * `pwm_.step()`

* 建议配置：

  * ControlLoop 目标频率：100 Hz
  * AB 分组后单个物理通道有效刷新频率：约 50 Hz

效果：

* 控制逻辑和安全层逻辑节拍统一
* 便于与 IMU(100 Hz)、DVL(10 Hz) 等传感器对齐
* 日后引入 MPC 时对采样周期假设更可靠

---

## 3. 使用与集成说明

### 3.1 底层 PWM 模块（orangepi_send）

适用于：只想做低层 **通信 + PWM 安全层验证** 的场景。

```bash
cd OrangePi_STM32_for_ROV
mkdir -p build && cd build
cmake ..
make -j4
# 将生成 orangepi_send 相关测试程序 / 静态库 libpwm_host.a
```

部分项目中仍可保留 `pwm_teleop` 这类 C 版小工具，用于单机联调与示波器测试。

---

### 3.2 上层控制程序（pwm_control_program）

适用于：**完整控制流程联调**（键盘 → 控制器 → 安全层 → STM32）。

```bash
cd OrangePi_STM32_for_ROV
mkdir -p build && cd build
cmake ..
make -j4
# 将生成：
#   orangepi_send_build/libpwm_host.a
#   pwm_control_program/pwm_control_program
```

运行示例：

```bash
cd pwm_control_program
./pwm_control_program   # 使用 config/pwm_client.yaml 中的 IP/端口 等配置
```

当前版本默认：

* 输入源：键盘 Teleop（TeleopInputProvider + teleop_keyboard）
* 控制器：ManualController（6DOF → 8 Thrusters）
* 输出：通过 PwmClient → pwm_control.c → libpwm_host → STM32

---

## 4. 键位控制速查表（当前实现）

| 键位        | 类型    | 行为说明                   |
| --------- | ----- | ---------------------- |
| `1..8`    | 单电机测试 | 调用 C 层测试逻辑，阻塞式单通道占空比变化 |
| `W` / `S` | 水平控制  | surge ±（前进/后退）         |
| `A` / `D` | 横向控制  | sway ±（左移/右移）          |
| `Q` / `E` | 航向控制  | yaw ±（左转/右转）           |
| `H` / `G` | 垂向控制  | heave ±（上浮/下潜）         |
| `R` / `T` | 大动作   | 纯 roll，独占触发，清空其他 DOF   |
| `F` / `V` | 大动作   | 纯 pitch，独占触发，清空其他 DOF  |
| `M`       | 清零    | 所有 DOF 清零（不直接操作 PWM）   |
| `Z`       | 帮助    | 打印 Teleop 使用说明         |
| `ESC`     | 退出    | 请求退出，由控制循环统一做停机与清理     |

---

## 5. 工程注意事项与后续协同

1. **测试时务必拆桨 / 无负载**

   * 尤其在验证大动作键（roll/pitch）与单电机测试逻辑时

2. **建议保持 ControlLoop 目标频率为 100 Hz**

   * 便于与 IMU 100 Hz 对齐
   * AB 分组后单个通道有效更新约 50 Hz

3. **底层 PWM 接口是稳定 ABI**

   * 上层可以放心替换控制算法（PID/MPC/SMC）和输入源（键盘 / 上位机 / 自动轨迹）

4. **如果后续将 MPC 控制加入**

   * 不需要更改 PWM 安全层和键盘模块
   * 只在 Controller 层与 ControlLoop 层做扩展即可
   * 日志字段可以直接复用当前预留格式，便于做对比实验与回放分析

---
