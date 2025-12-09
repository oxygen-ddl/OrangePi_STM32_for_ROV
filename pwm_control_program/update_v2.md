
---

# **PWM 控制系统三层结构说明（升级版）**

## **1. 变更摘要（相较于原版 PWM 模块）**

当前工程在原始 `orangepi_send + pwm_control.c` 的基础上，新增了**导航订阅功能**、**PWM 日志体系升级**等内容，并对软件模块结构进一步完善。

相比原始版本，本工程新增 / 升级了以下能力：

---

### **1. 控制链路分层更清晰**

底层仍由 **`pwm_control.c + libpwm_host`** 负责：

* 限斜率
* AB 分组
* 越零保护
* UDP 下发与心跳 ACK

上层 C++ 新增控制程序 **`pwm_control_program`**，负责：

* 控制主循环 `ControlLoop`
* 键盘 Teleop 输入 `TeleopInputProvider`
* 手动控制器 `ManualController`（6DOF → 8 推进器）
* **导航共享内存订阅 NavStateSubscriber（新）**
* **PWM Logger（cmd/applied 双轨记录）（新）**

---

### **2. 统一时间基 (timebase)**

保持原工程理念，C 层与 C++ 层共享统一时间基接口：

* C 层用于内部保护（心跳超时等）
* C++ 层用于：

  * `ControlLoop` 固定周期调度
  * 日志统一时间戳（控制循环启动以来的 t_s）

未来 IMU / DVL / USBL / 导航系统都可对齐该时间基。

---

### **3. Teleop 键盘控制逻辑彻底重构**

* 从“直接调占空比”升级为“产生 6DOF 归一化指令”
* 完全支持 **多自由度叠加**（surge+sway+yaw...）
* roll/pitch 为 **独占大动作键**
* 逻辑与安全层彻底解耦，更适合后续扩展自动控制（PID/MPC/SMC）

---

### **4. 控制频率统一由 ControlLoop 驱动**

* 安全层 `pwm_control.c` 根据 `ctrl_hz` 计算最大斜率
* 实际周期由 `ControlLoop` 保证（默认 100 Hz）
* 提升系统稳定性，并与 IMU 100 Hz 频率自然对齐

---

### **5. PWM 日志体系全面升级（新）**

新增 C++ 版本 `PwmLogger`：

* 支持两种模式：

  | 模式                  | 记录内容                     |
  | ------------------- | ------------------------ |
  | `AppliedOnly`       | 实际下发的安全层 PWM 指令          |
  | `CmdAndApplied`（默认） | 控制器输出 cmd + 实际下发 applied |

* 文件格式：

  ```
  t_s, ch1_cmd..ch8_cmd, ch1_applied..ch8_applied
  ```

此功能大幅提升实验数据可用性，为后续：

* 控制算法调参与验证
* MPC / PID / SMC 实际效果分析
* 神经网络动力学建模（cmd→accel）
  提供数据基础。

---

### **6. 导航共享内存订阅功能（NavStateSubscriber）（新增）**

控制循环现已支持读取导航程序发布的共享内存 `/rov_nav_state_v1`，内容包括：

* `pos[3]`（NED 位置）
* `vel[3]`（NED 速度）
* `rpy[3]`（roll/pitch/yaw）
* `depth`
* `status`
* `t_ns`（时间戳）

`ControlLoop` 每周期会将导航数据注入到：

```
ControlState state_
```

并由 `state_.nav_valid` 标识导航状态是否有效。

控制器（如 PID/MPC）未来可直接使用该状态闭环控制。

---

## **2. 具体改动项（含新增内容）**

---

## **2.1 统一时间基 (Timebase)**

（内容与你原稿一致，略）

---

## **2.2 PWM 日志记录（升级）**

**新增：**

### **PwmLogger 支持记录两路数据**

* 控制器输出的 8 路 `thruster_command`（cmd）
* 安全层实际下发的 8 路 PWM（applied）

### **控制循环日志调用点：**

在：

```cpp
int step_rc = pwm_.step();
```

之后调用：

```cpp
pwm_logger_.logCmdAndApplied(t_s, output_.thruster_command, applied_values);
```

未来你可以扩展：

* 记录导航状态
* 记录 IMU/DVL
* 记录控制器内部变量（如误差、MPC cost）

日志路径：

```
pwm_control_program/logs/pwm_log_*.csv
```

---

## **2.3 Teleop 键盘控制（内容同原稿）**

（略）

---

## **2.4 控制频率统一调度（内容同原稿）**

（略）

---

## **2.5 导航共享内存订阅（新增重点内容）**

控制循环现已加入完整共享内存订阅逻辑：

### **订阅初始化**

```cpp
nav_sub_.init("/rov_nav_state_v1");
```

失败时会继续运行，但 `nav_valid` 始终为 false。

---

### **周期性读取导航状态**

每个循环读取：

```cpp
shared::msg::NavState nav;
if (nav_sub_.read_latest(nav)) { ... }
```

并注入：

```cpp
state_.nav_pos_ned[i] = nav.pos[i];
state_.nav_vel_ned[i] = nav.vel[i];
state_.nav_rpy[i]     = nav.rpy[i];
state_.nav_depth      = nav.depth;
state_.nav_status     = nav.status;
state_.nav_valid      = true;
```

若持续 100 次读取失败，会输出：

```
Warning: no stable NavState for X cycles
```

---

### **控制器未来可直接使用导航状态**

依据：

```
state_.nav_valid
```

可以：

* 使用导航状态
* 或降级为 IMU-only 模式（本工程预留）

---

## **3. 使用与集成说明**

（保持原结构，此处省略）

---

## **4. 键位速查表**

（同原稿）

---

## **5. 工程注意事项（更新）**

新增两条：

### **5.6 导航共享内存必须保持与控制程序的时间基接近**

否则：

* 控制器会收到延迟状态
* logs 数据时间戳可能不对齐

建议导航程序与控制程序使用：

* 同一 CPU 绑定策略
* 同一时间基（steady_clock）

---

### **5.7 启用双轨 PWM 日志会产生更大文件**

100 Hz × 16 列 × CSV → 每小时可能 50–150 MB。

若需节省空间：

* 改为 AppliedOnly
* 改为二进制日志（未来可扩展）

---

# **文档升级完成**
