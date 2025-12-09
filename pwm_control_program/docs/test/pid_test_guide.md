下面我为你准备了一份**面向测试人员、结构清晰、可直接交付团队的专业文档**。
适用于你的当前架构（ControlLoop + ControllerManager + PID 控制 + 推力分配 + PWM 下发 + OrangePi + STM32）。

文档内容包括：

* 项目背景
* PID 控制模块的作用
* 测试人员需要做的事情
* 如何更新 PID 控制代码
* 如何在本地编译验证
* 如何在 OrangePi 上部署运行
* 如何执行实地（水池）测试
* 如何记录数据并判断 PID 效果
* 如何提交代码更新与问题报告

你可直接将文档放在项目根目录的 `docs/pid_test_guide.md` 中。

---

# **PID 控制模块测试与实验验证指南（面向测试人员）**

**项目：pwm_control_program + underwater navigation + OrangePi + STM32**

---

## 1. 文档目的

此文档用于指导测试人员执行以下任务：

1. **理解 PID 控制模块在整个系统中的作用**
2. **按照标准流程修改 / 调整 PID 参数或控制逻辑**
3. **在本地成功编译与运行控制程序**
4. **在 OrangePi 上进行部署并执行实地（水池）实验**
5. **评估 PID 控制效果并记录数据用于后续模型训练与参数优化**

测试人员无需深度理解 MPC、导航算法、推力分配等代码，只需根据本指南完成 PID 更新与验证流程即可。

---

## 2. 系统结构（测试人员需要知道的部分）

整个控制系统由以下模块组成：

### (1) **输入层 Input Provider**

* 键盘 / 自动指令生成参考量 reference（6DOF 期望）
* Teleop 模式：用于手动输入进行测试
* 自动模式：可切换 PID 控制

### (2) **控制器层 Controller**

* ManualController：人工手动控制
* PidController：本次测试的核心模块
* ControllerManager：控制器选择与统一调度

### (3) **推力分配 ThrusterAllocator**

* 将 PID 控制输出的 6DOF 力/力矩映射为 8 通道推进器推力
* 自动完成列重排、伪逆求解、推力限幅

### (4) **PWM 安全层 + STM32**

* 将 8 路归一化指令映射为 PWM，占空比由 STM32 驱动电机

### (5) **运行环境**

* 控制程序运行在 OrangePi（Ubuntu）
* 下游由 STM32 运行 PWM 驱动
* 上游可通过导航程序提供反馈（可选）

---

## 3. PID 控制模块测试人员需要做的事情

测试人员需要完成：

1. 拉取最新代码分支
2. 修改 PID 控制代码（主要包括 PID 参数和 step_axis 逻辑）
3. 本地编译调试
4. 上传到 OrangePi
5. 执行测试（陆上测试 → 水池测试）
6. 记录 IMU、电流、电压、导航状态、控制量等数据
7. 分析控制效果（超调、稳态误差、响应速度）
8. 将反馈提交到 GitHub issue 或开发者指定渠道

---

## 4. 如何更新 PID 控制代码

### PID 代码位置：

```
pwm_control_program/
│── include/controllers/pid_controller.hpp
└── src/controllers/pid_controller.cpp
```

测试人员可以修改两类内容：

---

### **(1) PID 参数调整**

在 `pid_controller.hpp` 中：

```cpp
struct PidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};
```

测试人员在 YAML（control_params.yaml）中调整 PID 参数：

```
controllers:
  pid:
    gains:
      surge:  {kp: 1.2, ki: 0.1, kd: 0.05}
      sway:   {kp: 1.0, ki: 0.1, kd: 0.05}
      heave:  {kp: 3.0, ki: 0.2, kd: 0.1}
      yaw:    {kp: 2.0, ki: 0.1, kd: 0.1}
```

修改后即可直接测试。

---

### **(2) 控制逻辑 step_axis 修改**

测试人员可以修改 `step_axis()`，例如加入积分限幅、抗饱和、滤波器等：

```cpp
double PidController::step_axis(PidAxisState& st,
                                const PidGains& g,
                                double error,
                                double dt)
{
    st.integral += error * dt;
    st.integral = clamp(st.integral, -max_i, +max_i);

    double derivative = (error - st.last_error) / dt;
    st.last_error = error;

    return g.kp*error + g.ki*st.integral + g.kd*derivative;
}
```

---

## 5. 如何本地编译验证

### 环境要求

* Ubuntu / OrangePi Linux
* CMake ≥ 3.10
* g++ ≥ 9
* 必须安装 Eigen3, yaml-cpp

---

### **编译步骤**

```
cd pwm_control_program
mkdir build && cd build
cmake ..
make -j4
```

成功后生成：

```
./pwm_control_program
```

---

## 6. 如何在 OrangePi 部署运行

### **(1) 上传代码**

```
scp -r pwm_control_program orangepi@192.168.x.x:~/rov/
```

### **(2) OrangePi 编译**

```
cd ~/rov/pwm_control_program
mkdir build && cd build
cmake ..
make -j4
```

---

### **(3) 启动程序**

```
./pwm_control_program --loop-hz 100 --control-config config/control_params.yaml
```

程序启动成功后会：

* 初始化 PWM 通信
* 加载 PID 控制参数
* 进入主循环

---

## 7. 实地实验验证流程（测试人员标准操作）

以下流程必须严格执行，确保硬件安全。

---

### **步骤 1：陆上静态测试（必须）**

目的：验证 PID 输出不会异常放大导致电机冲击。

操作：

1. 设置所有 PID 参数为低值
2. 给出小幅度阶跃输入（如 surge_ref = 0.2）
3. 查看电机输出是否连续稳定
4. 检查 CPU 占用与 loop timing 是否正常

---

### **步骤 2：水池低速测试**

目的：验证单轴、单方向 PID 控制效果

操作：

1. 控制 x 方向速度（surge）保持恒定
2. 记录以下数据：

   * PID 输出 wrench
   * ThrusterAllocator 输出的 8 路推力
   * PWM 最终指令
   * IMU 加速度 & 角速度
   * 若导航模块可用：速度、深度、姿态

成功标准：

* 输出不抖动
* 方向无明显偏转
* 控制响应平滑

---

### **步骤 3：闭环位置控制测试（可选）**

如果导航模块可用：

1. 设置目标 x = 1.0 m
2. 检查超调量、稳定时间、稳态误差
3. 分析 PID 参数是否需要调整

---

### **步骤 4：数据记录与分析**

OrangePi 会自动生成：

```
logs/2025-xx-xx/xxx_pwm_log.csv
logs/2025-xx-xx/xxx_nav_state.csv
logs/2025-xx-xx/xxx_control_output.csv
```

测试人员需要：

* 上传日志到实验服务器或 GitHub
* 填写实验报告

---

## 8. PID 效果判定标准（测试人员需记录）

| 指标     | 说明             | 合格标准       |
| ------ | -------------- | ---------- |
| 上升时间   | 输出从 10%→90% 目标 | < 1 s（按轴定） |
| 超调量    | 最大偏差 / 目标值     | < 15%      |
| 稳态误差   | 稳定后偏差          | < 5%       |
| 输出是否抖动 | PID 输出变化是否平滑   | 不出现剧烈振荡    |
| 电流偏差   | 是否明显偏高         | 正常范围内      |

测试人员应根据这些指标给出“合格 / 需要优化”。

---

## 9. 如何提交代码与测试反馈

测试人员需要：

1. 将修改后的代码推送到测试分支（如 `feature/pid-tuning`）

```
git checkout -b feature/pid-update
git add .
git commit -m "Update PID parameters and step_axis logic"
git push -u origin feature/pid-update
```

2. 在 GitHub 提交 issue，包括：

   * 本次测试目的
   * 修改内容
   * 控制效果评估表
   * 日志文件链接
   * 是否建议进一步参数调整

---

## 10. 结语

这份文档可以让测试人员：

* 按标准流程修改和验证 PID 控制器功能
* 有效理解控制流程和推力分配机制
* 顺利完成水池实验并提供高质量反馈

