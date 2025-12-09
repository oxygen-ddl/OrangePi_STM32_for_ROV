下面我为你准备一份**更高层次、面向你团队的完整控制算法开发文档**。
该文档可以直接交付，让你的下属理解：

* 整体控制系统的架构与理念
* 控制算法（PID / SMC / MPC）在系统中的位置
* 控制器应如何与导航模块配合
* 控制算法与 ThrusterAllocator、PwmClient 的标准接口
* 如何编写新的控制器（如 SMC、MPC）并无缝融入整个项目
* 控制器开发的规范流程

你可放入仓库 `docs/control_algorithm_development_guide.md`。

---

# **控制算法开发总指南：从 PID 到 SMC / MPC 的统一接口与实现规范**

**（面向控制算法研发人员与测试人员）**

---

## **1. 文档目的**

本指南旨在帮助团队成员系统地理解：

1. 我们的 **控制系统总架构**
2. 当前 **PID 控制器的完整功能与代码衔接关系**
3. 控制器如何与 **导航模块（nav_core）** 配合实现闭环轨迹跟踪
4. 控制器与 **推力分配器 ThrusterAllocator**、PWM 安全层之间的接口规范
5. 如何基于 PID 框架，实现新的控制器：

   * SMC（Sliding Mode Control）
   * MPC（Model Predictive Control）
   * Adaptive MPC、L1、自抗扰控制等
6. 项目的控制框架是可扩展、可维护、可调试的

阅读本指南，团队成员将能够：

* 理解所有控制算法应该放在哪里写
* 清楚控制流程数据如何流动
* 独立开发新控制器并集成进系统
* 在实地实验中配合导航模块进行轨迹跟踪验证

---

# **2. 控制系统总体架构（关键：理解数据流）**

下面展示控制程序的数据流：

![Image](https://www.researchgate.net/publication/319179579/figure/fig2/AS%3A667845256019969%401536238125197/Block-Diagram-of-ROV-control-system.ppm?utm_source=chatgpt.com)

![Image](https://www.researchgate.net/publication/288406660/figure/fig1/AS%3A551181409677312%401508423296942/Overall-architecture-of-the-ROV-control-system.png?utm_source=chatgpt.com)

![Image](https://docs.px4.io/main/assets/mc_control_arch.DPb5OeqV.jpg?utm_source=chatgpt.com)

![Image](https://www.researchgate.net/publication/228738505/figure/fig1/AS%3A341696874336256%401458478291128/A-block-diagram-of-the-control-system-d-denotes-the-vector-of-environmental.png?utm_source=chatgpt.com)

完整流程如下（非常重要）：

```
Navigation State (nav_core)  →  ControlLoop  →  Controller (PID/SMC/MPC)
         ↓                                                 ↓
   NavStateSubscriber                         body_wrench (Fx,Fy,Fz,Mx,My,Mz)
         ↓                                                 ↓
ControlState --------------------------→  ThrusterAllocator (伪逆分配)
                                                         ↓
                                  8 Thruster Commands [-1,1]
                                                         ↓
                                       PwmClient → STM32 → Thrusters
```

核心角色职责如下：

| 模块                           | 位置            | 作用                    |
| ---------------------------- | ------------- | --------------------- |
| **NavStateSubscriber**       | io/           | 读取导航模块的最新状态（姿态、速度、位置） |
| **ControlLoop**              | control_core/ | 调用控制器并调度整个循环          |
| **Controller (PID/SMC/MPC)** | controllers/  | 根据误差生成 6DOF wrench    |
| **ThrusterAllocator**        | control_core/ | 将 wrench 映射为 8 路推力    |
| **PwmClient**                | platform/     | 进行 PWM 分组下发、安全限幅、心跳   |

理解这些模块的连接，是下属未来开发新控制算法的基础。

---

# **3. 控制算法放在哪里写？**

所有控制算法类必须放在目录：

```
pwm_control_program/controllers/
```

对应头文件：

```
include/controllers/
```

每个控制器需要：

```
xxx_controller.hpp
xxx_controller.cpp
```

目前已有：

```
manual_controller.(hpp/cpp)
pid_controller.(hpp/cpp)
controller_manager.(hpp/cpp)
```

未来你们将添加：

```
smc_controller.(hpp/cpp)
mpc_controller.(hpp/cpp)
l1_controller.(hpp/cpp)
```

只要遵循统一接口，即可立即被 ControllerManager 调用。

---

# **4. 控制器必须遵守的统一接口（必须读）**

所有控制器必须继承 `IController`：

```cpp
class IController {
public:
    virtual ~IController() = default;

    virtual ControlMode mode() const noexcept = 0;

    virtual bool compute(const ControlState& state,
                        const ControlReference& ref,
                        ControlOutput& out,
                        double dt) = 0;

    virtual void reset() noexcept = 0;
    virtual const char* name() const noexcept = 0;
};
```

任何控制器必须实现 `compute()`：

```
输入:  state → 当前导航状态(state.pos, state.vel, state.rpy)
      ref   → 期望状态(x_ref, v_ref, yaw_ref...)
      dt    → 当前控制周期

输出: out.body_wrench (Fx,Fy,Fz,Mx,My,Mz)
```

控制器只能做一件事：

> 根据误差计算 “六自由度的力 / 力矩” 输出。

这是与导航模块协作的关键。

---

# **5. 控制器如何与导航模块联动？（闭环控制核心）**

导航模块 `nav_core` 提供状态：

```
pos_ned[3]    位置
vel_ned[3]    速度
rpy[3]        姿态
omega_b[3]    角速度
acc_b[3]      加速度
depth         深度
```

控制器使用这些状态计算误差：

```
ep = ref.pos - state.pos     // 位置误差
ev = ref.vel - state.vel     // 速度误差
er = ref.yaw - state.yaw     // 姿态误差
```

然后用于 PID/SMC/MPC 的 wretch 控制律。

**任何控制算法都必须遵循这个误差结构，否则无法实现轨迹跟踪。**

---

# **6. 控制器如何与推力分配器衔接？**

控制器输出：

```
Fx, Fy, Fz     → 线力
Mx, My, Mz     → 力矩
```

ThrusterAllocator 将其映射到：

```
8 路推进器推力（归一化）
```

因此：

> 控制器永远不需要知道推进器布局，也不需要计算 PWM。

这使得控制算法完全模块化。

---

# **7. 基于 PID 的控制器模板（下属可直接参考并复制实现 SMC/MPC）**

以下是完整的控制器模板结构：

```cpp
class PidController : public IController {
public:
    ControlMode mode() const noexcept override { return ControlMode::Auto; }
    const char* name() const noexcept override { return "pid"; }

    void reset() noexcept override {
        // 清除积分项
    }

    bool compute(const ControlState& state,
                const ControlReference& ref,
                ControlOutput& out,
                double dt) override
    {
        // Step 1: 计算误差
        double ex = ref.pos[0] - state.pos[0];
        double ey = ref.pos[1] - state.pos[1];
        double ez = ref.pos[2] - state.pos[2];

        // Step 2: PID 控制律
        double Fx = compute_pid(surge_axis, ex, dt);
        double Fy = compute_pid(sway_axis,  ey, dt);
        double Fz = compute_pid(heave_axis, ez, dt);

        // Step 3: 输出 body wrench
        out.body_wrench = {Fx, Fy, Fz, 0, 0, 0};
        out.has_body_wrench = true;
        return true;
    }
};
```

**SMC / MPC 只需要替换 “控制律部分”。**

例如：

### SMC：

```
u = -k * sign(s)
s = c1*e + c2*de
```

### MPC：

```
min (x - x_ref)'Q(x - x_ref) + u'Ru
subject to system dynamics
```

**但输入与输出接口必须保持一致。**

---

# **8. 如何让新的控制器（SMC / MPC）自动融入系统？**

只需两步：

---

## **步骤 1：在 controllers/ 中创建文件**

```
smc_controller.hpp
smc_controller.cpp
```

按 PID 模板实现 compute()。

---

## **步骤 2：在 controller_manager.cpp 注册新控制器**

```cpp
controllers_["smc"] = std::make_unique<SmcController>(cfg);
```

之后即可：

* 在 YAML 中选择控制器
* 在运行时通过键盘切换模式

---

# **9. 控制算法与系统整体配合的逻辑总结**

以下是算法开发者必须牢记的“任务边界”：

| 模块                    | 你应该做什么           | 你不需要做什么           |
| --------------------- | ---------------- | ----------------- |
| **控制算法（PID/SMC/MPC）** | 计算 6DOF wrench   | 不需要关心推进器布局        |
| **导航模块**              | 提供位置/速度/姿态       | 不需要知道控制算法         |
| **ThrusterAllocator** | wrench → 8 推进器指令 | 不需要知道 PID/SMC/MPC |
| **PWM 安全层**           | 控制推力平滑、安全        | 不需要知道控制律          |

这保证了模块的高内聚低耦合。

---

# **10. 实地实验对控制算法开发的意义**

控制器研发人员必须结合导航与实地数据：

1. 使用 **导航模块输出的速度、姿态、位置**
2. 生成参考轨迹
3. 使用 PID/SMC/MPC 计算 wrench
4. 分配为 8 路推力
5. 实地验证轨迹跟踪效果
6. 基于数据调整控制参数

简而言之：

> “算法 → 数据 → 反馈 → 更新参数” 是持续改进的核心流程。

---

# **11. 开发者工作流程（给你的下属）**

下面是下属开发 SMC/MPC 的完整步骤：

---

### **步骤 1：理解当前架构（本文档）**

---

### **步骤 2：阅读 PID 控制代码**

文件：

```
include/controllers/pid_controller.hpp
src/controllers/pid_controller.cpp
```

理解 compute() 中如何：

* 获取误差
* 计算 wrench
* 写入 ControlOutput

---

### **步骤 3：创建新控制器（例如 SMC）**

```
controllers/smc_controller.hpp
controllers/smc_controller.cpp
```

参考 PID 模板。

---

### **步骤 4：Visual Studio Code 构建单元测试（可选）**

---

### **步骤 5：在 OrangePi 上测试**

---

### **步骤 6：与导航模块配合执行轨迹跟踪实验**

---

### **步骤 7：分析数据与迭代算法**

---

# **12. 结语：文档的最终目的**

通过本指南，你的下属将能够：

* 系统理解整个控制系统的模块结构
* 基于统一接口规范开发 PID/SMC/MPC/L1/MRAC 等任何控制算法
* 与导航模块协同实现闭环控制与轨迹跟踪
* 将自己的控制器无缝集成到系统中
* 完成实地实验验证

