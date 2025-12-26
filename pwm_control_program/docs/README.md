下面给你一份**可直接落地、面向工程团队使用的 `docs/README.md` 索引模板**。
我按你们目前的项目成熟度，把**读者角色、阅读路径、文档边界**一次性理清，避免新人“翻文档迷路”。

你可以直接复制使用，后续只需要补充链接或微调说明文字。

---

# PWM Control Program – Documentation Index

本文档用于说明 `pwm_control_program/docs/` 目录中各类文档的**用途、适用对象与推荐阅读顺序**，帮助不同角色快速定位所需信息。

---

## 1. 文档面向对象（请先确认你的角色）

在阅读具体文档前，请先确认你属于以下哪一类角色：

* **A. 现场操作 / 调试人员**
  负责启动程序、使用遥控（Teleop）、进行安全测试与下水实验。

* **B. 控制算法 / 系统开发人员**
  负责控制逻辑（PID / MPC / Trajectory）、推力分配、系统架构设计。

* **C. 系统维护 / 新成员**
  负责理解整体架构、配置文件含义、参数修改影响范围。

---

## 2. 推荐阅读路径

### 2.1 现场操作 / 调试人员（A）

**目标**：安全启动程序，完成 Teleop 与基础测试，避免误操作。

**推荐阅读顺序：**

1. **`pwm_teleop_user_manual.md`**

   * Teleop 的完整用户手册
   * 键盘控制方式、模式切换、停止逻辑
   * *必读*

2. **`pwm_test_procedures.md`**

   * 工程级 PWM 安全测试流程
   * 推进器上电、干跑、下水前检查清单
   * *必读*

3. **`pwm_teleop_usage.md`**

   * 快速操作指南（命令、常见流程）
   * 适合现场查阅

4. **`PWM.md`**（如涉及硬件排障）

   * PWM 信号、电气层、通道说明
   * 用于硬件相关问题定位

---

### 2.2 控制算法 / 系统开发人员（B）

**目标**：理解控制闭环、主循环时序、推力分配与控制模式扩展方式。

**推荐阅读顺序：**

1. **`pwm_control_architecture.md`**

   * PWM 控制程序整体架构
   * 控制层 / IO 层 / 平台层划分
   * *核心文档*

2. **`architecture.svg`**

   * 模块关系与数据流示意图
   * 配合架构文档阅读

3. **`control_loop_sequence.svg`**

   * 控制主循环时序
   * 对应 `control_loop.cpp` 实现逻辑

4. **`docs/test/control_algorithm_development_guide.md`**

   * 控制算法开发与调试建议
   * PID / 控制策略测试流程

5. **`docs/test/pid_test_guide.md`**（如涉及 PID 调参）

   * PID 参数测试方法与经验说明

---

### 2.3 系统维护 / 新成员（C）

**目标**：快速理解项目边界、配置结构、文档体系，不误改关键逻辑。

**推荐阅读顺序：**

1. **本文件：`docs/README.md`**

   * 文档全局索引与阅读指引
   * *起点*

2. **`pwm_control_architecture.md`**

   * 理解系统整体职责与边界

3. **`pwm_test_procedures.md`**

   * 理解系统的安全保护设计与工程约束

4. **（建议）`config_reference.md`**

   * 配置文件字段说明（如已提供）

---

## 3. 文档分类说明

### 3.1 架构与设计文档

* `pwm_control_architecture.md`
* `architecture.svg`
* `control_loop_sequence.svg`

> 描述系统设计思想、模块边界与数据流，不涉及具体操作步骤。

---

### 3.2 操作与使用文档

* `pwm_teleop_user_manual.md`
* `pwm_teleop_usage.md`

> 面向实际操作者，关注“怎么用”“按什么顺序做”。

---

### 3.3 安全与测试文档

* `pwm_test_procedures.md`
* `docs/test/control_algorithm_development_guide.md`
* `docs/test/pid_test_guide.md`

> 面向工程安全、算法验证与实验规范。

---

### 3.4 硬件 / 协议说明

* `PWM.md`

> 面向 PWM 信号、电气接口、底层协议说明。

---

## 4. 维护说明

* 文档应与代码版本保持同步更新；
* 架构性变更请优先更新：

  * `pwm_control_architecture.md`
  * `architecture.svg`
* 操作流程或安全策略变更，请同步更新：

  * `pwm_test_procedures.md`
  * `pwm_teleop_user_manual.md`

---

## 5. 建议新增文档（可选）

以下文档建议在后续阶段补充，以提升可维护性：

* `config_reference.md`

  > 详细说明 `config/` 下各 YAML 文件字段含义、单位、取值范围与影响面。

* `troubleshooting.md`

  > 常见启动失败、PWM 无输出、通信异常的排查流程。

---

