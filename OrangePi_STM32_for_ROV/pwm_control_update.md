
````markdown
# PWM 安全层更新说明（pwm_control 安全封装）

版本：v1（与新版 `pwm_control.h/.cpp`、`teleop_keyboard.cpp` 对应）  
适用工程：`OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV`

---

## 1. 文档目的与适用范围

本说明文档用于介绍 **PWM 控制安全层（`pwm_control`）** 的最新设计与更新内容，帮助：

- 新成员理解：我们在 PWM 控制侧做了哪些安全保护；
- 控制算法同学理解：如何在不绕开安全层的前提下输出 4-DOF 指令；
- 硬件 / 调试同学理解：紧急情况如何快速、安全地停机。

涉及模块：

- `pwm_control.h / pwm_control.cpp`  
- `libpwm_host`（底层 UDP 通讯库）  
- STM32 端 PWM 输出程序（协议接收 + 8 路 PWM）  
- 键盘控制模块 `io/teleop_keyboard.*`（间接受影响）

---

## 2. 分层结构回顾

整体链路：

```text
控制算法（PID / MPC / Manual） → 4-DOF 命令
      ↓
pwm_control（安全层：限斜率 + 分组 + 反向保护 + 映射）
      ↓
libpwm_host（打包协议 + UDP 发送）
      ↓
STM32（解析协议 + 输出 8 路 PWM）
      ↓
ESC / 电机
````

关键原则：

1. **控制算法永远工作在“逻辑电机空间 + DOF 空间”，不直接碰裸 PWM**。
2. **所有 PWM 变动必须经过 `pwm_control_step()` 的限斜率与安全策略**。
3. **物理接线差异、反向、电机编号变更，都通过配置完成，不侵入控制逻辑。**

---

## 3. 核心更新点概览

本版本 `pwm_control` 相比旧版的主要升级包括：

1. 引入 **逻辑电机 / 物理 PWM 映射**：

   * 逻辑电机编号：`1..8`（代表“前左水平”、“后右垂向”等语义）
   * 物理 PWM 通道：`1..8`（STM32 引脚对应）

   通过 `pwm_ctrl_config_t.motorch_to_pwmch[8]` 做映射。

2. 支持 **电机反向配置 + 运行时开关**：

   * 配置结构体 `motor_reverse[8]`（上电即生效）
   * 运行时 API：`pwm_ctrl_set_motor_reverse(motor_id, enable)`

3. 完整的 **限斜率 + 禁止突然反向** 逻辑：

   * `max_step_pct` 控制单步占空比最大变化量（%）
   * `enable_reverse_protection`：正推 → 反推必须先过中位（7.5%）

4. 支持 **AB 分组交替更新**：

   * `group_mode` = `PWM_CTRL_GROUP_MODE_AB_ALTERNATE`
   * `groupA_mask` / `groupB_mask` 控制每一组逻辑通道
   * 一次只动一半电机，减小瞬时电流冲击

5. 提供 **4-DOF 统一入口**：

   * 新增 `pwm_ctrl_dof4_cmd_t` + `pwm_ctrl_set_targets_from_dof4()`
   * 接收 `surge/sway/heave/yaw ∈ [-1,1]`，内部映射成 8 通道目标占空比
   * 和键盘 teleop 使用同一套混控语义，便于控制器对齐

6. 完整的 **紧急停机接口**：

   * `pwm_ctrl_emergency_stop(seconds)`：在指定时间内平滑拉回中位
   * `pwm_ctrl_hold_pct_blocking()`：单电机阻塞测试（台架 / 示波器用）

---

## 4. 逻辑电机与物理 PWM 映射

### 4.1 逻辑电机编号语义（建议约定）

在控制栈中，我们推荐约定：

```text
逻辑电机 1..4：水平平面推进器
  CH1: 前左 (FL)
  CH2: 前右 (FR)
  CH3: 后左 (RL)
  CH4: 后右 (RR)

逻辑电机 5..8：垂向推进器（十字 / 对角布局）
  CH5: 前左垂向
  CH6: 前右垂向
  CH7: 后右垂向
  CH8: 后左垂向
```

实际接线不同，只需要在配置中调整映射，不需要修改任何控制代码。

### 4.2 配置示例

```c
pwm_ctrl_config_t cfg = {0};

cfg.ctrl_hz      = 50.0f;
cfg.max_step_pct = 0.2f;

cfg.group_mode   = PWM_CTRL_GROUP_MODE_AB_ALTERNATE;
cfg.groupA_mask  = PWM_CH_MASK_1_4;  // 水平组
cfg.groupB_mask  = PWM_CH_MASK_5_8;  // 垂向组

// 逻辑 1..8 → 物理 PWM 通道（例如：5,6,7,8,1,2,3,4）
int map[8] = {5,6,7,8,1,2,3,4};
memcpy(cfg.motorch_to_pwmch, map, sizeof(map));

// 若有电机安装反向，在此标记为 1:
cfg.motor_reverse[2] = 1;  // 逻辑 3 号电机反向
```

内部会做合法性检查：

* 映射必须覆盖 1..8 且无重复
* 否则回退成默认 1..8 对 1..8

---

## 5. 限斜率与禁止突然反向

### 5.1 限斜率（max_step_pct）

* `max_step_pct`：单步最大占空比变化量（%）
* 配合上层 `ctrl_hz` 得到“每秒最大变化量”

示例：

```c
cfg.ctrl_hz      = 50.0f;
cfg.max_step_pct = 0.2f;
// => 每秒最多变化 0.2% * 50 = 10%，非常柔和
```

上层每个周期必须调用：

```c
pwm_ctrl_step();  // 典型频率：50 Hz 或 100 Hz
```

只修改目标不调用 `step()`，电机不会动。

### 5.2 禁止突然反向（enable_reverse_protection）

场景：从前进 9.0% 直接给到后退 6.0%，如果不保护，可能在一两步内跨越中位，造成机械冲击和大电流。

逻辑：

* 当前占空比在中位上方，目标在下方（或反之）
* 本步“有效目标”被夹到中位
* 通过连续多步 `step()` 实现：正推 → 中位 → 反推

效果：在 `max_step_pct` 约束下，不会出现“直接穿过 7.5%”的危险瞬间。

---

## 6. AB 分组交替更新

为了进一步降低瞬时电流与供电冲击：

* 启用 `PWM_CTRL_GROUP_MODE_AB_ALTERNATE`
* `groupA_mask`：例如逻辑 CH1~CH4
* `groupB_mask`：逻辑 CH5~CH8

在这种模式下：

* 第一次 `pwm_ctrl_step()` 更新 group A（1..4）
* 第二次更新 group B（5..8）
* 如此往复

如果上层循环 100 Hz，那么每个通道实际更新频率 ~50 Hz，配合 STM32/ESC 的控制节奏。

---

## 7. 4-DOF 高层接口（控制栈入口）

为方便控制算法统一调用，安全层提供：

```c
typedef struct {
    float surge;  // 前(+)/后(-)   [-1,1]
    float sway;   // 右(+)/左(-)   [-1,1]
    float heave;  // 上(+)/下(-)   [-1,1]
    float yaw;    // 左(+)/右(-)   [-1,1]
} pwm_ctrl_dof4_cmd_t;

int pwm_ctrl_set_targets_from_dof4(const pwm_ctrl_dof4_cmd_t* cmd);
```

特点：

1. **控制器只关心 DOF 命令，不关心 8 路 PWM 分配**。
2. 内部用一套与键盘 teleop 一致的混控逻辑构造 8 路目标占空比。
3. 仍然只是“设置目标”，不会立刻下发，必须配合周期性的 `pwm_ctrl_step()`。

典型用法（在控制循环中）：

```c
pwm_ctrl_dof4_cmd_t cmd;
cmd.surge = surge_cmd;  // [-1,1]
cmd.sway  = sway_cmd;
cmd.heave = heave_cmd;
cmd.yaw   = yaw_cmd;

pwm_ctrl_set_targets_from_dof4(&cmd);  // 设置目标
pwm_ctrl_step();                       // 限斜率 + 映射 + 下发
```

---

## 8. 键盘 Teleop 与安全层的关系

当前版本中：

* `teleop_keyboard.cpp` 仍然直接调用 `pwm_ctrl_set_targets_mask()` 设置目标占空比；
* 内部 DOF→电机映射逻辑与 `pwm_ctrl_set_targets_from_dof4()` 保持一致风格。

后续计划：

1. Teleop 层只维护 `g_cmd_surge/sway/heave/yaw`。
2. 在每次按键后，构造 `pwm_ctrl_dof4_cmd_t`，调用 `pwm_ctrl_set_targets_from_dof4()`。
3. 最终形成：**键盘手动控制与控制器输出走同一条安全链路**。

这样可以保证：

* 手动 / 自动控制切换时，电机映射、反向、限斜率、分组等行为完全一致；
* 更容易定位问题（只剩上层控制逻辑不同）。

---

## 9. 紧急停机与测试接口

### 9.1 紧急归中位

```c
int pwm_ctrl_emergency_stop(float seconds);
```

含义：

* 将所有逻辑电机目标设为中位；
* 在 `seconds` 指定时间内，通过多次 `step()` 平滑拉回中位；
* `seconds <= 0` 时，根据当前偏差与 `max_step_pct` 自动估算需要的步数。

适用场景：

* 程序退出前的安全处理；
* 检测到上层控制异常、导航失效时的“受控软停机”。

### 9.2 单电机阻塞测试

```c
int pwm_ctrl_hold_pct_blocking(int ch, float pct, float seconds);
```

功能：

* 修改单个逻辑电机的目标占空比；
* 在指定时间内内部循环调用 `step()`，并适当 `sleep`；
* 用于示波器、台架单电机测试，不建议在实时控制循环中调用。

---

## 10. 初始化与使用顺序（规范）

推荐顺序：

1. 初始化底层通讯库：

   ```c
   pwm_host_init(&host_cfg);
   ```

2. 配置并初始化安全层：

   ```c
   pwm_ctrl_config_t cfg = {0};
   // 填 cfg.ctrl_hz / max_step_pct / group_mode / 映射 / 反向...
   pwm_ctrl_init(&cfg);
   ```

3. 控制循环中：

   * 控制器输出 4-DOF 命令：`pwm_ctrl_set_targets_from_dof4()`
   * Teleop 模式：键盘修改内部 DOF 状态 → 同样调用该接口（计划）
   * 固定频率调用 `pwm_ctrl_step()`（50~100 Hz）

4. 退出前：

   ```c
   pwm_ctrl_emergency_stop(1.0f);  // 1 秒平滑归中
   pwm_ctrl_deinit();
   pwm_host_deinit();              // 若有需要
   ```

---

## 11. 迁移指南（从旧版本到新版）

如果已有旧代码调用 `pwm_control`，迁移时请注意：

1. **检查头文件是否更新**：

   * 新增 `pwm_ctrl_dof4_cmd_t` / `pwm_ctrl_set_targets_from_dof4()`
   * 确保没有旧的“直接按 PWM 通道号发命令”的代码残留。

2. **在配置文件 / 初始化代码中明确 motor 映射：**

   * 不再默认为“逻辑 1..8 就是物理 1..8”，应明确写出映射
   * 记录在文档和图纸中：逻辑电机编号 → ROV 实际安装方位 → 物理通道

3. **统一 DOF 语义**：

   * 控制器输出、键盘 teleop、仿真中都使用统一的 `surge/sway/heave/yaw` 定义。
   * 一旦 DOF 定义确定，后续尽量避免频繁修改。

4. **验证紧急停机行为**：

   * 上水前，用空载/台架验证：`pwm_ctrl_emergency_stop()` 确实可以在期望时间内平滑归中。

---

## 12. 总结

新版 PWM 安全层的设计目标是：

* 把“如何安全驱动 8 路电机”这一问题 **封装在一个稳定的 C API 里**；
* 上层控制算法只需要关心：

  * 我要给 ROV 什么 4-DOF 命令？
  * 控制周期是多少？
* 所有“电机映射、电机反向、限斜率、分组更新、紧急停机”逻辑在这一层统一管理。

只要遵守：

* 不绕开 `pwm_control` 直接下裸 PWM
* 在固定频率下调用 `pwm_ctrl_step()`

就可以在后续长期迭代控制算法时，最大化复用当前这套工程化安全基础设施。

```

