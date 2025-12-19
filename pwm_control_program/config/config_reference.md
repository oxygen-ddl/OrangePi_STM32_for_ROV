下面给你一份**工程级、可长期维护的 `docs/config_reference.md` 设计稿**。
我按你们现在的 **控制 → 推力分配 → PWM 硬件 → 轨迹** 四个层次来组织，目标是：

* 新人**只改参数不改代码**
* 任何字段都能回答三个问题：**干什么 / 单位是什么 / 改了会影响什么**
* 明确哪些参数是**危险参数**

你可以直接原样落盘使用，后续只需要补具体数值。

---

# Configuration Reference

`pwm_control_program/config`

本文档用于说明 `config/` 目录下各 YAML 配置文件的**字段含义、单位、取值范围及修改影响**。
**在未充分理解字段作用前，请勿随意修改配置。**

---

## 0. 配置文件总览

| 文件名                   | 职责             | 修改风险  |
| --------------------- | -------------- | ----- |
| `control_params.yaml` | 控制算法与控制主循环参数   | 中     |
| `trajectory.yaml`     | 期望轨迹与参考输入      | 低     |
| `pwm_client.yaml`     | PWM 下发、硬件映射与保护 | **高** |

---

## 1. `control_params.yaml`

> **控制层核心配置**
> 影响控制频率、控制器行为、限幅与安全策略。

---

### 1.1 控制主循环参数（Control Loop）

```yaml
control_loop:
  loop_hz: 100
  max_step_pct: 0.2
```

| 字段             | 含义             | 单位  | 说明            |
| -------------- | -------------- | --- | ------------- |
| `loop_hz`      | 控制主循环频率        | Hz  | 应与 PWM 控制频率一致 |
| `max_step_pct` | 单周期 PWM 最大变化比例 | 0–1 | 防止突变冲击推进器     |

**注意事项：**

* `loop_hz` 改变需同步检查：

  * Teleop 输入
  * 控制器参数（尤其 PID）
* `max_step_pct` 过大可能导致：

  * 电流冲击
  * 推进器抖动

---

### 1.2 控制模式配置（Controllers）

```yaml
controller:
  default_mode: PID
```

| 字段             | 含义           |
| -------------- | ------------ |
| `default_mode` | 程序启动时的默认控制模式 |

支持模式应与 `controllers/` 中已实现的控制器一致，例如：

* `MANUAL`
* `PID`
* （预留）`MPC`

---

### 1.3 PID 控制器参数（如启用）

```yaml
pid:
  kp: [1.0, 1.0, 1.0, 0.8, 0.8, 0.6]
  ki: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  kd: [0.1, 0.1, 0.1, 0.05, 0.05, 0.05]
```

| 字段   | 含义   | 维度    |
| ---- | ---- | ----- |
| `kp` | 比例增益 | 6 DOF |
| `ki` | 积分增益 | 6 DOF |
| `kd` | 微分增益 | 6 DOF |

**DOF 顺序约定（示例）：**

```
[x, y, z, roll, pitch, yaw]
```

⚠️ **危险提示**
PID 参数调节请遵循 `docs/test/pid_test_guide.md`，禁止直接在水中大幅修改。

---

## 2. `trajectory.yaml`

> **轨迹与期望参考输入配置**
> 与硬件无关，用于测试、调试和轨迹跟踪。

---

### 2.1 轨迹类型

```yaml
trajectory:
  type: hold
```

支持类型示例：

* `hold`：定点保持
* `line`：直线轨迹
* `circle`：圆轨迹
* `custom`：用户自定义

---

### 2.2 定点保持（示例）

```yaml
trajectory:
  type: hold
  position: [0.0, 0.0, -1.0]
  attitude: [0.0, 0.0, 0.0]
```

| 字段         | 含义    | 单位  |
| ---------- | ----- | --- |
| `position` | 期望位置  | m   |
| `attitude` | 欧拉角姿态 | rad |

---

### 2.3 轨迹时间参数（如适用）

```yaml
time:
  duration: 30.0
```

| 字段         | 含义     |
| ---------- | ------ |
| `duration` | 轨迹持续时间 |

---

## 3. `pwm_client.yaml`（⚠️ 高风险配置）

> **PWM 下发与硬件映射配置**
> 错误配置可能直接损坏推进器或 ESC。

---

### 3.1 PWM 基本参数

```yaml
pwm:
  pwm_min: 1100
  pwm_max: 1900
  pwm_neutral: 1500
```

| 字段            | 含义     | 单位 |
| ------------- | ------ | -- |
| `pwm_min`     | 最小 PWM | μs |
| `pwm_max`     | 最大 PWM | μs |
| `pwm_neutral` | 空转 PWM | μs |

---

### 3.2 推进器映射（Logical → Physical）

```yaml
thrusters:
  - id: 1
    pwm_channel: 1
    reverse: false
  - id: 2
    pwm_channel: 2
    reverse: true
```

| 字段            | 含义          |
| ------------- | ----------- |
| `id`          | 逻辑推进器编号     |
| `pwm_channel` | 物理 PWM 输出通道 |
| `reverse`     | 是否反向        |

⚠️ **危险提示**
修改 `reverse` 后必须执行干跑测试。

---

### 3.3 安全保护参数（AB 通路）

```yaml
safety:
  enable_step_limit: true
  enable_output_clamp: true
```

| 字段                    | 含义           |
| --------------------- | ------------ |
| `enable_step_limit`   | 启用单步变化限制     |
| `enable_output_clamp` | 启用 PWM 上下限保护 |

---

## 4. 参数修改风险等级说明

| 等级 | 含义         |
| -- | ---------- |
| 低  | 仅影响轨迹或调试行为 |
| 中  | 影响控制性能，需谨慎 |
| 高  | 可能导致硬件损坏   |

---

## 5. 修改配置前的强制检查清单

修改任意配置前，请确认：

* [ ] 当前在 **干跑 / 上架** 状态
* [ ] 已备份原始配置文件
* [ ] 清楚该字段的影响范围
* [ ] 已阅读对应文档说明

---

## 6. 相关文档索引

* 控制架构：`pwm_control_architecture.md`
* 控制时序：`control_loop_sequence.svg`
* 安全测试流程：`pwm_test_procedures.md`
* PID 调试指南：`docs/test/pid_test_guide.md`

---

## 7. 维护说明

* 新增配置字段必须同步更新本文件；
* 字段语义变更必须记录在 CHANGELOG 中；
* 禁止“代码里偷偷吃字段、不写文档”。

---

如果你愿意，下一步我可以直接帮你做两件**非常实用**的事之一：

1. **把这份文档与你们现有 YAML 完整对齐（字段级别）**
2. **设计一个 `config_validate` 启动校验机制（哪些字段缺失直接拒绝启动）**

你选一个，我直接继续。
