# PWM 键盘遥控说明

## 适用场景

用于实验室或外场，从上位机（Windows/Linux/Mac）通过键盘控制 8 推进器的基础运动/姿态，测试通信 RTT、平滑斜率约束和分组更新策略。

## 动作组合控制

### 基本组合规则

1. 运动指令按键可以自由组合，同时影响的通道会合并到一帧 PWM 目标中。
2. 不允许冲突命令并发，例如 `W`（前进）和 `S`（后退）。
3. 横滚（ROLL）和翻转（FLIP）属于高风险大动作，独占触发，触发期间忽略其他按键的叠加输入。
4. 组合按键本次生效，不会锁存状态；松手后需重新按键。
5. 频率语义：
   - `pwm_ctrl_step()` 调用频率由 `cfg.ctrl_hz` 决定（推荐 100Hz）
   - 内部 A/B 组交替更新时，每路电机实际刷新约为 `ctrl_hz / 2`

### 连续更新效果示例（ctrl=100Hz, groupA=CH1-4, groupB=CH5-8）

| 上层 Step 次数 | 通道 1-4 组 | 通道 5-8 组 | 每路实际频率 |
|---|---|---|---|
| 1 | 更新 | 保持 | ~50 Hz |
| 2 | 保持 | 更新 | ~50 Hz |
| 3 | 更新 | 保持 | ~50 Hz |
| 4 | 保持 | 更新 | ~50 Hz |

## 键位定义

### 运动（可叠加）

| 按键 | 语义 | 影响通道目标（相对中位偏移） | 说明 |
|---|---|---|---|
| W | 前进 | CH3, CH4 → `+MOVE_DELTA` | 温和推力向前 |
| S | 后退 | CH1, CH2 → `+MOVE_DELTA` | 使用对称前推进器 |
| A | 右移 | CH1, CH3 → `+MOVE_DELTA` | 横向侧推（右） |
| D | 左移 | CH2, CH4 → `+MOVE_DELTA` | 横向侧推（左） |
| Q | 左偏航 | CH1, CH4 → `+MOVE_DELTA` | 船首向左转 |
| E | 右偏航 | CH2, CH3 → `+MOVE_DELTA` | 船首向右转 |
| G | 下潜 | CH5, CH8 正；CH6, CH7 反 | 垂向合力向下 |
| H | 上浮 | CH6, CH7 正；CH5, CH8 反 | 垂向合力向上 |


### 大动作（独占，禁止叠加）

| 按键 | 语义 | 覆盖所有输出 | 说明 |
|---|---|---|---|
| F | 俯仰（抬头） | CH6, CH7 → `+MOVE_DELTA`，CH5, CH8 → `-MOVE_DELTA` | 头部上仰 |
| V | 俯仰（低头） | CH5, CH8 → `+MOVE_DELTA`，CH6, CH7 → `-MOVE_DELTA` | 头部下沉 |
| R | 横滚（温和右翻） | CH5, CH8 → `+MOVE_DELTA`，CH6, CH7 → `-MOVE_DELTA` | 产生右侧滚力矩 |
| T | 横滚（温和左翻） | CH6, CH7 → `+MOVE_DELTA`，CH5, CH8 → `-MOVE_DELTA` | 产生左侧滚力矩 |

### 通用

| 按键 | 说明 |
|---|---|
| M | 所有通道目标回到中位（7.5%) |
| Z | 显示键位帮助 |
| ESC(27) | 退出遥控主循环 → 程序收尾进入 emergency stop |
| Ctrl+C(SIGINT) | 作为最高级别外部停止方式，触发 `g_running=false` |

## 斜率与保护策略

1. 所有按键改变的 `target_pct[]` 会经由 `pwm_ctrl_step()` 的**限斜率平滑逼近**，避免突变。
2. 启用 `enable_reverse_protection=1` 时，如果从正向转反向（或反之），会先平滑逼近中位，再反向。
3. AB 组默认掩码：
   - `groupA_mask = 0x0F` (CH1-4)
   - `groupB_mask = 0xF0` (CH5-8)
4. 默认模式：
   ```cpp
   group_mode = PWM_CTRL_GROUP_MODE_AB_ALTERNATE
   ctrl_hz = 100 Hz
   max_step_pct = 0.2%/step
