#pragma once
#ifndef ROVCTRL_IO_TELEOP_KEYBOARD_H
#define ROVCTRL_IO_TELEOP_KEYBOARD_H
/**
 * @file    teleop_keyboard.h
 * @brief   键盘 → ROV 手动控制的按键映射模块（C ABI 稳定接口）
 *
 * 模块定位（IO 输入层）
 * ------------------------------------------------------------------
 * teleop_keyboard 位于控制系统的“人机输入层”：
 *
 *   io/
 *     teleop_keyboard.h   ← 当前模块（纯键盘版本的输入后端）
 *
 * 职责：
 *   - 解析键盘输入（单个按键事件）；
 *   - 维护内部 DOF 档位（surge/sway/heave/yaw 的 [-1,1] 归一化命令）；
 *   - 触发“纯 roll / 纯 pitch”测试模式（在 DOF 空间内体现）；
 *
 * 不负责：
 *   - 控制主循环调度（周期性 tick）；
 *   - 传感器 / 导航状态读取；
 *   - 控制算法（PID / MPC / SMC 等）；
 *   - 推力分配 / PWM 映射 / pwm_control 调用；
 *
 * 使用方式：
 *   - 上层（例如 TeleopInputProvider）负责：
 *       * 调用 pwm_teleop_handle_key() 处理按键；
 *       * 调用 pwm_teleop_get_state() 获取当前 DOF 状态；
 *       * 将 DOF 状态写入 ControlReference::dof_cmd；
 *   - 实际的 DOF→推进器映射与 PWM 下发由控制器和 ControlLoop 负责。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* 枚举：返回值语义                                                         */
/* ------------------------------------------------------------------------- */

/**
 * @brief teleop 键处理函数的返回状态
 *
 * 注意：
 *   - pwm_teleop_handle_key() 实际返回 int；
 *   - 上层应使用此枚举来解读返回值；
 *   - <0 表示内部错误（例如参数非法），不涉及 pwm_control。
 */
typedef enum {
    PWM_TELEOP_IGNORED      = 0,  /**< 按键未被 Teleop 使用，主程序可自行处理           */
    PWM_TELEOP_HANDLED      = 1,  /**< 已处理（正常 Teleop 行为，不要求退出主循环）     */
    PWM_TELEOP_EXIT_REQUEST = 2   /**< 请求退出主程序（当前仅 ESC 使用）               */
} pwm_teleop_result_t;

/* ------------------------------------------------------------------------- */
/* DOF 命令状态结构（归一化 [-1,1]）                                        */
/* ------------------------------------------------------------------------- */

/**
 * @brief Teleop 内部维护的六自由度命令状态（归一化空间）
 *
 * 语义约定（需与控制栈对齐）：
 *   - cmd_surge : X 轴，前进(+) / 后退(-)
 *   - cmd_sway  : Y 轴，右移(+) / 左移(-)
 *   - cmd_heave : Z 轴，上升(+) / 下潜(-)
 *   - cmd_yaw   : 航向右转(+) / 左转(-)
 *
 *   - cmd_roll  : 横滚，当前仅在“纯 roll”测试模式下赋值，平时为 0
 *   - cmd_pitch : 俯仰，当前仅在“纯 pitch”测试模式下赋值，平时为 0
 *
 * 数值域：
 *   - 所有字段只在 [-1, 1] 范围内变化；
 *   - 实际占空比映射和限幅在控制器 / thrust allocator 内完成。
 *
 * 典型用途：
 *   - 调试 / 日志：上层可读取当前 Teleop 档位状态；
 *   - 作为手动模式 ControlReference::dof_cmd 的来源。
 */
typedef struct {
    float cmd_surge;  /**< X 轴：前进(+) / 后退(-) */
    float cmd_sway;   /**< Y 轴：右移(+) / 左移(-) */
    float cmd_heave;  /**< Z 轴：上升(+) / 下潜(-) */
    float cmd_yaw;    /**< 航向：右转(+) / 左转(-) */

    float cmd_roll;   /**< 横滚触发动作（纯 roll 模式下使用） */
    float cmd_pitch;  /**< 俯仰触发动作（纯 pitch 模式下使用） */
} pwm_teleop_state_t;

/* ------------------------------------------------------------------------- */
/* API 声明                                                                  */
/* ------------------------------------------------------------------------- */

/**
 * @brief 打印键位帮助信息（输出到 stdout）
 *
 * 建议：
 *   - 在程序启动时或用户按 Z 键时调用；
 *   - 提醒操作人员进行空载 / 无桨叶测试，避免误操作风险。
 */
void pwm_teleop_print_help(void);

/**
 * @brief 重置 Teleop 内部 DOF 状态（仅归一化命令，不操作 PWM）
 *
 * 行为：
 *   - 清零所有归一化 DOF 命令（surge/sway/heave/yaw/roll/pitch 全部置 0）；
 *
 * 说明：
 *   - 若需要同时将推进器 PWM 回中位，应在上层调用 PwmClient::setAllMid()
 *     或 PwmClient::emergencyStop()，而非由本模块直接操作硬件。
 */
void pwm_teleop_reset(void);

/**
 * @brief 查询当前 Teleop DOF 状态（只读，不下发 PWM）
 *
 * @param[out] out_state  调用方提供的结果存放结构体指针
 *
 * 注意：
 *   - 若 out_state 为 NULL，本函数不进行任何操作；
 *   - 本函数只读取内部状态，不触发任何 PWM 变化。
 */
void pwm_teleop_get_state(pwm_teleop_state_t* out_state);

/**
 * @brief 核心入口：处理单个键盘按键事件
 *
 * @param key 单次按键的字符编码（典型地来自 getchar() / _getch()）
 *
 * @return
 *   - PWM_TELEOP_IGNORED      : Teleop 未使用该按键，主程序可转交给其它逻辑
 *   - PWM_TELEOP_HANDLED      : 已处理（更新了内部 DOF 状态）
 *   - PWM_TELEOP_EXIT_REQUEST : 用户请求退出（ESC）
 *   - <0                      : 内部错误码（参数非法等）
 *
 * 键位约定（默认实现）：
 *   - 数字键 '1'..'8'：单电机阻塞测试触发（建议仅在实验/调试模式下启用）
 *   - W/S : surge 档位 ±1 step
 *   - A/D : sway  档位 ±1 step
 *   - Q/E : yaw   档位 ±1 step
 *   - H/G : heave 档位 ±1 step
 *   - R/T : 纯横滚（清零其它 DOF，仅用 roll 轴）
 *   - F/V : 纯俯仰（清零其它 DOF，仅用 pitch 轴）
 *   - M   : 全部 DOF 清零（调用 pwm_teleop_reset）
 *   - Z   : 打印帮助信息
 *   - ESC : 请求退出（返回 PWM_TELEOP_EXIT_REQUEST）
 *
 * 时序说明：
 *   - 本函数不会调用 pwm_ctrl_step() 或 PwmClient；
 *   - 只更新内部“归一化 DOF 档位”；
 *   - 真正的限斜率 / AB 保护 / PWM 下发由上层控制循环负责。
 */
int pwm_teleop_handle_key(int key);

#ifdef __cplusplus
}
#endif

#endif /* ROVCTRL_IO_TELEOP_KEYBOARD_H */
