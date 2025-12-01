#ifndef ROVCTRL_PLATFORM_IO_TELEOP_KEYBOARD_H
#define ROVCTRL_PLATFORM_IO_TELEOP_KEYBOARD_H
/**
 * @file    teleop_keyboard.h
 * @brief   键盘 → ROV 推进控制的按键映射模块（C ABI 稳定接口）
 *
 * 模块定位（Platform / IO 层）
 * ------------------------------------------------------------------
 * teleop_keyboard 位于控制系统的“平台输入层”：
 *
 *   platform/
 *     io/
 *       teleop_keyboard.h   ← 当前模块（纯键盘版本的控制入口）
 *
 * 职责：
 *   - 解析键盘输入（单个按键事件）
 *   - 维护内部 DOF 档位（surge/sway/heave/yaw 的 [-1,1] 归一化命令）
 *   - 触发“纯 roll / 纯 pitch”特殊姿态模式
 *   - 通过底层安全层 pwm_control 设置 PWM 目标（set_targets_from_dof 等）
 *
 * 不负责：
 *   - 控制主循环调度（周期性 tick）
 *   - 传感器 / 导航状态读取
 *   - 控制算法（PID / MPC / SMC 等）
 *   - thrust allocation 详细矩阵（位于更上层的控制栈）
 *
 * 使用前提：
 *   - 顶层主循环需要以固定频率调用 pwm_ctrl_step()，
 *     Teleop 只负责修改“目标值”，不直接驱动硬件。
 *
 * ABI 特性：
 *   - 提供 C linkage（extern "C"），便于 C / C++ 混合工程与长期稳定。
 *   - 将 Teleop 作为“人机输入后端”，后续上层可换成 UDP / 跨进程控制而保持 ABI 不变。
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
 *   - <0 表示底层错误（通常来自 pwm_control / libpwm_host）。
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
 *   - 实际占空比映射和限幅在 pwm_control 内完成。
 *
 * 典型用途：
 *   - 调试 / 日志：上层可读取当前 Teleop 档位状态；
 *   - 未来可以作为 ControlOutput 的一种来源（手动模式）。
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
 * @brief 重置 Teleop 内部 DOF 状态 + 将所有 PWM 通道目标设为中位
 *
 * 行为：
 *   - 清零所有归一化 DOF 命令（surge/sway/heave/yaw/roll/pitch 全部置 0）；
 *   - 调用底层 pwm_control，将所有通道目标设至 7.5% 中位；
 *
 * 典型使用场景：
 *   - 控制模式切换（例如 Teleop → PID / MPC / RL）前后，确保安全过渡；
 *   - 实验前/后，将 ROV 收回“安全静止状态”；
 *   - 遇到异常（但尚不需要紧急停机）时的人为复位。
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
 *   - PWM_TELEOP_HANDLED      : 已处理（可能修改 DOF，并调用了 pwm_control 设置目标）
 *   - PWM_TELEOP_EXIT_REQUEST : 用户请求退出（ESC）
 *   - <0                      : 底层错误码（pwm_control / libpwm_host 等）
 *
 * 键位约定（默认实现）：
 *   - 数字键 '1'..'8'：单电机阻塞测试（7.5% → 8.0% 保持 → 7.5%）
 *   - W/S : surge 档位 ±1 step
 *   - A/D : sway  档位 ±1 step
 *   - Q/E : yaw   档位 ±1 step
 *   - H/G : heave 档位 ±1 step
 *   - R/T : 纯横滚（清零其它 DOF，仅用垂向推进器做 roll）
 *   - F/V : 纯俯仰（清零其它 DOF，仅用垂向推进器做 pitch）
 *   - M   : 全部 DOF 清零 + PWM 回中位（调用 pwm_teleop_reset）
 *   - Z   : 打印帮助信息
 *   - ESC : 请求退出（返回 PWM_TELEOP_EXIT_REQUEST）
 *
 * 时序说明：
 *   - 本函数不会调用 pwm_ctrl_step()；
 *   - 只更新“目标占空比”和 Teleop 内部 DOF 状态；
 *   - 真正的平滑输出、AB 保护、电气安全节奏由上层周期循环负责。
 */
int pwm_teleop_handle_key(int key);

#ifdef __cplusplus
}
#endif

#endif /* ROVCTRL_PLATFORM_IO_TELEOP_KEYBOARD_H */
