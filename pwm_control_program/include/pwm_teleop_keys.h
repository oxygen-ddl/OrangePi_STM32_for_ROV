#ifndef PWM_TELEOP_KEYS_H
#define PWM_TELEOP_KEYS_H

/**
 * @file    pwm_teleop_keys.h
 * @brief   键盘 → ROV 推进控制的按键映射模块
 *
 * 职责：
 *   - 将键盘按键映射为：
 *       1) 单电机阻塞测试（数字 1..8）
 *       2) 多自由度叠加控制命令（surge / sway / yaw / heave）
 *       3) 特殊姿态动作（纯 roll / 纯 pitch）
 *   - 通过调用 pwm_control 接口，更新各通道的“目标占空比模式”；
 *   - 实际 PWM 平滑输出仍由上层主循环周期性调用 pwm_ctrl_step() 完成。
 *
 * 依赖：
 *   - libpwm_host.h
 *   - pwm_control.h
 *
 * 使用约定：
 *   - 外部负责：
 *       * 完成 libpwm_host / pwm_control 的初始化和收尾；
 *       * 在主循环中读取键值（非阻塞 read / getchar），并调用 pwm_teleop_handle_key()；
 *       * 周期性调用 pwm_ctrl_step() 下发 PWM；
 *   - 本模块：
 *       * 不直接操作 termios；
 *       * 不负责循环与 sleep，仅做“键 → 控制命令／占空比目标”的逻辑。
 */

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 打印键位帮助信息到 stdout
 *
 * 内容包括：
 *   - 数字键 1..8 的单电机测试说明（阻塞式）；
 *   - W/A/S/D/Q/E/H/G 的多自由度叠加控制规则：
 *       surge（前后）、sway（左右）、yaw（航向）、heave（上下）；
 *   - R/T/F/V 的“纯姿态模式”说明（只做 roll / pitch，不叠加其他 DOF）；
 *   - M/Z/ESC 等通用键说明。
 */
void pwm_teleop_print_help(void);

/**
 * @brief 处理一个按键（单字节字符）
 *
 * @param ch
 *    - 键盘按键（如 'w','1','q' 等），使用 int 是为了兼容 EOF；
 *    - 本函数内部会自动将 'a'..'z' 转为大写处理。
 *
 * 行为分类概述：
 *   1) 数字键 '1'..'8'：
 *       - 对应通道单电机测试（阻塞约 3~4 秒）：
 *           7.5% → 9%（受限斜率平滑上升）保持 3 s → 再平滑回 7.5%
 *       - 适合示波器、单电机台架测试。
 *
 *   2) 多自由度叠加控制键（非阻塞）：
 *       - W / S : 调整 surge 档位（前进 / 后退）
 *       - A / D : 调整 sway 档位（右移 / 左移）
 *       - Q / E : 调整 yaw  档位（左转 / 右转）
 *       - H / G : 调整 heave 档位（上浮 / 下潜）
 *       - 内部维护四个归一化命令：
 *           g_cmd_surge / g_cmd_sway / g_cmd_yaw / g_cmd_heave ∈ [-1, 1]
 *       - 每次按键增减一个“档位步长”，再调用 pwm_ctrl_set_targets_mask()
 *         为 8 路通道生成合成后的占空比目标模式；
 *       - 实际 PWM 输出由上层循环调用 pwm_ctrl_step() 平滑逼近。
 *
 *   3) 特殊姿态键（纯动作，不叠加其他 DOF）：
 *       - R / T : 横滚（Roll Right / Roll Left）
 *       - F / V : 俯仰（Pitch Forward / Pitch Backward）
 *       - 触发时会：
 *           * 清零 g_cmd_surge/sway/yaw/heave；
 *           * 所有通道先回中位；
 *           * 再仅用垂向推进器构造“纯 roll / 纯 pitch”的目标占空比；
 *       - 用于安全的横滚／俯仰测试，避免与其他 DOF 叠加导致异常姿态。
 *
 *   4) 通用键：
 *       - M : 清零所有 DOF 命令，并将所有通道目标设为中位（7.5%）；
 *       - Z : 打印键位帮助；
 *       - ESC(27) : 请求退出（返回值特别约定为 2）。
 *
 * 返回值约定：
 *   >0 :
 *       - 已成功处理该按键；
 *       - 其中返回值 2 有特殊含义：表示“请求主程序退出”：
 *           * 当前实现中仅 ESC 键返回 2，供主循环识别后 break。
 *    0 :
 *       - 按键未被本模块识别／使用（上层可继续使用该按键做其他功能）；
 *   <0 :
 *       - 错误（例如底层 pwm_control 调用失败等），具体含义参考 pwm_control / libpwm_host。
 *
 * 注意事项：
 *   - 数字键单电机测试是阻塞操作，期间主循环不会处理其他按键；
 *   - 运动键和姿态键是非阻塞的，只是修改目标占空比（或 DOF 命令），
 *     实际 PWM 由上层固定频率调用 pwm_ctrl_step() 发出；
 *   - 建议在首次上电测试时拆下螺旋桨或在空载环境中进行。
 */
int pwm_teleop_handle_key(int ch);

#ifdef __cplusplus
}
#endif

#endif /* PWM_TELEOP_KEYS_H */
