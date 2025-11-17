#ifndef PWM_TELEOP_KEYS_H
#define PWM_TELEOP_KEYS_H

/**
 * @file    pwm_teleop_keys.h
 * @brief   键盘→电机测试 & 运动控制的按键映射模块
 *
 * 依赖：
 *   - libpwm_host.h
 *   - pwm_control.h
 *
 * 约定：
 *   - 外部负责初始化 libpwm_host / pwm_control；
 *   - 外部负责读取键值（非阻塞 read / getchar），然后交给 handle_key；
 *   - 本模块不直接操作 termios，只做“键→动作”的逻辑。
 */

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 打印键位帮助信息到 stdout
 */
void pwm_teleop_print_help(void);

/**
 * @brief 处理一个按键（单字节字符）
 *
 * @param ch    按键（如 'w','1','q' 等），传入 int 方便兼容 EOF
 *
 * 行为分类：
 *   - '1'..'8': 对应通道的“单电机测试”（阻塞约 3~4 秒）：
 *        7.5% → 9%（受限斜率平滑上升）保持 3s → 再平滑回 7.5%
 *   - 'W','A','S','D','M','Q','E' 等：设置“运动模式”，
 *        内部通过 pwm_ctrl_set_targets_mask() 设置各通道目标占空比，
 *        由 pwm_ctrl_step() 在主循环中平滑逼近。
 *
 * 返回值：
 *   >0 : 成功处理了一个已知按键
 *    0 : 未识别的按键（忽略）
 *   <0 : 错误（例如未初始化、底层返回错误码）
 *
 * 注意：
 *   - 数字键测试是阻塞操作，适合实验室示波器/单机测试；
 *   - 运动键位是非阻塞的，只是修改目标，实际输出由主循环 step()。
 */
int pwm_teleop_handle_key(int ch);

#ifdef __cplusplus
}
#endif

#endif /* PWM_TELEOP_KEYS_H */
