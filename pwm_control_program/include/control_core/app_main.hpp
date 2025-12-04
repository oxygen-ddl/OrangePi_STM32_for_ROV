// include/control_core/app_main.hpp
#pragma once

/**
 * @file app_main.hpp
 * @brief pwm_control_program 的高层入口封装
 *
 * 设计目的：
 *   - 将具体的初始化逻辑（PwmClient / InputProvider / Controller / ControlLoop）
 *     隔离在 control_core 内部；
 *   - 顶层 main.cpp 只需要调用 app_main(argc, argv)，便于后续嵌入到更大系统；
 *   - 将来如果要支持多种运行模式（Teleop / PID / MPC），可以在这里扩展参数解析。
 */

namespace rovctrl::control_core {

/**
 * @brief 程序主入口
 *
 * @param argc  命令行参数个数
 * @param argv  命令行参数数组
 *
 * 支持的命令行（当前简化版本）：
 *   --loop-hz <Hz>        主循环频率（缺省 100）
 *   --pwm-hz <Hz>         安全层配置给 PwmClient 的 ctrl_hz（缺省 100）
 *   --max-step <pct>      每步最大占空比变化（缺省 0.2）
 *
 * 返回值：
 *   0   正常退出
 *   非0 发生错误（初始化失败 / 控制循环异常退出）
 */
int app_main(int argc, char** argv);

} // namespace rovctrl::control_core
