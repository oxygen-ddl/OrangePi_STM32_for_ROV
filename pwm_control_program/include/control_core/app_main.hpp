#pragma once
/**
 * @file app_main.hpp
 * @brief pwm_control_program 的高层入口封装
 *
 * 设计目的：
 *   - 将初始化逻辑集中封装（PwmClient / InputProvider / Controller / ControlLoop）；
 *   - 顶层 main.cpp 只调用 app_main(argc, argv)；
 *   - 未来扩展 Teleop、PID、MPC 等控制模式时都从这里统一入口。
 */

#include <string>

#include "control_core/thruster_allocation.hpp"   // 声明 ThrusterAllocationConfig

namespace rovctrl::control_core {

/**
 * @brief 解析 YAML 中的 thruster allocation 配置
 *
 * @param path YAML 文件路径
 * @param cfg  输出解析后的配置
 * @return true  解析成功
 * @return false 解析失败
 *
 * 注意：app_main.cpp 会调用该接口，因此必须在头文件中声明。
 */
bool load_thruster_allocation_from_yaml(
    const std::string& path,
    ThrusterAllocationConfig& cfg);

/**
 * @brief 程序主入口（由 main.cpp 调用）
 *
 * 支持命令行参数：
 *   --config <yaml>       PWM 安全层配置文件
 *   --alloc <yaml>        推力分配配置文件（解析 ThrusterAllocationConfig）
 *   --loop-hz <Hz>        主控制循环频率
 *   --pwm-hz <Hz>         PWM 安全层内部频率
 *   --max-step <pct>      每步 PWM 最大变化量限制
 *
 * @return 0 成功退出；非 0 表示错误。
 */
int app_main(int argc, char** argv);

} // namespace rovctrl::control_core
