#pragma once
#ifndef ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
#define ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP

/**
 * @file    manual_controller.hpp
 * @brief   手动模式控制器：将 DOF 命令映射为 8 路推进器指令
 *
 * 职责：
 *   - 读取 ControlReference 中的手动 DOF 命令（通常来自键盘 Teleop）；
 *   - 将 surge / sway / heave / yaw / roll / pitch 映射到 8 个推进器归一化指令 [-1, 1]；
 *   - 填写 ControlOutput::thruster_command，并设置 has_thruster_command = true；
 *
 * 不负责：
 *   - 读取键盘 / 上位机（由 InputProvider 完成）；
 *   - PWM 下发 / 限斜率 / AB 分组（由 PwmClient + pwm_control 完成）。
 */

#include "controllers/controller_base.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::controllers {

class ManualControllerConfig {
public:
    // 各 DOF → 推进器映射时的增益（可在 config/controller_manual.yaml 里配）
    double surge_gain  = 1.0;   ///< 前进/后退
    double sway_gain   = 1.0;   ///< 左右侧移
    double heave_gain  = 1.0;   ///< 上浮/下潜
    double yaw_gain    = 1.0;   ///< 航向
    double roll_gain   = 1.0;   ///< 横滚（主要用于纯 roll 测试）
    double pitch_gain  = 1.0;   ///< 俯仰（主要用于纯 pitch 测试）

    // 输出归一化指令的限幅（通常保持 1.0 即 [-1,1]）
    double max_cmd_abs = 1.0;
};

class ManualController : public ControllerBase {
public:
    explicit ManualController(const ManualControllerConfig& cfg = ManualControllerConfig());
    ~ManualController() override = default;

    /**
     * @brief 计算推进器指令
     *
     * 输入：
     *   - state : 当前导航状态（本控制器暂不使用，可为将来扩展预留）
     *   - ref   : 控制参考（这里主要使用 ref.surge/sway/heave/yaw/roll/pitch）
     *   - dt    : 控制周期（秒），当前实现中未使用
     *
     * 输出：
     *   - out.thruster_command   : 归一化 8 推进器指令 [-1,1]；
     *   - out.has_thruster_command = true；
     */
    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                          dt) override;

private:
    ManualControllerConfig cfg_;

    // 对 thruster_command 做统一限幅
    void clamp_output(rovctrl::control_core::ControlOutput& out) const;
};

} // namespace rovctrl::controllers

#endif // ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
