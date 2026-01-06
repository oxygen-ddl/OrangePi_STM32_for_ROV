#pragma once
#ifndef ROVCTRL_CONTROL_CORE_TELEOP_MIXER_HPP
#define ROVCTRL_CONTROL_CORE_TELEOP_MIXER_HPP

#include <array>
#include <cstddef>
#include <algorithm>

namespace rovctrl::control_core {

/**
 * @brief 手动遥控 6DOF → 8 推进器指令的混合配置
 *
 * 含义：
 *   - surge/sway/heave/yaw/roll/pitch_gain：对 ref.dof_cmd 的每个分量施加的增益；
 *   - max_cmd_abs：所有通道统一的绝对值上限（类似 ManualControllerConfig.max_cmd_abs）。
 */
struct TeleopMixConfig {
    double surge_gain  = 1.0;
    double sway_gain   = 1.0;
    double heave_gain  = 1.0;
    double yaw_gain    = 1.0;
    double roll_gain   = 1.0;
    double pitch_gain  = 1.0;
    double max_cmd_abs = 1.0;
};

/**
 * @brief 6DOF 遥控指令混合为 8 路推进器命令（与 ManualController 拓扑保持一致）
 *
 * 要求 TeleopCmd 类型具有成员：
 *   - surge, sway, heave, roll, pitch, yaw （float/double 皆可）
 *
 * @tparam TeleopCmd  包含 6DOF 字段的类型（如 ControlReference::dof_cmd）
 * @param cfg         混合增益与限幅配置
 * @param cmd         6DOF 遥控指令（通常 ∈ [-1,1]）
 * @param thr_out     输出：8 路归一化推进器指令（会被本函数完全写满）
 */
template <typename TeleopCmd>
inline void mix_6dof_to_8thrusters(const TeleopMixConfig&      cfg,
                                   const TeleopCmd&            cmd,
                                   std::array<float, 8>&       thr_out) noexcept
{
    const double surge = static_cast<double>(cmd.surge) * cfg.surge_gain;
    const double sway  = static_cast<double>(cmd.sway)  * cfg.sway_gain;
    const double heave = static_cast<double>(cmd.heave) * cfg.heave_gain;
    const double yaw   = static_cast<double>(cmd.yaw)   * cfg.yaw_gain;
    const double roll  = static_cast<double>(cmd.roll)  * cfg.roll_gain;
    const double pitch = static_cast<double>(cmd.pitch) * cfg.pitch_gain;

    thr_out.fill(0.0f);

    // 水平推进器 0–3：surge / sway / yaw 合成
    //
    // 约定（俯视，机体朝上）：
    //   0: 左后, 1: 右后, 2: 左前, 3: 右前

    // 1) surge：4 个水平推进器同向
    thr_out[0] += static_cast<float>(surge);
    thr_out[1] += static_cast<float>(surge);
    thr_out[2] += static_cast<float>(surge);
    thr_out[3] += static_cast<float>(surge);

    // 2) sway：左右差动
    thr_out[0] += static_cast<float>(sway);
    thr_out[1] += static_cast<float>(-sway);
    thr_out[2] += static_cast<float>(-sway);
    thr_out[3] += static_cast<float>(sway);

    // 3) yaw：左右反向产生偏航力矩
    thr_out[0] += static_cast<float>(-yaw);
    thr_out[1] += static_cast<float>(+yaw);
    thr_out[2] += static_cast<float>(-yaw);
    thr_out[3] += static_cast<float>(+yaw);

    // 垂向推进器 4–7：heave / roll / pitch 合成
    //
    // 约定（俯视）：
    //   4: 前左 FL, 5: 前右 FR, 6: 后左 RL, 7: 后右 RR
    //
    // heave：整体上下；roll：左右差动；pitch：前后差动。
    thr_out[4] += static_cast<float>(heave - roll - pitch); // FL
    thr_out[5] += static_cast<float>(heave + roll - pitch); // FR
    thr_out[6] += static_cast<float>(heave - roll + pitch); // RL
    thr_out[7] += static_cast<float>(heave + roll + pitch); // RR

    // 统一限幅
    const double limit = (cfg.max_cmd_abs > 0.0) ? cfg.max_cmd_abs : 1.0;
    for (auto& v : thr_out) {
        double dv = static_cast<double>(v);
        dv = std::max(-limit, std::min(limit, dv));
        v  = static_cast<float>(dv);
    }
}

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_TELEOP_MIXER_HPP
