#pragma once
#ifndef ROVCTRL_CONTROL_CORE_TELEOP_MIXER_HPP
#define ROVCTRL_CONTROL_CORE_TELEOP_MIXER_HPP

#include <array>
#include <cstddef>
#include <cmath>

namespace rovctrl::control_core {

struct TeleopMixerConfig {
    static constexpr std::size_t kNumDof        = 6;  // surge,sway,heave,roll,pitch,yaw
    static constexpr std::size_t kNumThrusters  = 8;  // 8 推进器

    bool   enable           = true;
    float  input_deadzone   = 0.05f;
    float  output_limit_abs = 1.0f;

    // mix_matrix[thruster][dof]
    std::array<std::array<float, kNumDof>, kNumThrusters> mix_matrix{};

    TeleopMixerConfig()
    {
        // 对应你原来的硬编码逻辑：
        // DOF 顺序: [surge, sway, heave, roll, pitch, yaw]

        // 水平推进器 0–3：surge / sway / yaw
        // 0: 左后, 1: 右后, 2: 左前, 3: 右前
        mix_matrix[0] = { 1.f,  1.f, 0.f,  0.f,  0.f, -1.f}; // THR0
        mix_matrix[1] = { 1.f, -1.f, 0.f,  0.f,  0.f,  1.f}; // THR1
        mix_matrix[2] = { 1.f, -1.f, 0.f,  0.f,  0.f, -1.f}; // THR2
        mix_matrix[3] = { 1.f,  1.f, 0.f,  0.f,  0.f,  1.f}; // THR3

        // 垂向推进器 4–7：heave / roll / pitch
        // 4: 前左 FL, 5: 前右 FR, 6: 后左 RL, 7: 后右 RR
        mix_matrix[4] = { 0.f, 0.f,  1.f, -1.f, -1.f, 0.f}; // FL
        mix_matrix[5] = { 0.f, 0.f,  1.f,  1.f, -1.f, 0.f}; // FR
        mix_matrix[6] = { 0.f, 0.f,  1.f, -1.f,  1.f, 0.f}; // RL
        mix_matrix[7] = { 0.f, 0.f,  1.f,  1.f,  1.f, 0.f}; // RR
    }
};

/**
 * @brief 将 6-DOF wrench 映射到 8 路推进器（线性混合）
 *
 * @param cfg     TeleopMixerConfig（通常由默认值 + YAML 覆盖而来）
 * @param u_dof   [6] = [surge, sway, heave, roll, pitch, yaw]
 * @param thr_out [8] 推进器归一化输出 [-1,1]
 */
inline void mix_6dof_to_8thrusters(
    const TeleopMixerConfig&                                           cfg,
    const std::array<double, TeleopMixerConfig::kNumDof>&              u_dof,
    std::array<float, TeleopMixerConfig::kNumThrusters>&               thr_out) noexcept
{
    thr_out.fill(0.0f);

    if (!cfg.enable) {
        return; // mixer 关掉，直接输出 0 → PWM 归中
    }

    // 1) 输入做死区 + 限幅
    std::array<double, TeleopMixerConfig::kNumDof> u{};
    for (std::size_t j = 0; j < TeleopMixerConfig::kNumDof; ++j) {
        double v = u_dof[j];
        if (!std::isfinite(v)) {
            v = 0.0;
        }
        if (std::fabs(v) < cfg.input_deadzone) {
            v = 0.0;
        }
        const double lim = (cfg.output_limit_abs > 0.0f)
                               ? static_cast<double>(cfg.output_limit_abs)
                               : 1.0;
        if (v >  lim) v =  lim;
        if (v < -lim) v = -lim;
        u[j] = v;
    }

    // 2) 矩阵乘：thr = mix_matrix * u
    const double lim = (cfg.output_limit_abs > 0.0f)
                           ? static_cast<double>(cfg.output_limit_abs)
                           : 1.0;

    for (std::size_t i = 0; i < TeleopMixerConfig::kNumThrusters; ++i) {
        double sum = 0.0;
        for (std::size_t j = 0; j < TeleopMixerConfig::kNumDof; ++j) {
            sum += static_cast<double>(cfg.mix_matrix[i][j]) * u[j];
        }
        if (sum >  lim) sum =  lim;
        if (sum < -lim) sum = -lim;
        thr_out[i] = static_cast<float>(sum);
    }
}

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_TELEOP_MIXER_HPP
