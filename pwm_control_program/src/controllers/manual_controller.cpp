// controllers/manual_controller.cpp
//
// ManualController
// ----------------
// 手动模式控制器：
//   - 从 ControlReference 中读取操作者 6-DOF 归一化指令（假定范围 [-1, 1]）；
//   - 按 TeleopMixerConfig.gains 对各个 DOF 应用增益（surge/sway/heave/roll/pitch/yaw）；
//   - 按 teleop_mixer.yaml 中配置的混合矩阵将 6 个 DOF 线性混合为 8 路推进器命令；
//   - 对输出进行统一限幅（[-max_cmd_abs, max_cmd_abs]）；
//   - 写入 ControlOutput::thruster_command，由上层 PWM 客户端转换为占空比。
//
// 详细坐标系与拓扑约定请参考 manual_controller.hpp 中的文档注释。

#include "controllers/manual_controller.hpp"
#include "control_core/teleop_mixer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace rovctrl::controllers {

using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlOutput;
using rovctrl::control_core::TeleopMixerConfig;

// 若需要完全静默，可以改为 false 或在编译期用宏控制
static constexpr bool kManualCtrlDebugLog = false;

// =============================================================================
// Ctor
// =============================================================================

ManualController::ManualController(const ManualControllerConfig& cfg)
    : cfg_(cfg)
{
    // 如果以后需要，可以在这里根据 cfg_.teleop_mixer_cfg 打一行摘要日志
    // 例如：enable / output_limit_abs / gains 等
}

// =============================================================================
// Helpers
// =============================================================================

// 统一限幅到 [-max_cmd_abs, max_cmd_abs]
void ManualController::clamp_output(ControlOutput& out) const noexcept
{
    const double limit = (cfg_.max_cmd_abs > 0.0) ? cfg_.max_cmd_abs : 1.0;

    for (auto& v : out.thruster_command) {
        const double dv = static_cast<double>(v);
        v = static_cast<float>(std::clamp(dv, -limit, limit));
    }
}

// =============================================================================
// Core compute
// =============================================================================
bool ManualController::compute(const ControlState&     /*state*/,
                               const ControlReference& ref,
                               ControlOutput&          out,
                               double                  /*dt*/) noexcept
{
    // 目前我们完全走 “thruster_command 直写” 路径
    out = ControlOutput{};
    out.has_thruster_command = true;

    auto& u = out.thruster_command;
    if (u.size() < TeleopMixerConfig::kNumThrusters) {
        std::cerr << "[ManualController] thruster_command size < "
                  << TeleopMixerConfig::kNumThrusters << ", got "
                  << u.size() << "\n";
        out.has_thruster_command = false;
        return false;
    }

    // 1) 如果没有 DOF 指令，就输出 0（手动模式空闲）
    //    （你之前把 use_dof_cmd 的防御注释掉了，这里保持简单处理）
    if (!ref.use_dof_cmd) {
        u.fill(0.0f);
        return true;
    }

    // 2) 取出 TeleopMixerConfig（从 cfg_ 里带进来的那份）
    const TeleopMixerConfig& mix_cfg = cfg_.teleop_mixer_cfg;

    // 3) 把 DofCommand 映射成固定顺序的 6 维数组：
    //    [surge, sway, heave, roll, pitch, yaw]
    std::array<double, TeleopMixerConfig::kNumDof> u_dof{{
        static_cast<double>(ref.dof_cmd.surge),
        static_cast<double>(ref.dof_cmd.sway),
        static_cast<double>(ref.dof_cmd.heave),
        static_cast<double>(ref.dof_cmd.roll),
        static_cast<double>(ref.dof_cmd.pitch),
        static_cast<double>(ref.dof_cmd.yaw),
    }};

    // 4) 调用通用 mixer 做 6DOF → 8Thrusters
    std::array<float, TeleopMixerConfig::kNumThrusters> thr_tmp{};
    rovctrl::control_core::mix_6dof_to_8thrusters(mix_cfg, u_dof, thr_tmp);

    // 5) 把 thr_tmp 拷贝到 ControlOutput 的 thruster_command
    for (std::size_t i = 0; i < TeleopMixerConfig::kNumThrusters; ++i) {
        u[i] = thr_tmp[i];
    }

    // 6) 额外限幅（双保险；如果你觉得重复，可以删掉这一行调用）
    clamp_output(out);

    // 调试打印（需要时再打开）
    // std::cout << "[ManualController] thr_cmd "
    //           << "u0=" << u[0]
    //           << " u1=" << u[1]
    //           << " u2=" << u[2]
    //           << " u3=" << u[3]
    //           << " u4=" << u[4]
    //           << " u5=" << u[5]
    //           << " u6=" << u[6]
    //           << " u7=" << u[7]
    //           << "\n";

    return true;
}

} // namespace rovctrl::controllers
