// controllers/manual_controller.cpp
//
// ManualController
// ----------------
// 手动模式控制器：
//   - 从 ControlReference 中读取操作者 6-DOF 归一化指令（假定范围 [-1, 1]）；
//   - 按配置对各个 DOF 应用增益（surge/sway/heave/roll/pitch/yaw）；
//   - 按固定拓扑将 6 个 DOF 线性混合为 8 路推进器命令；
//   - 对输出进行统一限幅（[-max_cmd_abs, max_cmd_abs]）；
//   - 写入 ControlOutput::thruster_command，由上层 PWM 客户端转换为占空比。
//
// 详细坐标系与拓扑约定请参考 manual_controller.hpp 中的文档注释。

#include "controllers/manual_controller.hpp"
#include "control_core/teleop_mixer.hpp"


#include <algorithm>
#include <iostream>

namespace rovctrl::controllers {

using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlOutput;

// 若需要完全静默，可以改为 false 或在编译期用宏控制
static constexpr bool kManualCtrlDebugLog = true;

// =============================================================================
// Ctor
// =============================================================================

ManualController::ManualController(const ManualControllerConfig& cfg)
    : cfg_(cfg)
{
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
    // 调试：打印输入 DOF（可以后面再酌情关掉）
    // std::cout << "[ManualController] input dof "
    //           << "s=" << ref.dof_cmd.surge
    //           << " sw=" << ref.dof_cmd.sway
    //           << " h=" << ref.dof_cmd.heave
    //           << " r=" << ref.dof_cmd.roll
    //           << " p=" << ref.dof_cmd.pitch
    //           << " y=" << ref.dof_cmd.yaw
    //           << " use_dof_cmd=" << ref.use_dof_cmd
    //           << "\n";

    // 基本防御：确保确实是在用 DOF 参考
    if (!ref.use_dof_cmd) {
        std::cerr << "[ManualController] ref.use_dof_cmd == false, nothing to do.\n";
        out = ControlOutput{};
        return true;  // 对于手动模式，可以视为“零输出”也是合法
    }

    out = ControlOutput{};
    out.has_thruster_command = true;

    auto& u = out.thruster_command;
    if (u.size() < 8) {
        std::cerr << "[ManualController] thruster_command size < 8, got "
                  << u.size() << "\n";
        out.has_thruster_command = false;
        return false;
    }

    // 通过 TeleopMixConfig + mix_6dof_to_8thrusters 实现混合
    rovctrl::control_core::TeleopMixConfig mix_cfg{};
    mix_cfg.surge_gain  = cfg_.surge_gain;
    mix_cfg.sway_gain   = cfg_.sway_gain;
    mix_cfg.heave_gain  = cfg_.heave_gain;
    mix_cfg.yaw_gain    = cfg_.yaw_gain;
    mix_cfg.roll_gain   = cfg_.roll_gain;
    mix_cfg.pitch_gain  = cfg_.pitch_gain;
    mix_cfg.max_cmd_abs = cfg_.max_cmd_abs;

    rovctrl::control_core::mix_6dof_to_8thrusters(mix_cfg, ref.dof_cmd, u);

    // 调试：打印输出 thruster_command
    std::cout << "[ManualController] thr_cmd "
              << "u0=" << u[0]
              << " u1=" << u[1]
              << " u2=" << u[2]
              << " u3=" << u[3]
              << " u4=" << u[4]
              << " u5=" << u[5]
              << " u6=" << u[6]
              << " u7=" << u[7]
              << "\n";

    return true;
}


} // namespace rovctrl::controllers
