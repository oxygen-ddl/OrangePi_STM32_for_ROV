#pragma once
#ifndef ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
#define ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP

#include <cmath>
#include <cstdint>
#include <string>

#include "controllers/controller_base.hpp"      // IController
#include "control_core/control_types.hpp"       // ControlState/Reference/Output/DofCommand
#include "control_core/control_mode.hpp"        // ControlMode

namespace rovctrl::controllers {

struct ManualControllerConfig {
    // DOF 指令 -> 输出缩放（默认 1.0）
    double surge_gain  = 1.0;
    double sway_gain   = 1.0;
    double heave_gain  = 1.0;
    double roll_gain   = 1.0;
    double pitch_gain  = 1.0;
    double yaw_gain    = 1.0;

    // 输出限幅：[-max_cmd_abs, +max_cmd_abs]
    double max_cmd_abs = 1.0;

    // 输出层级选择：
    // - true ：输出 out.thruster_command（要求上游已决定用“DOF->Thruster”的路径，通常不在 controller 做分配）
    // - false：输出 out.body_wrench（交给 thruster_allocation 做 6x8 分配）
    bool output_body_wrench = true;

    // 异常输入策略：
    // - true：如果 ref 没有 dof_cmd，则返回 false（让 manager 统计失败/触发 failsafe）
    // - false：如果 ref 没有 dof_cmd，则输出全零并返回 true（更“现场友好”）
    bool missing_input_is_error = false;
};

class ManualController final : public IController {
public:
    explicit ManualController(const ManualControllerConfig& cfg = ManualControllerConfig());
    ~ManualController() override = default;

    const char* name() const noexcept override { return "manual"; }

    rovctrl::control_core::ControlMode mode() const noexcept override {
        return rovctrl::control_core::ControlMode::kManual;
    }

    void reset() noexcept override {}

    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt) noexcept override;

    const ManualControllerConfig& config() const noexcept { return cfg_; }

private:
    ManualControllerConfig cfg_{};

    static double clamp(double v, double lo, double hi) noexcept {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    static bool is_finite(double v) noexcept {
        return std::isfinite(v);
    }

    void zero_output(rovctrl::control_core::ControlOutput& out) const noexcept;
    bool build_from_dof(const rovctrl::control_core::DofCommand& dof,
                        rovctrl::control_core::ControlOutput& out) const noexcept;

    void clamp_output(rovctrl::control_core::ControlOutput& out) const noexcept;
};

} // namespace rovctrl::controllers

#endif // ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
