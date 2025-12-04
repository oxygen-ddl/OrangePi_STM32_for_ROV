#pragma once
#ifndef ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
#define ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP

#include "controllers/controller_base.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::controllers {

class ManualControllerConfig {
public:
    double surge_gain  = 1.0;
    double sway_gain   = 1.0;
    double heave_gain  = 1.0;
    double yaw_gain    = 1.0;
    double roll_gain   = 1.0;
    double pitch_gain  = 1.0;

    double max_cmd_abs = 1.0;   // 输出限幅（在 [-max_cmd_abs, +max_cmd_abs] 之间）
};

class ManualController : public IController {
public:
    explicit ManualController(const ManualControllerConfig& cfg = ManualControllerConfig());
    ~ManualController() override = default;

    const char* name() const noexcept override {
        return "ManualController";
    }

    rovctrl::control_core::ControlMode mode() const noexcept override {
        // 注意：这里要和 control_types.hpp 里的枚举值一致
        // 如果那里是 enum class ControlMode { Manual, Auto, Failsafe };
        // 这里就应该是 ControlMode::Manual（而不是 MANUAL）
        return rovctrl::control_core::ControlMode::Manual;
    }

    void reset() noexcept override {
        // 手动模式当前无内部状态，可留空
    }

    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt) noexcept override;

private:
    ManualControllerConfig cfg_;

    void clamp_output(rovctrl::control_core::ControlOutput& out) const;
};

} // namespace rovctrl::controllers

#endif // ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
