#include "controllers/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rovctrl::controllers {

using rovctrl::control_core::ControlMode;
using rovctrl::control_core::ControlOutput;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlState;

PidController::PidController(const PidControllerConfig& cfg) noexcept
    : cfg_(cfg)
{
    reset();
}

ControlMode PidController::mode() const noexcept
{
    // 占位实现：先返回枚举的第 0 个值，确保编译通过
    // TODO: 后续根据 ControlMode 的实际定义，改成真正的 PID 模式枚举
    return static_cast<ControlMode>(0);
}

void PidController::reset() noexcept
{
    auto reset_axis = [](PidAxisState& st) noexcept {
        st.integ    = 0.0;
        st.prev_err = 0.0;
        st.has_prev = false;
    };

    reset_axis(surge_);
    reset_axis(sway_);
    reset_axis(heave_);
    reset_axis(yaw_);
}

bool PidController::compute(const ControlState&     state,
                            const ControlReference& ref,
                            ControlOutput&          out,
                            double                  dt) noexcept
{
    (void)state;
    (void)ref;
    (void)dt;

    out.has_body_wrench      = false;
    out.has_thruster_command = false;

    out.body_wrench.fill(0.0);
    out.thruster_command.fill(0.0f);

    return true;
}

} // namespace rovctrl::controllers
