// pwm_control_program/src/controllers/manual_controller.cpp

#include "controllers/manual_controller.hpp"

#include <algorithm>  // std::clamp

namespace rovctrl::controllers {

using rovctrl::control_core::ControlMode;
using rovctrl::control_core::ControlOutput;
using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;

// ------------------------ 小工具函数 ------------------------ //

namespace {

inline double clamp_unit(double v)
{
    // 将指令限制在 [-1, 1]，避免 teleop 偶然越界
    return std::clamp(v, -1.0, 1.0);
}

} // anonymous namespace

// ------------------------ ManualController 实现 ------------------------ //

ManualController::ManualController()  = default;
ManualController::~ManualController() = default;

ControlMode ManualController::mode() const
{
    return ControlMode::MANUAL;
}

void ManualController::reset()
{
    // 手动模式没有内部状态，无需做任何事情
    // 保留接口是为了和其他控制器保持一致
}

bool ManualController::update(const ControllerContext& ctx,
                              ControlOutput& out)
{
    const ControlState&     state = ctx.state;
    const ControlReference& ref   = ctx.ref;

    // 1) 模式与时间戳
    out.mode = ControlMode::MANUAL;

    // 优先使用导航状态时间，如果没有导航（纯手动/台架），退回 ref.t_ns
    if (state.t_ns != 0) {
        out.t_ns = state.t_ns;
    } else {
        out.t_ns = ref.t_ns;
    }

    // 2) 手动模式的约定：teleop_keyboard.cpp 侧按如下方式写入 ControlReference：
    //
    //   - ref.vel_ref[0] : surge 档位指令  (前(+)/后(-))，范围建议 [-1, 1]
    //   - ref.vel_ref[1] : sway  档位指令  (右(+)/左(-))，范围建议 [-1, 1]
    //   - ref.vel_ref[2] : heave 档位指令  (上(+)/下(-))，范围建议 [-1, 1]
    //   - ref.rpy_ref[2] : yaw   档位指令  (左(+)/右(-))，范围建议 [-1, 1]
    //
    // 这里只做“直接透传 + 限幅”，不做任何 PID/MPC 运算。

    const double surge_raw = ref.vel_ref[0];
    const double sway_raw  = ref.vel_ref[1];
    const double heave_raw = ref.vel_ref[2];
    const double yaw_raw   = ref.rpy_ref[2];

    out.surge_cmd = clamp_unit(surge_raw);
    out.sway_cmd  = clamp_unit(sway_raw);
    out.heave_cmd = clamp_unit(heave_raw);
    out.yaw_cmd   = clamp_unit(yaw_raw);

    // 3) 标记输出有效
    out.valid = true;

    return true;
}

const char* ManualController::name() const
{
    return "ManualController";
}

} // namespace rovctrl::controllers
