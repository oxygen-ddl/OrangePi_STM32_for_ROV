#pragma once

/**
 * @file    manual_controller.hpp
 * @brief   手动模式控制器 —— 不做闭环控制，直接传递输入
 *
 * MANUAL 模式的本质：
 *   - 用户输入就是控制输出；
 *   - 不涉及 PID/MPC；
 *   - 仅在内部做限幅、过滤（可选）。
 */

#include "controllers/controller_base.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::controllers {

using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlOutput;
using rovctrl::control_core::ControlMode;

class ManualController : public IController {
public:
    ManualController()  = default;
    ~ManualController() override = default;

    ControlMode mode() const override {
        return ControlMode::MANUAL;
    }

    void reset() override {
        // Manual 模式通常不需要 reset
    }

    /**
     * @brief 直接将 ControlReference 中的虚拟 DOF 映射到 ControlOutput
     *
     * @note Manual 模式的核心：不做控制律，只做传递。
     */
    bool update(const ControllerContext& ctx,
                ControlOutput& out) override
    {
        const auto& ref = ctx.ref;

        out.mode  = ControlMode::MANUAL;
        out.t_ns  = ctx.state.t_ns;

        // 直接传递虚拟 DOF 指令
        out.surge_cmd = ref.vel_ref[0];   // 或者 surge_ref（取决于键盘输入设计）
        out.sway_cmd  = ref.vel_ref[1];
        out.heave_cmd = ref.vel_ref[2];
        out.yaw_cmd   = ref.rpy_ref[2];   // yaw 输入通常单独处理

        out.valid = true;
        return true;
    }

    const char* name() const override { return "ManualController"; }
};

} // namespace rovctrl::controllers
