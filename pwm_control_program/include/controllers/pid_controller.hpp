#pragma once

/**
 * @file    pid_controller.hpp
 * @brief   4-DOF PID 控制器（surge / sway / heave / yaw）
 *
 * 设计要点：
 *   - 以 ControlState + ControlReference 为输入；
 *   - 输出 ControlOutput 中的虚拟 4-DOF 指令（surge/sway/heave/yaw）；
 *   - 先不做 6-DoF 力级别，只作为 teleop / 简易闭环的控制器；
 *   - 参数从 YAML / 上层配置加载到 PidGains / PidAxisConfig。
 */

#include <array>
#include <cstdint>
#include "control_core/control_types.hpp"
#include "controllers/controller_base.hpp"

namespace rovctrl::controllers {

using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlOutput;
using rovctrl::control_core::ControlMode;

/// 单轴 PID 参数
struct PidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    /// 输出限幅（在映射到虚拟 DOF 前的内部限幅）
    double u_min{-1.0};
    double u_max{ 1.0};
};

/// 单轴 PID 内部状态
struct PidAxisState {
    double integrator{0.0};
    double prev_error{0.0};
    bool   has_prev{false};
};

/// PID 控制器总体配置
struct PidControllerConfig {
    double loop_hz{50.0};     ///< 控制循环频率（Hz），用于 dt 估计

    /// 四个 DOF 的 PID 参数：
    ///   idx=0: surge, 1: sway, 2: heave, 3: yaw
    std::array<PidGains, 4> gains{};
};

class PidController : public IController {
public:
    PidController() = default;
    ~PidController() override = default;

    /// 初始化/配置 PID 控制器（通常由 YAML 配置填充 cfg 后调用）
    bool init(const PidControllerConfig& cfg);

    /// IController 接口实现
    ControlMode mode() const override {
        return ControlMode::PID_POSITION;   // 当前阶段视作“位置/姿态 PID”
    }

    void reset() override;

    bool update(const ControllerContext& ctx,
                ControlOutput& out) override;

    const char* name() const override {
        return "PIDController";
    }

private:
    PidControllerConfig cfg_{};
    bool inited_{false};

    // 四个 DOF 的 PID 内部状态：0: surge, 1: sway, 2: heave, 3: yaw
    std::array<PidAxisState, 4> axes_{};

    /// 单轴 PID 步进
    double step_one_axis(PidAxisState& st,
                         const PidGains& gains,
                         double error,
                         double dt_sec);
};

} // namespace rovctrl::controllers
