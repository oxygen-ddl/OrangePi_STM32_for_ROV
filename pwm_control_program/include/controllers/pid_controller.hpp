#pragma once

/**
 * @file   pid_controller.hpp
 * @brief  位置 / 深度 / 航向 PID 控制器（占位实现，接口与框架对齐）
 *
 * 说明：
 *   - 当前版本的目标是先让整个 pwm_control_program 可以顺利编译、运行；
 *   - 控制律实现部分暂时做成“空实现”（输出置零），后续根据实际需求再完善；
 *   - 接口完全对齐 controllers/controller_base.hpp 中的 IController 要求：
 *       * name()        noexcept
 *       * mode()        noexcept
 *       * reset()       noexcept
 *       * compute(...)  noexcept
 */

#include <array>

#include "controllers/controller_base.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::controllers {

/**
 * @brief 单轴 PID 配置
 *
 * 预留字段，后续可以按需使用。
 */
struct PidAxisConfig {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    /// 积分限幅
    double i_limit{0.0};

    /// 输出限幅
    double out_limit{0.0};
};

/**
 * @brief PID 控制器整体配置
 *
 * 当前假定控制 4 个自由度：surge / sway / heave / yaw。
 * 后续如果需要滚转 / 俯仰，可以继续扩展。
 */
struct PidControllerConfig {
    double loop_hz{100.0};

    PidAxisConfig surge;  ///< x 方向位置 /速度 PID
    PidAxisConfig sway;   ///< y 方向
    PidAxisConfig heave;  ///< z / 深度
    PidAxisConfig yaw;    ///< 航向

    // 预留：对 body wrench 输出做限幅
    double max_abs_fx{100.0};
    double max_abs_fy{100.0};
    double max_abs_fz{100.0};
    double max_abs_mz{50.0};
};

/**
 * @brief 单轴 PID 的内部状态
 */
struct PidAxisState {
    double integ{0.0};      ///< 积分项
    double prev_err{0.0};   ///< 上一次误差
    bool   has_prev{false}; ///< 是否已有上一时刻数据
};

/**
 * @class PidController
 * @brief 实现 IController 接口的 PID 控制器（当前为占位实现）
 */
class PidController : public IController {
public:
    explicit PidController(const PidControllerConfig& cfg) noexcept;

    const char* name() const noexcept override {
        return "PidController";
    }

    rovctrl::control_core::ControlMode mode() const noexcept override;

    void reset() noexcept override;

    /**
     * @brief 计算控制输出
     *
     * @param state 当前系统状态（来自 ControlLoop）
     * @param ref   参考指令（期望位置 / 姿态等）
     * @param out   控制输出（将被写入 body_wrench / thruster_command）
     * @param dt    本周期时间步长，单位秒
     *
     * 当前版本为了先打通编译流程，暂时实现为：
     *   - 输出置零；
     *   - 标记为无 body_wrench / 无 thruster_command。
     */
    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt) noexcept override;

private:
    PidControllerConfig cfg_{};

    PidAxisState surge_{};
    PidAxisState sway_{};
    PidAxisState heave_{};
    PidAxisState yaw_{};

    // 后续如果要实现真正的 PID，可以增加内部辅助函数：
    // double step_axis(PidAxisState& st, const PidAxisConfig& cfg, double err, double dt) noexcept;
};

} // namespace rovctrl::controllers
