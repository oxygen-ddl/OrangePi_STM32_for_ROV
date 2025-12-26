/**
 * @file   control_loop_helpers.cpp
 * @brief  helper utilities for ControlLoop
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <cstdint>
#include <cstddef>

// 兜底：若 ControllerManager 析构仍需完整类型
//#include "controllers/controller_base.hpp"

// 统一时间基
#include "platform/timebase.hpp"

namespace rovctrl::control_core {

namespace {

inline std::uint64_t now_mono_ns()
{
    return static_cast<std::uint64_t>(rovctrl::platform::timebase::now_ns());
}

} // namespace

// ============================================================================
// 构造 8 路推进器归一化指令
// ============================================================================

bool ControlLoop::build_thruster_command_(ThrusterArray& thr_out)
{
    if (output_.has_thruster_command) {
        thr_out = output_.thruster_command;
        return true;
    }

    if (output_.has_body_wrench) {
        std::array<double, 6> wrench{};
        for (std::size_t i = 0; i < 6; ++i) {
            wrench[i] = output_.body_wrench[i];
        }

        std::array<float, 8> thr{};
        if (!allocator_.allocate(wrench, thr)) {
            return false;
        }

        thr_out = thr;
        return true;
    }

    return false;
}

// ============================================================================
// 执行 failsafe（Guard 只建议，ControlLoop 执行）
// ============================================================================

void ControlLoop::execute_failsafe_(FailsafeAction a)
{
    output_ = ControlOutput{};
    (void)ctrl_mgr_.set_mode(ControlMode::kFailsafe);

    ThrusterArray zero{};
    zero.fill(0.0f);

    (void)a;

    (void)pwm_.setTargets(zero);
    (void)pwm_.step();
}

// ============================================================================
// GuardResult -> reference
// ============================================================================

void ControlLoop::build_reference_from_guard_()
{
    ref_ = ControlReference{};

    const auto& eff = guard_result_.effective_intent;

    if (eff.has_ref) {
        ref_ = eff.ref;
    }

    if (eff.has_teleop_dof) {
        ref_.dof_cmd     = eff.teleop_dof_cmd;
        ref_.use_dof_cmd = true;
    }

    // ref_delta 如需叠加，可后续扩展；此处保持原逻辑
}

// ============================================================================
// 给 run() 使用的“当前时间”
// ============================================================================

std::uint64_t ControlLoop::now_mono_ns_() const
{
    return now_mono_ns();
}

} // namespace rovctrl::control_core
