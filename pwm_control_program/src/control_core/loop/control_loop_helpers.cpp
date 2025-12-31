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

/// 根据 motor_test 覆盖 thruster 命令：
/// - 仅当 has_motor_test 且 enable!=0 时生效；
/// - 其他模式/控制结果在本周期被忽略；
/// - motor_id 为 1..N（逻辑电机编号），N>范围则忽略 motor_test。
///
/// 约定：
///   mode = 0 : value ∈ [-1, +1] 归一化指令（你可以在这里换算成 PWM 或直接当作 "throttle"）
///   mode = 1 : value 为绝对 PWM 值（需要与 PwmClientConfig 对应）
///
/// 注意：真正“执行 2s 并自动停止”的时间逻辑建议放在 gateway 侧；
/// 这里假设收到的每一个 motor_test 都是“当前周期要执行的最新期望”。
void ControlLoop::apply_motor_test_override(
    const ControlIntent& intent,
    ThrusterArray& thr_cmd) noexcept
{
    if (!intent.has_motor_test) return;
    const auto& mt = intent.motor_test;
    if (!mt.enable) return;

    // 简单示例：1..8 → 0..7
    const std::uint8_t id = mt.motor_id;
    if (id == 0 || id > ThrusterArray::value_type(thr_cmd.size())) {
        return;  // 非法 id，忽略
    }
    const std::size_t idx = static_cast<std::size_t>(id - 1);

    // 这里根据 mode 决定如何覆盖 thr_cmd[idx]
    if (mt.mode == 0) {
        // 归一化 [-1..1] → 直接覆盖
        thr_cmd[idx] = static_cast<float>(mt.value);
    } else if (mt.mode == 1) {
        // 如果你这边 thr_cmd 是 [-1..1]，那 mode=1 也可以先简单映射/忽略
        // TODO：将绝对 PWM 映射为 normalized 值
        // 暂时示例：仍旧当作 [-1..1]
        thr_cmd[idx] = static_cast<float>(mt.value);
    }

    // 也可以选择：单电机测试时把其他通道归零
    // for (std::size_t i = 0; i < thr_cmd.size(); ++i) {
    //     if (i == idx) continue;
    //     thr_cmd[i] = 0.0f;
    // }
}


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
