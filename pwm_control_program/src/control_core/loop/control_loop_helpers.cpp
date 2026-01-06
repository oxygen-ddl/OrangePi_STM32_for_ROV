/**
 * @file   control_loop_helpers.cpp
 * @brief  helper utilities for ControlLoop
 */

#include "control_core/control_loop.hpp"
#include "control_core/control_mode.hpp"  // 若本文件还没 include 这个，需要补上
#include "control_core/teleop_mixer.hpp"


#include <array>
#include <cstdint>
#include <cstddef>
#include <iostream>  // ← 新增这一行

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
    // 1) 控制器直接给了 thruster_command：优先使用
    if (output_.has_thruster_command) {
        thr_out = output_.thruster_command;
        return true;
    }

    // 2) 控制器给了 body_wrench：走分配矩阵
    if (output_.has_body_wrench) {
        std::array<double, 6> wrench{};
        for (std::size_t i = 0; i < 6; ++i) {
            wrench[i] = output_.body_wrench[i];
        }

        std::array<float, 8> thr{};
        if (!allocator_.allocate(wrench, thr)) {
            std::cerr << "[ControlLoop][ALLOC] allocator_.allocate() failed.\n";
            return false;
        }

        thr_out = thr;
        return true;
    }

    // 3) 遥控兜底：只要有 teleop DOF，就直接按 6DOF → 8 推进器混配
    const auto mode = ctrl_mgr_.mode();
    const auto& eff = guard_result_.effective_intent;

    if (eff.has_teleop_dof) {
        rovctrl::control_core::TeleopMixConfig mix_cfg{};
        mix_cfg.surge_gain  = 1.0;
        mix_cfg.sway_gain   = 1.0;
        mix_cfg.heave_gain  = 1.0;
        mix_cfg.yaw_gain    = 1.0;
        mix_cfg.roll_gain   = 1.0;
        mix_cfg.pitch_gain  = 1.0;
        mix_cfg.max_cmd_abs = 1.0;

        // 注意：这里用 ref_.dof_cmd（build_reference_from_guard_ 已经把 effective_intent 写进去）
        rovctrl::control_core::mix_6dof_to_8thrusters(mix_cfg,
                                                      ref_.dof_cmd,
                                                      thr_out);

        std::cout << "[ControlLoop][ALLOC_FALLBACK] teleop -> thr_cmd "
                  << "mode=" << static_cast<int>(mode)
                  << " u0=" << thr_out[0]
                  << " u1=" << thr_out[1]
                  << " u2=" << thr_out[2]
                  << " u3=" << thr_out[3]
                  << " u4=" << thr_out[4]
                  << " u5=" << thr_out[5]
                  << " u6=" << thr_out[6]
                  << " u7=" << thr_out[7]
                  << "\n";

        return true;
    }

    // 4) 仍然没有任何输出，保留原始告警和 0 推力行为
    std::cout << "[ControlLoop][ALLOC] no thruster_command and no body_wrench\n";
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
    // 调试：看写进 ref_ 之后的 DOF
    // if (ref_.use_dof_cmd) {
    //     std::cout << "[ControlLoop][REF] use_dof_cmd=1 "
    //               << "s=" << ref_.dof_cmd.surge
    //               << " sw=" << ref_.dof_cmd.sway
    //               << " h=" << ref_.dof_cmd.heave
    //               << " r=" << ref_.dof_cmd.roll
    //               << " p=" << ref_.dof_cmd.pitch
    //               << " y=" << ref_.dof_cmd.yaw
    //               << "\n";
    // } else {
    //     std::cout << "[ControlLoop][REF] use_dof_cmd=0\n";
    // }

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
