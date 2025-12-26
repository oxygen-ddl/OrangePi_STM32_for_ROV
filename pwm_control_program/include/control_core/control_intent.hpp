#pragma once
#ifndef ROVCTRL_CONTROL_CORE_CONTROL_INTENT_HPP
#define ROVCTRL_CONTROL_CORE_CONTROL_INTENT_HPP

/**
 * @file    control_intent.hpp
 * @brief   控制栈统一的“控制意图”消息（唯一真源，做法 A）
 *
 * 设计定位：
 *   - ControlIntent 是控制核心消费的“输入表达”，它描述“想让系统做什么”，而不是“输入从哪里来”；
 *   - IO 层（键盘/GCS/脚本）负责把外部输入解析为本结构；
 *   - ControlLoop / ControlGuard / ControllerManager 只处理本结构，不接触协议/键盘/网络细节。
 *
 * 约束：
 *   - 只依赖 control_core 的公共类型（ControlMode / ControlTypes）与标准库；
 *   - 不包含 socket/session/线程等 IO 细节字段；
 *   - 字段配套 has_* 有效位，避免默认值误触发。
 */

#include <cstdint>

#include "control_core/control_mode.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::control_core {

/**
 * @brief 一次 poll 输出的“控制意图”
 *
 * intent 是“输入侧”对控制侧的意图表达，不直接包含控制算法细节。
 * 允许同时存在 teleop 与 setpoint：由控制核心（Guard/Controller）决定如何融合。
 */
struct ControlIntent final
{
    // ---- 元信息（调试/时序） ----
    std::uint64_t seq      = 0;   ///< 输入序列号（输入源自增）
    std::uint64_t stamp_ns = 0;   ///< 输入产生时间（steady ns；若无法提供可置 0）
    std::uint32_t ttl_ms   = 0;   ///< 输入有效期（0 表示“使用控制侧默认 TTL”）

    // ---- 生命周期/安全相关 ----
    bool request_exit   = false;  ///< 请求退出主循环（例如按下 quit）

    bool estop          = false;  ///< 急停（强制进入安全输出）
    bool clear_estop    = false;  ///< 解除急停（建议显式解除；Guard 决定是否允许）
    bool has_estop_cmd  = false;  ///< estop/clear_estop 是否有效（避免默认 false 误触发）

    bool arm            = false;  ///< Arm（允许输出）
    bool disarm         = false;  ///< Disarm（禁止输出）
    bool has_arm_cmd    = false;  ///< arm/disarm 是否有效（避免默认 false 误触发）

    // ---- 模式请求 ----
    ControlMode mode_request     = ControlMode::kNone; ///< kNone 表示“无请求/不改变当前模式”
    bool        has_mode_request = false;

    // ---- 遥控输入（手动） ----
    DofCommand teleop_dof_cmd{};
    bool       has_teleop_dof = false;

    // ---- 参考量 / setpoint ----
    ControlReference ref{};
    bool             has_ref = false;

    // ---- 辅助：增量式 setpoint 调整 ----
    ControlReference ref_delta{};
    bool             has_ref_delta = false;

    /**
     * @brief 清空为“无有效载荷”，但保留 seq/stamp/ttl（方便调试与链路对齐）
     */
    void clear_payload() noexcept
    {
        request_exit = false;

        estop = false;
        clear_estop = false;
        has_estop_cmd = false;

        arm = false;
        disarm = false;
        has_arm_cmd = false;

        mode_request = ControlMode::kNone;
        has_mode_request = false;

        teleop_dof_cmd = DofCommand{};
        has_teleop_dof = false;

        ref = ControlReference{};
        has_ref = false;

        ref_delta = ControlReference{};
        has_ref_delta = false;
    }

    /**
     * @brief 便捷工具：清空全部（含 seq/stamp/ttl）
     */
    void clear_all() noexcept
    {
        seq = 0;
        stamp_ns = 0;
        ttl_ms = 0;
        clear_payload();
    }
};

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_CONTROL_INTENT_HPP
