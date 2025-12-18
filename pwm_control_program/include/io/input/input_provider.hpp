#pragma once
#ifndef ROVCTRL_IO_INPUT_PROVIDER_HPP
#define ROVCTRL_IO_INPUT_PROVIDER_HPP

/**
 * @file    input_provider.hpp
 * @brief   控制栈的“输入源”抽象接口（键盘 / 上位机 / 自动脚本等）
 *
 * 设计目标：
 *   - ControlLoop 只消费「control_core::ControlIntent」这一种输入表达；
 *   - 旧接口 poll(state, ref, request_exit) 作为兼容层保留，但不再作为主路径；
 *   - 模式枚举与控制核心保持一致：不在 io 层重复定义模式枚举，避免漂移；
 *   - 为超时保护/链路诊断提供最小必要字段（stamp_ns + ttl_ms）。
 *
 * 重要约束（做法 A）：
 *   - ControlIntent 的唯一真源在 control_core（control_core/control_intent.hpp）；
 *   - io 层仅负责“采集/解析/注入”，不定义控制意图的数据结构本体。
 */

#include <cstdint>
#include <memory>

#include "control_core/control_intent.hpp"    // ★ ControlIntent/ControlState/ControlReference
                                              //   (应在该头内自行 include 所需 control_types / control_mode)

namespace rovctrl::io {

namespace cc = rovctrl::control_core;

// ============================
// 输入源接口
// ============================
class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    /// 初始化输入源（打开串口/UDP/TTY 等）。失败返回 false。
    virtual bool init() = 0;

    /**
     * @brief 主接口：输出 ControlIntent（ControlLoop 只调用该接口）
     *
     * @param[in,out] state  当前控制状态（输入源可读取；不建议在此接口内修改核心状态）
     * @param[out]    intent 本周期输入意图（实现方需负责设置 stamp_ns/seq/ttl_ms 等元信息）
     * @return true 正常；false 致命错误
     */
    virtual bool poll(cc::ControlState& state, cc::ControlIntent& intent) = 0;

    /**
     * @brief 兼容旧接口：输出 ControlReference（Legacy）
     *
     * NOTE:
     *   - 仅用于旧代码/旧测试；ControlLoop 不应再使用该接口作为主路径。
     *   - 默认实现：poll(state,intent) + 最保守映射。
     *   - ref_delta / mode_request / arm/estop 等“系统级意图”在旧接口语义下无法可靠表达，
     *     默认不做隐式处理（避免行为不透明）。
     */
    virtual bool poll(cc::ControlState&      state,
                      cc::ControlReference& ref,
                      bool&                 request_exit)
    {
        cc::ControlIntent intent{};
        if (!poll(state, intent)) {
            return false;
        }

        request_exit = intent.request_exit;

        // 旧 ref 的生成策略（保守）：
        // - intent.has_ref：覆盖 ref
        // - intent.has_teleop_dof：填 ref.dof_cmd 并置 use_dof_cmd
        // - 其它系统级意图（mode/arm/estop/ref_delta）：旧接口默认不处理
        if (intent.has_ref) {
            ref = intent.ref;
        }
        if (intent.has_teleop_dof) {
            ref.dof_cmd     = intent.teleop_dof_cmd;
            ref.use_dof_cmd = true;
        }

        return true;
    }

    /// 重置输入源内部状态（不一定释放系统资源）
    virtual void reset() = 0;
};

using InputProviderPtr = std::shared_ptr<IInputProvider>;

} // namespace rovctrl::io

#endif // ROVCTRL_IO_INPUT_PROVIDER_HPP
