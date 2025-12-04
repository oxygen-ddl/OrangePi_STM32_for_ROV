// 
#pragma once
#ifndef ROVCTRL_IO_INPUT_PROVIDER_HPP
#define ROVCTRL_IO_INPUT_PROVIDER_HPP

/**
 * @file    input_provider.hpp
 * @brief   控制栈的“输入源”抽象接口（键盘 / 上位机 / 自动脚本等）
 *
 * 设计目标：
 *   - 给控制主循环提供统一的输入入口：
 *       * 更新 ControlState（可选）
 *       * 更新 ControlReference（尤其是 dof_cmd / pose_ref 等）
 *       * 告知是否请求退出；
 *   - 不直接做控制算法 / PWM 下发，这些交给 Controller 和 ControlLoop；
 *   - 不负责“当前控制模式”决策，模式切换交给控制核心（Mode / Controller）。
 */

#include <memory>
#include "control_core/control_types.hpp"

namespace rovctrl::io {

namespace cc = rovctrl::control_core;

/**
 * @brief 输入源接口：控制循环每个周期调用 poll() 拉取一次输入
 */
class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    /**
     * @brief 初始化输入源（例如设置终端 raw 模式、打开网络等）
     *
     * @return true 成功；false 失败（控制循环可认为是致命错误）
     */
    virtual bool init() = 0;

    /**
     * @brief 拉取一次输入（由控制循环每个周期调用）
     *
     * @param[in,out] state        当前控制状态（可选，输入源可根据需要修改或忽略）
     * @param[in,out] ref          当前控制参考（例如 dof_cmd / pose_ref 等）
     * @param[out]    request_exit 若本次调用后请求退出主循环，则置为 true
     *
     * 返回值：
     *   - true  正常工作；
     *   - false 发生致命错误，控制循环可立即退出。
     *
     * 典型实现：
     *   - TeleopInput：读键盘 → 调 teleop_keyboard → 填 ref.dof_cmd / use_dof_cmd；
     *   - AutoScriptInput：根据时间或内部脚本推进 ref.pose_ref 等。
     */
    virtual bool poll(cc::ControlState&     state,
                      cc::ControlReference& ref,
                      bool&                 request_exit) = 0;

    /**
     * @brief 重置内部状态（例如切换模式 / 恢复安全状态时调用）
     *
     * 建议行为：
     *   - 清零所有手动 DOF；
     *   - 清除退出请求标志；
     *   - 清空内部缓冲。
     */
    virtual void reset() = 0;
};

using InputProviderPtr = std::shared_ptr<IInputProvider>;

} // namespace rovctrl::io

#endif // ROVCTRL_IO_INPUT_PROVIDER_HPP
