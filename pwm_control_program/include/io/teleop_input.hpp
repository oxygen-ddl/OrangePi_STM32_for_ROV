#pragma once
#ifndef ROVCTRL_IO_TELEOP_INPUT_HPP
#define ROVCTRL_IO_TELEOP_INPUT_HPP

#include <atomic>

#include "io/input_provider.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::io {

/**
 * @brief 键盘 Teleop 输入源实现
 *
 * 职责：
 *   - 设置/恢复终端 raw 模式（仅在本类内部处理）；
 *   - 非阻塞读取键盘按键；
 *   - 调用 C 层 teleop_keyboard 模块更新内部 DOF 状态；
 *   - 将 teleop_keyboard 的 DOF 状态写入 ControlReference::dof_cmd；
 *   - 捕获 ESC 等退出请求，通过 request_exit 返回给控制循环。
 *
 * 不负责：
 *   - DOF → 推进器映射（由 ManualController / ThrustAllocator 完成）；
 *   - PWM 下发（由 ControlLoop + PwmClient 完成）。
 */
class TeleopInputProvider : public IInputProvider {
public:
    TeleopInputProvider();
    ~TeleopInputProvider() override;

    bool init() override;

    bool poll(rovctrl::control_core::ControlState&     state,
              rovctrl::control_core::ControlReference& ref,
              bool&                                    request_exit) override;

    void reset() override;

private:
    bool initialized_ = false;
    bool raw_mode_    = false;

    // 是否曾经请求过退出（可用于做一次性 clean-up）
    bool exit_requested_ = false;

    // 可选：维护一个本地时间戳，用于调试
    std::int64_t last_t_ns_{0};

    // 内部帮助函数：进入/退出终端 raw 模式
    bool enter_raw_mode();
    void leave_raw_mode();

    // 内部帮助函数：读取一个键（非阻塞），没有按键时返回 EOF
    int  read_key_nonblock();
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_TELEOP_INPUT_HPP
