#pragma once
#ifndef ROVCTRL_IO_TELEOP_INPUT_HPP
#define ROVCTRL_IO_TELEOP_INPUT_HPP

#include <atomic>

#include "io/input_provider.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::io {

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
    bool initialized_    = false;
    bool raw_mode_       = false;
    bool exit_requested_ = false;

    // 可选：维护一个本地时间戳，用于调试
    std::int64_t last_t_ns_{0};

    // 标记这个输入源对应的控制模式（这里只是“标签”）
    rovctrl::control_core::ControlMode mode_{rovctrl::control_core::ControlMode::Manual};

    // 内部帮助函数：进入/退出终端 raw 模式
    bool enter_raw_mode();
    void leave_raw_mode();

    // 内部帮助函数：读取一个键（非阻塞），没有按键时返回 EOF
    int  read_key_nonblock();
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_TELEOP_INPUT_HPP
