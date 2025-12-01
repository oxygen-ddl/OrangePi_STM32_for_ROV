// include/platform/io/teleop_input.hpp
#pragma once

#include "platform/io/input_provider.hpp"
#include "control_core/control_types.hpp"

namespace rovctrl::io {

class TeleopInputProvider : public IInputProvider {
public:
    TeleopInputProvider();
    ~TeleopInputProvider() override;

    bool init() override;
    bool poll(rovctrl::control_core::ControlState& state,
              rovctrl::control_core::ControlReference& ref,
              bool& request_exit) override;
    void reset() override;

private:
    bool initialized_ = false;
    bool raw_mode_    = false;

    // 当前控制模式（手动 / PID Demo 等），暂时可以不在这里切模式
    rovctrl::control_core::ControlMode mode_;
};

} // namespace rovctrl::io
:contentReference[oaicite:4]{index=4}
