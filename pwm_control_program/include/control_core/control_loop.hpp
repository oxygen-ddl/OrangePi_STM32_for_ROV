// include/control_core/control_loop.hpp
#pragma once

#include "control_core/control_types.hpp"
#include "platform/pwm_client.hpp"
#include "platform/io/input_provider.hpp"
#include "controllers/controller_base.hpp"

namespace rovctrl::control_core {

class ControlLoop {
public:
    struct Config {
        double loop_hz = 100.0;
    };

    ControlLoop(const Config& cfg,
                rovctrl::platform::PwmClient& pwm,
                rovctrl::io::InputProviderPtr input,
                std::shared_ptr<rovctrl::controllers::ControllerBase> controller);

    int run();  // 阻塞运行，直到退出

private:
    Config cfg_;
    rovctrl::platform::PwmClient& pwm_;
    rovctrl::io::InputProviderPtr input_;
    std::shared_ptr<rovctrl::controllers::ControllerBase> controller_;
};

} // namespace rovctrl::control_core
