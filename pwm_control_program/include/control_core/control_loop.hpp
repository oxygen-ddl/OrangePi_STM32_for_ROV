#pragma once

#include <atomic>
#include <memory>
#include <chrono>   // 必须：因为头文件里直接使用了 std::chrono

#include "control_core/control_types.hpp"
#include "platform/pwm_client.hpp"
#include "io/input_provider.hpp"
#include "io/pwm_logger.hpp"              // 必须：确保 PwmLogger 可见
#include "controllers/controller_base.hpp" // 必须：引入 IController

namespace rovctrl::control_core {

class ControlLoop {
public:
    struct Config {
        double loop_hz = 100.0;

        int  max_step_errors         = 1000;
        int  step_error_log_interval = 100;
        bool log_timing              = false;
        bool enable_pwm_log          = true;
    };

    using ControllerPtr = std::shared_ptr<rovctrl::controllers::IController>;

    ControlLoop(const Config& cfg,
                rovctrl::platform::PwmClient& pwm,
                rovctrl::io::InputProviderPtr input,
                ControllerPtr controller,
                std::atomic_bool* external_stop_flag = nullptr);

    int run();

    const Config& config() const noexcept { return cfg_; }

private:
    Config cfg_;
    rovctrl::platform::PwmClient& pwm_;
    rovctrl::io::InputProviderPtr input_;
    ControllerPtr controller_;
    std::atomic_bool* external_stop_;

    ControlState     state_{};
    ControlReference ref_{};
    ControlOutput    output_{};

    // PWM 日志
    rovctrl::io::PwmLogger pwm_logger_;
    std::chrono::steady_clock::time_point start_time_{};
};

} // namespace rovctrl::control_core
