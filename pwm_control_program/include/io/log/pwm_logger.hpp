#pragma once
#ifndef ROVCTRL_IO_PWM_LOGGER_HPP
#define ROVCTRL_IO_PWM_LOGGER_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace rovctrl::io {

class PwmLogger final {
public:
    enum class Mode : std::uint8_t {
        AppliedOnly   = 0,
        CmdAndApplied = 1
    };

    PwmLogger();
    ~PwmLogger();

    PwmLogger(const PwmLogger&)            = delete;
    PwmLogger& operator=(const PwmLogger&) = delete;
    PwmLogger(PwmLogger&&) noexcept;
    PwmLogger& operator=(PwmLogger&&) noexcept;

    bool init(const std::string& root_dir,
              Mode               mode   = Mode::CmdAndApplied,
              const std::string& prefix = "pwm_log");

    void close() noexcept;

    bool is_open() const noexcept;
    Mode mode() const noexcept;

    void logApplied(double t_s, const std::array<float, 8>& applied);

    void logCmdAndApplied(double t_s,
                          const std::array<float, 8>& cmd,
                          const std::array<float, 8>& applied);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_PWM_LOGGER_HPP
