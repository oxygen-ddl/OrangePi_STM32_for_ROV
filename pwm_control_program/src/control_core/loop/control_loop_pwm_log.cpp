/**
 * @file   control_loop_pwm_log.cpp
 * @brief  PwmLog PIMPL implementation
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <string>
#include <memory>   // +++

// 日志实现（cpp 内部使用，不透传到头文件）
#include "io/log/pwm_logger.hpp"

namespace rovctrl::control_core {

namespace {

class PwmLogImpl final : public ControlLoop::PwmLog {
public:
    bool init(const std::string& root_dir,
              Mode               mode,
              const std::string& prefix) override
    {
        const rovctrl::io::PwmLogger::Mode m =
            (mode == Mode::AppliedOnly) ? rovctrl::io::PwmLogger::Mode::AppliedOnly
                                        : rovctrl::io::PwmLogger::Mode::CmdAndApplied;

        return logger_.init(root_dir, m, prefix);
    }

    bool is_open() const noexcept override { return logger_.is_open(); }

    void logApplied(double t_s,
                    const std::array<float, 8>& applied) override
    {
        logger_.logApplied(t_s, applied);
    }

    void logCmdAndApplied(double t_s,
                          const std::array<float, 8>& cmd,
                          const std::array<float, 8>& applied) override
    {
        logger_.logCmdAndApplied(t_s, cmd, applied);
    }

    
    void close() noexcept override

    {
        // 若 PwmLogger 没有 close()，这里可留空
        // 如果 PwmLogger 提供 close/flush，建议调用
        // logger_.close();
    }

private:
    rovctrl::io::PwmLogger logger_;
};

} // namespace

// 工厂：在 run() 中创建（保持你原来的“按开关创建”的行为）
std::unique_ptr<ControlLoop::PwmLog> ControlLoop::make_pwm_logger_()
{
    return std::make_unique<PwmLogImpl>();
}

} // namespace rovctrl::control_core
