#pragma once
#ifndef ROVCTRL_CONTROL_CORE_CONTROL_LOOP_HPP
#define ROVCTRL_CONTROL_CORE_CONTROL_LOOP_HPP

/**
 * @file   control_loop.hpp
 * @brief  顶层控制循环（定时执行：读取输入 → Guard 仲裁 → 计算控制 → 推力分配 → 下发 PWM）。
 *
 * 设计原则：
 *  - 头文件只暴露“控制循环的公共接口与必要类型”；
 *  - 导航共享内存/消息类型不透传到头文件（cpp 内部使用 PIMPL）；
 *  - IO/日志等实现细节尽量留在 .cpp，减少 include 扇出与耦合。
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "control_core/control_guard.hpp"
#include "control_core/control_intent.hpp"
#include "control_core/thruster_allocation.hpp"
#include "controllers/controller_manager.hpp"
#include "io/input/input_provider.hpp" // for rovctrl::io::InputProviderPtr
#include "platform/pwm_client.hpp"

namespace shared::msg {
struct NavState; // forward decl only; definition stays in .cpp
}

namespace rovctrl::control_core {

class ControlLoop {
public:
    struct Config {
        double loop_hz = 100.0;

        // --- runtime robustness ---
        int  max_step_errors         = 1000;
        int  step_error_log_interval = 100;
        bool log_timing              = false;
        bool enable_pwm_log          = true;

        // dt clamp（避免卡顿导致 dt 巨大）
        double dt_clamp_max_sec = 0.2;

        // --- safety & arbitration ---
        ControlGuardConfig guard_cfg{};

        bool allow_run_without_nav              = true;
        bool enter_failsafe_on_controller_error = true;

        // --- allocation ---
        ThrusterAllocationConfig thruster_alloc{};
    };

    // ================= PWM 日志后端接口（方案 A：对外可见，cpp 里实现） =================
    struct PwmLog {
        enum class Mode : std::uint8_t {
            AppliedOnly,
            CmdAndApplied
        };

        virtual ~PwmLog() = default;

        virtual bool init(const std::string& root_dir,
                          Mode               mode,
                          const std::string& prefix) = 0;

        virtual bool is_open() const noexcept = 0;

        virtual void logApplied(double t_s,
                                const std::array<float, 8>& applied) = 0;

        virtual void logCmdAndApplied(double t_s,
                                      const std::array<float, 8>& cmd,
                                      const std::array<float, 8>& applied) = 0;

        virtual void close() noexcept = 0;
    };

public:
    ControlLoop(const Config&                  cfg,
                rovctrl::platform::PwmClient&  pwm,
                rovctrl::io::InputProviderPtr  input,
                ControllerManager&&            ctrl_mgr,
                std::atomic_bool*              external_stop_flag = nullptr);

    // NOTE: Defined in .cpp (NavSub is incomplete here).
    ~ControlLoop() noexcept;

    ControlLoop(const ControlLoop&)            = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    // 保守做法：先禁用 move（你当前策略）
    ControlLoop(ControlLoop&&)                 = delete;
    ControlLoop& operator=(ControlLoop&&)      = delete;

    // 若你决定允许 move（推荐的工程做法），用下面替换上面两行：
    // ControlLoop(ControlLoop&&) noexcept        = default;
    // ControlLoop& operator=(ControlLoop&&) noexcept = default;

    int run();

    const Config& config() const noexcept { return cfg_; }

private:
    // =============== 配置与依赖对象 ===============
    Config                         cfg_{};
    rovctrl::platform::PwmClient&  pwm_;
    rovctrl::io::InputProviderPtr  input_;
    ControllerManager              ctrl_mgr_;
    std::atomic_bool*              external_stop_{nullptr};

    // =============== Guard（安全/仲裁）==============
    ControlGuard guard_;

    // =============== 控制状态与 I/O 缓存 ===============
    ControlState  state_{};
    ControlIntent intent_{};        // raw input
    GuardResult   guard_result_{};  // post-arbitration

    ControlReference ref_{};
    ControlOutput    output_{};

    // =============== 计时 ===============
    std::chrono::steady_clock::time_point start_time_{};

    // =============== 导航状态反馈（PIMPL） ===============
    struct NavSub;
    struct NavSubDeleter {
    void operator()(NavSub*) noexcept;
};
    std::unique_ptr<NavSub, NavSubDeleter> nav_sub_;

    bool last_nav_valid_{false};

    // =============== PWM 日志（PIMPL，避免头文件引入 <fstream>） ===============
    std::unique_ptr<PwmLog> pwm_logger_;

    // =============== 推力分配器 ===============
    ThrusterAllocator allocator_;

private:
    // 与 .cpp 保持一致的私有步骤接口（避免签名漂移）
    bool update_nav_feedback_(shared::msg::NavState& nav_out); // returns nav_ok
    void build_reference_from_guard_();
    bool build_thruster_command_(ThrusterArray& thr_out);
    void execute_failsafe_(FailsafeAction a);

    // 由 pwm_control_program/src/control_core/control_loop_pwm_log.cpp 提供实现
    std::unique_ptr<PwmLog> make_pwm_logger_();

    // 由 pwm_control_program/src/control_core/control_loop_helpers.cpp 提供实现
    std::uint64_t now_mono_ns_() const;
};

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROL_CORE_CONTROL_LOOP_HPP
