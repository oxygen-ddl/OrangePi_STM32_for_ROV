#include "platform/pwm_client.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <cmath>


// C 接口头文件只在实现层可见
extern "C" {
#include "libpwm_host.h"
#include "pwm_control.h"
}

namespace rovctrl::platform {

namespace {

// 将 C++ 侧 motor 映射配置拷贝到 C 侧 pwm_ctrl_config_t，并做边界检查
static void sanitize_motor_mapping(const PwmClientConfig& cfg,
                                   pwm_ctrl_config_t&     ctrl_cfg)
{
    const std::size_t n =
        (kNumPwmChannels < static_cast<std::size_t>(PWM_HOST_CH_NUM))
            ? kNumPwmChannels
            : static_cast<std::size_t>(PWM_HOST_CH_NUM);

    for (std::size_t i = 0; i < n; ++i) {
        const int map_val = cfg.motorch_to_pwmch[i];

        if (map_val < 0 || map_val > PWM_HOST_CH_NUM) {
            std::cerr << "[PwmClient] WARN: motorch_to_pwmch[" << i
                      << "]=" << map_val
                      << " out of range (0.." << PWM_HOST_CH_NUM
                      << "), fallback to 0\n";
            ctrl_cfg.motorch_to_pwmch[i] = 0; // 0 => default mapping
        } else {
            ctrl_cfg.motorch_to_pwmch[i] = map_val; // 0 or 1..N
        }

        ctrl_cfg.motor_reverse[i] = (cfg.motor_reverse[i] ? 1 : 0);
    }

    for (std::size_t i = n; i < static_cast<std::size_t>(PWM_HOST_CH_NUM); ++i) {
        ctrl_cfg.motorch_to_pwmch[i] = 0;
        ctrl_cfg.motor_reverse[i]    = 0;
    }
}

static int clamp_nonneg_int(int v) { return (v < 0) ? 0 : v; }

static const char* backend_str(bool dummy) noexcept { return dummy ? "DUMMY" : "STM32"; }

static float clampf(float v, float lo, float hi) noexcept
{
    return std::max(lo, std::min(v, hi));
}

} // namespace

// ============================================================================
// 析构 / 状态管理
// ============================================================================

PwmClient::~PwmClient()
{
    if (inited_) shutdown();
}

void PwmClient::set_error(int err_code, const std::string& msg)
{
    status_.ok             = false;
    status_.last_error     = err_code;
    status_.last_error_msg = msg;
}

void PwmClient::clear_error()
{
    status_.ok             = true;
    status_.last_error     = 0;
    status_.last_error_msg.clear();
}

// ============================================================================
// 初始化 / 关闭
// ============================================================================

bool PwmClient::init(const PwmClientConfig& cfg)
{
    if (inited_) shutdown();

    cfg_ = cfg;
    clear_error();

    // ------------------------------
    // Dummy backend
    // ------------------------------
    if (cfg_.dummy_backend) {
        dummy_  = true;
        inited_ = true;

        for (std::size_t i = 0; i < kNumPwmChannels; ++i) {
            target_pct_[i]  = cfg_.mid_pct;
            current_pct_[i] = cfg_.mid_pct;
        }

        std::cerr << "[PwmClient] backend=" << backend_str(dummy_)
                  << " dummy_print_frames=" << (cfg_.dummy_print_frames ? 1 : 0)
                  << " (no STM32)\n";
        return true;
    }

    dummy_ = false;
    debug_print_targets_ = cfg.debug_print_targets;

    // ------------------------------------------------------------------------
    // 1) libpwm_host init (UDP -> STM32)
    // ------------------------------------------------------------------------
    pwm_host_config_t host_cfg{};
    pwm_host_default_config(&host_cfg);

    // Override from cfg_.transport
    if (!cfg_.transport.stm32.ip.empty()) {
        host_cfg.stm32_ip = cfg_.transport.stm32.ip.c_str();
    }
    if (cfg_.transport.stm32.port != 0) {
        host_cfg.stm32_port = cfg_.transport.stm32.port;
    }

    if (cfg_.transport.send_hz > 0) {
        host_cfg.send_hz = clamp_nonneg_int(cfg_.transport.send_hz);
    }

    if (cfg_.transport.socket_sndbuf != 0) {
        host_cfg.socket_sndbuf = cfg_.transport.socket_sndbuf;
    }

    host_cfg.nonblock_send = cfg_.transport.nonblock_send ? 1 : 0;

    std::cerr << "[PwmClient] backend=" << backend_str(dummy_) << " libpwm_host target: "
              << (host_cfg.stm32_ip ? host_cfg.stm32_ip : "<null>")
              << ":" << host_cfg.stm32_port
              << " send_hz=" << host_cfg.send_hz
              << " nonblock_send=" << host_cfg.nonblock_send
              << " socket_sndbuf=" << host_cfg.socket_sndbuf
              << "\n";

    const pwmh_result_t rc_host = pwm_host_init(&host_cfg);
    if (rc_host != 0) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_host_init failed, rc=" << static_cast<int>(rc_host);
        set_error(static_cast<int>(rc_host), oss.str());
        pwm_host_close(); // defensive
        inited_ = false;
        return false;
    }

    // ------------------------------------------------------------------------
    // 2) pwm_control init (safety layer)
    // ------------------------------------------------------------------------
    pwm_ctrl_config_t ctrl_cfg;
    std::memset(&ctrl_cfg, 0, sizeof(ctrl_cfg));

    ctrl_cfg.ctrl_hz      = cfg_.ctrl_hz;
    ctrl_cfg.max_step_pct = cfg_.max_step_pct;
    ctrl_cfg.min_pct      = cfg_.min_pct;
    ctrl_cfg.mid_pct      = cfg_.mid_pct;
    ctrl_cfg.max_pct      = cfg_.max_pct;

    ctrl_cfg.enable_reverse_protection = cfg_.enable_reverse_protection ? 1 : 0;
    ctrl_cfg.groupA_mask               = static_cast<pwm_channel_mask_t>(cfg_.groupA_mask);
    ctrl_cfg.groupB_mask               = static_cast<pwm_channel_mask_t>(cfg_.groupB_mask);
    ctrl_cfg.group_mode                = static_cast<pwm_ctrl_group_mode_t>(cfg_.group_mode);

    sanitize_motor_mapping(cfg_, ctrl_cfg);

    const int rc_ctrl = pwm_ctrl_init(&ctrl_cfg);
    if (rc_ctrl != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_init failed, rc=" << rc_ctrl;
        set_error(rc_ctrl, oss.str());
        pwm_host_close();
        inited_ = false;
        return false;
    }

    inited_ = true;
    clear_error();

    std::cerr << "[PwmClient] backend=" << backend_str(dummy_) << " init OK\n";
    return true;
}

void PwmClient::shutdown()
{
    if (!inited_) return;

    if (dummy_) {
        std::cerr << "[PwmClient] shutdown backend=" << backend_str(dummy_) << "\n";
        inited_ = false;
        dummy_  = false;
        clear_error();
        return;
    }

    (void)pwm_ctrl_emergency_stop(1.0f);
    pwm_ctrl_deinit();
    pwm_host_close();

    std::cerr << "[PwmClient] shutdown backend=" << backend_str(dummy_) << "\n";
    inited_ = false;
    clear_error();
}

// ============================================================================
// 目标设置接口
// ============================================================================

int PwmClient::setAllMid()
{
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setAllMid: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    if (dummy_) {
        for (std::size_t i = 0; i < kNumPwmChannels; ++i) target_pct_[i] = cfg_.mid_pct;
        clear_error();
        return 0;
    }

    const int rc = pwm_ctrl_set_all_target_mid();
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_set_all_target_mid failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

int PwmClient::setTarget(int ch, float pct)
{
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setTarget: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    if (ch < 1 || ch > PWM_HOST_CH_NUM) {
        std::ostringstream oss;
        oss << "[PwmClient] setTarget: invalid channel " << ch
            << " (must be 1.." << PWM_HOST_CH_NUM << ")";
        set_error(PWM_CTRL_ERR_INVALID_ARG, oss.str());
        return PWM_CTRL_ERR_INVALID_ARG;
    }

    if (dummy_) {
        const float v = clampf(pct, cfg_.min_pct, cfg_.max_pct);
        target_pct_[static_cast<std::size_t>(ch - 1)] = v;
        clear_error();
        return 0;
    }

    const int rc = pwm_ctrl_set_target_pct(ch, pct);
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_set_target_pct(ch=" << ch
            << ", pct=" << pct << ") failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

int PwmClient::setTargets(const std::array<float, kNumPwmChannels>& cmd_norm)
{
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setTargets: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    constexpr float kDutyMin  = 5.0f;
    constexpr float kDutyMid  = 7.5f;
    constexpr float kDutyMax  = 10.0f;
    constexpr float kDutySpan = 2.5f; // kDutyMid ± kDutySpan

    auto clampf_local = [](float v, float lo, float hi) noexcept {
        return (v < lo) ? lo : (v > hi ? hi : v);
    };

    auto norm_to_duty = [&](float u) noexcept -> float {
        u = clampf_local(u, -1.0f, 1.0f);
        return clampf_local(kDutyMid + kDutySpan * u, kDutyMin, kDutyMax);
    };

    // ---------- Dummy 模式 ----------
    if (dummy_) {
        for (std::size_t i = 0; i < kNumPwmChannels; ++i) {
            target_pct_[i] = norm_to_duty(cmd_norm[i]);
        }

        if (debug_print_targets_) {
            std::cout << "[PwmClient][DUMMY_SET] cmd_norm=";
            for (std::size_t i = 0; i < kNumPwmChannels; ++i) {
                std::cout << cmd_norm[i] << (i + 1 == kNumPwmChannels ? "" : ",");
            }
            std::cout << " | duty=";
            for (std::size_t i = 0; i < kNumPwmChannels; ++i) {
                std::cout << target_pct_[i] << "%"
                          << (i + 1 == kNumPwmChannels ? "" : ",");
            }
            std::cout << "\n";
        }

        clear_error();
        return 0;
    }

    // ---------- 真正后端 ----------
    float arr[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        arr[i] = kDutyMid; // 默认中立
    }

    const std::size_t n =
        (kNumPwmChannels < static_cast<std::size_t>(PWM_HOST_CH_NUM))
            ? kNumPwmChannels
            : static_cast<std::size_t>(PWM_HOST_CH_NUM);

    for (std::size_t i = 0; i < n; ++i) {
        arr[i]        = norm_to_duty(cmd_norm[i]);
        target_pct_[i] = arr[i];
    }

    // ★ 调试：每次都清晰打印“输入”和“映射后 PWM”
    if (debug_print_targets_) {
        std::cout << "[PwmClient][SET_TARGETS] cmd_norm=";
        for (std::size_t i = 0; i < n; ++i) {
            std::cout << cmd_norm[i] << (i + 1 == n ? "" : ",");
        }
        std::cout << " | duty=";
        for (std::size_t i = 0; i < n; ++i) {
            std::cout << arr[i] << "%" << (i + 1 == n ? "" : ",");
        }
        std::cout << "\n";
    }

    const int rc = pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, arr);
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_set_targets_mask failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}



// ============================================================================
// step / emergencyStop / 反向开关
// ============================================================================

int PwmClient::step()
{
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] step: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    if (dummy_) {
        const float max_d = (cfg_.max_step_pct > 0.0f) ? cfg_.max_step_pct : 0.2f;

        for (std::size_t i = 0; i < kNumPwmChannels; ++i) {
            const float t = target_pct_[i];
            float&      c = current_pct_[i];

            float d = t - c;
            if (d >  max_d) d =  max_d;
            if (d < -max_d) d = -max_d;
            c += d;

            c = clampf(c, cfg_.min_pct, cfg_.max_pct);
        }

        if (cfg_.dummy_print_frames) {
            std::cerr << "[PwmClient][Dummy] current_pct=[";
            for (std::size_t i = 0; i < kNumPwmChannels; ++i) {
                std::cerr << current_pct_[i] << (i + 1 == kNumPwmChannels ? "" : ", ");
            }
            std::cerr << "]\n";
        }

        clear_error();
        return 0;
    }

    const int poll_rc = pwm_host_poll(1);
    if (poll_rc < 0) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_host_poll error, rc=" << poll_rc;
        status_.ok             = false;
        status_.last_error     = poll_rc;
        status_.last_error_msg = oss.str();
    }

    const int rc = pwm_ctrl_step();
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_step failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

int PwmClient::emergencyStop(float seconds)
{
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] emergencyStop: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    if (dummy_) {
        for (std::size_t i = 0; i < kNumPwmChannels; ++i) target_pct_[i] = cfg_.mid_pct;
        clear_error();
        return 0;
    }

    const int rc = pwm_ctrl_emergency_stop(seconds);
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_emergency_stop failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

int PwmClient::setMotorReverse(int motor_id, bool enable)
{
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setMotorReverse: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    if (motor_id < 1 || motor_id > PWM_HOST_CH_NUM) {
        std::ostringstream oss;
        oss << "[PwmClient] setMotorReverse: invalid motor_id " << motor_id
            << " (must be 1.." << PWM_HOST_CH_NUM << ")";
        set_error(PWM_CTRL_ERR_INVALID_ARG, oss.str());
        return PWM_CTRL_ERR_INVALID_ARG;
    }

    if (dummy_) {
        cfg_.motor_reverse[static_cast<std::size_t>(motor_id - 1)] = enable ? 1 : 0;
        clear_error();
        return 0;
    }

    const int rc = pwm_ctrl_set_motor_reverse(motor_id, enable ? 1 : 0);
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_set_motor_reverse(motor_id=" << motor_id
            << ", enable=" << (enable ? 1 : 0) << ") failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

// ============================================================================
// 最近一次“实际下发”占空比查询
// ============================================================================

bool PwmClient::getLastApplied(std::array<float, kNumPwmChannels>& out_pct)
{
    if (!inited_) return false;

    if (dummy_) {
        out_pct = current_pct_;
        return true;
    }

    pwm_ctrl_state_t st{};
    pwm_ctrl_get_state(&st);

    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        out_pct[static_cast<std::size_t>(i)] = st.current_pct[i];
    }

    return true;
}

} // namespace rovctrl::platform
