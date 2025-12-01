#include "platform/pwm_client.hpp"

#include <sstream>
#include <cstring>

// C 接口头文件只在实现层可见
extern "C" {
#include "libpwm_host.h"
#include "pwm_control.h"
}

namespace rovctrl::platform {

PwmClient::~PwmClient() {
    // RAII：对象析构时自动做一次安全关闭
    if (inited_) {
        shutdown();
    }
}

void PwmClient::set_error(int err_code, const std::string& msg) {
    status_.ok            = false;
    status_.last_error    = err_code;
    status_.last_error_msg = msg;
}

void PwmClient::clear_error() {
    status_.ok            = true;
    status_.last_error    = 0;
    status_.last_error_msg.clear();
}

bool PwmClient::init(const PwmClientConfig& cfg) {
    // 如果已经初始化，先关掉旧的
    if (inited_) {
        shutdown();
    }

    cfg_ = cfg;
    clear_error();

    // 1) 初始化 libpwm_host（使用默认配置，后续如需自定义 IP/端口再扩展）
    pwm_host_config_t host_cfg;
    pwm_host_default_config(&host_cfg);

    // TODO（如有需要）：这里可以从 cfg_ 补充 host_cfg.stm32_ip / stm32_port

    pwmh_result_t rc_host = pwm_host_init(&host_cfg);
    if (rc_host != PWMH_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_host_init failed, rc=" << rc_host
            << " (" << pwm_host_strerror(rc_host) << ")";
        set_error(-rc_host, oss.str());
        inited_ = false;
        return false;
    }

    // 2) 初始化 pwm_control 安全层
    pwm_ctrl_config_t ctrl_cfg;
    std::memset(&ctrl_cfg, 0, sizeof(ctrl_cfg));

    ctrl_cfg.ctrl_hz      = cfg_.ctrl_hz;
    ctrl_cfg.max_step_pct = cfg_.max_step_pct;
    ctrl_cfg.min_pct      = cfg_.min_pct;
    ctrl_cfg.mid_pct      = cfg_.mid_pct;
    ctrl_cfg.max_pct      = cfg_.max_pct;

    ctrl_cfg.enable_reverse_protection = cfg_.enable_reverse_protection ? 1 : 0;
    ctrl_cfg.groupA_mask = static_cast<pwm_channel_mask_t>(cfg_.groupA_mask);
    ctrl_cfg.groupB_mask = static_cast<pwm_channel_mask_t>(cfg_.groupB_mask);
    ctrl_cfg.group_mode  = static_cast<pwm_ctrl_group_mode_t>(cfg_.group_mode);

    // 逻辑电机映射 / 反向标志
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        ctrl_cfg.motorch_to_pwmch[i] = cfg_.motorch_to_pwmch[i];
        ctrl_cfg.motor_reverse[i]    = cfg_.motor_reverse[i];
    }

    int rc_ctrl = pwm_ctrl_init(&ctrl_cfg);
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
    return true;
}

void PwmClient::shutdown() {
    if (!inited_) {
        return;
    }

    // 1) 尝试安全归中位（忽略错误，毕竟是在退出）
    (void)pwm_ctrl_emergency_stop(1.0f);

    // 2) 关闭安全层与 UDP 库
    pwm_ctrl_deinit();
    pwm_host_close();

    inited_ = false;
    clear_error();
}

int PwmClient::setAllMid() {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setAllMid: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    int rc = pwm_ctrl_set_all_target_mid();
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_set_all_target_mid failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

int PwmClient::setTarget(int ch, float pct) {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setTarget: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    int rc = pwm_ctrl_set_target_pct(ch, pct);
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

int PwmClient::setTargets(const std::array<float, kNumPwmChannels>& pct) {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setTargets: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    float arr[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        arr[i] = pct[static_cast<std::size_t>(i)];
    }

    int rc = pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, arr);
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_set_targets_mask failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

int PwmClient::step() {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] step: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    // 先处理一下心跳 ACK / 统计信息（非阻塞）
    int poll_rc = pwm_host_poll(0);
    if (poll_rc < 0) {
        // 这里视为“软性告警”：记录错误信息，但不直接把客户端标为失效
        std::ostringstream oss;
        oss << "[PwmClient] pwm_host_poll error, rc=" << poll_rc;
        status_.last_error     = poll_rc;
        status_.last_error_msg = oss.str();
        // status_.ok 保持当前值，让控制层 step 尽量继续执行
    }

    int rc = pwm_ctrl_step();
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_step failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    // 只有核心控制 step 成功，才认为客户端处于“健康”状态
    clear_error();
    return rc;
}

int PwmClient::emergencyStop(float seconds) {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] emergencyStop: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    int rc = pwm_ctrl_emergency_stop(seconds);
    if (rc != PWM_CTRL_OK) {
        std::ostringstream oss;
        oss << "[PwmClient] pwm_ctrl_emergency_stop failed, rc=" << rc;
        set_error(rc, oss.str());
        return rc;
    }

    clear_error();
    return rc;
}

int PwmClient::setMotorReverse(int motor_id, bool enable) {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setMotorReverse: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    int rc = pwm_ctrl_set_motor_reverse(motor_id, enable ? 1 : 0);
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

} // namespace rovctrl::platform
