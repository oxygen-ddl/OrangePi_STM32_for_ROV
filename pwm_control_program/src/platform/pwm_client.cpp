#include "platform/pwm_client.hpp"

#include <sstream>
#include <cstring>
#include <iostream>

// C 接口头文件只在实现层可见
extern "C" {
#include "libpwm_host.h"
#include "pwm_control.h"
}

namespace rovctrl::platform {

namespace {

// 简单工具：检查并规范 motorch_to_pwmch / motor_reverse
void sanitize_motor_mapping(const PwmClientConfig& cfg, pwm_ctrl_config_t& ctrl_cfg)
{
    // 说明：
    //  - motorch_to_pwmch[i] 语义：逻辑电机 (i+1) → 物理 PWM 通道号
    //  - 合法范围：1..PWM_HOST_CH_NUM；0 表示“默认映射 i+1，由 pwm_control 自行处理”
    //  - motor_reverse[i]：0 正向，非 0 视为反向（统一归一为 0/1）

    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        int map_val = cfg.motorch_to_pwmch[i];

        if (map_val < 0 || map_val > PWM_HOST_CH_NUM) {
            // 非法映射：打印告警并回退为 0（由 pwm_control 使用默认 1..N）
            std::cerr << "[PwmClient] WARN: motorch_to_pwmch[" << i
                      << "] = " << map_val
                      << " out of range, fallback to 0 (default mapping)\n";
            ctrl_cfg.motorch_to_pwmch[i] = 0;  // 0 -> 由 pwm_control 解释为默认
        } else {
            ctrl_cfg.motorch_to_pwmch[i] = map_val; // 0 或 1..N
        }

        // 归一 motor_reverse：0 正向，其它都视为 1
        ctrl_cfg.motor_reverse[i] = (cfg.motor_reverse[i] ? 1 : 0);
    }
}

} // anonymous namespace

PwmClient::~PwmClient() {
    // RAII：对象析构时自动做一次安全关闭
    if (inited_) {
        shutdown();
    }
}

void PwmClient::set_error(int err_code, const std::string& msg) {
    status_.ok             = false;
    status_.last_error     = err_code;
    status_.last_error_msg = msg;
}

void PwmClient::clear_error() {
    status_.ok             = true;
    status_.last_error     = 0;
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
    pwm_host_config_t host_cfg{};
    pwm_host_default_config(&host_cfg);

    // 如有需要，可在这里根据 cfg_ 设置 host_cfg.stm32_ip / stm32_port 等
    // host_cfg.stm32_port = ...;
    // std::snprintf(host_cfg.stm32_ip, sizeof(host_cfg.stm32_ip), "%s", cfg_.stm32_ip.c_str());

    const pwmh_result_t rc_host = pwm_host_init(&host_cfg);
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

    // 逻辑电机 ↔ 物理通道映射 + 方向反转归一
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

int PwmClient::setTarget(int ch, float pct) {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setTarget: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    // 通道号保护：底层 API 以 1..PWM_HOST_CH_NUM 为合法范围
    if (ch < 1 || ch > PWM_HOST_CH_NUM) {
        std::ostringstream oss;
        oss << "[PwmClient] setTarget: invalid channel " << ch
            << " (must be 1.." << PWM_HOST_CH_NUM << ")";
        set_error(PWM_CTRL_ERR_INVALID_ARG, oss.str());
        return PWM_CTRL_ERR_INVALID_ARG;
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

int PwmClient::setTargets(const std::array<float, kNumPwmChannels>& pct) {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setTargets: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    float arr[PWM_HOST_CH_NUM];
    for (int i = 0; i < PWM_HOST_CH_NUM; ++i) {
        arr[i] = pct[static_cast<std::size_t>(i)];
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

int PwmClient::step() {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] step: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    // 先处理一下心跳 ACK / 统计信息（非阻塞）
    const int poll_rc = pwm_host_poll(1);
    if (poll_rc < 0) {
        // 软性告警：记录错误但不直接标记为失效，交给上层决定是否终止
        std::ostringstream oss;
        oss << "[PwmClient] pwm_host_poll error, rc=" << poll_rc;
        status_.last_error     = poll_rc;
        status_.last_error_msg = oss.str();
        // status_.ok 保持当前值
    }

    const int rc = pwm_ctrl_step();
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

int PwmClient::setMotorReverse(int motor_id, bool enable) {
    if (!inited_) {
        set_error(PWM_CTRL_ERR_NOT_INIT, "[PwmClient] setMotorReverse: not initialized");
        return PWM_CTRL_ERR_NOT_INIT;
    }

    // motor_id 语义：逻辑电机编号 1..8
    if (motor_id < 1 || motor_id > PWM_HOST_CH_NUM) {
        std::ostringstream oss;
        oss << "[PwmClient] setMotorReverse: invalid motor_id " << motor_id
            << " (must be 1.." << PWM_HOST_CH_NUM << ")";
        set_error(PWM_CTRL_ERR_INVALID_ARG, oss.str());
        return PWM_CTRL_ERR_INVALID_ARG;
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

} // namespace rovctrl::platform
