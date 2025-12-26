#pragma once
#ifndef ROVCTRL_PLATFORM_PWM_CLIENT_HPP
#define ROVCTRL_PLATFORM_PWM_CLIENT_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace rovctrl::platform {

/// 逻辑电机通道数（与 STM32 / libpwm_host / pwm_control 保持一致：8 路）
inline constexpr std::size_t kNumPwmChannels = 8;

/**
 * @brief PWM 客户端配置（控制侧视角）
 *
 * 设计约束：
 *  - 头文件不包含 C 侧头文件（libpwm_host.h / pwm_control.h），避免污染上层依赖；
 *  - transport 作为 cfg 的成员存在（Scheme B），禁止在头文件里定义任何命名空间级全局对象。
 */
struct PwmClientConfig {
    // === 控制频率与斜率 ===
    float ctrl_hz      = 100.0f; ///< 上层调用 step() 的频率；推荐与你主控制循环一致
    float max_step_pct = 0.2f;   ///< 每步最大占空比变化（%），由安全层进行限斜率

    // === 占空比范围（与 STM32 约定：5%/7.5%/10% ≈ 1000/1500/2000us） ===
    float min_pct = 5.0f;
    float mid_pct = 7.5f;
    float max_pct = 10.0f;

    bool enable_reverse_protection = true; ///< 是否启用“禁止突然反向”保护

    // === 分组（逻辑电机空间掩码） ===
    // bit0→CH1, bit7→CH8
    std::uint8_t groupA_mask = 0x0F; ///< CH1-4
    std::uint8_t groupB_mask = 0xF0; ///< CH5-8
    int          group_mode  = 1;    ///< 对应 PWM_CTRL_GROUP_MODE_AB_ALTERNATE

    // === 映射与反向 ===
    // motorch_to_pwmch[i]：逻辑电机(i+1) → 物理PWM通道号
    //  - 合法：0 或 1..8；0 表示默认映射 i+1
    std::array<int, kNumPwmChannels> motorch_to_pwmch{0,0,0,0,0,0,0,0};

    // motor_reverse[i]：0 正向；非 0 反向（内部会归一为 0/1）
    std::array<std::uint8_t, kNumPwmChannels> motor_reverse{0,0,0,0,0,0,0,0};

    // === Dummy backend（无 STM32 测试用） ===
    bool dummy_backend      = false; ///< true: 不连接 STM32，不调用 C 后端，内部仿真 step/限斜率
    bool dummy_print_frames = false; ///< true: 每次 step 打印 8 路 current_pct

    // === Transport (Scheme B) ===
    // 注意：这里是“配置字段”，不是“全局对象”。不要在 namespace 作用域定义 transport{}。
    struct TransportConfig {
        struct Endpoint {
            std::string   ip{};
            std::uint16_t port{0}; // 0 = use lib default
        };

        // 默认直连 STM32（与你 YAML 一致）
        Endpoint stm32{ "192.168.2.16", static_cast<std::uint16_t>(8000) };

        // 可选：显式绑定本地网卡（仅当 libpwm_host 支持相关字段时才会用到）
        Endpoint orangepi{};

        // 传输参数
        int  send_hz{0};         // 0 = keep lib default
        int  socket_sndbuf{0};   // 0 = do not modify
        bool nonblock_send{false};
    };

    TransportConfig transport{}; // Scheme B：统一从 cfg.transport 读取
};

/**
 * @brief PWM 客户端状态（最近错误快照）
 */
struct PwmClientStatus {
    bool        ok          = false;  ///< 是否处于“工作正常”状态（最近一次操作成功）
    int         last_error  = 0;      ///< 最近一次错误代码（来自 pwm_control 或 libpwm_host）
    std::string last_error_msg;       ///< 简要错误信息（带来源描述）
};

/**
 * @brief 面向控制算法 / Teleop 的 PWM 客户端（C++ RAII 封装）
 *
 * 非线程安全：默认假定在单线程控制循环中使用；若未来需要多线程，需要外部加锁。
 */
class PwmClient {
public:
    PwmClient() = default;
    ~PwmClient();

    PwmClient(const PwmClient&)            = delete;
    PwmClient& operator=(const PwmClient&) = delete;

    /// 初始化 PWM 客户端（libpwm_host + pwm_control）
    bool init(const PwmClientConfig& cfg);

    /// 关闭 PWM 客户端（尽量安全归中位并释放底层资源）
    void shutdown();

    /// 将所有逻辑通道目标占空比设为中位（7.5%）；只改目标，不立即下发
    int setAllMid();

    /// 设置单通道（逻辑电机 1..8）的目标占空比（单位：%）
    int setTarget(int ch, float pct);

    /// 设置 8 路逻辑电机的目标占空比（单位：%）；只改目标，不立即下发
    int setTargets(const std::array<float, kNumPwmChannels>& pct);

    /// 执行一次“安全层 step + UDP 下发”
    int step();

    /// 紧急停车：在指定时间内平滑拉回中位
    int emergencyStop(float seconds);

    /// 运行时调整某个逻辑电机是否反向（1..8）
    int setMotorReverse(int motor_id, bool enable);

    /// 获取最近一次 step 后“逻辑电机视角”的当前占空比（单位：%）
    bool getLastApplied(std::array<float, kNumPwmChannels>& out_pct);

    const PwmClientStatus& status() const { return status_; }
    bool is_ok() const { return status_.ok; }

private:
    bool            inited_ = false;
    PwmClientConfig cfg_{};
    PwmClientStatus status_{};

    void set_error(int err_code, const std::string& msg);
    void clear_error();

    // === Dummy backend state ===
    bool dummy_ = false;
    std::array<float, kNumPwmChannels> target_pct_{};   // dummy: 目标 %
    std::array<float, kNumPwmChannels> current_pct_{};  // dummy: 当前 %
};

} // namespace rovctrl::platform

#endif // ROVCTRL_PLATFORM_PWM_CLIENT_HPP
