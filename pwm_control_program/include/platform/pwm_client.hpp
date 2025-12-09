#pragma once
#ifndef ROVCTRL_PLATFORM_PWM_CLIENT_HPP
#define ROVCTRL_PLATFORM_PWM_CLIENT_HPP

#include <array>
#include <cstdint>
#include <string>

namespace rovctrl::platform {

/// 逻辑电机通道数（与 STM32 / libpwm_host / pwm_control 保持一致：8 路）
inline constexpr std::size_t kNumPwmChannels = 8;

/**
 * @brief PWM 客户端配置（控制侧视角）
 *
 * 说明：
 *   - 本结构与 pwm_control.h 中的 pwm_ctrl_config_t 一一对应，但以 C++ 形式暴露；
 *   - 不在头文件中直接包含 C 头（libpwm_host.h / pwm_control.h），
 *     所有 C 依赖都放在 cpp 里，避免上层被 C API 污染；
 *   - 字段都有工程级默认值，上层可以只改局部参数。
 */
struct PwmClientConfig {
    // === 控制频率与斜率 ===
    float ctrl_hz          = 100.0f;  ///< 上层调用 step() 的频率；推荐与你主控制循环一致
    float max_step_pct     = 0.2f;    ///< 每步最大占空比变化（%），由安全层进行限斜率

    // === 占空比范围（与 STM32 约定：5%/7.5%/10% ≈ 1000/1500/2000us） ===
    float min_pct          = 5.0f;
    float mid_pct          = 7.5f;
    float max_pct          = 10.0f;

    bool  enable_reverse_protection = true; ///< 是否启用“禁止突然反向”保护

    // === 分组（逻辑电机空间掩码） ===
    // bit0→CH1, bit7→CH8
    uint8_t groupA_mask    = 0x0F;    ///< CH1-4
    uint8_t groupB_mask    = 0xF0;    ///< CH5-8
    int     group_mode     = 1;       ///< 对应 PWM_CTRL_GROUP_MODE_AB_ALTERNATE

    // === 逻辑电机 → 物理 PWM 通道映射（1..8，0 表示使用默认 1..N）===
    int     motorch_to_pwmch[kNumPwmChannels] = {0,0,0,0,0,0,0,0};

    // === 电机方向反向标志：0 正向，非 0 反向 ===
    uint8_t motor_reverse[kNumPwmChannels]    = {0,0,0,0,0,0,0,0};
};

/**
 * @brief PWM 客户端状态（最近错误快照）
 */
struct PwmClientStatus {
    bool        ok          = false;      ///< 是否处于“工作正常”状态（最近一次操作成功）
    int         last_error  = 0;          ///< 最近一次错误代码（来自 pwm_control 或 libpwm_host）
    std::string last_error_msg;           ///< 简要错误信息（带来源描述）
};

/**
 * @brief 面向控制算法 / Teleop 的 PWM 客户端（C++ RAII 封装）
 *
 * 职责：
 *   - 初始化/关闭 libpwm_host 与 pwm_control 安全层；
 *   - 提供“设中位”、“设目标占空比数组”、“单步 step”、“紧急停车”等接口；
 *   - 将底层 C API 的错误状态封装成 C++ 形式，便于上层记录日志和决策；
 *   - 提供查询最近一次“实际下发占空比”的接口，便于日志与诊断；
 *
 * 非线程安全：
 *   - 默认假定在单线程控制循环中使用；若未来需要多线程，需要外部加锁。
 */
class PwmClient {
public:
    PwmClient() = default;
    ~PwmClient();

    PwmClient(const PwmClient&)            = delete;
    PwmClient& operator=(const PwmClient&) = delete;

    /**
     * @brief 初始化 PWM 客户端（libpwm_host + pwm_control）
     *
     * @return true 初始化成功；false 失败（可通过 status() 查看错误信息）
     *
     * 行为说明：
     *   - 内部会调用 libpwm_host_init(...) 打开 UDP 连接；
     *   - 根据 cfg 填充 pwm_ctrl_config_t 并调用 pwm_ctrl_init(...)；
     *   - 初始化成功后会推送一次“全部中位”的 PWM 帧到 STM32；
     *   - 多次重复调用会先自动 shutdown() 再重新 init()。
     */
    bool init(const PwmClientConfig& cfg);

    /**
     * @brief 关闭 PWM 客户端
     *
     * 行为：
     *   - 尽量调用 emergencyStop(1.0f) 将所有通道平滑拉回中位；
     *   - 调用 pwm_ctrl_deinit()；
     *   - 调用 pwm_host_close()；
     *   - 清空内部状态标记（inited_ = false, status_.ok = false）。
     */
    void shutdown();

    /**
     * @brief 将所有逻辑通道目标占空比设为中位（7.5%）
     *
     * @return 0 或正数：成功；负数：错误（同时写入 status_）
     *
     * 说明：
     *   - 只修改“逻辑目标值”，不会立即下发；下发由后续 step() 完成。
     */
    int setAllMid();

    /**
     * @brief 设置单通道（逻辑电机）的目标占空比（单位：%）
     *
     * @param ch  逻辑通道号（1..8）
     * @param pct 目标占空比（会在安全层裁剪到 [min_pct, max_pct]；pct < 0 视为 mid_pct）
     *
     * 典型用途：
     *   - 调试单个电机；
     *   - 某些模式想对部分通道做微调。
     */
    int setTarget(int ch, float pct);

    /**
     * @brief 设置 8 路逻辑电机的目标占空比（单位：%）
     *
     * @param pct 长度为 8 的数组，对应逻辑电机 1..8；
     *            数值会在安全层中裁剪到 [min_pct, max_pct]。
     *
     * 说明：
     *   - 此调用只设置目标值，不会立即下发；需要后续调用 step() 才会逐步下发；
     *   - 对应 pwm_ctrl_set_targets_mask(PWM_CH_MASK_ALL, ...) 的 C 封装。
     */
    int setTargets(const std::array<float, kNumPwmChannels>& pct);

    /**
     * @brief 执行一次“安全层 step + UDP 下发”的循环
     *
     * 行为：
     *   - 内部调用 pwm_ctrl_step() 完成限斜率 / 分组 / 映射 / 反向并下发一帧 PWM；
     *   - 同时让底层 libpwm_host 处理 ACK 等（等价于周期性轮询）；
     *
     * 典型调用频率：
     *   - cfg.ctrl_hz = 100.0f 时，在主控制循环中以 100Hz 调用本函数；
     *   - 时间控制由上层负责（例如使用 sleep_until）。
     */
    int step();

    /**
     * @brief 紧急停车：在指定时间内平滑拉回中位
     *
     * @param seconds 期望收敛时间（秒），<=0 表示“尽快”（仍受 max_step_pct 限制）
     *
     * 典型使用场景：
     *   - 程序退出前 / ROV 上岸前；
     *   - 控制器异常、传感器失效、通讯异常时。
     */
    int emergencyStop(float seconds);

    /**
     * @brief 运行时调整某个逻辑电机是否反向
     *
     * @param motor_id 逻辑电机编号（1..8）
     * @param enable   true: 反向；false: 正向
     *
     * 对应底层 pwm_ctrl_set_motor_reverse() 封装。
     */
    int setMotorReverse(int motor_id, bool enable);

    /**
     * @brief 获取最近一次安全层 step 后“逻辑电机视角”的当前占空比
     *
     * @param out_pct 长度为 8 的数组，返回 current_pct（单位：%），
     *                即 pwm_ctrl_get_state() 中的 current_pct[]。
     *
     * @return true 成功获取；false 客户端未初始化或底层返回错误
     *
     * 典型用途：
     *   - 与控制器输出（目标 cmd）一起写入日志，用于分析限斜率 / 反向保护等效果；
     *   - 调试 AB 分组、映射和反向逻辑；
     *   - 运行中做健康检查或安全监控。
     */
    bool getLastApplied(std::array<float, kNumPwmChannels>& out_pct);

    /**
     * @brief 查询当前状态（错误快照）
     */
    const PwmClientStatus& status() const { return status_; }

    /**
     * @brief 客户端是否“健康”
     *
     * 条件：
     *   - 已成功 init()；
     *   - 最近一次操作未报错。
     */
    bool is_ok() const { return status_.ok; }

private:
    bool            inited_ = false;
    PwmClientConfig cfg_{};
    PwmClientStatus status_{};

    /// 记录错误并将 ok 置为 false
    void set_error(int err_code, const std::string& msg);

    /// 清除错误状态（一般在 init 成功或 emergencyStop 成功后调用）
    void clear_error();
};

} // namespace rovctrl::platform

#endif // ROVCTRL_PLATFORM_PWM_CLIENT_HPP
