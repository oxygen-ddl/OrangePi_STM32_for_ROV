#pragma once

#include <array>
#include <fstream>
#include <string>

namespace rovctrl::io {

/**
 * @brief PWM 日志记录器
 *
 * 设计目标：
 *  - 简单可靠：控制循环每步调用一次 log*() 即可；
 *  - 支持两种模式：
 *      1) 仅记录“安全层实际下发值”（applied）；
 *      2) 同时记录“控制器输出 cmd”和“安全层实际下发值 applied”；
 *  - 输出 CSV 文件，方便 Python / Excel 后处理。
 */
class PwmLogger {
public:
    /// 日志模式
    enum class Mode {
        AppliedOnly,      ///< 仅记录实际下发值：t_s,ch1..ch8
        CmdAndApplied     ///< 同时记录：t_s,ch1_cmd..ch8_cmd,ch1_applied..ch8_applied
    };

    PwmLogger() = default;
    ~PwmLogger();

    /**
     * @brief 初始化日志文件
     *
     * @param root_dir 日志根目录（如 "./logs"），内部会在其下创建文件。
     * @param mode     日志模式（默认：CmdAndApplied，同时记录 cmd 和 applied）
     * @param prefix   文件名前缀（默认 "pwm_log"）
     *
     * 行为：
     *  - 若 root_dir 不存在，尝试创建；
     *  - 文件名形如：root_dir/prefix_YYYYMMDD_HHMMSS.csv；
     *  - 根据 mode 写入不同的表头。
     */
    bool init(const std::string& root_dir,
              Mode               mode   = Mode::CmdAndApplied,
              const std::string& prefix = "pwm_log");

    /// 是否已成功打开文件
    bool is_open() const noexcept { return static_cast<bool>(ofs_); }

    /**
     * @brief 记录一条“实际下发值”样本
     *
     * @param t_s      相对时间（秒），由调用方定义时间零点
     * @param applied  安全层当前实际下发的 8 路归一化指令（或占空比）
     *
     * 说明：
     *  - 在 Mode::AppliedOnly 下：写一行 t_s,ch1..ch8；
     *  - 在 Mode::CmdAndApplied 下：写
     *      t_s,NaN..NaN,ch1_applied..ch8_applied （cmd 部分填 NaN）。
     */
    void logApplied(double t_s, const std::array<float, 8>& applied);

    /**
     * @brief 同时记录“控制器输出 cmd”和“安全层实际下发值 applied”
     *
     * @param t_s      相对时间（秒）
     * @param cmd      控制器输出的 8 路指令（通常是 DOF→thruster 映射后的结果）
     * @param applied  安全层当前实际下发的 8 路指令
     *
     * 说明：
     *  - 推荐在 Mode::CmdAndApplied 模式下使用；
     *  - 表头为：
     *      t_s,ch1_cmd,...,ch8_cmd,ch1_applied,...,ch8_applied。
     *  - 若当前模式为 AppliedOnly，则退化为调用 logApplied()。
     */
    void logCmdAndApplied(double t_s,
                          const std::array<float, 8>& cmd,
                          const std::array<float, 8>& applied);

private:
    std::ofstream ofs_;
    Mode          mode_{Mode::CmdAndApplied};
};

} // namespace rovctrl::io
