#include "io/pwm_logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>

namespace fs = std::filesystem;

namespace rovctrl::io {

/*
 * PwmLogger 日志约定（改版）：
 *
 * 1. 传入 logApplied / logCmdAndApplied 的 cmd/applied 被视为
 *    “归一化指令量 u”，范围约在 [-1.0, 1.0]：
 *        u =  0   → 中位
 *        u = +1   → 最大前推
 *        u = -1   → 最大反推
 *
 * 2. 在 CSV 文件中，我们不再直接记录 u，而是记录“占空比百分数”
 *    duty_pct（单位：%），映射关系为：
 *
 *      PWM 频率 f ≈ 50 Hz，周期 T = 1/f ≈ 20 ms；
 *
 *      min_pct = 5.0  → τ_min = 0.05 * 20 ms ≈ 1.0 ms
 *      mid_pct = 7.5  → τ_mid = 0.075 * 20 ms ≈ 1.5 ms
 *      max_pct = 10.0 → τ_max = 0.10 * 20 ms ≈ 2.0 ms
 *
 *      u =  0   → duty_pct =  7.5 % → τ = 1.5 ms（中位）
 *      u = +1   → duty_pct = 10.0 % → τ = 2.0 ms（最大）
 *      u = -1   → duty_pct =  5.0 % → τ = 1.0 ms（最小）
 *
 *    这样，日志中的 ch1/ch2/.../ch8 值一眼就能对应物理脉宽：
 *
 *      duty_pct → τ_ms = duty_pct / 100 * 20 ms
 */

namespace {

// u ∈ [-1,1] → 占空比百分数（%）
static float norm_to_duty_pct(float u)
{
    constexpr float min_pct = 5.0f;
    constexpr float mid_pct = 7.5f;
    constexpr float max_pct = 10.0f;

    if (u > 1.0f)  u = 1.0f;
    if (u < -1.0f) u = -1.0f;

    if (u >= 0.0f) {
        // 0 ~ 1 映射到 [mid_pct, max_pct]
        return mid_pct + u * (max_pct - mid_pct);
    } else {
        // -1 ~ 0 映射到 [min_pct, mid_pct]
        return mid_pct + u * (mid_pct - min_pct);
    }
}

} // anonymous namespace

PwmLogger::~PwmLogger()
{
    if (ofs_.is_open()) {
        ofs_.flush();
        ofs_.close();
    }
}

bool PwmLogger::init(const std::string& root_dir,
                     Mode               mode,
                     const std::string& prefix)
{
    mode_ = mode;

    try {
        fs::create_directories(root_dir);
    } catch (const std::exception& e) {
        std::cerr << "[PwmLogger] create_directories failed: " << e.what() << "\n";
        return false;
    }

    // 生成时间戳文件名
    auto       now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    char time_buf[32];
    std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", &tm);

    // 文件名：<root_dir>/<prefix>_YYYYMMDD_HHMMSS.csv
    char file_buf[128];
    std::snprintf(file_buf, sizeof(file_buf),
                  "%s_%s.csv", prefix.c_str(), time_buf);

    fs::path filepath = fs::path(root_dir) / file_buf;

    ofs_.open(filepath, std::ios::out | std::ios::trunc);
    if (!ofs_) {
        std::cerr << "[PwmLogger] failed to open file: " << filepath << "\n";
        return false;
    }

    // 数字格式统一一下，方便后续分析
    ofs_ << std::fixed << std::setprecision(4);

    // 表头约定：
    //   - t_s 单位：秒
    //   - ch* / ch*_cmd / ch*_applied 单位：占空比百分数（%），例如 7.5 → 7.5%
    if (mode_ == Mode::AppliedOnly) {
        ofs_ << "t_s";
        for (int i = 1; i <= 8; ++i) {
            ofs_ << ",ch" << i;  // chX = 占空比百分数（%）
        }
        ofs_ << "\n";
    } else { // Mode::CmdAndApplied
        ofs_ << "t_s";
        for (int i = 1; i <= 8; ++i) {
            ofs_ << ",ch" << i << "_cmd";      // 期望占空比（%）
        }
        for (int i = 1; i <= 8; ++i) {
            ofs_ << ",ch" << i << "_applied";  // 实际占空比（%）
        }
        ofs_ << "\n";
    }

    ofs_.flush();
    std::cout << "[PwmLogger] logging to: " << filepath << "\n";
    return true;
}

void PwmLogger::logApplied(double t_s, const std::array<float, 8>& applied)
{
    if (!ofs_) {
        return;
    }

    ofs_ << t_s;

    if (mode_ == Mode::AppliedOnly) {
        // 仅记录“实际下发”的占空比百分数（由归一化指令 u 映射而来）
        for (std::size_t i = 0; i < applied.size(); ++i) {
            const float duty_pct = norm_to_duty_pct(applied[i]);
            ofs_ << "," << duty_pct;
        }
    } else {
        // Mode::CmdAndApplied：cmd 部分填 NaN，applied 部分填占空比百分数
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (std::size_t i = 0; i < applied.size(); ++i) {
            ofs_ << "," << nan;  // ch*_cmd 列，此时无有效值
        }
        for (std::size_t i = 0; i < applied.size(); ++i) {
            const float duty_pct = norm_to_duty_pct(applied[i]);
            ofs_ << "," << duty_pct; // ch*_applied 占空比百分数（%）
        }
    }

    ofs_ << "\n";
}

void PwmLogger::logCmdAndApplied(double t_s,
                                 const std::array<float, 8>& cmd,
                                 const std::array<float, 8>& applied)
{
    if (!ofs_) {
        return;
    }

    if (mode_ != Mode::CmdAndApplied) {
        // 当前模式不支持 cmd 列，退化为只记 applied
        logApplied(t_s, applied);
        return;
    }

    ofs_ << t_s;

    // 期望：归一化指令 → 占空比百分数
    for (std::size_t i = 0; i < cmd.size(); ++i) {
        const float duty_pct_cmd = norm_to_duty_pct(cmd[i]);
        ofs_ << "," << duty_pct_cmd;
    }
    // 实际：归一化指令 → 占空比百分数
    for (std::size_t i = 0; i < applied.size(); ++i) {
        const float duty_pct_applied = norm_to_duty_pct(applied[i]);
        ofs_ << "," << duty_pct_applied;
    }

    ofs_ << "\n";
}

} // namespace rovctrl::io
