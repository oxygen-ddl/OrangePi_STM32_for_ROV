#include "io/pwm_logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>

namespace fs = std::filesystem;

namespace rovctrl::io {

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

    // 写表头
    if (mode_ == Mode::AppliedOnly) {
        ofs_ << "t_s";
        for (int i = 1; i <= 8; ++i) {
            ofs_ << ",ch" << i;
        }
        ofs_ << "\n";
    } else { // Mode::CmdAndApplied
        ofs_ << "t_s";
        for (int i = 1; i <= 8; ++i) {
            ofs_ << ",ch" << i << "_cmd";
        }
        for (int i = 1; i <= 8; ++i) {
            ofs_ << ",ch" << i << "_applied";
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
        // 仅记录实际下发值
        for (std::size_t i = 0; i < applied.size(); ++i) {
            ofs_ << "," << applied[i];
        }
    } else {
        // Mode::CmdAndApplied：cmd 部分填 NaN，applied 部分填实际值
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (std::size_t i = 0; i < applied.size(); ++i) {
            ofs_ << "," << nan;          // ch*_cmd
        }
        for (std::size_t i = 0; i < applied.size(); ++i) {
            ofs_ << "," << applied[i];   // ch*_applied
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

    for (std::size_t i = 0; i < cmd.size(); ++i) {
        ofs_ << "," << cmd[i];          // ch*_cmd
    }
    for (std::size_t i = 0; i < applied.size(); ++i) {
        ofs_ << "," << applied[i];      // ch*_applied
    }

    ofs_ << "\n";
}

} // namespace rovctrl::io
