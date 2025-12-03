#include "io/pwm_logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace fs = std::filesystem;

namespace rovctrl::io {

PwmLogger::~PwmLogger()
{
    if (ofs_.is_open()) {
        ofs_.flush();
        ofs_.close();
    }
}

bool PwmLogger::init(const std::string& root_dir)
{
    try {
        fs::create_directories(root_dir);
    } catch (const std::exception& e) {
        std::cerr << "[PwmLogger] create_directories failed: " << e.what() << "\n";
        return false;
    }

    // 生成时间戳文件名
    auto now       = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    char buf[64];
    std::strftime(buf, sizeof(buf), "pwm_log_%Y%m%d_%H%M%S.csv", &tm);

    fs::path filepath = fs::path(root_dir) / buf;

    ofs_.open(filepath, std::ios::out | std::ios::trunc);
    if (!ofs_) {
        std::cerr << "[PwmLogger] failed to open file: " << filepath << "\n";
        return false;
    }

    // 写表头
    ofs_ << "t_s,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8\n";
    ofs_.flush();

    std::cout << "[PwmLogger] logging to: " << filepath << "\n";
    return true;
}

void PwmLogger::log(double t_s, const std::array<float, 8>& u)
{
    if (!ofs_) {
        return;
    }
    ofs_ << t_s;
    for (std::size_t i = 0; i < u.size(); ++i) {
        ofs_ << "," << u[i];
    }
    ofs_ << "\n";
}

} // namespace rovctrl::io
