#include <fstream>      // 必须放第一：避免被项目头文件宏污染
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "io/log/pwm_logger.hpp"

namespace fs = std::filesystem;

namespace rovctrl::io {

struct PwmLogger::Impl {
    std::ofstream ofs;
    Mode mode{Mode::CmdAndApplied};

    static std::string make_timestamp_()
    {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    static bool ensure_dir_(const fs::path& p)
    {
        std::error_code ec;
        if (fs::exists(p, ec)) return true;
        return fs::create_directories(p, ec);
    }

    void write_header_()
    {
        if (!ofs.is_open()) return;

        ofs << "t_s";
        if (mode == Mode::AppliedOnly) {
            for (int i = 1; i <= 8; ++i) ofs << ",ch" << i;
        } else {
            for (int i = 1; i <= 8; ++i) ofs << ",ch" << i << "_cmd";
            for (int i = 1; i <= 8; ++i) ofs << ",ch" << i << "_applied";
        }
        ofs << "\n";
        ofs.flush();
    }

    static void write_nan_(std::ofstream& ofs, int n)
    {
        for (int i = 0; i < n; ++i) ofs << ",nan";
    }
};

PwmLogger::PwmLogger() : impl_(std::make_unique<Impl>()) {}
PwmLogger::~PwmLogger() { close(); }

PwmLogger::PwmLogger(PwmLogger&&) noexcept = default;
PwmLogger& PwmLogger::operator=(PwmLogger&&) noexcept = default;

bool PwmLogger::init(const std::string& root_dir, Mode mode, const std::string& prefix)
{
    if (!impl_) impl_ = std::make_unique<Impl>();
    close();

    impl_->mode = mode;

    fs::path root(root_dir);
    if (!Impl::ensure_dir_(root)) {
        std::cerr << "[PwmLogger] ensure_dir failed: " << root << "\n";
        return false;
    }

    const std::string ts = Impl::make_timestamp_();
    fs::path file = root / (prefix + "_" + ts + ".csv");

    impl_->ofs.open(file, std::ios::out | std::ios::trunc);
    if (!impl_->ofs.is_open()) {
        std::cerr << "[PwmLogger] open failed: " << file << "\n";
        return false;
    }

    impl_->write_header_();
    return true;
}

void PwmLogger::close() noexcept
{
    if (impl_ && impl_->ofs.is_open()) {
        impl_->ofs.flush();
        impl_->ofs.close();
    }
}

bool PwmLogger::is_open() const noexcept
{
    return impl_ && impl_->ofs.is_open();
}

PwmLogger::Mode PwmLogger::mode() const noexcept
{
    return impl_ ? impl_->mode : Mode::CmdAndApplied;
}

void PwmLogger::logApplied(double t_s, const std::array<float, 8>& applied)
{
    if (!is_open()) return;

    auto& ofs = impl_->ofs;
    ofs << t_s;

    if (impl_->mode == Mode::AppliedOnly) {
        for (float v : applied) ofs << "," << v;
        ofs << "\n";
        return;
    }

    // CmdAndApplied: cmd 用 NaN 填充
    Impl::write_nan_(ofs, 8);
    for (float v : applied) ofs << "," << v;
    ofs << "\n";
}

void PwmLogger::logCmdAndApplied(double t_s,
                                 const std::array<float, 8>& cmd,
                                 const std::array<float, 8>& applied)
{
    if (!is_open()) return;

    if (impl_->mode == Mode::AppliedOnly) {
        logApplied(t_s, applied);
        return;
    }

    auto& ofs = impl_->ofs;
    ofs << t_s;
    for (float v : cmd)     ofs << "," << v;
    for (float v : applied) ofs << "," << v;
    ofs << "\n";
}

} // namespace rovctrl::io
