#pragma once

#include <array>
#include <fstream>
#include <string>

namespace rovctrl::io {

class PwmLogger {
public:
    PwmLogger() = default;
    ~PwmLogger();

    /// 在给定根目录下创建 logs 文件夹并生成带时间戳的 csv
    bool init(const std::string& root_dir);

    bool is_open() const noexcept { return static_cast<bool>(ofs_); }

    /// 写入一行：t_s,ch1..ch8
    void log(double t_s, const std::array<float, 8>& u);

private:
    std::ofstream ofs_;
};

} // namespace rovctrl::io
