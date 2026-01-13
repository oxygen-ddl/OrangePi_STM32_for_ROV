#pragma once
#ifndef ROVCTRL_UTILS_DETAIL_CONFIG_LOG_HPP
#define ROVCTRL_UTILS_DETAIL_CONFIG_LOG_HPP

#include <ostream>
#include <string>

namespace rovctrl::utils::detail {

enum class LogLevel { Debug, Info, Warn, Err };

inline const char* level_str(LogLevel lv)
{
    switch (lv) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Err:   return "ERR";
        default:              return "INFO";
    }
}

inline void log_line(std::ostream& os,
                     const char*   tag,
                     LogLevel      lv,
                     const std::string& msg)
{
    os << "[" << tag << "] [" << level_str(lv) << "] " << msg << "\n";
}

} // namespace rovctrl::utils::detail

#endif
