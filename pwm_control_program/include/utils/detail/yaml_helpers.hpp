#pragma once
#ifndef ROVCTRL_UTILS_DETAIL_YAML_HELPERS_HPP
#define ROVCTRL_UTILS_DETAIL_YAML_HELPERS_HPP

#include <exception>
#include <ostream>
#include <string>
#include <yaml-cpp/yaml.h>

#include "utils/detail/config_log.hpp"

namespace rovctrl::utils::detail {

// yaml_as<T>(node, "path", out, log, "TAG", required)
template <typename T>
inline bool yaml_as(const YAML::Node& node,
                    const char*       path,
                    T&                out,
                    std::ostream&     log,
                    const char*       tag,
                    bool              required = true)
{
    if (!node) {
        if (required) {
            log_line(log, tag, LogLevel::Err,
                     std::string("Missing required node: ") + path);
        }
        return false;
    }
    try {
        out = node.as<T>();
        return true;
    } catch (const std::exception& e) {
        log_line(log, tag, LogLevel::Err,
                 std::string("Type conversion failed at '") + path + "': " + e.what());
        return false;
    }
}

inline double get_double_or(const YAML::Node& node,
                            const char*       key,
                            double            default_val)
{
    if (node && node[key]) return node[key].as<double>();
    return default_val;
}

} // namespace rovctrl::utils::detail

#endif
