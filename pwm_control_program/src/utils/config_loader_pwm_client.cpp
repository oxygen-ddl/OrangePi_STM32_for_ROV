// pwm_control_program/src/utils/config_loader_pwm_client.cpp

#include "utils/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <ostream>
#include <string>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include "platform/pwm_client.hpp"
#include "utils/detail/config_log.hpp"

namespace rovctrl::utils {

namespace fs = std::filesystem;
using detail::LogLevel;
using detail::log_line;

namespace {

static std::uint16_t clamp_u16_port(int v, std::uint16_t fallback = 0)
{
    if (v <= 0 || v > 65535) return fallback;
    return static_cast<std::uint16_t>(v);
}

static std::string to_lower_ascii(std::string s)
{
    for (auto& ch : s) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return s;
}

// Accept: bool / int / string ("0","1","true","false","on","off","yes","no")
static bool parse_bool_flexible(const YAML::Node& n, bool fallback)
{
    if (!n) return fallback;

    try {
        // 1) bool
        try { return n.as<bool>(); } catch (...) {}

        // 2) int
        try { return n.as<int>() != 0; } catch (...) {}

        // 3) string
        try {
            const std::string raw = n.as<std::string>();
            const std::string s   = to_lower_ascii(raw);

            if (s == "1" || s == "true" || s == "on"  || s == "yes") return true;
            if (s == "0" || s == "false"|| s == "off" || s == "no")  return false;
        } catch (...) {}
    } catch (...) {
        // ignore
    }

    return fallback;
}

static std::uint8_t parse_u8_mask(const YAML::Node& n, std::uint8_t fallback)
{
    if (!n) return fallback;
    try {
        const int v = n.as<int>();
        return static_cast<std::uint8_t>(v & 0xFF);
    } catch (...) {
        return fallback;
    }
}

static int parse_nonneg_int(const YAML::Node& n, int fallback)
{
    if (!n) return fallback;
    try {
        const int v = n.as<int>();
        return (v >= 0) ? v : 0;
    } catch (...) {
        return fallback;
    }
}

} // namespace

bool load_pwm_client_config(const fs::path&             path,
                            platform::PwmClientConfig& cfg,
                            std::ostream&              log)
{
    try {
        log_line(log, "PwmClient", LogLevel::Info,
                 std::string("Loading pwm_client.yaml: ") + path.string());

        YAML::Node root = YAML::LoadFile(path.string());

        auto get_scalar = [&](const char* key, auto& target) {
            if (!root[key]) return;
            using T = std::decay_t<decltype(target)>;
            try {
                target = root[key].as<T>();
            } catch (const std::exception& e) {
                log_line(log, "PwmClient", LogLevel::Warn,
                         std::string("Key '") + key + "' parse failed: " + e.what());
            }
        };

        // ------------------------------------------------------------
        // Top-level scalars
        // ------------------------------------------------------------
        get_scalar("ctrl_hz",      cfg.ctrl_hz);
        get_scalar("max_step_pct", cfg.max_step_pct);
        get_scalar("min_pct",      cfg.min_pct);
        get_scalar("mid_pct",      cfg.mid_pct);
        get_scalar("max_pct",      cfg.max_pct);

        if (root["enable_reverse_protection"]) {
            cfg.enable_reverse_protection =
                parse_bool_flexible(root["enable_reverse_protection"], cfg.enable_reverse_protection);
        }

        if (root["dummy_backend"]) {
            cfg.dummy_backend = parse_bool_flexible(root["dummy_backend"], cfg.dummy_backend);
        }
        if (root["dummy_print_frames"]) {
            cfg.dummy_print_frames = parse_bool_flexible(root["dummy_print_frames"], cfg.dummy_print_frames);
        }

        if (root["groupA_mask"]) cfg.groupA_mask = parse_u8_mask(root["groupA_mask"], cfg.groupA_mask);
        if (root["groupB_mask"]) cfg.groupB_mask = parse_u8_mask(root["groupB_mask"], cfg.groupB_mask);

        if (root["group_mode"]) {
            try { cfg.group_mode = root["group_mode"].as<int>(); } catch (...) {}
        }

        // ------------------------------------------------------------
        // Arrays
        // ------------------------------------------------------------
        if (root["motorch_to_pwmch"]) {
            const auto node = root["motorch_to_pwmch"];
            if (!node.IsSequence()) {
                log_line(log, "PwmClient", LogLevel::Err, "'motorch_to_pwmch' must be a sequence.");
                return false;
            }

            const std::size_t n = std::min<std::size_t>(node.size(), platform::kNumPwmChannels);
            for (std::size_t i = 0; i < n; ++i) {
                try {
                    cfg.motorch_to_pwmch[i] = node[i].as<int>();
                } catch (const std::exception& e) {
                    log_line(log, "PwmClient", LogLevel::Warn,
                             "motorch_to_pwmch[" + std::to_string(i) + "] parse failed: " + e.what());
                }
            }
        }

        if (root["motor_reverse"]) {
            const auto node = root["motor_reverse"];
            if (!node.IsSequence()) {
                log_line(log, "PwmClient", LogLevel::Err, "'motor_reverse' must be a sequence.");
                return false;
            }

            const std::size_t n = std::min<std::size_t>(node.size(), platform::kNumPwmChannels);
            for (std::size_t i = 0; i < n; ++i) {
                // 支持 0/1、true/false、"on/off"
                cfg.motor_reverse[i] = parse_bool_flexible(node[i], cfg.motor_reverse[i] != 0) ? 1 : 0;
            }
        }

        // ------------------------------------------------------------
        // transport (optional) - Scheme B
        // ------------------------------------------------------------
        if (root["transport"]) {
            const auto t = root["transport"];

            // stm32 endpoint
            if (t["stm32"]) {
                const auto s = t["stm32"];
                if (s["ip"]) {
                    try { cfg.transport.stm32.ip = s["ip"].as<std::string>(); } catch (...) {}
                }
                if (s["port"]) {
                    cfg.transport.stm32.port =
                        clamp_u16_port(s["port"].as<int>(), cfg.transport.stm32.port);
                }
            }

            // orangepi bind (optional)
            if (t["orangepi"] && t["orangepi"]["ip"]) {
                try { cfg.transport.orangepi.ip = t["orangepi"]["ip"].as<std::string>(); } catch (...) {}
            }

            // knobs
            if (t["send_hz"]) {
                const int v = parse_nonneg_int(t["send_hz"], cfg.transport.send_hz);
                cfg.transport.send_hz = (v > 0) ? v : 0;
            }
            if (t["socket_sndbuf"]) {
                // 0 allowed => do not modify
                cfg.transport.socket_sndbuf = parse_nonneg_int(t["socket_sndbuf"], cfg.transport.socket_sndbuf);
            }
            if (t["nonblock_send"]) {
                cfg.transport.nonblock_send =
                    parse_bool_flexible(t["nonblock_send"], cfg.transport.nonblock_send);
            }
        }

        log_line(log, "PwmClient", LogLevel::Info,
                 std::string("pwm_client.yaml loaded OK. backend=") +
                 (cfg.dummy_backend ? "DUMMY" : "STM32") +
                 (cfg.dummy_backend && cfg.dummy_print_frames ? " (print=on)" : ""));

        return true;
    } catch (const std::exception& e) {
        log_line(log, "PwmClient", LogLevel::Err,
                 std::string("Failed to load pwm_client.yaml: ") + e.what());
        return false;
    }
}

} // namespace rovctrl::utils
