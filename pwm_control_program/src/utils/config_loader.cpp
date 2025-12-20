// pwm_control_program/src/utils/config_loader.cpp

#include "utils/config_loader.hpp"

#include <cstdlib>      // std::getenv
#include <exception>
#include <filesystem>
#include <initializer_list>
#include <string>

#include "utils/detail/config_log.hpp"

namespace rovctrl::utils {

namespace fs = std::filesystem;
using detail::LogLevel;
using detail::log_line;

bool resolve_config_path(const std::string& cli_opt,
                         const char*        argv0,
                         const char*        env_name,
                         std::initializer_list<std::string> rel_candidates,
                         fs::path&          out_path,
                         std::ostream&      log)
{
    // 1) CLI 优先
    if (!cli_opt.empty()) {
        fs::path p = cli_opt;
        if (fs::exists(p)) {
            out_path = fs::canonical(p);
            log_line(log, "CONFIG", LogLevel::Info,
                     std::string("Using config from CLI: ") + out_path.string());
            return true;
        }
        log_line(log, "CONFIG", LogLevel::Warn,
                 std::string("CLI config path does not exist: ") + p.string());
    }

    // 2) 环境变量
    if (env_name && env_name[0] != '\0') {
        if (const char* env_val = std::getenv(env_name)) {
            fs::path p = env_val;
            if (fs::exists(p)) {
                out_path = fs::canonical(p);
                log_line(log, "CONFIG", LogLevel::Info,
                         std::string("Using config from env ") + env_name + ": " + out_path.string());
                return true;
            }
            log_line(log, "CONFIG", LogLevel::Warn,
                     std::string("Env ") + env_name + " points to non-existing file: " + p.string());
        } else {
            log_line(log, "CONFIG", LogLevel::Debug,
                     std::string("Env not set: ") + env_name);
        }
    }

    // 3) 相对可执行文件目录
    try {
        fs::path exe_path = fs::canonical(fs::path(argv0 ? argv0 : ""));
        fs::path exe_dir  = exe_path.parent_path();

        log_line(log, "CONFIG", LogLevel::Debug,
                 std::string("Executable dir: ") + exe_dir.string());

        for (const auto& rel : rel_candidates) {
            fs::path cand = exe_dir / rel;
            if (fs::exists(cand)) {
                out_path = fs::canonical(cand);
                log_line(log, "CONFIG", LogLevel::Info,
                         std::string("Using config near executable: ") + out_path.string());
                return true;
            }
            log_line(log, "CONFIG", LogLevel::Debug,
                     std::string("Candidate not found: ") + cand.string());
        }
    } catch (const std::exception& e) {
        log_line(log, "CONFIG", LogLevel::Err,
                 std::string("Exception while resolving config path: ") + e.what());
    }

    log_line(log, "CONFIG", LogLevel::Err,
             "Failed to resolve config path from CLI/env/candidates.");
    return false;
}

bool resolve_pwm_client_config_path(const std::string& cli_opt,
                                   const char*        argv0,
                                   fs::path&          out_path,
                                   std::ostream&      log)
{
    return resolve_config_path(cli_opt, argv0, "PWM_CLIENT_CONFIG",
                               {"config/pwm_client.yaml",
                                "../../pwm_control_program/config/pwm_client.yaml"},
                               out_path, log);
}

bool resolve_control_config_path(const std::string& cli_opt,
                                 const char*        argv0,
                                 fs::path&          out_path,
                                 std::ostream&      log)
{
    return resolve_config_path(cli_opt, argv0, "ROV_CONTROL_CONFIG",
                               {"config/control_params.yaml",
                                "../../pwm_control_program/config/control_params.yaml"},
                               out_path, log);
}

bool resolve_trajectory_config_path(const std::string& cli_opt,
                                   const char*        argv0,
                                   fs::path&          out_path,
                                   std::ostream&      log)
{
    return resolve_config_path(cli_opt, argv0, "ROV_TRAJECTORY_CONFIG",
                               {"config/trajectory.yaml",
                                "../../pwm_control_program/config/trajectory.yaml"},
                               out_path, log);
}

bool resolve_alloc_config_path(const std::string& cli_opt,
                              const char*        argv0,
                              fs::path&          out_path,
                              std::ostream&      log)
{
    return resolve_config_path(cli_opt, argv0, "ROV_ALLOC_CONFIG",
                               {"config/alloc.yaml",
                                "../../pwm_control_program/config/alloc.yaml"},
                               out_path, log);
}

} // namespace rovctrl::utils
