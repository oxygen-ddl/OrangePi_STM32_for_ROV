// pwm_control_program/src/utils/config_loader.cpp

#include "utils/config_loader.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>      // std::getenv
#include <exception>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "platform/pwm_client.hpp"
#include "control_core/thruster_allocation.hpp"
#include "control_core/trajectory_tracking.hpp"

namespace rovctrl::utils {

namespace fs = std::filesystem;

// ============================================================
// 小型日志工具：统一前缀 + 分级，便于新人排障
// ============================================================

namespace {

enum class LogLevel { Debug, Info, Warn, Err };

const char* level_str(LogLevel lv)
{
    switch (lv) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Err:   return "ERR";
        default:              return "INFO";
    }
}

void log_line(std::ostream& os,
              const char*   tag,
              LogLevel      lv,
              const std::string& msg)
{
    os << "[" << tag << "] [" << level_str(lv) << "] " << msg << "\n";
}

// YAML 常见异常：BadConversion / InvalidNode / ParserException
// 这里统一包装为“字段路径+期望类型”的报错信息
template <typename T>
bool yaml_as(const YAML::Node& node,
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
                 std::string("Type conversion failed at '") + path +
                 "': " + e.what());
        return false;
    }
}

// DOF 名称 -> index，固定顺序 [Fx,Fy,Fz,Mx,My,Mz]
int dof_index(const std::string& name)
{
    if      (name == "Fx") return 0;
    else if (name == "Fy") return 1;
    else if (name == "Fz") return 2;
    else if (name == "Mx") return 3;
    else if (name == "My") return 4;
    else if (name == "Mz") return 5;
    return -1;
}

bool any_active(const std::array<bool, 6>& a)
{
    for (bool v : a) { if (v) return true; }
    return false;
}

} // namespace

// ============================================================
// 通用路径解析
// ============================================================

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
                         std::string("Using config from env ") + env_name + ": " +
                         out_path.string());
                return true;
            }
            log_line(log, "CONFIG", LogLevel::Warn,
                     std::string("Env ") + env_name +
                     " points to non-existing file: " + p.string());
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

// ============================================================
// 专用路径解析封装
// ============================================================

bool resolve_pwm_client_config_path(const std::string& cli_opt,
                                    const char*        argv0,
                                    fs::path&          out_path,
                                    std::ostream&      log)
{
    return resolve_config_path(
        cli_opt,
        argv0,
        "PWM_CLIENT_CONFIG",
        {
            "config/pwm_client.yaml",
            "../../pwm_control_program/config/pwm_client.yaml"
        },
        out_path,
        log
    );
}

bool resolve_control_config_path(const std::string& cli_opt,
                                 const char*        argv0,
                                 fs::path&          out_path,
                                 std::ostream&      log)
{
    return resolve_config_path(
        cli_opt,
        argv0,
        "ROV_CONTROL_CONFIG",
        {
            "config/control_params.yaml",
            "../../pwm_control_program/config/control_params.yaml"
        },
        out_path,
        log
    );
}

bool resolve_trajectory_config_path(const std::string& cli_opt,
                                    const char*        argv0,
                                    fs::path&          out_path,
                                    std::ostream&      log)
{
    return resolve_config_path(
        cli_opt,
        argv0,
        "ROV_TRAJECTORY_CONFIG",
        {
            "config/trajectory.yaml",
            "../../pwm_control_program/config/trajectory.yaml"
        },
        out_path,
        log
    );
}

bool resolve_alloc_config_path(const std::string& cli_opt,
                               const char*        argv0,
                               fs::path&          out_path,
                               std::ostream&      log)
{
    return resolve_config_path(
        cli_opt,
        argv0,
        "ROV_ALLOC_CONFIG",
        {
            "config/alloc.yaml",
            "../../pwm_control_program/config/alloc.yaml"
        },
        out_path,
        log
    );
}

// ============================================================
// pwm_client.yaml 加载
// ============================================================

bool load_pwm_client_config(const fs::path&                 path,
                            platform::PwmClientConfig&     cfg,
                            std::ostream&                  log)
{
    try {
        log_line(log, "PwmClient", LogLevel::Info,
                 std::string("Loading pwm_client.yaml: ") + path.string());

        YAML::Node root = YAML::LoadFile(path.string());

        auto get_scalar = [&](const char* key, auto& target) {
            if (root[key]) {
                using T = std::decay_t<decltype(target)>;
                try {
                    target = root[key].as<T>();
                } catch (const std::exception& e) {
                    log_line(log, "PwmClient", LogLevel::Err,
                             std::string("Bad type for key '") + key + "': " + e.what());
                    throw;
                }
            }
        };

        // 基本标量（缺省则保持 cfg 的默认值）
        get_scalar("ctrl_hz",      cfg.ctrl_hz);
        get_scalar("max_step_pct", cfg.max_step_pct);
        get_scalar("min_pct",      cfg.min_pct);
        get_scalar("mid_pct",      cfg.mid_pct);
        get_scalar("max_pct",      cfg.max_pct);

        if (root["enable_reverse_protection"]) {
            cfg.enable_reverse_protection =
                root["enable_reverse_protection"].as<bool>();
        }

        // 分组掩码：写成 0x0F / 0xF0 / 1 这样的 int 即可
        if (root["groupA_mask"]) {
            cfg.groupA_mask =
                static_cast<std::uint8_t>(root["groupA_mask"].as<int>());
        }
        if (root["groupB_mask"]) {
            cfg.groupB_mask =
                static_cast<std::uint8_t>(root["groupB_mask"].as<int>());
        }
        if (root["group_mode"]) {
            cfg.group_mode = root["group_mode"].as<int>();
        }

        // 逻辑电机 → 物理 PWM 通道映射
        if (root["motorch_to_pwmch"]) {
            auto node = root["motorch_to_pwmch"];
            if (!node.IsSequence()) {
                log_line(log, "PwmClient", LogLevel::Err,
                         "'motorch_to_pwmch' must be a sequence.");
                return false;
            }
            const std::size_t n = std::min<std::size_t>(
                node.size(), platform::kNumPwmChannels);
            for (std::size_t i = 0; i < n; ++i) {
                cfg.motorch_to_pwmch[i] = node[i].as<int>();
            }
        }

        // 电机方向：0 正向，非 0 视为反向
        if (root["motor_reverse"]) {
            auto node = root["motor_reverse"];
            if (!node.IsSequence()) {
                log_line(log, "PwmClient", LogLevel::Err,
                         "'motor_reverse' must be a sequence.");
                return false;
            }
            const std::size_t n = std::min<std::size_t>(
                node.size(), platform::kNumPwmChannels);
            for (std::size_t i = 0; i < n; ++i) {
                cfg.motor_reverse[i] =
                    static_cast<std::uint8_t>(node[i].as<int>() ? 1 : 0);
            }
        }

        log_line(log, "PwmClient", LogLevel::Info, "pwm_client.yaml loaded OK.");
        return true;
    } catch (const std::exception& e) {
        log_line(log, "PwmClient", LogLevel::Err,
                 std::string("Failed to load pwm_client.yaml: ") + e.what());
        return false;
    }
}

// ============================================================
// alloc.yaml 加载（Thruster Allocation）
// ============================================================

bool load_thruster_allocation_config(
    const fs::path&                         path,
    control_core::ThrusterAllocationConfig& alloc_cfg,
    std::ostream&                           log)
{
    try {
        log_line(log, "Alloc", LogLevel::Info,
                 std::string("Loading alloc.yaml: ") + path.string());

        YAML::Node root = YAML::LoadFile(path.string());

        // ---------- thrusters ----------
        if (!root["thrusters"]) {
            log_line(log, "Alloc", LogLevel::Err,
                     "Missing top-level key: thrusters");
            return false;
        }
        YAML::Node thr = root["thrusters"];

        // 1) count
        int count = 0;
        if (!yaml_as<int>(thr["count"], "thrusters.count", count, log, "Alloc", true)) {
            return false;
        }
        if (count != 8) {
            log_line(log, "Alloc", LogLevel::Err,
                     "thrusters.count must be 8 for current build. Got: " + std::to_string(count));
            return false;
        }

        // 2) order
        if (!thr["order"] || !thr["order"].IsSequence()) {
            log_line(log, "Alloc", LogLevel::Err,
                     "thrusters.order missing or not a sequence. Expected 8 strings.");
            return false;
        }
        if (thr["order"].size() != 8) {
            log_line(log, "Alloc", LogLevel::Err,
                     "thrusters.order size must be 8. Got: " + std::to_string(thr["order"].size()));
            return false;
        }
        for (std::size_t i = 0; i < 8; ++i) {
            alloc_cfg.thruster_order_yaml[i] = thr["order"][i].as<std::string>();
        }

        // 3) allocation_matrix
        if (!thr["allocation_matrix"]) {
            log_line(log, "Alloc", LogLevel::Err,
                     "Missing key: thrusters.allocation_matrix");
            return false;
        }
        YAML::Node mat = thr["allocation_matrix"];

        // 3.1 rows（可选，但若提供则必须严格匹配固定顺序）
        if (mat["rows"]) {
            if (!mat["rows"].IsSequence() || mat["rows"].size() != 6) {
                log_line(log, "Alloc", LogLevel::Err,
                         "allocation_matrix.rows must be a 6-element sequence: [Fx,Fy,Fz,Mx,My,Mz].");
                return false;
            }
            static const std::array<std::string, 6> kExpected = {
                "Fx","Fy","Fz","Mx","My","Mz"
            };
            for (std::size_t i = 0; i < 6; ++i) {
                const std::string s = mat["rows"][i].as<std::string>();
                if (s != kExpected[i]) {
                    log_line(log, "Alloc", LogLevel::Err,
                             "allocation_matrix.rows mismatch at index " + std::to_string(i) +
                             ": got '" + s + "', expected '" + kExpected[i] + "'.");
                    return false;
                }
            }
        } else {
            log_line(log, "Alloc", LogLevel::Debug,
                     "allocation_matrix.rows not provided; assume fixed order [Fx,Fy,Fz,Mx,My,Mz].");
        }

        // 3.2 data: 6x8
        if (!mat["data"] || !mat["data"].IsSequence()) {
            log_line(log, "Alloc", LogLevel::Err,
                     "allocation_matrix.data missing or not a sequence. Expected 6 rows, each 8 numbers.");
            return false;
        }
        if (mat["data"].size() != 6) {
            log_line(log, "Alloc", LogLevel::Err,
                     "allocation_matrix.data must have 6 rows. Got: " + std::to_string(mat["data"].size()));
            return false;
        }
        for (std::size_t r = 0; r < 6; ++r) {
            YAML::Node row = mat["data"][r];
            if (!row.IsSequence() || row.size() != 8) {
                log_line(log, "Alloc", LogLevel::Err,
                         "allocation_matrix.data row " + std::to_string(r) +
                         " must have 8 elements.");
                return false;
            }
            for (std::size_t c = 0; c < 8; ++c) {
                alloc_cfg.allocation_matrix_yaml[r][c] = row[c].as<double>();
            }
        }

        // 4) active_rows -> active_dof
        alloc_cfg.active_dof = {false, false, false, false, false, false};
        if (mat["active_rows"]) {
            if (!mat["active_rows"].IsSequence()) {
                log_line(log, "Alloc", LogLevel::Err,
                         "allocation_matrix.active_rows must be a sequence, e.g. [Fx,Fy,Fz,Mz].");
                return false;
            }
            for (std::size_t i = 0; i < mat["active_rows"].size(); ++i) {
                const std::string name = mat["active_rows"][i].as<std::string>();
                const int idx = dof_index(name);
                if (idx < 0) {
                    log_line(log, "Alloc", LogLevel::Warn,
                             "Unknown DOF in active_rows: '" + name + "' (ignored)");
                    continue;
                }
                alloc_cfg.active_dof[static_cast<std::size_t>(idx)] = true;
            }
        } else {
            log_line(log, "Alloc", LogLevel::Warn,
                     "allocation_matrix.active_rows missing; all DOF inactive (may output zeros).");
        }

        if (!any_active(alloc_cfg.active_dof)) {
            log_line(log, "Alloc", LogLevel::Warn,
                     "No active DOF enabled. Verify allocation_matrix.active_rows.");
        }

        // 5) limits.norm_min / norm_max
        if (thr["limits"]) {
            YAML::Node lim = thr["limits"];
            if (lim["norm_min"]) alloc_cfg.norm_min = lim["norm_min"].as<double>();
            if (lim["norm_max"]) alloc_cfg.norm_max = lim["norm_max"].as<double>();
        } else {
            log_line(log, "Alloc", LogLevel::Warn,
                     "thrusters.limits missing; keep defaults norm_min/norm_max.");
        }

        if (!(alloc_cfg.norm_min < alloc_cfg.norm_max)) {
            log_line(log, "Alloc", LogLevel::Err,
                     "Invalid norm range: norm_min=" + std::to_string(alloc_cfg.norm_min) +
                     ", norm_max=" + std::to_string(alloc_cfg.norm_max));
            return false;
        }

        // 6) 同步到 thrust_model（你们已有字段）
        alloc_cfg.thrust_model.norm_min = alloc_cfg.norm_min;
        alloc_cfg.thrust_model.norm_max = alloc_cfg.norm_max;

        // 输出摘要便于排障
        {
            std::string order = "order=[";
            for (std::size_t i = 0; i < 8; ++i) {
                order += alloc_cfg.thruster_order_yaml[i];
                if (i + 1 < 8) order += ",";
            }
            order += "]";
            log_line(log, "Alloc", LogLevel::Info,
                     "alloc.yaml loaded OK: " + order +
                     ", norm=[" + std::to_string(alloc_cfg.norm_min) + "," +
                     std::to_string(alloc_cfg.norm_max) + "]");
        }

        return true;
    } catch (const std::exception& e) {
        log_line(log, "Alloc", LogLevel::Err,
                 std::string("Failed to load alloc.yaml: ") + e.what());
        return false;
    }
}

// ============================================================
// trajectory.yaml 加载（保持你现有逻辑，略作日志风格对齐）
// ============================================================

namespace {

control_core::TrajectoryFrame parse_frame(const std::string& s)
{
    if (s == "NED" || s == "ned") return control_core::TrajectoryFrame::NED;
    if (s == "ENU" || s == "enu") return control_core::TrajectoryFrame::ENU;
    return control_core::TrajectoryFrame::Unknown;
}

control_core::AngleUnit parse_angle_unit(const std::string& s)
{
    if (s == "rad" || s == "RAD") return control_core::AngleUnit::Rad;
    if (s == "deg" || s == "DEG") return control_core::AngleUnit::Deg;
    return control_core::AngleUnit::Unknown;
}

control_core::TrajectoryType parse_traj_type(const std::string& s)
{
    if (s == "piecewise" || s == "PIECEWISE") {
        return control_core::TrajectoryType::Piecewise;
    }
    return control_core::TrajectoryType::Unknown;
}

double get_double_or(const YAML::Node& node,
                     const char*       key,
                     double            default_val)
{
    if (node[key]) return node[key].as<double>();
    return default_val;
}

} // anonymous namespace

bool load_trajectory_config(const fs::path&                 path,
                            control_core::TrajectoryConfig& cfg,
                            std::ostream&                   log)
{
    try {
        log_line(log, "Traj", LogLevel::Info,
                 std::string("Loading trajectory.yaml: ") + path.string());
        YAML::Node root = YAML::LoadFile(path.string());

        if (!root["trajectory"]) {
            log_line(log, "Traj", LogLevel::Err,
                     "YAML missing 'trajectory' root node.");
            return false;
        }

        YAML::Node traj_node = root["trajectory"];

        // frame / angle_unit / type
        if (traj_node["frame"]) {
            cfg.frame_raw = traj_node["frame"].as<std::string>();
            cfg.frame     = parse_frame(cfg.frame_raw);
        } else {
            cfg.frame_raw = "";
            cfg.frame     = control_core::TrajectoryFrame::Unknown;
        }

        if (traj_node["angle_unit"]) {
            cfg.angle_unit_raw = traj_node["angle_unit"].as<std::string>();
            cfg.angle_unit     = parse_angle_unit(cfg.angle_unit_raw);
        } else {
            cfg.angle_unit_raw = "";
            cfg.angle_unit     = control_core::AngleUnit::Unknown;
        }

        if (traj_node["type"]) {
            cfg.type_raw = traj_node["type"].as<std::string>();
            cfg.type     = parse_traj_type(cfg.type_raw);
        } else {
            cfg.type_raw = "";
            cfg.type     = control_core::TrajectoryType::Unknown;
        }

        // waypoints
        if (!traj_node["waypoints"] || !traj_node["waypoints"].IsSequence()) {
            log_line(log, "Traj", LogLevel::Err,
                     "'trajectory.waypoints' missing or not a sequence.");
            return false;
        }

        YAML::Node wps = traj_node["waypoints"];
        if (wps.size() == 0) {
            log_line(log, "Traj", LogLevel::Err,
                     "'trajectory.waypoints' is empty.");
            return false;
        }

        cfg.points.clear();
        cfg.points.reserve(wps.size());

        constexpr double PI = 3.14159265358979323846;
        auto yaw_to_rad = [&](double yaw_val) {
            if (cfg.angle_unit == control_core::AngleUnit::Deg) return yaw_val * PI / 180.0;
            return yaw_val;
        };

        for (std::size_t i = 0; i < wps.size(); ++i) {
            YAML::Node wp = wps[i];
            if (!wp["t"]) {
                log_line(log, "Traj", LogLevel::Err,
                         "waypoint[" + std::to_string(i) + "] missing 't'.");
                return false;
            }

            control_core::TrajectoryPoint pt;
            pt.t_s = wp["t"].as<double>();

            double x   = get_double_or(wp, "x", 0.0);
            double y   = get_double_or(wp, "y", 0.0);
            double z   = get_double_or(wp, "z", 0.0);
            double yaw = get_double_or(wp, "yaw", 0.0);

            pt.pose.x     = x;
            pt.pose.y     = y;
            pt.pose.z     = z;
            pt.pose.roll  = 0.0;
            pt.pose.pitch = 0.0;
            pt.pose.yaw   = yaw_to_rad(yaw);

            double vx       = get_double_or(wp, "vx", 0.0);
            double vy       = get_double_or(wp, "vy", 0.0);
            double vz       = get_double_or(wp, "vz", 0.0);
            double yaw_rate = get_double_or(wp, "yaw_rate", 0.0);

            pt.vel.surge      = vx;
            pt.vel.sway       = vy;
            pt.vel.heave      = vz;
            pt.vel.roll_rate  = 0.0;
            pt.vel.pitch_rate = 0.0;
            pt.vel.yaw_rate   = yaw_rate;

            pt.accel.surge     = 0.0;
            pt.accel.sway      = 0.0;
            pt.accel.heave     = 0.0;
            pt.accel.roll_acc  = 0.0;
            pt.accel.pitch_acc = 0.0;
            pt.accel.yaw_acc   = 0.0;

            cfg.points.push_back(pt);
        }

        std::sort(cfg.points.begin(), cfg.points.end(),
                  [](const control_core::TrajectoryPoint& a,
                     const control_core::TrajectoryPoint& b) {
                      return a.t_s < b.t_s;
                  });

        log_line(log, "Traj", LogLevel::Info,
                 "trajectory.yaml loaded OK: waypoints=" + std::to_string(cfg.points.size()) +
                 ", frame=" + cfg.frame_raw +
                 ", angle_unit=" + cfg.angle_unit_raw +
                 ", type=" + cfg.type_raw);

        return true;
    } catch (const std::exception& e) {
        log_line(log, "Traj", LogLevel::Err,
                 std::string("Failed to load trajectory.yaml: ") + e.what());
        return false;
    }
}

} // namespace rovctrl::utils
