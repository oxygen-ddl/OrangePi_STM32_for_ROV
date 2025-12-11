// pwm_control_program/src/utils/config_loader.cpp

#include "utils/config_loader.hpp"

#include <algorithm>
#include <cstdlib>      // std::getenv
#include <exception>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include "platform/pwm_client.hpp"
#include "control_core/thruster_allocation.hpp"
#include "control_core/trajectory_tracking.hpp"

namespace rovctrl::utils {

namespace fs = std::filesystem;

// ===================== 通用路径解析 =====================

bool resolve_config_path(const std::string& cli_opt,
                         const char*        argv0,
                         const char*        env_name,
                         std::initializer_list<std::string> rel_candidates,
                         fs::path&          out_path,
                         std::ostream&      log)
{
    // 1) 命令行优先
    if (!cli_opt.empty()) {
        fs::path p = cli_opt;
        if (fs::exists(p)) {
            out_path = fs::canonical(p);
            log << "[CONFIG] Using config from CLI: " << out_path << "\n";
            return true;
        } else {
            log << "[CONFIG] [WARN] CLI config path does not exist: "
                << p << "\n";
        }
    }

    // 2) 环境变量
    if (env_name) {
        if (const char* env_val = std::getenv(env_name)) {
            fs::path p = env_val;
            if (fs::exists(p)) {
                out_path = fs::canonical(p);
                log << "[CONFIG] Using config from env " << env_name
                    << ": " << out_path << "\n";
                return true;
            } else {
                log << "[CONFIG] [WARN] Env " << env_name
                    << " points to non-existing file: " << p << "\n";
            }
        }
    }

    // 3) 相对可执行文件目录
    try {
        fs::path exe_path = fs::canonical(fs::path(argv0 ? argv0 : ""));
        fs::path exe_dir  = exe_path.parent_path();

        for (const auto& rel : rel_candidates) {
            fs::path cand = exe_dir / rel;
            if (fs::exists(cand)) {
                out_path = fs::canonical(cand);
                log << "[CONFIG] Using config near executable: "
                    << out_path << "\n";
                return true;
            } else {
                log << "[CONFIG] [DEBUG] Candidate not found: "
                    << cand << "\n";
            }
        }
    } catch (const std::exception& e) {
        log << "[CONFIG] [ERR] Exception while resolving config path: "
            << e.what() << "\n";
    }

    return false;
}

// ===================== 专用路径解析封装 =====================

bool resolve_pwm_client_config_path(const std::string& cli_opt,
                                    const char*        argv0,
                                    fs::path&          out_path,
                                    std::ostream&      log)
{
    // 环境变量：PWM_CLIENT_CONFIG
    // 候选路径：
    //   exe_dir / "config/pwm_client.yaml"
    //   exe_dir / "../../pwm_control_program/config/pwm_client.yaml"
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
    // 环境变量：ROV_CONTROL_CONFIG
    // 候选路径：
    //   exe_dir / "config/control_params.yaml"
    //   exe_dir / "../../pwm_control_program/config/control_params.yaml"
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
    // 环境变量：ROV_TRAJECTORY_CONFIG
    // 候选路径：
    //   exe_dir / "config/trajectory.yaml"
    //   exe_dir / "../../pwm_control_program/config/trajectory.yaml"
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

// ===================== pwm_client.yaml 加载 =====================

bool load_pwm_client_config(const fs::path&         path,
                            platform::PwmClientConfig& cfg,
                            std::ostream&           log)
{
    try {
        log << "[PwmClient] Loading config from " << path << "\n";
        YAML::Node root = YAML::LoadFile(path.string());

        auto get_scalar = [&](const char* key, auto& target) {
            if (root[key]) {
                using T = std::decay_t<decltype(target)>;
                target  = root[key].as<T>();
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
            if (node.IsSequence()) {
                const std::size_t n = std::min<std::size_t>(
                    node.size(), platform::kNumPwmChannels);
                for (std::size_t i = 0; i < n; ++i) {
                    cfg.motorch_to_pwmch[i] = node[i].as<int>();
                }
            }
        }

        // 电机方向：0 正向，非 0 视为反向
        if (root["motor_reverse"]) {
            auto node = root["motor_reverse"];
            if (node.IsSequence()) {
                const std::size_t n = std::min<std::size_t>(
                    node.size(), platform::kNumPwmChannels);
                for (std::size_t i = 0; i < n; ++i) {
                    cfg.motor_reverse[i] =
                        static_cast<std::uint8_t>(node[i].as<int>() ? 1 : 0);
                }
            }
        }

        log << "[PwmClient] pwm_client.yaml loaded OK.\n";
        return true;
    } catch (const std::exception& e) {
        log << "[PwmClient] ERROR: failed to load config " << path
            << " : " << e.what() << "\n";
        return false;
    }
}

// ===================== control_params.yaml 加载 =====================

bool load_thruster_allocation_config(
    const fs::path&                         path,
    control_core::ThrusterAllocationConfig& alloc_cfg,
    std::ostream&                           log)
{
    using control_core::ThrusterAllocationConfig;

    try {
        log << "[Control] Loading control params from " << path << "\n";
        YAML::Node root = YAML::LoadFile(path.string());

        if (!root["vehicle"]) {
            log << "[Control] YAML missing 'vehicle' section in "
                << path << "\n";
            return false;
        }
        YAML::Node vehicle  = root["vehicle"];
        YAML::Node thr_node = vehicle["thrusters"];
        if (!thr_node) {
            log << "[Control] YAML missing 'vehicle.thrusters' section in "
                << path << "\n";
            return false;
        }

        // --- 1) 读取 order: [P5, P6, P7, P8, P1, P2, P3, P4] ---
        if (!thr_node["order"] || !thr_node["order"].IsSequence()) {
            log << "[Control] 'vehicle.thrusters.order' missing or not a sequence.\n";
            return false;
        }
        {
            auto seq = thr_node["order"];
            if (seq.size() != 8) {
                log << "[Control] 'vehicle.thrusters.order' size != 8, got "
                    << seq.size() << "\n";
                return false;
            }
            for (std::size_t i = 0; i < 8; ++i) {
                alloc_cfg.thruster_order_yaml[i] = seq[i].as<std::string>();
            }
        }

        // --- 2) 读取 allocation_matrix.data (6x8) ---
        YAML::Node mat_node = thr_node["allocation_matrix"];
        if (!mat_node || !mat_node["data"] || !mat_node["data"].IsSequence()) {
            log << "[Control] 'vehicle.thrusters.allocation_matrix.data' "
                   "missing or invalid.\n";
            return false;
        }
        {
            auto rows = mat_node["data"];
            if (rows.size() != 6) {
                log << "[Control] allocation_matrix.data must have 6 rows, got "
                    << rows.size() << "\n";
                return false;
            }
            for (std::size_t r = 0; r < 6; ++r) {
                auto row = rows[r];
                if (!row.IsSequence() || row.size() != 8) {
                    log << "[Control] allocation_matrix.data row " << r
                        << " must have 8 elements.\n";
                    return false;
                }
                for (std::size_t c = 0; c < 8; ++c) {
                    alloc_cfg.allocation_matrix_yaml[r][c] =
                        row[c].as<double>();
                }
            }
        }

        // --- 3) active_rows: [Fx, Fy, Fz, Mz] → active_dof[6] ---
        alloc_cfg.active_dof = {false, false, false, false, false, false};
        if (mat_node["active_rows"] && mat_node["active_rows"].IsSequence()) {
            auto active_rows = mat_node["active_rows"];
            for (std::size_t i = 0; i < active_rows.size(); ++i) {
                const std::string name = active_rows[i].as<std::string>();
                if      (name == "Fx") alloc_cfg.active_dof[0] = true;
                else if (name == "Fy") alloc_cfg.active_dof[1] = true;
                else if (name == "Fz") alloc_cfg.active_dof[2] = true;
                else if (name == "Mx") alloc_cfg.active_dof[3] = true;
                else if (name == "My") alloc_cfg.active_dof[4] = true;
                else if (name == "Mz") alloc_cfg.active_dof[5] = true;
                else {
                    log << "[Control] unknown DOF name in active_rows: "
                        << name << "\n";
                }
            }
        } else {
            log << "[Control] 'active_rows' missing, all DOF inactive.\n";
        }

        // --- 4) limits.norm_min / norm_max ---
        if (thr_node["limits"]) {
            auto lim = thr_node["limits"];
            if (lim["norm_min"]) {
                alloc_cfg.norm_min = lim["norm_min"].as<double>();
            }
            if (lim["norm_max"]) {
                alloc_cfg.norm_max = lim["norm_max"].as<double>();
            }
        }

        // --- 5) thrust_model 可选参数（如有） ---
        if (thr_node["thrust_model"]) {
            auto tm = thr_node["thrust_model"];
            if (tm["max_forward_N"]) {
                alloc_cfg.thrust_model.max_forward_N =
                    tm["max_forward_N"].as<double>();
            }
            if (tm["max_reverse_N"]) {
                alloc_cfg.thrust_model.max_reverse_N =
                    tm["max_reverse_N"].as<double>();
            }
        }

        // 同步限幅到 thrust_model（内部 ThrusterAllocator::init 也会再同步一次）
        alloc_cfg.thrust_model.norm_min = alloc_cfg.norm_min;
        alloc_cfg.thrust_model.norm_max = alloc_cfg.norm_max;

        log << "[Control] control_params.yaml loaded OK.\n";
        return true;
    } catch (const std::exception& e) {
        log << "[Control] ERROR: failed to load control params "
            << path << " : " << e.what() << "\n";
        return false;
    }
}

// ===================== trajectory.yaml 加载 =====================

// 一些小工具：字符串 → 枚举

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

/// 安全提取 double，如果缺失则返回默认值
double get_double_or(const YAML::Node& node,
                     const char*       key,
                     double            default_val)
{
    if (node[key]) {
        return node[key].as<double>();
    }
    return default_val;
}

} // anonymous namespace

bool load_trajectory_config(const fs::path&                 path,
                            control_core::TrajectoryConfig& cfg,
                            std::ostream&                   log)
{
    try {
        log << "[Traj] Loading trajectory from " << path << "\n";
        YAML::Node root = YAML::LoadFile(path.string());

        if (!root["trajectory"]) {
            log << "[Traj] YAML missing 'trajectory' root node.\n";
            return false;
        }

        YAML::Node traj_node = root["trajectory"];

        // --- 基本元信息：frame / angle_unit / type ---
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

        // --- waypoints 列表 ---
        if (!traj_node["waypoints"] || !traj_node["waypoints"].IsSequence()) {
            log << "[Traj] 'trajectory.waypoints' missing or not a sequence.\n";
            return false;
        }

        YAML::Node wps = traj_node["waypoints"];
        if (wps.size() == 0) {
            log << "[Traj] 'trajectory.waypoints' is empty.\n";
            return false;
        }

        cfg.points.clear();
        cfg.points.reserve(wps.size());

        // 角度单位换算辅助
        constexpr double PI = 3.14159265358979323846;
        auto yaw_to_rad = [&](double yaw_val) {
            if (cfg.angle_unit == control_core::AngleUnit::Deg) {
                return yaw_val * PI / 180.0;
            }
            // 默认为 rad
            return yaw_val;
        };

        // 遍历每一个 waypoint
        for (std::size_t i = 0; i < wps.size(); ++i) {
            YAML::Node wp = wps[i];

            if (!wp["t"]) {
                log << "[Traj] waypoint[" << i << "] missing 't'.\n";
                return false;
            }

            control_core::TrajectoryPoint pt;

            // 时间
            pt.t_s = wp["t"].as<double>();

            // 位置
            double x   = get_double_or(wp, "x", 0.0);
            double y   = get_double_or(wp, "y", 0.0);
            double z   = get_double_or(wp, "z", 0.0);
            double yaw = get_double_or(wp, "yaw", 0.0);

            pt.pose.x    = x;
            pt.pose.y    = y;
            pt.pose.z    = z;
            pt.pose.roll = 0.0;
            pt.pose.pitch= 0.0;
            pt.pose.yaw  = yaw_to_rad(yaw);

            // 速度（可选）
            double vx       = get_double_or(wp, "vx", 0.0);
            double vy       = get_double_or(wp, "vy", 0.0);
            double vz       = get_double_or(wp, "vz", 0.0);
            double yaw_rate = get_double_or(wp, "yaw_rate", 0.0);

            pt.vel.surge      = vx;
            pt.vel.sway       = vy;
            pt.vel.heave      = vz;
            pt.vel.roll_rate  = 0.0;
            pt.vel.pitch_rate = 0.0;
            pt.vel.yaw_rate   = yaw_rate;  // 这里假定 yaw_rate 已经是 rad/s 或 deg/s？
                                           // 更严格可以根据 angle_unit 再转换，这里先认为是 rad/s。

            // 加速度（当前暂不使用，全部置 0）
            pt.accel.surge     = 0.0;
            pt.accel.sway      = 0.0;
            pt.accel.heave     = 0.0;
            pt.accel.roll_acc  = 0.0;
            pt.accel.pitch_acc = 0.0;
            pt.accel.yaw_acc   = 0.0;

            cfg.points.push_back(pt);
        }

        // 排序（按 t_s 升序）以保证后续插值逻辑安全
        std::sort(cfg.points.begin(), cfg.points.end(),
                  [](const control_core::TrajectoryPoint& a,
                     const control_core::TrajectoryPoint& b) {
                      return a.t_s < b.t_s;
                  });

        log << "[Traj] trajectory.yaml loaded OK, waypoints="
            << cfg.points.size()
            << ", frame=" << cfg.frame_raw
            << ", angle_unit=" << cfg.angle_unit_raw
            << ", type=" << cfg.type_raw
            << "\n";

        return true;
    } catch (const std::exception& e) {
        log << "[Traj] ERROR: failed to load trajectory from "
            << path << " : " << e.what() << "\n";
        return false;
    }
}

} // namespace rovctrl::utils
