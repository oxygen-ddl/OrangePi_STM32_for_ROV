#include "control_core/app_context.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <utility>
#include <algorithm>  // +
#include <cstddef>    // +
#include <limits>     // +


#include <yaml-cpp/yaml.h>

#include "control_core/control_mode.hpp"
#include "control_core/trajectory_tracking.hpp"

#include "controllers/manual_controller.hpp"

#include "io/input/gcs_input_provider.hpp"
#include "io/input/multi_input_provider.hpp"
#include "io/input/teleop_input.hpp"

#include "utils/config_loader.hpp"

namespace rovctrl::control_core {

namespace fs = std::filesystem;

namespace {

// 复用你之前 app_main.cpp 里的打印（搬到 context 内部）
void print_pwm_mapping(const rovctrl::platform::PwmClientConfig& cfg, std::ostream& os)
{
    os << "[PwmClient] Logical motor -> physical PWM mapping:\n";
    for (std::size_t i = 0; i < rovctrl::platform::kNumPwmChannels; ++i) {
        const int logical_id = static_cast<int>(i) + 1;
        int pwm_ch           = cfg.motorch_to_pwmch[i];
        const int rev        = cfg.motor_reverse[i] ? 1 : 0;

        if (pwm_ch == 0) {
            pwm_ch = logical_id;
            os << "  motor " << logical_id << " -> PWM " << pwm_ch
               << " (default), reverse=" << rev << "\n";
        } else {
            os << "  motor " << logical_id << " -> PWM " << pwm_ch
               << ", reverse=" << rev << "\n";
        }
    }
    os.flush();
}

inline std::uint16_t clamp_u16_from_int(int v, std::uint16_t fallback) noexcept
{
    if (v < 0) return fallback;
    if (v > static_cast<int>(std::numeric_limits<std::uint16_t>::max()))
        return std::numeric_limits<std::uint16_t>::max();
    return static_cast<std::uint16_t>(v);
}

inline std::uint32_t clamp_u32_from_int(int v, std::uint32_t fallback) noexcept
{
    if (v < 0) return fallback;
    return static_cast<std::uint32_t>(v);
}

// alloc.yaml 解析：保持与你旧版本一致（仍走 load_thruster_allocation_from_yaml）
bool load_alloc_yaml_compat(const fs::path& alloc_cfg_path,
                            rovctrl::control_core::ThrusterAllocationConfig& out,
                            std::ostream& log)
{
    if (alloc_cfg_path.empty()) {
        log << "[Alloc] [WARN] alloc.yaml path empty; keep default allocation.\n";
        return true;
    }

    try {
        YAML::Node root = YAML::LoadFile(alloc_cfg_path.string());
        if (!root || !root["thrusters"]) {
            log << "[Alloc] [ERR] Missing top-level key 'thrusters' in: "
                << alloc_cfg_path << "\n";
            return false;
        }

        if (!rovctrl::control_core::load_thruster_allocation_from_yaml(root, out)) {
            log << "[Alloc] [ERR] load_thruster_allocation_from_yaml failed: "
                << alloc_cfg_path << "\n";
            return false;
        }

        log << "[Alloc] [INFO] alloc.yaml loaded OK: " << alloc_cfg_path << "\n";
        return true;
    } catch (const std::exception& e) {
        log << "[Alloc] [ERR] YAML::LoadFile failed: " << alloc_cfg_path
            << " err=" << e.what() << "\n";
        return false;
    }
}

} // namespace

void AppContext::shutdown(std::ostream& log, float estop_seconds)
{
    // “尽量做”，不强制依赖状态
    (void)log;
    if (pwm_client.is_ok()) {
        (void)pwm_client.emergencyStop(estop_seconds);
    }
    pwm_client.shutdown();
}

AppBuildResult build_app_context(const AppBuildOptions& opt,
                                 const char*            argv0,
                                 std::atomic_bool*      stop_flag,
                                 AppContext&            out_ctx,
                                 std::ostream&          log)
{
    AppBuildResult br{};
    out_ctx.stop_flag = stop_flag;

    // 输入全关：保持旧逻辑一致
    if (!opt.enable_gcs && !opt.enable_teleop) {
        br.ok       = false;
        br.err_code = 62;
        br.err_msg  = "both inputs disabled (--no-gcs and --no-teleop).";
        return br;
    }

    // ===================== PWM config + PwmClient =====================
    fs::path pwm_cfg_path;
    (void)rovctrl::utils::resolve_pwm_client_config_path(opt.pwm_config_cli, argv0, pwm_cfg_path, log);

    rovctrl::platform::PwmClientConfig pwm_cfg{};
    if (!pwm_cfg_path.empty()) {
        if (!rovctrl::utils::load_pwm_client_config(pwm_cfg_path, pwm_cfg, log)) {
            log << "[WARN] load_pwm_client_config failed, fallback to defaults.\n";
        }
    } else {
        log << "[WARN] pwm_client.yaml not resolved, using defaults.\n";
    }

    // CLI override（保持旧行为）
    if (opt.pwm_ctrl_hz > 0.0) pwm_cfg.ctrl_hz = static_cast<float>(opt.pwm_ctrl_hz);
    if (opt.max_step_pct > 0.0) pwm_cfg.max_step_pct = static_cast<float>(opt.max_step_pct);

    // 方式2：显式启用 dummy backend（保持旧行为）
    pwm_cfg.dummy_backend      = opt.pwm_dummy;
    pwm_cfg.dummy_print_frames = opt.pwm_dummy_print;

    log << "[PwmClient] backend=" << (pwm_cfg.dummy_backend ? "DUMMY" : "STM32")
        << (pwm_cfg.dummy_backend && pwm_cfg.dummy_print_frames ? " (print=on)" : "")
        << "\n";

    print_pwm_mapping(pwm_cfg, log);

    if (!out_ctx.pwm_client.init(pwm_cfg)) {
        br.ok       = false;
        br.err_code = 12;
        br.err_msg  = std::string("PwmClient init failed: ") + out_ctx.pwm_client.status().last_error_msg;
        out_ctx.shutdown(log, 1.0f);
        return br;
    }

    if (out_ctx.pwm_client.setAllMid() < 0) {
        br.ok       = false;
        br.err_code = 13;
        br.err_msg  = std::string("setAllMid failed: ") + out_ctx.pwm_client.status().last_error_msg;
        out_ctx.shutdown(log, 1.0f);
        return br;
    }

    // ===================== alloc.yaml -> thruster allocation =====================
    fs::path alloc_cfg_path;
    (void)rovctrl::utils::resolve_alloc_config_path(opt.alloc_config_cli, argv0, alloc_cfg_path, log);

    ThrusterAllocationConfig alloc_cfg{};
    if (!load_alloc_yaml_compat(alloc_cfg_path, alloc_cfg, log)) {
        br.ok       = false;
        br.err_code = 61;
        br.err_msg  = "alloc.yaml load failed.";
        out_ctx.shutdown(log, 1.0f);
        return br;
    }

    // ===================== trajectory.yaml (optional) =====================
    // 保持旧行为：加载成功也仅提示（目前未接入 ControlLoop）
    fs::path traj_cfg_path;
    rovctrl::control_core::TrajectoryTracking traj_tracking;
    rovctrl::control_core::TrajectoryConfig   traj_cfg;
    bool traj_loaded = false;

    if (rovctrl::utils::resolve_trajectory_config_path(opt.traj_config_cli, argv0, traj_cfg_path, log)) {
        if (!traj_cfg_path.empty() &&
            rovctrl::utils::load_trajectory_config(traj_cfg_path, traj_cfg, log)) {
            traj_tracking.set_trajectory(traj_cfg);
            traj_loaded = traj_tracking.has_trajectory();
        }
    }
    if (traj_loaded) {
        log << "[INFO] trajectory loaded (currently NOT wired into ControlLoop).\n";
    }

    // ===================== Build InputProvider chain =====================
    rovctrl::io::InputProviderPtr teleop;
    rovctrl::io::InputProviderPtr gcs;

    if (opt.enable_teleop) {
        teleop = std::make_shared<rovctrl::io::TeleopInputProvider>();
    }

    if (opt.enable_gcs) {
        rovctrl::io::GcsInputProvider::Config gcs_cfg{};
        gcs_cfg.bind_port      = clamp_u16_from_int(opt.gcs_bind_port, 14600); // 旧行为默认 14600
        gcs_cfg.default_ttl_ms = clamp_u32_from_int(opt.gcs_ttl_ms, 200);    // 旧行为默认 200
        gcs = std::make_shared<rovctrl::io::GcsInputProvider>(gcs_cfg);
    }

    if (teleop && gcs) {
        rovctrl::io::MultiInputProvider::Config mix_cfg{};
        mix_cfg.gcs_priority   = true;
        mix_cfg.default_ttl_ms = clamp_u32_from_int(opt.gcs_ttl_ms, 200);
        out_ctx.input = std::make_shared<rovctrl::io::MultiInputProvider>(teleop, gcs, mix_cfg);
    } else if (gcs) {
        out_ctx.input = gcs;
    } else {
        out_ctx.input = teleop;
    }

    if (!out_ctx.input) {
        br.ok       = false;
        br.err_code = 62;
        br.err_msg  = "Input provider not constructed.";
        out_ctx.shutdown(log, 1.0f);
        return br;
    }



    // ===================== Manual controller + ControllerManager =====================
    // 保持旧行为：init_manual_only + 手动模式
    rovctrl::controllers::ManualControllerConfig mc_cfg{};
    mc_cfg.surge_gain  = 1.0;
    mc_cfg.sway_gain   = 1.0;
    mc_cfg.heave_gain  = 1.0;
    mc_cfg.yaw_gain    = 1.0;
    mc_cfg.roll_gain   = 1.0;
    mc_cfg.pitch_gain  = 1.0;
    mc_cfg.max_cmd_abs = 1.0;

    rovctrl::control_core::ControllerManagerOptions cm_opt{};
    cm_opt.default_auto_controller = "pid";
    cm_opt.failsafe_zero_output    = true;
    cm_opt.min_switch_interval_sec = 0.2;
    cm_opt.auto_fail_limit         = 3;

    out_ctx.ctrl_mgr = ControllerManager(cm_opt);

    auto manual_ctrl =
        rovctrl::control_core::ControllerManager::make_controller<rovctrl::controllers::ManualController>(mc_cfg);

    if (!out_ctx.ctrl_mgr.init_manual_only(std::move(manual_ctrl))) {
        br.ok       = false;
        br.err_code = 50;
        br.err_msg  = std::string("ControllerManager init_manual_only failed: ") +
                      out_ctx.ctrl_mgr.status().last_error;
        out_ctx.shutdown(log, 1.0f);
        return br;
    }

    (void)out_ctx.ctrl_mgr.set_mode(rovctrl::control_core::ControlMode::kManual);

    // ===================== ControlLoop config =====================
    out_ctx.loop_cfg = rovctrl::control_core::ControlLoop::Config{};
    out_ctx.loop_cfg.loop_hz                 = opt.loop_hz;
    out_ctx.loop_cfg.max_step_errors         = 1000;
    out_ctx.loop_cfg.step_error_log_interval = 100;
    out_ctx.loop_cfg.log_timing              = false;
    out_ctx.loop_cfg.enable_pwm_log          = true;
    out_ctx.loop_cfg.thruster_alloc          = alloc_cfg;

    br.ok       = true;
    br.err_code = 0;
    br.err_msg.clear();
    return br;
}

} // namespace rovctrl::control_core
