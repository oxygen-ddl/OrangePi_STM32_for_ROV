#include "control_core/app_main.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "platform/pwm_client.hpp"

#include "control_core/control_loop.hpp"
#include "control_core/thruster_allocation.hpp"
#include "control_core/trajectory_tracking.hpp"

#include "controllers/manual_controller.hpp"
#include "controllers/controller_manager.hpp"

#include "io/input/teleop_input.hpp"
#include "utils/config_loader.hpp"

// ---- IMPORTANT: make sure these headers exist in your tree ----
// If names/paths differ, adjust accordingly.
#include "io/input/gcs_input_provider.hpp"
#include "io/input/multi_input_provider.hpp"

namespace {
namespace fs = std::filesystem;

std::atomic_bool g_stop_flag{false};
void signal_handler(int) { g_stop_flag.store(true); }

enum class AppError : int {
    Ok                   = 0,
    PwmClientInitFailed  = 12,
    SetAllMidFailed      = 13,
    ControlLoopException = 50,

    AllocLoadFailed      = 61,
    InputInitFailed      = 62
};

struct CmdArgs {
    double loop_hz      = 100.0;
    double pwm_ctrl_hz  = 100.0;
    double max_step_pct = 0.2;

    std::string pwm_config;
    std::string control_config;
    std::string traj_config;
    std::string alloc_config;

    bool enable_gcs    = true;
    bool enable_teleop = true;
    bool show_help     = false;
};

static void print_help()
{
    std::cout
        << "Usage: pwm_control_program [options]\n"
        << "  --loop-hz <Hz>           Control loop frequency (default 100)\n"
        << "  --pwm-hz <Hz>            PWM safety layer frequency (default 100)\n"
        << "  --max-step <pct>         Max duty step per cycle (default 0.2)\n"
        << "  --config <file>          pwm_client.yaml path\n"
        << "  --control-config <file>  control_params.yaml path\n"
        << "  --alloc-config <file>    alloc.yaml path\n"
        << "  --traj-config <file>     trajectory.yaml path (optional)\n"
        << "  --no-gcs                 Disable GCS UDP input\n"
        << "  --no-teleop              Disable keyboard teleop input\n"
        << "  -h, --help               Show this help\n";
}

CmdArgs parse_args(int argc, char** argv)
{
    CmdArgs args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];

        auto need_value = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[ERR] missing value after " << opt << "\n";
                args.show_help = true;
                return nullptr;
            }
            return argv[++i];
        };

        if (a == "--loop-hz") {
            if (const char* v = need_value("--loop-hz")) args.loop_hz = std::atof(v);
        } else if (a == "--pwm-hz") {
            if (const char* v = need_value("--pwm-hz")) args.pwm_ctrl_hz = std::atof(v);
        } else if (a == "--max-step") {
            if (const char* v = need_value("--max-step")) args.max_step_pct = std::atof(v);
        } else if (a == "--config") {
            if (const char* v = need_value("--config")) args.pwm_config = v;
        } else if (a == "--control-config") {
            if (const char* v = need_value("--control-config")) args.control_config = v;
        } else if (a == "--traj-config") {
            if (const char* v = need_value("--traj-config")) args.traj_config = v;
        } else if (a == "--alloc-config") {
            if (const char* v = need_value("--alloc-config")) args.alloc_config = v;
        } else if (a == "--no-gcs") {
            args.enable_gcs = false;
        } else if (a == "--no-teleop") {
            args.enable_teleop = false;
        } else if (a == "--help" || a == "-h") {
            args.show_help = true;
        } else {
            std::cerr << "[WARN] unknown option: " << a << "\n";
            args.show_help = true;
        }
    }
    return args;
}

void print_pwm_mapping(const rovctrl::platform::PwmClientConfig& cfg)
{
    std::cout << "[PwmClient] Logical motor -> physical PWM mapping:\n";
    for (std::size_t i = 0; i < rovctrl::platform::kNumPwmChannels; ++i) {
        const int logical_id = static_cast<int>(i) + 1;
        int pwm_ch = cfg.motorch_to_pwmch[i];
        const int rev = cfg.motor_reverse[i] ? 1 : 0;

        if (pwm_ch == 0) {
            pwm_ch = logical_id;
            std::cout << "  motor " << logical_id << " -> PWM " << pwm_ch
                      << " (default), reverse=" << rev << "\n";
        } else {
            std::cout << "  motor " << logical_id << " -> PWM " << pwm_ch
                      << ", reverse=" << rev << "\n";
        }
    }
    std::cout.flush();
}

static bool load_alloc_yaml(const fs::path& alloc_cfg_path,
                            rovctrl::control_core::ThrusterAllocationConfig& out,
                            std::ostream& log)
{
    if (alloc_cfg_path.empty()) {
        log << "[Alloc] [WARN] alloc.yaml path empty; keep default allocation.\n";
        return true; // allow default
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

namespace rovctrl::control_core {

int app_main(int argc, char** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const CmdArgs args = parse_args(argc, argv);
    if (args.show_help) {
        print_help();
        return static_cast<int>(AppError::Ok);
    }

    std::cout << "[INFO] pwm_control_program starting...\n"
              << "       loop_hz=" << args.loop_hz
              << ", pwm_ctrl_hz=" << args.pwm_ctrl_hz
              << ", max_step_pct=" << args.max_step_pct
              << ", pwm_config_cli=" << (args.pwm_config.empty() ? "<none>" : args.pwm_config)
              << ", traj_config_cli=" << (args.traj_config.empty() ? "<none>" : args.traj_config)
              << ", control_config_cli=" << (args.control_config.empty() ? "<none>" : args.control_config)
              << ", alloc_config_cli=" << (args.alloc_config.empty() ? "<none>" : args.alloc_config)
              << ", enable_gcs=" << (args.enable_gcs ? "1" : "0")
              << ", enable_teleop=" << (args.enable_teleop ? "1" : "0")
              << "\n";

    if (!args.enable_gcs && !args.enable_teleop) {
        std::cerr << "[ERR] both inputs disabled (--no-gcs and --no-teleop). Nothing to do.\n";
        return static_cast<int>(AppError::InputInitFailed);
    }

    // ===================== PWM config + PwmClient =====================
    fs::path pwm_cfg_path;
    (void)rovctrl::utils::resolve_pwm_client_config_path(args.pwm_config, argv[0], pwm_cfg_path, std::cerr);

    rovctrl::platform::PwmClientConfig pwm_cfg{};
    if (!pwm_cfg_path.empty()) {
        if (!rovctrl::utils::load_pwm_client_config(pwm_cfg_path, pwm_cfg, std::cerr)) {
            std::cerr << "[WARN] load_pwm_client_config failed, fallback to defaults.\n";
        }
    } else {
        std::cerr << "[WARN] pwm_client.yaml not resolved, using defaults.\n";
    }

    if (args.pwm_ctrl_hz > 0.0) pwm_cfg.ctrl_hz = static_cast<float>(args.pwm_ctrl_hz);
    if (args.max_step_pct > 0.0) pwm_cfg.max_step_pct = static_cast<float>(args.max_step_pct);

    print_pwm_mapping(pwm_cfg);

    rovctrl::platform::PwmClient pwm_client;
    if (!pwm_client.init(pwm_cfg)) {
        std::cerr << "[ERR] PwmClient init failed: " << pwm_client.status().last_error_msg << "\n";
        return static_cast<int>(AppError::PwmClientInitFailed);
    }

    if (pwm_client.setAllMid() < 0) {
        std::cerr << "[ERR] setAllMid failed: " << pwm_client.status().last_error_msg << "\n";
        pwm_client.shutdown();
        return static_cast<int>(AppError::SetAllMidFailed);
    }

    // ===================== alloc.yaml -> thruster allocation =====================
    fs::path alloc_cfg_path;
    (void)rovctrl::utils::resolve_alloc_config_path(args.alloc_config, argv[0], alloc_cfg_path, std::cerr);

    ThrusterAllocationConfig alloc_cfg{};
    if (!load_alloc_yaml(alloc_cfg_path, alloc_cfg, std::cerr)) {
        pwm_client.emergencyStop(1.0f);
        pwm_client.shutdown();
        return static_cast<int>(AppError::AllocLoadFailed);
    }

    // ===================== trajectory.yaml (optional) =====================
    fs::path traj_cfg_path;
    rovctrl::control_core::TrajectoryTracking traj_tracking;
    rovctrl::control_core::TrajectoryConfig   traj_cfg;
    bool traj_loaded = false;

    if (rovctrl::utils::resolve_trajectory_config_path(args.traj_config, argv[0], traj_cfg_path, std::cerr)) {
        if (!traj_cfg_path.empty() &&
            rovctrl::utils::load_trajectory_config(traj_cfg_path, traj_cfg, std::cerr)) {
            traj_tracking.set_trajectory(traj_cfg);
            traj_loaded = traj_tracking.has_trajectory();
        }
    }
    if (traj_loaded) {
        std::cout << "[INFO] trajectory loaded (currently NOT wired into ControlLoop).\n";
    }

    // ===================== Build InputProvider chain =====================
    rovctrl::io::InputProviderPtr input;

    std::shared_ptr<rovctrl::io::TeleopInputProvider> teleop;
    std::shared_ptr<rovctrl::io::GcsInputProvider>    gcs;

    if (args.enable_teleop) {
        teleop = std::make_shared<rovctrl::io::TeleopInputProvider>();
    }
    if (args.enable_gcs) {
        rovctrl::io::GcsInputProvider::Config gcs_cfg{};
        gcs_cfg.bind_port      = 14600;
        gcs_cfg.default_ttl_ms = 200;
        gcs = std::make_shared<rovctrl::io::GcsInputProvider>(gcs_cfg);
    }

    if (teleop && gcs) {
        rovctrl::io::MultiInputProvider::Config mix_cfg{};
        mix_cfg.gcs_priority   = true;
        mix_cfg.default_ttl_ms = 200;
        input = std::make_shared<rovctrl::io::MultiInputProvider>(teleop, gcs, mix_cfg);
    } else if (gcs) {
        input = gcs;
    } else {
        input = teleop;
    }

    // ===================== Manual controller =====================
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

    rovctrl::control_core::ControllerManager ctrl_mgr(cm_opt);

// IMPORTANT: build ControllerPtr (unique_ptr<IController, IControllerDeleter>)
auto manual_ctrl =
    rovctrl::control_core::ControllerManager::make_controller<rovctrl::controllers::ManualController>(mc_cfg);

if (!ctrl_mgr.init_manual_only(std::move(manual_ctrl))) {
    std::cerr << "[ERR] ControllerManager init_manual_only failed: "
              << ctrl_mgr.status().last_error << "\n";
    pwm_client.emergencyStop(1.0f);
    pwm_client.shutdown();
    return -60;
}

(void)ctrl_mgr.set_mode(rovctrl::control_core::ControlMode::kManual);


    // ===================== ControlLoop =====================
    rovctrl::control_core::ControlLoop::Config loop_cfg{};
    loop_cfg.loop_hz                 = args.loop_hz;
    loop_cfg.max_step_errors         = 1000;
    loop_cfg.step_error_log_interval = 100;
    loop_cfg.log_timing              = false;
    loop_cfg.enable_pwm_log          = true;
    loop_cfg.thruster_alloc          = alloc_cfg;

    rovctrl::control_core::ControlLoop loop(
        loop_cfg,
        pwm_client,
        input,
        std::move(ctrl_mgr),
        &g_stop_flag
    );

    int rc = 0;
    try {
        rc = loop.run();
    } catch (const std::exception& e) {
        std::cerr << "[ERR] Exception in ControlLoop::run(): " << e.what() << "\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    } catch (...) {
        std::cerr << "[ERR] Unknown exception in ControlLoop::run().\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    }

    std::cout << "[INFO] ControlLoop exited with rc=" << rc << "\n";
    pwm_client.emergencyStop(1.0f);
    pwm_client.shutdown();
    return rc;
}

} // namespace rovctrl::control_core
