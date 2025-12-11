#include "control_core/app_main.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>

#include "platform/pwm_client.hpp"
#include "control_core/control_loop.hpp"
#include "control_core/thruster_allocation.hpp"
#include "control_core/trajectory_tracking.hpp"
#include "controllers/manual_controller.hpp"
#include "io/teleop_input.hpp"

#include "utils/config_loader.hpp"   // 新增：统一配置加载入口

namespace {

namespace fs = std::filesystem;

// 全局停止标志，用于响应 Ctrl+C / SIGTERM
std::atomic_bool g_stop_flag{false};

void signal_handler(int)
{
    g_stop_flag.store(true);
}

// ===================== 错误码（便于快速定位模块） =====================

enum class AppError {
    Ok                      = 0,

    BadPwmConfigPath        = 10,  // STEP1: 找不到 pwm_client.yaml
    LoadPwmConfigFailed     = 11,  // STEP2: 解析 pwm_client.yaml 失败
    PwmClientInitFailed     = 12,  // STEP2: PwmClient 初始化失败
    SetAllMidFailed         = 13,  // STEP2: 上电归中失败

    BadControlConfigPath    = 30,  // STEP3: 找不到 control_params.yaml
    LoadControlConfigFailed = 31,  // STEP3: 解析控制参数 YAML 失败

    BadTrajConfigPath       = 40,  // STEP4: 找不到 trajectory.yaml
    LoadTrajConfigFailed    = 41,  // STEP4: 解析轨迹 YAML 失败

    ControlLoopException    = 50,  // STEP7: 控制主循环异常
};

// ===================== 命令行解析 =====================

struct CmdArgs {
    double      loop_hz        = 100.0;  // 控制主循环频率
    double      pwm_ctrl_hz    = 100.0;  // PwmClient 安全层频率
    double      max_step_pct   = 0.2;   // 每步最大占空比变化

    // 为空 → 不触发 CLI override
    std::string pwm_config     = "";
    std::string control_config = "";
    std::string traj_config    = "";
};

CmdArgs parse_args(int argc, char** argv)
{
    CmdArgs args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--loop-hz" && i + 1 < argc) {
            args.loop_hz = std::atof(argv[++i]);
        } else if (a == "--pwm-hz" && i + 1 < argc) {
            args.pwm_ctrl_hz = std::atof(argv[++i]);
        } else if (a == "--max-step" && i + 1 < argc) {
            args.max_step_pct = std::atof(argv[++i]);
        } else if (a == "--config" && i + 1 < argc) {
            args.pwm_config = argv[++i];   // 例如: --config config/pwm_client.yaml
        } else if (a == "--control-config" && i + 1 < argc) {
            args.control_config = argv[++i];   // 控制参数 YAML
        } else if (a == "--traj-config" && i + 1 < argc) {
            args.traj_config = argv[++i];      // 轨迹 YAML
        } else if (a == "--help" || a == "-h") {
            std::cout
                << "Usage: pwm_control_program [options]\n"
                << "  --loop-hz <Hz>           Control loop frequency (default 100)\n"
                << "  --pwm-hz <Hz>            PWM safety layer frequency (default 100)\n"
                << "  --max-step <pct>         Max duty step per cycle (default 0.2)\n"
                << "  --config <file>          pwm_client.yaml path\n"
                << "  --control-config <file>  control_params.yaml path\n"
                << "  --traj-config <file>     trajectory.yaml path (optional)\n";
        }
    }
    return args;
}

// 打印映射，方便现场校验（沿用你之前的实现）
void print_pwm_mapping(const rovctrl::platform::PwmClientConfig& cfg)
{
    using rovctrl::platform::kNumPwmChannels;

    std::cout << "[PwmClient] Logical motor → physical PWM mapping:\n";
    for (std::size_t i = 0; i < kNumPwmChannels; ++i) {
        int logical_id = static_cast<int>(i) + 1;
        int pwm_ch     = cfg.motorch_to_pwmch[i];
        int rev        = cfg.motor_reverse[i] ? 1 : 0;

        // 0 表示默认映射（由底层解释为 logical_id）
        if (pwm_ch == 0) {
            pwm_ch = logical_id;
            std::cout << "  motor " << logical_id
                      << " -> PWM " << pwm_ch
                      << " (default), reverse=" << rev << "\n";
        } else {
            std::cout << "  motor " << logical_id
                      << " -> PWM " << pwm_ch
                      << ", reverse=" << rev << "\n";
        }
    }
    std::cout.flush();
}

} // anonymous namespace

namespace rovctrl::control_core {

int app_main(int argc, char** argv)
{
    using rovctrl::platform::PwmClient;
    using rovctrl::platform::PwmClientConfig;
    using rovctrl::controllers::ManualController;
    using rovctrl::controllers::ManualControllerConfig;
    using rovctrl::io::TeleopInputProvider;

    using rovctrl::utils::resolve_pwm_client_config_path;
    using rovctrl::utils::load_pwm_client_config;
    using rovctrl::utils::resolve_control_config_path;
    using rovctrl::utils::load_thruster_allocation_config;
    using rovctrl::utils::resolve_trajectory_config_path;
    using rovctrl::utils::load_trajectory_config;

    // 0) 安装信号处理，支持 Ctrl+C 安全退出
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    CmdArgs args = parse_args(argc, argv);

    std::cout << "[INFO] pwm_control_program starting...\n";
    std::cout << "       loop_hz="        << args.loop_hz
              << ", pwm_ctrl_hz="         << args.pwm_ctrl_hz
              << ", max_step_pct="        << args.max_step_pct
              << ", pwm_config_cli="      << (args.pwm_config.empty()
                                               ? "<none>" : args.pwm_config)
              << ", control_config_cli="  << args.control_config
              << ", traj_config_cli="     << args.traj_config
              << "\n";

    // ===================== STEP1: 解析 pwm_client.yaml 路径 =====================
    std::cout << "[STEP1] Resolving pwm_client.yaml path...\n";
    fs::path pwm_cfg_path;
    if (!resolve_pwm_client_config_path(args.pwm_config, argv[0], pwm_cfg_path, std::cerr)) {
        std::cerr << "[ERR][E10] Failed to resolve pwm_client.yaml.\n"
                  << "          Tried: CLI (--config), env PWM_CLIENT_CONFIG, "
                     "paths relative to executable.\n"
                  << "          Fallback to PwmClientConfig defaults.\n";
        // 不直接退出，继续用默认 pwm_cfg
    } else {
        std::cout << "[STEP1] pwm_client.yaml = " << pwm_cfg_path << "\n";
    }

    // ===================== STEP2: 加载 PWM 配置并初始化 PwmClient ==============
    PwmClientConfig pwm_cfg;  // 默认值在结构体内已设好

    if (!pwm_cfg_path.empty()) {
        std::cout << "[STEP2] Loading PWM config from YAML...\n";
        if (!load_pwm_client_config(pwm_cfg_path, pwm_cfg, std::cerr)) {
            std::cerr << "[ERR][E11] Failed to load PWM config, fallback to defaults.\n";
        }
    } else {
        std::cout << "[STEP2] No valid pwm_client.yaml resolved, using defaults.\n";
    }

    // 命令行覆盖 YAML：优先级最高
    if (args.pwm_ctrl_hz > 0.0) {
        pwm_cfg.ctrl_hz = static_cast<float>(args.pwm_ctrl_hz);
    }
    if (args.max_step_pct > 0.0) {
        pwm_cfg.max_step_pct = static_cast<float>(args.max_step_pct);
    }

    // 映射打印
    print_pwm_mapping(pwm_cfg);

    std::cout << "[STEP2] Initializing PwmClient...\n";
    PwmClient pwm_client;
    if (!pwm_client.init(pwm_cfg)) {
        std::cerr << "[ERR][E12] PwmClient init failed: "
                  << pwm_client.status().last_error_msg << "\n";
        return static_cast<int>(AppError::PwmClientInitFailed);
    }

    std::cout << "[STEP2] PwmClient initialized, setAllMid()...\n";
    int rc_mid = pwm_client.setAllMid();
    if (rc_mid < 0) {
        std::cerr << "[ERR][E13] setAllMid failed: "
                  << pwm_client.status().last_error_msg << "\n";
        pwm_client.shutdown();
        return static_cast<int>(AppError::SetAllMidFailed);
    }

    // ===================== STEP3: 解析 & 加载推力分配配置 =======================
    std::cout << "[STEP3] Resolving control_params.yaml path...\n";
    fs::path control_cfg_path;
    if (!resolve_control_config_path(args.control_config, argv[0], control_cfg_path, std::cerr)) {
        std::cerr << "[ERR][E30] Failed to resolve control_params.yaml.\n"
                  << "          Thruster allocation will be invalid; "
                     "manual teleop still works.\n";
        // 这里不退出，允许纯 teleop 测试
    } else {
        std::cout << "[STEP3] control_params.yaml = " << control_cfg_path << "\n";
    }

    ThrusterAllocationConfig alloc_cfg{};
    if (!control_cfg_path.empty()) {
        std::cout << "[STEP3] Loading thruster allocation config...\n";
        if (!load_thruster_allocation_config(control_cfg_path, alloc_cfg, std::cerr)) {
            std::cerr << "[ERR][E31] Thruster allocation config load failed, "
                         "ControlLoop will still run but allocator may be invalid.\n";
        }
    } else {
        std::cerr << "[STEP3] No valid control_params.yaml resolved, "
                     "thruster allocation kept default.\n";
    }

    // ===================== STEP4: 解析 & 加载轨迹配置（可选） ===================
    std::cout << "[STEP4] Resolving trajectory.yaml path (optional)...\n";
    fs::path traj_cfg_path;
    TrajectoryTracking traj_tracking;
    TrajectoryConfig   traj_cfg;
    bool               traj_enabled = false;

    if (!resolve_trajectory_config_path(args.traj_config, argv[0], traj_cfg_path, std::cerr)) {
        std::cerr << "[WARN][E40] Failed to resolve trajectory.yaml. "
                     "Trajectory-based control will be disabled.\n";
    } else {
        std::cout << "[STEP4] trajectory.yaml = " << traj_cfg_path << "\n";
        if (!load_trajectory_config(traj_cfg_path, traj_cfg, std::cerr)) {
            std::cerr << "[WARN][E41] Failed to load trajectory.yaml. "
                         "Trajectory-based control will be disabled.\n";
        } else {
            traj_tracking.set_trajectory(traj_cfg);
            traj_enabled = traj_tracking.has_trajectory();
            if (!traj_enabled) {
                std::cerr << "[WARN] TrajectoryConfig loaded but no points.\n";
            } else {
                std::cout << "[STEP4] trajectory loaded: points="
                          << traj_cfg.points.size()
                          << ", frame=" << traj_cfg.frame_raw
                          << ", angle_unit=" << traj_cfg.angle_unit_raw
                          << ", type=" << traj_cfg.type_raw
                          << "\n";
            }
        }
    }

    // ===================== STEP5: 构造输入源 & 控制器 ===========================
    std::cout << "[STEP5] Creating TeleopInput and ManualController...\n";

    // 5.1 输入源：键盘 Teleop
    auto input = std::make_shared<TeleopInputProvider>();

    // 5.2 控制器：ManualController（手动模式）
    ManualControllerConfig mc_cfg;
    mc_cfg.surge_gain  = 1.0;  // W/S 档位整体缩放
    mc_cfg.sway_gain   = 1.0;  // A/D
    mc_cfg.heave_gain  = 1.0;  // G/H
    mc_cfg.yaw_gain    = 1.0;  // Q/E
    mc_cfg.roll_gain   = 1.0;  // R/T（横滚）
    mc_cfg.pitch_gain  = 1.0;  // F/V（俯仰）
    mc_cfg.max_cmd_abs = 1.0;  // 手动模式下归一化输出 [-1,1]

    auto controller = std::make_shared<ManualController>(mc_cfg);

    // ===================== STEP6: 构造控制主循环 ===============================
    std::cout << "[STEP6] Creating ControlLoop...\n";

    ControlLoop::Config loop_cfg;
    loop_cfg.loop_hz                 = args.loop_hz;
    loop_cfg.max_step_errors         = 1000;
    loop_cfg.step_error_log_interval = 100;
    loop_cfg.log_timing              = false;
    loop_cfg.enable_pwm_log          = true;
    loop_cfg.thruster_alloc          = alloc_cfg;

    // 目前还没把 TrajectoryTracking 接到 ControlLoop 内部，这里先不传；
    // 将来如果在 ControlLoop 中加入轨迹引用，可以扩展构造函数。
    ControlLoop loop(loop_cfg,
                     pwm_client,
                     input,
                     controller,
                     &g_stop_flag);

    // ===================== STEP7: 进入主循环 ===============================
    std::cout << "[STEP7] Entering main control loop...\n";
    int rc = 0;
    try {
        rc = loop.run();
    } catch (const std::exception& e) {
        std::cerr << "[ERR][E50] Exception in ControlLoop::run(): "
                  << e.what() << "\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    } catch (...) {
        std::cerr << "[ERR][E51] Unknown exception in ControlLoop::run().\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    }

    std::cout << "[INFO] ControlLoop exited with rc=" << rc << "\n";
    std::cout << "[INFO] emergencyStop(1.0s) before shutdown...\n";

    // ===================== STEP8: 退出前安全收尾 ==============================
    pwm_client.emergencyStop(1.0f);
    pwm_client.shutdown();

    std::cout << "[INFO] pwm_control_program exited.\n";
    return rc;
}

} // namespace rovctrl::control_core
