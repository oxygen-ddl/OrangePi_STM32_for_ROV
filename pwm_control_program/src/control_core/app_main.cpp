// control_core/app_main.cpp

#include "control_core/app_main.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "control_core/app_context.hpp"
#include "control_core/control_loop.hpp"
#include "controllers/controller_manager.hpp"

namespace {

std::atomic_bool g_stop_flag{false};

void signal_handler(int) { g_stop_flag.store(true); }

enum class AppError : int {
    Ok                   = 0,
    ControlLoopException = 50,
};

struct CmdArgs {
    double loop_hz      = 100.0;
    double pwm_ctrl_hz  = 100.0;
    double max_step_pct = 0.2;

    // 配置文件 CLI 覆盖
    std::string pwm_config;           // pwm_client.yaml
    std::string control_config;       // 目前预留（control_params.yaml）
    std::string traj_config;          // trajectory.yaml
    std::string alloc_config;         // alloc.yaml
    std::string teleop_mixer_config;  // NEW: teleop_mixer.yaml

    bool enable_gcs    = true;
    bool enable_teleop = true;
    bool show_help     = false;

    bool pwm_dummy       = false;
    bool pwm_dummy_print = false;
};

static void print_help()
{
    std::cout
        << "Usage: pwm_control_program [options]\n"
        << "\n"
        << "  Frequency & PWM safety:\n"
        << "    --loop-hz <Hz>              Control loop frequency (default 100)\n"
        << "    --pwm-hz <Hz>               PWM safety layer frequency (default 100)\n"
        << "    --max-step <pct>            Max duty step per cycle (default 0.2)\n"
        << "\n"
        << "  Config files:\n"
        << "    --config <file>             pwm_client.yaml path\n"
        << "    --alloc-config <file>       alloc.yaml path\n"
        << "    --traj-config <file>        trajectory.yaml path (optional)\n"
        << "    --control-config <file>     control_params.yaml path (reserved; not yet used)\n"
        << "    --teleop-mixer-config <file> teleop_mixer.yaml path (6DOF->8Thr mapping)\n"
        << "\n"
        << "  Input sources:\n"
        << "    --no-gcs                    Disable GCS shared-memory input\n"
        << "    --no-teleop                 Disable keyboard teleop input\n"
        << "\n"
        << "  PWM backend:\n"
        << "    --pwm-dummy                 Use dummy PWM backend (no STM32 required)\n"
        << "    --pwm-dummy-print           Print dummy PWM frames/state (debug)\n"
        << "\n"
        << "  Misc:\n"
        << "    -h, --help                  Show this help\n";
}

static CmdArgs parse_args(int argc, char** argv)
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

        // 配置文件 CLI
        } else if (a == "--config") {
            if (const char* v = need_value("--config")) args.pwm_config = v;
        } else if (a == "--control-config") {
            if (const char* v = need_value("--control-config")) args.control_config = v;
        } else if (a == "--traj-config") {
            if (const char* v = need_value("--traj-config")) args.traj_config = v;
        } else if (a == "--alloc-config") {
            if (const char* v = need_value("--alloc-config")) args.alloc_config = v;
        } else if (a == "--teleop-mixer-config") {           // NEW: teleop mixer 配置
            if (const char* v = need_value("--teleop-mixer-config")) args.teleop_mixer_config = v;

        // 输入源开关
        } else if (a == "--no-gcs") {
            args.enable_gcs = false;
        } else if (a == "--no-teleop") {
            args.enable_teleop = false;

        // PWM backend
        } else if (a == "--pwm-dummy") {
            args.pwm_dummy = true;
        } else if (a == "--pwm-dummy-print") {
            args.pwm_dummy_print = true;

        } else if (a == "--help" || a == "-h") {
            args.show_help = true;
        } else {
            std::cerr << "[WARN] unknown option: " << a << "\n";
            args.show_help = true;
        }
    }
    return args;
}

// RAII：确保退出时一定 shutdown（包含 emergencyStop + pwm_client.shutdown）
class AppShutdownGuard {
public:
    AppShutdownGuard(rovctrl::control_core::AppContext& ctx,
                     std::ostream&                      log,
                     float                              estop_seconds)
        : ctx_(ctx), log_(log), estop_seconds_(estop_seconds) {}

    ~AppShutdownGuard()
    {
        // 无论 run() 怎么退出，都确保执行停机
        try {
            ctx_.shutdown(log_, estop_seconds_);
        } catch (...) {
            // destructor 不抛异常
        }
    }

    AppShutdownGuard(const AppShutdownGuard&)            = delete;
    AppShutdownGuard& operator=(const AppShutdownGuard&) = delete;

private:
    rovctrl::control_core::AppContext& ctx_;
    std::ostream&                      log_;
    float                              estop_seconds_{1.0f};
};

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
              << "\n"
              << "       pwm_config_cli=" << (args.pwm_config.empty() ? "<none>" : args.pwm_config)
              << ", alloc_config_cli=" << (args.alloc_config.empty() ? "<none>" : args.alloc_config)
              << ", traj_config_cli=" << (args.traj_config.empty() ? "<none>" : args.traj_config)
              << ", control_config_cli=" << (args.control_config.empty() ? "<none>" : args.control_config)
              << ", teleop_mixer_config_cli="
              << (args.teleop_mixer_config.empty() ? "<none>" : args.teleop_mixer_config)
              << "\n"
              << "       enable_gcs=" << (args.enable_gcs ? "1" : "0")
              << ", enable_teleop=" << (args.enable_teleop ? "1" : "0")
              << ", pwm_dummy=" << (args.pwm_dummy ? "1" : "0")
              << ", pwm_dummy_print=" << (args.pwm_dummy_print ? "1" : "0")
              << "\n";

    // ---------- Build context ----------
    AppBuildOptions opt{};
    opt.loop_hz      = args.loop_hz;
    opt.pwm_ctrl_hz  = args.pwm_ctrl_hz;
    opt.max_step_pct = args.max_step_pct;

    opt.pwm_config_cli     = args.pwm_config;
    opt.control_config_cli = args.control_config;    // 当前预留（后续可在 build_app_context 中使用）
    opt.traj_config_cli    = args.traj_config;
    opt.alloc_config_cli   = args.alloc_config;

    opt.teleop_mixer_config_cli = args.teleop_mixer_config;  // NEW: 传入 teleop_mixer.yaml

    opt.enable_gcs    = args.enable_gcs;
    opt.enable_teleop = args.enable_teleop;

    opt.pwm_dummy       = args.pwm_dummy;
    opt.pwm_dummy_print = args.pwm_dummy_print;

    AppContext ctx;
    const auto br = build_app_context(opt, argv[0], &g_stop_flag, ctx, std::cerr);
    if (!br.ok) {
        std::cerr << "[ERR] build_app_context failed: " << br.err_msg
                  << " (err_code=" << br.err_code << ")\n";
        // 即使 build 失败，也尽量做停机（shutdown 自身是“尽量做”）
        ctx.shutdown(std::cerr, 1.0f);
        return br.err_code;
    }

    // 退出时强制停机（RAII 确保 run() 任何路径都 shutdown）
    AppShutdownGuard shutdown_guard(ctx, std::cerr, 1.0f);

    // ---------- 强制默认遥控模式 ----------
    // 这里的 set_mode 在 build_app_context 之后调用，
    // 确保即使配置/后续扩展了自动模式，启动时仍默认 Manual。
    try {
        (void)ctx.ctrl_mgr.set_mode(ControlMode::kManual);
    } catch (...) {
        // 若 set_mode 不抛异常，可以去掉 try/catch
    }

    int rc = 0;
    try {
        ControlLoop loop(
            ctx.loop_cfg,
            ctx.pwm_client,
            ctx.input,
            std::move(ctx.ctrl_mgr),
            &g_stop_flag
        );

        rc = loop.run();

    } catch (const std::exception& e) {
        std::cerr << "[ERR] Exception in ControlLoop::run(): " << e.what() << "\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    } catch (...) {
        std::cerr << "[ERR] Unknown exception in ControlLoop::run().\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    }

    std::cout << "[INFO] ControlLoop exited with rc=" << rc << "\n";
    std::cout << "[INFO] program exit.\n";
    return rc;
}

} // namespace rovctrl::control_core
