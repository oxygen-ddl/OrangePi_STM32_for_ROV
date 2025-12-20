#include "control_core/app_main.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include "control_core/app_context.hpp"

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

    std::string pwm_config;
    std::string control_config;
    std::string traj_config;
    std::string alloc_config;

    bool enable_gcs    = true;
    bool enable_teleop = true;
    bool show_help     = false;

    // --- PWM dummy backend (方式2：显式开关) ---
    bool pwm_dummy       = false; // --pwm-dummy
    bool pwm_dummy_print = false; // --pwm-dummy-print
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
        << "  --pwm-dummy              Use dummy PWM backend (no STM32 required)\n"
        << "  --pwm-dummy-print        Print dummy PWM frames/state (debug)\n"
        << "  -h, --help               Show this help\n";
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
              << ", pwm_dummy=" << (args.pwm_dummy ? "1" : "0")
              << ", pwm_dummy_print=" << (args.pwm_dummy_print ? "1" : "0")
              << "\n";

    // ---------- Build context ----------
    AppBuildOptions opt{};
    opt.loop_hz            = args.loop_hz;
    opt.pwm_ctrl_hz        = args.pwm_ctrl_hz;
    opt.max_step_pct       = args.max_step_pct;

    opt.pwm_config_cli     = args.pwm_config;
    opt.control_config_cli = args.control_config;
    opt.traj_config_cli    = args.traj_config;
    opt.alloc_config_cli   = args.alloc_config;

    opt.enable_gcs         = args.enable_gcs;
    opt.enable_teleop      = args.enable_teleop;

    opt.pwm_dummy          = args.pwm_dummy;
    opt.pwm_dummy_print    = args.pwm_dummy_print;

    AppContext ctx;
    const auto br = build_app_context(opt, argv[0], &g_stop_flag, ctx, std::cerr);
    if (!br.ok) {
        std::cerr << "[ERR] build_app_context failed: " << br.err_msg
                  << " (err_code=" << br.err_code << ")\n";
        ctx.shutdown(std::cerr, 1.0f);
        return br.err_code;
    }

    // ---------- Run loop ----------
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
    ctx.shutdown(std::cerr, 1.0f);
    return rc;
}

} // namespace rovctrl::control_core
