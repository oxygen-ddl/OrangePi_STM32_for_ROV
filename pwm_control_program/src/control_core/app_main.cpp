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

    std::string pwm_config;
    std::string control_config;
    std::string traj_config;
    std::string alloc_config;

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

// RAII：确保退出时一定 shutdown（包含 emergencyStop + pwm_client.shutdown）
class AppShutdownGuard {
public:
    AppShutdownGuard(rovctrl::control_core::AppContext& ctx, std::ostream& log, float estop_seconds)
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

    AppShutdownGuard(const AppShutdownGuard&) = delete;
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
        // 即使 build 失败，也尽量做停机（你的 shutdown 已经做“尽量做”）
        ctx.shutdown(std::cerr, 1.0f);
        return br.err_code;
    }

    // 退出时强制停机（RAII 确保 run() 任何路径都 shutdown）
    AppShutdownGuard shutdown_guard(ctx, std::cerr, 1.0f);

    // ---------- 强制默认遥控模式 ----------
    // 你希望“默认遥控模式，其他模式手动切换”——最稳妥的做法：
    // - app 启动后直接把 ControllerManager 切到 Manual
    // - 这样即使配置里写错，默认仍是 manual
    //
    // 如果你不想强制，改成：只有当当前 mode 是 Unknown/None 时才 set。
    try {
        (void)ctx.ctrl_mgr.set_mode(ControlMode::kManual);
    } catch (...) {
        // 若 set_mode 不抛异常，可删掉 try/catch
    }

    // ---------- 可选：遥控模式下不要求导航 ----------
    // 更合理的实现位置在 ControlLoop::run() 中：当 effective_mode==kManual 时，
    // 直接不检查 nav_missing、也不打印 warning。
    //
    // 如果你们 loop_cfg 有类似字段，这里可以做兜底覆盖（否则删掉此块）。
    //
    // ctx.loop_cfg.allow_run_without_nav = true;
    // ctx.loop_cfg.suppress_nav_warn_in_manual = true;

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

        // 关键：如果 loop 因为 teleop ESC 退出，teleop_input 已经 disable raw mode；
        // 这里仍可以额外做一次“温和 reset”，确保终端已恢复。
        //
        // 注意：InputProviderPtr 的真实类型你们可能是多态接口，未必有 reset()。
        // 若你们有统一 reset() 接口，就调用；否则删掉这一段。
        if (ctx.input) {
            try {
                ctx.input->reset(); // 如果接口不存在，删掉
            } catch (...) {
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[ERR] Exception in ControlLoop::run(): " << e.what() << "\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    } catch (...) {
        std::cerr << "[ERR] Unknown exception in ControlLoop::run().\n";
        rc = static_cast<int>(AppError::ControlLoopException);
    }

    // shutdown_guard 会在函数结束时执行 ctx.shutdown()
    std::cout << "[INFO] ControlLoop exited with rc=" << rc << "\n";
    std::cout << "[INFO] program exit.\n";
    return rc;
}

} // namespace rovctrl::control_core
