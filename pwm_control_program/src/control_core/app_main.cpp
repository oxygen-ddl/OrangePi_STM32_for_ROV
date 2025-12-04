// src/control_core/app_main.cpp

#include "control_core/app_main.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "platform/pwm_client.hpp"
#include "control_core/control_loop.hpp"
#include "controllers/manual_controller.hpp"
#include "io/teleop_input.hpp"

// YAML 解析
#include <yaml-cpp/yaml.h>

namespace {

// 全局停止标志，用于响应 Ctrl+C / SIGTERM
std::atomic_bool g_stop_flag{false};

void signal_handler(int)
{
    g_stop_flag.store(true);
}

// ===================== 命令行解析 =====================

struct CmdArgs {
    double      loop_hz      = 100.0;   // 控制主循环频率
    double      pwm_ctrl_hz  = 100.0;   // PwmClient 安全层频率
    double      max_step_pct = 0.2;     // 每步最大占空比变化
    std::string pwm_config;             // pwm_client.yaml 路径（可选）
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
        }
        // 以后可以在这里扩展更多参数（例如选择控制器类型等）
    }
    return args;
}

// ===================== YAML 加载工具 =====================

bool load_pwm_config_from_yaml(const std::string& path,
                               rovctrl::platform::PwmClientConfig& cfg)
{
    try {
        YAML::Node root = YAML::LoadFile(path);
        std::cout << "[PwmClient] Loading config from " << path << "\n";

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
            cfg.enable_reverse_protection = root["enable_reverse_protection"].as<bool>();
        }

        // 分组掩码：写成 0x0F / 0xF0 / 1 这样的 int 即可
        if (root["groupA_mask"]) {
            cfg.groupA_mask = static_cast<std::uint8_t>(root["groupA_mask"].as<int>());
        }
        if (root["groupB_mask"]) {
            cfg.groupB_mask = static_cast<std::uint8_t>(root["groupB_mask"].as<int>());
        }
        if (root["group_mode"]) {
            cfg.group_mode  = root["group_mode"].as<int>();
        }

        // 逻辑电机 → 物理 PWM 映射
        if (root["motorch_to_pwmch"]) {
            auto node = root["motorch_to_pwmch"];
            if (node.IsSequence()) {
                const std::size_t n = std::min<std::size_t>(
                    node.size(), rovctrl::platform::kNumPwmChannels);
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
                    node.size(), rovctrl::platform::kNumPwmChannels);
                for (std::size_t i = 0; i < n; ++i) {
                    cfg.motor_reverse[i] =
                        static_cast<std::uint8_t>(node[i].as<int>() ? 1 : 0);
                }
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[PwmClient] ERROR: failed to load config " << path
                  << " : " << e.what() << "\n";
        return false;
    }
}

// ===================== 打印映射，方便现场校验 =====================

void print_motor_mapping(const rovctrl::platform::PwmClientConfig& cfg)
{
    std::cout << "[PwmClient] Logical motor → physical PWM mapping:\n";
    for (std::size_t i = 0; i < rovctrl::platform::kNumPwmChannels; ++i) {
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
    // 1) 安装信号处理，支持 Ctrl+C 安全退出
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    CmdArgs args = parse_args(argc, argv);

    std::cout << "[INFO] pwm_control_program starting...\n";
    std::cout << "       loop_hz="      << args.loop_hz
              << ", pwm_ctrl_hz="       << args.pwm_ctrl_hz
              << ", max_step_pct="      << args.max_step_pct
              << ", config="            << (args.pwm_config.empty()
                                             ? "<none>" : args.pwm_config)
              << "\n";

    // 2) 构造 PwmClientConfig：先用默认值，再由 YAML / 命令行覆盖
    platform::PwmClientConfig pwm_cfg;  // 默认值在结构体内已设好

    // 如果指定了 --config，则尝试从 YAML 加载
    if (!args.pwm_config.empty()) 
    {
        if (!load_pwm_config_from_yaml(args.pwm_config, pwm_cfg)) {
            std::cerr << "[WARN] Failed to load PWM config, fallback to defaults.\n";
        }
    }

    // 命令行优先级最高：覆盖 ctrl_hz / max_step_pct
    if (args.pwm_ctrl_hz > 0.0) {
        pwm_cfg.ctrl_hz = static_cast<float>(args.pwm_ctrl_hz);
    }
    if (args.max_step_pct > 0.0) {
        pwm_cfg.max_step_pct = static_cast<float>(args.max_step_pct);
    }

    print_motor_mapping(pwm_cfg);

    // 3) 初始化 PwmClient（底层安全层 + UDP）
    platform::PwmClient pwm_client;
    if (!pwm_client.init(pwm_cfg)) {
        std::cerr << "[ERR] PwmClient init failed: "
                  << pwm_client.status().last_error_msg << "\n";
        return 1;
    }

    // 上电后先全部回中位
    int rc_mid = pwm_client.setAllMid();
    if (rc_mid < 0) {
        std::cerr << "[ERR] setAllMid failed: "
                  << pwm_client.status().last_error_msg << "\n";
        return 1;
    }

    // 4) 构造输入源：键盘 Teleop
    auto input = std::make_shared<rovctrl::io::TeleopInputProvider>();

    // 5) 构造控制器：ManualController（手动模式）
    rovctrl::controllers::ManualControllerConfig mc_cfg;
    mc_cfg.surge_gain  = 1.0;  // W/S 档位整体缩放
    mc_cfg.sway_gain   = 1.0;  // A/D
    mc_cfg.heave_gain  = 1.0;  // G/H
    mc_cfg.yaw_gain    = 1.0;  // Q/E
    mc_cfg.roll_gain   = 1.0;  // R/T（横滚）后面可以开
    mc_cfg.pitch_gain  = 1.0;  // F/V（俯仰）后面可以开
    mc_cfg.max_cmd_abs = 1.0;  // 手动模式下归一化输出 [-1,1]

    auto controller = std::make_shared<rovctrl::controllers::ManualController>(mc_cfg);

    // 6) 构造控制主循环
    control_core::ControlLoop::Config loop_cfg;
    loop_cfg.loop_hz                 = args.loop_hz;
    loop_cfg.max_step_errors         = 1000;
    loop_cfg.step_error_log_interval = 100;
    loop_cfg.log_timing              = false;

    control_core::ControlLoop loop(loop_cfg,
                                   pwm_client,
                                   input,
                                   controller,
                                   &g_stop_flag);

    // 7) 进入主循环（阻塞直到退出）
    int rc = loop.run();

    std::cout << "[INFO] ControlLoop exited with rc=" << rc << "\n";
    std::cout << "[INFO] emergencyStop(1.0s) before shutdown...\n";

    // 8) 退出前确保平滑归中并关闭资源
    pwm_client.emergencyStop(1.0f);
    pwm_client.shutdown();

    std::cout << "[INFO] pwm_control_program exited.\n";
    return rc;
}

} // namespace rovctrl::control_core
