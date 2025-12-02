// src/main.cpp
//
// 顶层入口：只负责基础的 --help / --version 解析，
// 其余逻辑全部委托给 control_core::app_main。

#include <iostream>
#include <string>

#include "control_core/app_main.hpp"

namespace {

// 简单版本号（可以改成由 CMake 注入）
constexpr const char* kProgramName    = "pwm_control_program";
constexpr const char* kProgramVersion = "0.1.0";

// 打印使用说明
void print_usage(const char* argv0)
{
    std::string exe = argv0 ? argv0 : kProgramName;

    std::cout
        << "\n"
        << "Usage: " << exe << " [options]\n"
        << "\n"
        << "Options:\n"
        << "  --help, -h              Show this help message and exit\n"
        << "  --version, -v           Show program version and exit\n"
        << "\n"
        << "  --config <file>         YAML config for PWM client\n"
        << "                          e.g. config/pwm_client.yaml\n"
        << "  --loop-hz <Hz>          Control loop frequency (default: 100)\n"
        << "  --pwm-hz <Hz>           PWM safety layer freq (default: 100)\n"
        << "  --max-step <pct>        Max duty change per step (default: 0.2)\n"
        << "\n"
        << "Example:\n"
        << "  " << exe
        << " --config config/pwm_client.yaml --loop-hz 100 --pwm-hz 100\n"
        << "\n";
}

} // anonymous namespace

int main(int argc, char** argv)
{
    // 先扫描是否有 --help / --version，这两类选项由 main 自己处理
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (a == "--version" || a == "-v") {
            std::cout << kProgramName << " version " << kProgramVersion << "\n";
            return 0;
        }
    }

    try {
        // 其余参数交给 app_main（控制栈真正入口）
        return rovctrl::control_core::app_main(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] Unhandled std::exception: " << e.what() << "\n";
    } catch (...) {
        std::cerr << "[FATAL] Unknown exception caught in main()\n";
    }

    return 1;
}
