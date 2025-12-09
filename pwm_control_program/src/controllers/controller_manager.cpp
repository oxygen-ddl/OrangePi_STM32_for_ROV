#include "controllers/controller_manager.hpp"

#include <iostream>
#include <utility>      // std::move

#include "controllers/controller_base.hpp"
// 将来如果需要在这里直接 new PidController / SmcController / MpcController，
// 再按需包含对应头文件即可：
// #include "controllers/pid_controller.hpp"

namespace rovctrl::control_core {

// ============================================================================
// 内部工具
// ============================================================================

void ControllerManager::set_error(std::string msg)
{
    status_.ok         = false;
    status_.last_error = std::move(msg);

    // 简单打印到 stderr，便于调试
    std::cerr << "[ControllerManager] ERROR: " << status_.last_error << "\n";
}

bool ControllerManager::switch_active_controller(const std::string& name)
{
    auto it = controllers_.find(name);
    if (it == controllers_.end() || !it->second) {
        set_error("controller '" + name + "' not found or null");
        active_      = nullptr;
        active_name_.clear();
        status_.active_controller.clear();
        return false;
    }

    active_      = it->second.get();
    active_name_ = name;

    status_.active_controller = active_name_;
    status_.ok                = true;
    status_.last_error.clear();
    return true;
}

// 目前只做占位，不依赖 ControlParams 的具体字段，后续你再补充 PID/SMC/MPC 创建逻辑即可
bool ControllerManager::register_builtin_controllers(const ControlParams& params)
{
    // 避免未使用参数的编译告警
    (void)params;

    // 示例（将来可以这样启用 PID 控制器）：
    //
    // if (params.pid.enabled) {
    //     auto pid = std::make_unique<rovctrl::controllers::PidController>(params.pid);
    //     controllers_.emplace("pid", std::move(pid));
    // }
    //
    // if (params.smc.enabled) { ... }
    // if (params.mpc.enabled) { ... }

    return true;
}

// ============================================================================
// 对外接口实现
// ============================================================================

bool ControllerManager::init_manual_only(ControllerPtr manual_ctrl)
{
    controllers_.clear();
    active_          = nullptr;
    active_name_.clear();
    status_          = ControllerManagerStatus{};
    mode_            = ControlMode::Unknown;
    default_auto_name_ = options_.default_auto_controller;

    if (!manual_ctrl) {
        set_error("init_manual_only: manual controller is null");
        return false;
    }

    controllers_.emplace("manual", std::move(manual_ctrl));

    // 默认进入手动模式
    mode_         = ControlMode::Manual;
    status_.mode  = mode_;

    if (!switch_active_controller("manual")) {
        // switch_active_controller 已经设置了错误信息
        return false;
    }

    status_.ok = true;
    return true;
}

bool ControllerManager::init_from_params(const ControlParams& params,
                                         ControllerPtr        manual_ctrl)
{
    controllers_.clear();
    active_          = nullptr;
    active_name_.clear();
    status_          = ControllerManagerStatus{};
    mode_            = ControlMode::Unknown;
    default_auto_name_ = options_.default_auto_controller;

    if (!manual_ctrl) {
        set_error("init_from_params: manual controller is null");
        return false;
    }

    // 1) 注册手动控制器（键盘/手柄 teleop）
    controllers_.emplace("manual", std::move(manual_ctrl));

    // 2) 按配置注册 PID/SMC/MPC 等自动控制器
    if (!register_builtin_controllers(params)) {
        // register_builtin_controllers 内部应当设置错误信息
        return false;
    }

    // 3) 设置默认模式：先进入 Manual，保证安全
    mode_        = ControlMode::Manual;
    status_.mode = mode_;

    if (!switch_active_controller("manual")) {
        return false;
    }

    status_.ok = true;
    return true;
}

bool ControllerManager::set_mode(ControlMode mode)
{
    // 记录目标模式
    mode_        = mode;
    status_.mode = mode_;

    switch (mode_) {
    case ControlMode::Manual:
        // 手动模式 → 使用 "manual" 控制器
        if (!switch_active_controller("manual")) {
            // 保守策略：如果切换失败，回退为 Unknown
            mode_        = ControlMode::Unknown;
            status_.mode = mode_;
            return false;
        }
        return true;

    case ControlMode::Auto:
        // 自动模式 → 使用当前选择好的自动控制器（默认 "pid"）
        if (!switch_active_controller(default_auto_name_)) {
            // 如果失败，可以回退为 Manual 或 Unknown，这里先回退为 Manual
            std::cerr << "[ControllerManager] Auto mode fallback to Manual.\n";
            mode_        = ControlMode::Manual;
            status_.mode = mode_;
            return switch_active_controller("manual");
        }
        return true;

    case ControlMode::Failsafe:
        // Failsafe 模式：当前可以先不绑定任何控制器，由 compute() 做特例处理
        active_      = nullptr;
        active_name_.clear();
        status_.active_controller.clear();
        status_.ok         = true;
        status_.last_error.clear();
        return true;

    case ControlMode::Unknown:
    default:
        active_      = nullptr;
        active_name_.clear();
        status_.active_controller.clear();
        status_.ok         = false;
        status_.last_error = "set_mode: Unknown mode selected";
        return false;
    }
}

bool ControllerManager::select_auto_controller(const std::string& name)
{
    // 如果对应控制器不存在，直接报错
    auto it = controllers_.find(name);
    if (it == controllers_.end() || !it->second) {
        set_error("select_auto_controller: controller '" + name + "' not found");
        return false;
    }

    default_auto_name_ = name;

    // 如果当前已经处于 Auto 模式，则立即切换 active 控制器
    if (mode_ == ControlMode::Auto) {
        return switch_active_controller(default_auto_name_);
    }

    // 非 Auto 模式下，仅记录，待下次进入 Auto 时生效
    status_.ok = true;
    return true;
}

bool ControllerManager::compute(const ControlState&    state,
                                const ControlReference& ref,
                                ControlOutput&          out,
                                double                  dt)
{
    // Failsafe 模式：当前简单处理为“输出清零 + 返回 true”
    if (mode_ == ControlMode::Failsafe) {
        out = ControlOutput{};   // 清零 wrench & thruster_command
        status_.ok         = true;
        status_.last_error.clear();
        return true;
    }

    if (!active_) {
        set_error("compute: no active controller");
        return false;
    }

    // 委托给当前激活控制器
    if (!active_->compute(state, ref, out, dt)) {
        set_error("compute: active controller '" + active_name_ + "' returned failure");
        return false;
    }

    status_.ok         = true;
    status_.last_error.clear();
    return true;
}

} // namespace rovctrl::control_core
