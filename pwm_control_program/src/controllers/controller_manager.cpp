#include "controllers/controller_manager.hpp"

#include <iostream>
#include <utility>

// IMPORTANT: must include the full definition of IController here
// to allow unique_ptr<IController> to be destroyed safely.
#include "controllers/controller_base.hpp"

namespace rovctrl::controllers {
void IControllerDeleter::operator()(IController* p) const noexcept {
    delete p;
}
} // namespace rovctrl::controllers

namespace rovctrl::control_core {

// ============================================================================
// Out-of-line special members (required by forward-declared IController)
// ============================================================================

ControllerManager::~ControllerManager() noexcept = default;

ControllerManager::ControllerManager(ControllerManager&&) noexcept = default;
ControllerManager& ControllerManager::operator=(ControllerManager&&) noexcept = default;

// ============================================================================
// Initialization
// ============================================================================

bool ControllerManager::init_manual_only(ControllerPtr manual_ctrl)
{
    controllers_.clear();
    active_      = nullptr;
    active_name_.clear();

    status_ = ControllerManagerStatus{};
    mode_   = ControlMode::kUnknown;

    if (!manual_ctrl) {
        set_error("manual controller is null");
        return false;
    }

    // NOTE:
    // This file assumes IController exposes name() (string).
    // If your base interface differs, keep your original logic and only
    // update ControlMode enum names.
    const std::string name = manual_ctrl->name();
    controllers_.emplace(name, std::move(manual_ctrl));

    if (!switch_active_controller(name)) {
        set_error("failed to activate manual controller: " + name);
        return false;
    }

    mode_        = ControlMode::kManual;
    status_.mode = mode_;
    status_.ok   = true;

    return true;
}

bool ControllerManager::init_from_params(const ControlParams& params,
                                        ControllerPtr        manual_ctrl)
{
    controllers_.clear();
    active_      = nullptr;
    active_name_.clear();

    status_ = ControllerManagerStatus{};
    mode_   = ControlMode::kUnknown;

    if (!manual_ctrl) {
        set_error("manual controller is null");
        return false;
    }

    // Register manual controller first
    {
        const std::string name = manual_ctrl->name();
        controllers_.emplace(name, std::move(manual_ctrl));
    }

    // Register builtins based on params
    if (!register_builtin_controllers(params)) {
        // register_builtin_controllers already sets error
        return false;
    }

    // Choose defaults
    default_auto_name_ = options_.default_auto_controller.empty()
                             ? std::string{"pid"}
                             : options_.default_auto_controller;

    // Activate manual by default if present
    if (controllers_.empty()) {
        set_error("no controllers registered");
        return false;
    }

    // Prefer "manual" if exists; otherwise pick first
    if (has_controller("manual")) {
        if (!switch_active_controller("manual")) {
            set_error("failed to activate manual controller");
            return false;
        }
        mode_ = ControlMode::kManual;
    } else {
        // fallback
        const auto it = controllers_.begin();
        if (!switch_active_controller(it->first)) {
            set_error("failed to activate controller: " + it->first);
            return false;
        }
        mode_ = ControlMode::kManual; // treat as manual-like
    }

    status_.mode = mode_;
    status_.ok   = true;
    return true;
}

// ============================================================================
// Commands
// ============================================================================

bool ControllerManager::apply_command(const ControlCommand& cmd,
                                     std::uint64_t         now_ns)
{
    switch (cmd.kind) {
    case ControlCommand::Kind::None:
        return true;

    case ControlCommand::Kind::SetManual:
        if (!cmd.force && !allow_switch_now(now_ns)) {
            set_error("mode switch throttled (manual)");
            return false;
        }
        status_.last_switch_t_ns = now_ns;
        return set_mode(ControlMode::kManual);

    case ControlCommand::Kind::SetAuto:
        if (!cmd.force && !allow_switch_now(now_ns)) {
            set_error("mode switch throttled (auto)");
            return false;
        }
        status_.last_switch_t_ns = now_ns;
        return set_mode(ControlMode::kAuto);

    case ControlCommand::Kind::SetFailsafe:
        status_.last_switch_t_ns = now_ns;
        return set_mode(ControlMode::kFailsafe);

    case ControlCommand::Kind::SelectAutoController:
        if (!cmd.force && !allow_switch_now(now_ns)) {
            set_error("controller switch throttled");
            return false;
        }
        status_.last_switch_t_ns = now_ns;
        return select_auto_controller(cmd.controller_name);

    case ControlCommand::Kind::QueryStatus:
        return true;

    case ControlCommand::Kind::EmergencyStop:
        status_.last_switch_t_ns = now_ns;
        return set_mode(ControlMode::kFailsafe);

    default:
        set_error("unknown command kind");
        return false;
    }
}

// ============================================================================
// Legacy / direct APIs
// ============================================================================

bool ControllerManager::set_mode(ControlMode mode)
{
    // kNone means "no request": keep current mode
    if (mode == ControlMode::kNone) {
        return true;
    }

    // Normalize unknown -> failsafe (defensive)
    if (mode == ControlMode::kUnknown) {
        set_error("set_mode called with kUnknown");
        return false;
    }

    mode_        = mode;
    status_.mode = mode_;
    clear_error();

    // When switching to auto, ensure desired controller exists
    if (mode_ == ControlMode::kAuto) {
        if (!status_.desired_controller.empty()) {
            if (!has_controller(status_.desired_controller)) {
                set_error("desired auto controller not found: " + status_.desired_controller);
                return false;
            }
            if (!switch_active_controller(status_.desired_controller)) {
                set_error("failed to switch to desired auto controller: " + status_.desired_controller);
                return false;
            }
        } else if (!default_auto_name_.empty() && has_controller(default_auto_name_)) {
            status_.desired_controller = default_auto_name_;
            if (!switch_active_controller(default_auto_name_)) {
                set_error("failed to switch to default auto controller: " + default_auto_name_);
                return false;
            }
        } else {
            // Auto requested but no controller specified
            set_error("auto mode requested but no auto controller available");
            return false;
        }
    }

    // Manual mode: prefer "manual" controller if it exists
    if (mode_ == ControlMode::kManual) {
        if (has_controller("manual")) {
            if (!switch_active_controller("manual")) {
                set_error("failed to switch to manual controller");
                return false;
            }
        }
    }

    return true;
}

bool ControllerManager::select_auto_controller(const std::string& name)
{
    if (name.empty()) {
        set_error("select_auto_controller: empty name");
        return false;
    }
    if (!has_controller(name)) {
        set_error("auto controller not found: " + name);
        return false;
    }

    status_.desired_controller = name;

    // If already in auto mode, switch immediately
    if (mode_ == ControlMode::kAuto) {
        if (!switch_active_controller(name)) {
            set_error("failed to switch auto controller: " + name);
            return false;
        }
    }

    clear_error();
    return true;
}

// ============================================================================
// Compute
// ============================================================================

bool ControllerManager::compute(const ControlState&     state,
                               const ControlReference& ref,
                               ControlOutput&          out,
                               double                  dt)
{
    status_.last_compute_ok = false;

    // Failsafe: output policy
    if (mode_ == ControlMode::kFailsafe) {
        apply_failsafe_output(out);
        status_.last_compute_ok = true;
        clear_error();
        return true;
    }

    // Unknown or no active controller -> failsafe output
    if (mode_ == ControlMode::kUnknown || active_ == nullptr) {
        apply_failsafe_output(out);
        set_error("no active controller (unknown mode or null active)");
        return false;
    }

    // Manual/Auto: delegate to active controller
    const bool ok = active_->compute(state, ref, out, dt);
    status_.last_compute_ok = ok;

    if (ok) {
        status_.consecutive_failures = 0;
        clear_error();
        return true;
    }

    // Failure handling
    status_.consecutive_failures += 1;

    set_error("controller compute failed: " + active_name_);

    if (status_.consecutive_failures >= options_.auto_fail_limit) {
        // Enter failsafe on repeated failures
        mode_        = ControlMode::kFailsafe;
        status_.mode = mode_;
        apply_failsafe_output(out);
        std::cerr << "[ControllerManager] Auto fail limit reached -> FAILSAFE\n";
    }

    return false;
}

// ============================================================================
// Query helpers
// ============================================================================

std::vector<std::string> ControllerManager::list_controllers() const
{
    std::vector<std::string> names;
    names.reserve(controllers_.size());
    for (const auto& kv : controllers_) {
        names.push_back(kv.first);
    }
    return names;
}

bool ControllerManager::has_controller(std::string_view name) const
{
    return controllers_.find(std::string(name)) != controllers_.end();
}

// ============================================================================
// Internals
// ============================================================================

bool ControllerManager::register_builtin_controllers(const ControlParams& params)
{
    (void)params;

    // TODO: build pid/smc/mpc ... from params and register into controllers_.
    // NOTE: keep your existing implementation here; this stub remains intentionally minimal.

    // If your existing project already registers PID/Manual controllers elsewhere,
    // you can keep that logic. This function must return true when registration is OK.

    // For now, assume manual controller already exists and PID controller is optional.
    // If your build expects PID always, implement it here using your pid_controller.hpp.

    clear_error();
    return true;
}

bool ControllerManager::switch_active_controller(const std::string& name)
{
    auto it = controllers_.find(name);
    if (it == controllers_.end() || !it->second) {
        set_error("switch_active_controller: controller not found or null: " + name);
        return false;
    }

    active_      = it->second.get();
    active_name_ = name;

    status_.active_controller  = active_name_;
    status_.consecutive_failures = 0;

    clear_error();
    return true;
}

void ControllerManager::set_error(std::string msg)
{
    status_.ok         = false;
    status_.last_error = std::move(msg);

    std::cerr << "[ControllerManager] ERROR: " << status_.last_error << "\n";
}

void ControllerManager::clear_error()
{
    status_.ok = true;
    status_.last_error.clear();
}

void ControllerManager::apply_failsafe_output(ControlOutput& out)
{
    if (options_.failsafe_zero_output) {
        out = ControlOutput{}; // rely on default-zero initialization
    }
}

bool ControllerManager::allow_switch_now(std::uint64_t now_ns) const
{
    if (options_.min_switch_interval_sec <= 0.0) {
        return true;
    }
    const double min_ns = options_.min_switch_interval_sec * 1e9;
    const auto   last   = status_.last_switch_t_ns;
    if (last == 0) return true;
    return static_cast<double>(now_ns - last) >= min_ns;
}

} // namespace rovctrl::control_core
