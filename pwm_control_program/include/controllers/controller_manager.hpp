#pragma once
#ifndef ROVCTRL_CONTROLLERS_CONTROLLER_MANAGER_HPP
#define ROVCTRL_CONTROLLERS_CONTROLLER_MANAGER_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_core/control_mode.hpp"
#include "control_core/control_types.hpp"

// -----------------------------------------------------------------------------
// Forward declarations + deleter (keeps header clean; avoids incomplete-type UB)
// -----------------------------------------------------------------------------
namespace rovctrl::controllers {

class IController;

// 自定义 deleter：声明在头文件，定义放到 .cpp 里（包含 IController 完整定义后 delete）
struct IControllerDeleter final {
    void operator()(IController* p) const noexcept;
};

} // namespace rovctrl::controllers

namespace rovctrl::control_core {

struct ControlParams;

// =============================================================================
// Options / Status / Command
// =============================================================================

struct ControllerManagerOptions {
    std::string  default_auto_controller{"pid"};
    bool         failsafe_zero_output{true};
    double       min_switch_interval_sec{0.2};
    std::uint32_t auto_fail_limit{3};
};

struct ControllerManagerStatus {
    bool        ok{false};
    ControlMode mode{ControlMode::kUnknown};

    std::string active_controller;
    std::string desired_controller;

    std::string last_error;

    std::uint64_t last_switch_t_ns{0};
    std::uint32_t consecutive_failures{0};
    bool          last_compute_ok{false};
};

struct ControlCommand {
    enum class Kind : std::uint8_t {
        None = 0,
        SetManual,
        SetAuto,
        SetFailsafe,
        SelectAutoController,
        QueryStatus,
        EmergencyStop
    };

    Kind        kind{Kind::None};
    std::string controller_name;
    std::string source;
    bool        force{false};
};

// =============================================================================
// ControllerManager
// =============================================================================

class ControllerManager final {
public:
    using ControllerPtr =
        std::unique_ptr<rovctrl::controllers::IController,
                        rovctrl::controllers::IControllerDeleter>;

    ControllerManager() = default;
    explicit ControllerManager(const ControllerManagerOptions& opt)
        : options_(opt) {}

    ~ControllerManager() noexcept; // out-of-line in .cpp

    ControllerManager(const ControllerManager&)            = delete;
    ControllerManager& operator=(const ControllerManager&) = delete;

    ControllerManager(ControllerManager&&) noexcept;            // out-of-line in .cpp
    ControllerManager& operator=(ControllerManager&&) noexcept; // out-of-line in .cpp

    // ----------------- Controller factory (optional but recommended) -----------------
    template <class T, class... Args>
    static ControllerPtr make_controller(Args&&... args) {
        static_assert(std::is_base_of_v<rovctrl::controllers::IController, T>,
                      "T must derive from rovctrl::controllers::IController");
        return ControllerPtr(new T(std::forward<Args>(args)...));
    }

    // ================= Initialization =================
    bool init_manual_only(ControllerPtr manual_ctrl);

    bool init_from_params(const ControlParams& params,
                          ControllerPtr        manual_ctrl);

    // ================= Commands (Scheme A) =================
    bool apply_command(const ControlCommand& cmd,
                       std::uint64_t         now_ns);

    // ================= Legacy / direct APIs =================
    bool set_mode(ControlMode mode);
    [[nodiscard]] ControlMode mode() const noexcept { return mode_; }

    bool select_auto_controller(const std::string& name);

    [[nodiscard]] const std::string& active_controller_name() const noexcept {
        return active_name_;
    }

    [[nodiscard]] bool has_active_controller() const noexcept {
        return active_ != nullptr;
    }

    bool compute(const ControlState&     state,
                 const ControlReference& ref,
                 ControlOutput&          out,
                 double                  dt);

    [[nodiscard]] const ControllerManagerStatus& status() const noexcept {
        return status_;
    }

    // ================= Query helpers =================
    std::vector<std::string> list_controllers() const;
    bool has_controller(std::string_view name) const;

    [[nodiscard]] std::string default_auto_controller() const {
        return default_auto_name_;
    }

private:
    using ControllerMap = std::unordered_map<std::string, ControllerPtr>;

    bool register_builtin_controllers(const ControlParams& params);
    bool switch_active_controller(const std::string& name);

    void set_error(std::string msg);
    void clear_error();

    void apply_failsafe_output(ControlOutput& out);
    bool allow_switch_now(std::uint64_t now_ns) const;

private:
    ControllerManagerOptions options_{};
    ControllerManagerStatus  status_{};

    ControlMode mode_{ControlMode::kUnknown};

    ControllerMap                      controllers_;
    rovctrl::controllers::IController* active_{nullptr}; // non-owning
    std::string                        active_name_;

    std::string default_auto_name_{"pid"};
};

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROLLERS_CONTROLLER_MANAGER_HPP
