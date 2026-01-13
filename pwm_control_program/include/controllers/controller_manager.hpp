#pragma once
#ifndef ROVCTRL_CONTROLLERS_CONTROLLER_MANAGER_HPP
#define ROVCTRL_CONTROLLERS_CONTROLLER_MANAGER_HPP

/**
 * @file  controllers/controller_manager.hpp
 * @brief 控制器管理与模式调度核心接口声明
 *
 * 模块定位（业务视角）：
 *   - 负责“控制模式 + 控制器实例”的统一调度：
 *       * Manual / Auto / Failsafe 等控制模式；
 *       * 多种 Auto 控制器（PID / MPC / RL …）的注册、切换、状态管理；
 *   - 对上暴露统一的 `compute(state, ref, out, dt)` 接口：
 *       * ControlLoop 只关心“当前模式 + 当前控制器输出”；
 *       * 具体控制算法细节封装在各个 IController 实现内部；
 *   - 同时承担“基本 failsafe 策略”和“切换节流策略”：
 *       * 通过 options.failsafe_zero_output 控制错误时是否强制零输出；
 *       * 通过 options.min_switch_interval_sec 限制频繁切换控制器；
 *       * 维护 consecutive_failures / last_compute_ok 等诊断信息。
 *
 * 与其他模块的关系：
 *   - 输入：
 *       * ControlState：由 ControlLoop 整理的系统状态（含导航、解锁、急停等）；
 *       * ControlReference：由 ControlGuard / 轨迹模块生成的参考（teleop_dof / ref_track 等）；
 *   - 输出：
 *       * ControlOutput：包含 body_wrench / thruster_command 等结果；
 *   - 配置：
 *       * ControlParams：在 init_from_params() 阶段传入，用于创建内置 Auto 控制器等。
 *
 * 使用约定（典型流程）：
 *   1) 启动阶段：
 *        ControllerManager mgr(opt);
 *        mgr.init_from_params(params, manual_ctrl_ptr);
 *
 *   2) 模式/控制器管理：
 *        mgr.set_mode(ControlMode::kManual / kAuto / kFailsafe);
 *        mgr.select_auto_controller("mpc"); // 仅在 Auto 模式下生效
 *
 *   3) 控制主循环内：
 *        ControlOutput out{};
 *        if (!mgr.compute(state, ref, out, dt)) {
 *            // 根据 mgr.status().last_error 做日志 / failsafe
 *        }
 *
 *   4) 诊断与可视化：
 *        const auto& st = mgr.status();
 *        // st.mode / st.active_controller / st.consecutive_failures ...
 */

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
// Forward declarations + 自定义 deleter（只在 .cpp 里包含 IController 定义）
// -----------------------------------------------------------------------------
namespace rovctrl::controllers {

class IController;

/**
 * @brief IController 专用 deleter
 *
 * 设计目的：
 *   - 头文件中仅 forward declare IController，避免在所有使用处强行 include
 *     其完整定义；
 *   - 自定义 deleter 在 .cpp 中实现，保证 delete 时 IController 为完整类型，
 *     避免不完整类型的 UB。
 */
struct IControllerDeleter final {
    void operator()(IController* p) const noexcept;
};

} // namespace rovctrl::controllers

namespace rovctrl::control_core {

struct ControlParams;

// =============================================================================
// 配置 / 状态结构体
// =============================================================================

/**
 * @brief ControllerManager 运行选项
 */
struct ControllerManagerOptions {
    /**
     * @brief 默认 Auto 控制器名称
     *
     * 用途：
     *   - 在 Auto 模式首次启用或未显式指定控制器时，尝试激活该控制器；
     *   - 对齐配置文件中的默认控制器名（如 "pid" / "mpc"）。
     */
    std::string  default_auto_controller{"pid"};

    /**
     * @brief 当控制器计算失败时，是否强制输出零（安全保底）
     *
     * true：
     *   - compute() 失败或 active controller 不存在时，对输出调用 apply_failsafe_output()
     *   - 通常意味着“清零推力 / 中立 PWM”；
     *
     * false：
     *   - 保留 out 中已有内容，由上层决定如何处理（仅适用于高度自定义场景）。
     */
    bool         failsafe_zero_output{true};

    /**
     * @brief 连续切换控制器的最小间隔（秒）
     *
     * 用途：
     *   - 防止 UI / GCS 抖动导致控制器频繁来回切换；
     *   - 仅在需要切换 active controller 时检查。
     */
    double       min_switch_interval_sec{0.2};

    /**
     * @brief 连续 compute 失败次数上限
     *
     * 用途：
     *   - 可用于触发自动进入 Failsafe 模式（由上层或 ControlLoop 读取该计数后决定）。
     */
    std::uint32_t auto_fail_limit{3};
};

/**
 * @brief ControllerManager 运行状态 / 诊断信息
 */
struct ControllerManagerStatus {
    bool        ok{false};                 ///< 当前整体状态是否“健康”（最近一次 compute 是否成功）
    ControlMode mode{ControlMode::kUnknown}; ///< 当前控制模式（Manual / Auto / Failsafe / Unknown）

    std::string active_controller;         ///< 当前生效控制器名称（Manual / Auto 对应不同实现）
    std::string desired_controller;        ///< 最近一次“期望切换”的控制器名（未必成功）

    std::string last_error;                ///< 最近一次错误信息（如 compute 失败原因）

    std::uint64_t last_switch_t_ns{0};     ///< 最近一次控制器切换时间（mono ns）
    std::uint32_t consecutive_failures{0}; ///< 连续 compute 失败次数
    bool          last_compute_ok{false};  ///< 最近一次 compute() 返回值
};

/**
 * @brief 高层控制命令对象（Scheme A：命令式接口）
 *
 * 与 `set_mode()` / `select_auto_controller()` 等“直接 API”相比，
 * `ControlCommand` 更适合作为“消息对象”在不同模块之间传递（如从 GCS / 任务管理器传下）。
 */
struct ControlCommand {
    enum class Kind : std::uint8_t {
        None = 0,              ///< 空命令，占位
        SetManual,             ///< 切换到 Manual 模式
        SetAuto,               ///< 切换到 Auto 模式
        SetFailsafe,           ///< 切换到 Failsafe 模式
        SelectAutoController,  ///< 选择某个 Auto 控制器（需在 Auto 模式下生效）
        QueryStatus,           ///< 请求当前状态（通常仅触发一次性返回）
        EmergencyStop          ///< 紧急停止（由上层解释为具体策略）
    };

    Kind        kind{Kind::None}; ///< 命令类型
    std::string controller_name;  ///< 目标控制器名（仅对 SelectAutoController 有意义）
    std::string source;           ///< 命令来源（GCS / local / script），仅用于日志
    bool        force{false};     ///< 是否强制执行（绕过某些“节流/保护”条件）
};

// =============================================================================
// ControllerManager
// =============================================================================

/**
 * @brief 控制器管理器（ControlMode + IController 的统一调度中枢）
 *
 * 职责：
 *   - 维护当前控制模式（Manual / Auto / Failsafe）；
 *   - 按名称注册/查找/切换多个控制器实例（如 "manual" / "pid" / "mpc" / "rl"）；
 *   - 对外暴露统一的 compute(...) 接口：
 *       * 根据当前 mode 和 active_controller 分发到具体控制器；
 *       * 在错误或空输出情况下，应用基本 failsafe 策略；
 *   - 记录状态与诊断信息（ControllerManagerStatus）。
 *
 * 线程模型：
 *   - 设计为在单线程控制循环内调用，不做内部锁保护；
 *   - 若未来需要多线程使用，应由调用方保证同步。
 */
class ControllerManager final {
public:
    /// 控制器智能指针类型（带自定义 deleter）
    using ControllerPtr =
        std::unique_ptr<rovctrl::controllers::IController,
                        rovctrl::controllers::IControllerDeleter>;

    ControllerManager() = default;

    explicit ControllerManager(const ControllerManagerOptions& opt)
        : options_(opt) {}

    ~ControllerManager() noexcept; // 在 .cpp 中定义（IController 完整可见）

    ControllerManager(const ControllerManager&)            = delete;
    ControllerManager& operator=(const ControllerManager&) = delete;

    ControllerManager(ControllerManager&&) noexcept;            // 在 .cpp 中定义
    ControllerManager& operator=(ControllerManager&&) noexcept; // 在 .cpp 中定义

    // -------------------------------------------------------------------------
    // Controller factory（推荐用来构造控制器，保持创建方式统一）
    // -------------------------------------------------------------------------
    template <class T, class... Args>
    static ControllerPtr make_controller(Args&&... args) {
        static_assert(std::is_base_of_v<rovctrl::controllers::IController, T>,
                      "T must derive from rovctrl::controllers::IController");
        return ControllerPtr(new T(std::forward<Args>(args)...));
    }

    // ================= 初始化 =================

    /**
     * @brief 仅注册一个 Manual 控制器（最小可用配置）
     *
     * 适用场景：
     *   - 早期 bring-up：只使用 Manual 模式，不启用任意 Auto 控制器；
     *   - 测试硬件 / 推力映射时，只依赖键盘/手柄 teleop。
     */
    bool init_manual_only(ControllerPtr manual_ctrl);

    /**
     * @brief 基于 ControlParams 完成控制器注册与默认设置
     *
     * 典型工作：
     *   - 注册 Manual 控制器（由调用方提供指针）；
     *   - 调用 register_builtin_controllers() 注册 PID / MPC / RL 等内置 Auto 控制器；
     *   - 设置默认 Auto 控制器名等。
     */
    bool init_from_params(const ControlParams& params,
                          ControllerPtr        manual_ctrl);

    // ================= 命令式接口（Scheme A） =================

    /**
     * @brief 处理一条高层控制命令（SetManual / SetAuto / SelectAutoController / ...）
     *
     * @param cmd    控制命令对象（见 ControlCommand）
     * @param now_ns 当前时间（mono ns，用于节流控制）
     */
    bool apply_command(const ControlCommand& cmd,
                       std::uint64_t         now_ns);

    // ================= 直接 API（兼容旧逻辑 / 简单场景） =================

    /**
     * @brief 直接设置控制模式（不经由 ControlCommand）
     *
     * 注意：
     *   - 不自动选择 Auto 控制器，仅改变 mode_；
     *   - 若需要同时指定 Auto 控制器，请配合 select_auto_controller() 使用。
     */
    bool set_mode(ControlMode mode);

    /// @brief 当前控制模式（只读）
    [[nodiscard]] ControlMode mode() const noexcept { return mode_; }

    /**
     * @brief 指定当前 Auto 模式下希望使用的控制器名称
     *
     * 行为：
     *   - 若 name 已注册且允许切换，则更新 active_ / active_name_；
     *   - 若 name 未注册，返回 false 并更新 status_.last_error。
     */
    bool select_auto_controller(const std::string& name);

    /// @brief 当前 active controller 名称（Manual / 某 Auto 控制器）
    [[nodiscard]] const std::string& active_controller_name() const noexcept {
        return active_name_;
    }

    /// @brief 是否存在已激活的控制器实例
    [[nodiscard]] bool has_active_controller() const noexcept {
        return active_ != nullptr;
    }

    /**
     * @brief 控制主循环入口：根据当前 mode + active controller 计算控制输出
     *
     * 逻辑概要：
     *   1) 根据 mode_ 决定调用哪个 IController 实例；
     *   2) 如无 active controller 或 compute() 返回失败：
     *        - 记录错误信息与 consecutive_failures；
     *        - 如 options_.failsafe_zero_output 为 true，则调用 apply_failsafe_output(out)；
     *   3) 成功时：
     *        - status_.last_compute_ok = true；
     *        - 如有需要，由调用方根据 status_.consecutive_failures 决定是否进入更激进的 failsafe 模式。
     *
     * @return true 表示调用成功（不代表控制器认为状态“正常”，仅代表框架未崩溃）；
     *         false 通常表示内部逻辑错误或严重配置问题。
     */
    bool compute(const ControlState&     state,
                 const ControlReference& ref,
                 ControlOutput&          out,
                 double                  dt);

    /// @brief 获取当前 ControllerManagerStatus（供日志 / Telemetry 使用）
    [[nodiscard]] const ControllerManagerStatus& status() const noexcept {
        return status_;
    }

    // ================= 查询辅助函数 =================

    /**
     * @brief 列出当前已注册的控制器名称（包含 Manual + 各 Auto 控制器）
     */
    std::vector<std::string> list_controllers() const;

    /**
     * @brief 查询是否存在名为 name 的控制器
     */
    bool has_controller(std::string_view name) const;

    /**
     * @brief 默认 Auto 控制器名称（通常来自 options_.default_auto_controller）
     */
    [[nodiscard]] std::string default_auto_controller() const {
        return default_auto_name_;
    }

private:
    using ControllerMap = std::unordered_map<std::string, ControllerPtr>;

    /**
     * @brief 注册内置 Auto 控制器（PID / MPC / RL 等）
     *
     * 约定：
     *   - 在 init_from_params() 中被调用；
     *   - 根据 ControlParams 中的配置决定实际注册哪些控制器。
     */
    bool register_builtin_controllers(const ControlParams& params);

    /**
     * @brief 内部实现：切换 active controller 到指定名称
     *
     * 不做 mode_ 检查，仅负责：
     *   - 在 controllers_ 中查找 name；
     *   - 更新 active_ / active_name_；
     *   - 更新 status_.last_switch_t_ns。
     */
    bool switch_active_controller(const std::string& name);

    /// @brief 设置错误信息并标记 status_.ok = false
    void set_error(std::string msg);

    /// @brief 清除错误标记与 last_error 内容
    void clear_error();

    /**
     * @brief 失败或未激活控制器时应用基础 failsafe 输出
     *
     * 默认行为：
     *   - 清空 ControlOutput 中的 body_wrench / thruster_command 等；
     *   - 具体策略可在 .cpp 实现中根据工程需要调整。
     */
    void apply_failsafe_output(ControlOutput& out);

    /**
     * @brief 判断当前时间点是否允许切换控制器
     *
     * 条件：
     *   - now_ns - status_.last_switch_t_ns >= options_.min_switch_interval_sec；
     */
    bool allow_switch_now(std::uint64_t now_ns) const;

private:
    ControllerManagerOptions options_{};
    ControllerManagerStatus  status_{};

    ControlMode mode_{ControlMode::kUnknown};

    ControllerMap                      controllers_; ///< 所有已注册控制器（名称 → 实例）
    rovctrl::controllers::IController* active_{nullptr}; ///< 当前激活控制器（非 owning 指针）
    std::string                        active_name_;     ///< 当前激活控制器名

    std::string default_auto_name_{"pid"}; ///< 默认 Auto 控制器名（通常与 options_ 一致）
};

} // namespace rovctrl::control_core

#endif // ROVCTRL_CONTROLLERS_CONTROLLER_MANAGER_HPP
