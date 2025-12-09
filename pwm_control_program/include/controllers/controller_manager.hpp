#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "control_core/control_types.hpp"      // ControlState / ControlReference / ControlOutput / ControlMode
#include "controllers/controller_base.hpp"     // IController 接口定义

namespace rovctrl::control_core {

/**
 * @brief 控制参数总结构前向声明
 *
 * 说明：
 *  - 该结构用于承载从 YAML 读取的控制相关配置（vehicle / pid / smc / mpc 等）；
 *  - 具体定义可以放在单独的 control_params.hpp 中；
 *  - ControllerManager 只在接口层引用，不依赖其具体内容。
 */
struct ControlParams;

/**
 * @brief 控制器管理器配置选项
 *
 * 说明：
 *  - 用于调整 ControllerManager 自身的一些行为；
 *  - 后续可按需扩展。
 */
struct ControllerManagerOptions {
    /// 默认自动控制器名称（例如 "pid" / "smc" / "mpc"），init_from_params() 时可使用
    std::string default_auto_controller{"pid"};
};

/**
 * @brief 控制器管理器运行状态快照（只读信息）
 */
struct ControllerManagerStatus {
    bool        ok{false};                             ///< 是否初始化成功且当前无致命错误
    ControlMode mode{ControlMode::Unknown};            ///< 当前控制模式（Manual / Auto / Failsafe 等）
    std::string active_controller;                     ///< 当前选中的控制器名称（如 "manual" / "pid"）
    std::string last_error;                            ///< 最近一次错误信息（如有）
};

/**
 * @brief 控制器管理器（Controller Manager）
 *
 * 设计职责：
 *  - 统一管理“控制器对象”的生命周期与选择（Manual / PID / SMC / MPC 等）；
 *  - 从高层配置（ControlParams）中构造自动控制器实例；
 *  - 根据 ControlMode / 控制器名称选择当前 active 控制器；
 *  - 对外提供统一的 compute() 接口，供 ControlLoop 调用。
 *
 * 使用建议（典型流程）：
 *  1. 在主程序启动时：
 *      - 先构造 ManualController（键盘 Teleop 对应的 6DOF → 8 推进器控制器）；
 *      - 调用 init_manual_only() 或 init_from_params() 进行初始化；
 *  2. 在控制循环中：
 *      - 根据当前 mode（Manual / Auto）调用 set_mode()；
 *      - 调用 compute() 得到 ControlOutput，再交给 PWM 客户端。
 *
 * 注意：
 *  - ControllerManager 自身不直接实现任何控制算法；
 *    所有算法都在派生自 IController 的类中（例如 ManualController / PidController / MpcController）。
 */
class ControllerManager {
public:
    using ControllerPtr = std::unique_ptr<rovctrl::controllers::IController>;

    ControllerManager() = default;

    explicit ControllerManager(const ControllerManagerOptions& opt)
        : options_(opt)
    {}

    ~ControllerManager() = default;

    ControllerManager(const ControllerManager&)            = delete;
    ControllerManager& operator=(const ControllerManager&) = delete;

    /**
     * @brief 仅初始化“手动控制”场景
     *
     * 典型用法：
     *  - 仅有 ManualController（键盘/手柄 teleop），不启用自动控制。
     *
     * @param manual_ctrl 手动控制器实例（通常为 ManualController）
     * @return true 初始化成功
     * @return false 初始化失败（比如传入空指针）
     */
    bool init_manual_only(ControllerPtr manual_ctrl);

    /**
     * @brief 从 ControlParams 初始化管理器（自动控制 + 手动控制）
     *
     * 典型用法：
     *  - 从 YAML 加载 ControlParams；
     *  - 传入 manual_ctrl 作为 Manual 模式的控制器；
     *  - ControllerManager 内部根据 params 构造 PID / SMC / MPC 等自动控制器。
     *
     * @param params       控制参数总结构（由外部 YAML 解析而来）
     * @param manual_ctrl  手动控制器实例（不可为空）
     * @return true 初始化成功
     * @return false 初始化失败（可通过 status().last_error 了解原因）
     */
    bool init_from_params(const ControlParams& params, ControllerPtr manual_ctrl);

    /**
     * @brief 切换当前控制模式
     *
     * 约定：
     *  - ControlMode::Manual   → 使用名为 "manual" 的控制器（如果存在）；
     *  - ControlMode::Auto     → 使用当前选定的自动控制器（如 "pid"）；
     *  - ControlMode::Failsafe → 可扩展为特殊保护控制器（暂可保持不变或关闭输出）。
     *
     * @param mode 目标控制模式
     * @return true 切换成功
     * @return false 切换失败（如目标模式无对应控制器）
     */
    bool set_mode(ControlMode mode);

    /// 获取当前控制模式
    ControlMode mode() const noexcept { return mode_; }

    /**
     * @brief 在 Auto 模式下选择具体的自动控制器（例如 "pid" / "smc" / "mpc"）
     *
     * 说明：
     *  - 仅改变“自动模式下默认控制器”的名称；
     *  - 若当前处于 Auto 模式，将立即切换 active 控制器；
     *  - 若当前处于 Manual / Failsafe，仅更新内部记录，待下次进入 Auto 时生效。
     *
     * @param name 控制器名称（例如 "pid"）
     * @return true 选择成功
     * @return false 未找到对应名称的控制器
     */
    bool select_auto_controller(const std::string& name);

    /**
     * @brief 获取当前激活的控制器名称
     */
    const std::string& active_controller_name() const noexcept {
        return active_name_;
    }

    /**
     * @brief 当前是否存在激活控制器
     */
    bool has_active_controller() const noexcept {
        return active_ != nullptr;
    }

    /**
     * @brief 统一的控制计算接口
     *
     * 说明：
     *  - 内部会根据当前 mode_ 和 active_ 控制器指针进行判断；
     *  - 若没有激活控制器，返回 false，并在 status_.last_error 中写入信息；
     *  - 若 compute() 内部报错，也会返回 false。
     *
     * @param state 当前系统状态（导航 + 传感器融合结果）
     * @param ref   控制参考量（轨迹/期望状态 或 6DOF 指令）
     * @param out   控制输出（wrench + thruster_command）
     * @param dt    控制周期（秒）
     * @return true 计算成功
     * @return false 计算失败
     */
    bool compute(const ControlState&     state,
                 const ControlReference& ref,
                 ControlOutput&          out,
                 double                  dt);

    /**
     * @brief 获取管理器当前运行状态快照
     */
    const ControllerManagerStatus& status() const noexcept {
        return status_;
    }

private:
    using ControllerMap = std::unordered_map<std::string, ControllerPtr>;

    /// 内部注册各类控制器实例（PID / SMC / MPC 等），由 init_from_params() 调用
    bool register_builtin_controllers(const ControlParams& params);

    /// 切换当前 active 控制器指针
    bool switch_active_controller(const std::string& name);

    /// 设置错误信息并标记 status_.ok = false
    void set_error(std::string msg);

    ControllerManagerOptions options_{};
    ControllerManagerStatus  status_{};

    ControlMode mode_{ControlMode::Unknown};

    ControllerMap                          controllers_;   ///< 所有已创建控制器（包含 "manual" / "pid" / "smc" / "mpc"...）
    rovctrl::controllers::IController*     active_{nullptr}; ///< 当前激活控制器的裸指针（由 controllers_ 托管）
    std::string                            active_name_;   ///< 当前激活控制器名称

    std::string default_auto_name_{"pid"}; ///< 当前自动模式下默认控制器名称（可通过 select_auto_controller 修改）
};

} // namespace rovctrl::control_core
