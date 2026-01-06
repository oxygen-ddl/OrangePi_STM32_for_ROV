#pragma once
#ifndef ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
#define ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP

#include <cmath>
#include <cstdint>
#include <string>

#include "controllers/controller_base.hpp"      // IController
#include "control_core/control_types.hpp"       // ControlState/Reference/Output/DofCommand
#include "control_core/control_mode.hpp"        // ControlMode

namespace rovctrl::controllers {

/**
 * @brief 手动模式控制器（ManualController）
 *
 * 设计定位：
 *  - 面向“操作员遥控”的简单控制器；
 *  - 上层（Teleop/GCS）已经将键盘/手柄解析为 6-DOF 归一化指令 DofCommand ∈ [-1,1]^6；
 *  - 本控制器仅做：
 *      1）对 DOF 乘增益（surge/sway/heave/roll/pitch/yaw）；
 *      2）按固定推进器拓扑，将 6 个 DOF 线性组合为 8 路推进器命令；
 *      3）统一限幅到 [-max_cmd_abs, +max_cmd_abs]；
 *  - 不做闭环控制、不依赖导航反馈，只是“遥控 → 推进器输出”的线性映射。
 *
 * 坐标与 DOF 约定（与 DofCommand 对齐）：
 *  - surge : 机体系 X 轴前后   (+ 前进,     - 后退)
 *  - sway  : 机体系 Y 轴左右   (+ 右移,     - 左移)
 *  - heave : 机体系 Z 轴上下   (+ 上浮,     - 下潜)
 *  - roll  : 绕 X 轴横滚      (+ 右侧下沉, - 左侧下沉)
 *  - pitch : 绕 Y 轴俯仰      (+ 头上尾下, - 头下尾上)
 *  - yaw   : 绕 Z 轴偏航      (+ 左转,     - 右转)
 *
 * 推进器拓扑约定（与当前 alloc.yaml / 机械布置保持一致）：
 *
 *  水平推进器（逻辑索引 0–3），俯视图（机体指向 ↑ 为前方）：
 *
 *        前方
 *         ↑
 *     [2]   [3]
 *
 *     [0]   [1]
 *         ↓ 后方
 *
 *  垂向推进器（逻辑索引 4–7），俯视图：
 *
 *        前方
 *         ↑
 *     [4]   [5]
 *
 *     [6]   [7]
 *         ↓ 后方
 *
 *  当前线性组合策略（在 ManualController::compute 中实现）：
 *
 *  1）水平 0–3：surge / sway / yaw
 *      - surge: 4 个水平推进器同向，加法：
 *          u[0] += surge; u[1] += surge; u[2] += surge; u[3] += surge;
 *
 *      - sway: 左右差动（右移为例）：
 *          u[0] += +sway;  // 右移时，右后
 *          u[1] += -sway;
 *          u[2] += -sway;
 *          u[3] += +sway;
 *
 *      - yaw: 产生绕 Z 轴偏航力矩：
 *          u[0] += -yaw;
 *          u[1] += +yaw;
 *          u[2] += -yaw;
 *          u[3] += +yaw;
 *
 *  2）垂向 4–7：heave / roll / pitch
 *      - heave：整体上下 → 所有垂向推进器同向分量；
 *      - roll ：左右差动；
 *      - pitch：前后差动；
 *
 *      组合模板：
 *          u[4] = heave - roll - pitch;   // 前左 FL
 *          u[5] = heave + roll - pitch;   // 前右 FR
 *          u[6] = heave - roll + pitch;   // 后左 RL
 *          u[7] = heave + roll + pitch;   // 后右 RR
 *
 *      这样：
 *        - 纯 heave 时：4 个垂向推进器同号 → 纯升沉；
 *        - 纯 roll  时：左右异号 → 产生绕 X 轴力矩；
 *        - 纯 pitch 时：前后异号 → 产生绕 Y 轴力矩。
 *
 * 使用方式（控制侧）：
 *  - ControlLoop 已经从 Guard 得到 ControlReference ref：
 *      - ref.dof_cmd    填好 6-DOF ∈ [-1,1]；
 *      - ref.use_dof_cmd = true（手动模式下）；
 *  - ManualController::compute(state, ref, out, dt)：
 *      - 忽略 state（不依赖导航）；
 *      - 基于 ref.dof_cmd 计算 out.thruster_command[8]；
 *      - out.has_thruster_command = true；
 *      - 交给 ThrusterAllocator / PwmClient 下发。
 *
 * 注意：
 *  - 当前实现只输出 thruster_command，不构造 body_wrench；
 *  - 若将来希望在 ManualController 输出 body_wrench 由通用分配器处理，
 *    可以扩展 output_body_wrench / build_from_dof 等逻辑。
 */

struct ManualControllerConfig {
    // ==== 1. DOF 指令缩放（单位：无量纲增益） ====

    /// surge DOF 增益，1.0 表示 DofCommand.surge 直接作为归一化 thrust 输入。
    double surge_gain  = 1.0;

    /// sway DOF 增益。
    double sway_gain   = 1.0;

    /// heave DOF 增益。
    double heave_gain  = 1.0;

    /// roll DOF 增益。
    double roll_gain   = 1.0;

    /// pitch DOF 增益。
    double pitch_gain  = 1.0;

    /// yaw DOF 增益。
    double yaw_gain    = 1.0;

    // ==== 2. 输出限幅 ====

    /**
     * @brief 推进器归一化输出绝对值上限。
     *
     * - ManualController 在 compute() 末尾会调用 clamp_output()；
     * - 所有 thruster_command[i] ∈ [-max_cmd_abs, +max_cmd_abs]；
     * - 通常设为 1.0；如需“半油门上限”可以设为 0.5。
     */
    double max_cmd_abs = 1.0;

    // ==== 3. 输出层级选择（当前实现保留扩展用） ====

    /**
     * @brief 是否输出 body_wrench 而不是 thruster_command。
     *
     * 当前实现说明：
     *  - ManualController 现阶段始终直接写 thruster_command[8]（水平 4 + 垂向 4）；
     *  - output_body_wrench 字段暂未在 compute() 中起作用，仅作为未来扩展预留；
     *  - 若后续引入通用 ThrusterAllocator，并希望统一由其做 6xN 分配，
     *    可以在 compute() 中根据该标志选择：
     *      - output_body_wrench=true  → 填 ref/输出 body_wrench；
     *      - output_body_wrench=false → 直接填 thruster_command。
     */
    bool output_body_wrench = true;

    // ==== 4. 异常输入策略（预留） ====

    /**
     * @brief ref 中缺少有效 dof_cmd 时的策略。
     *
     * 建议行为（当前实现可以按项目需求扩展）：
     *  - true  ：视为错误，compute() 返回 false，交由 ControlLoop/Guard 进入 failsafe；
     *  - false ：视为“无遥控输入”，输出全零（thruster_command=0），返回 true，
     *            更接近“松开手柄即归中”的语义。
     *
     * 当前版本的 ManualController 代码只假定 ref.dof_cmd 存在，
     * 上游 Guard / ControlLoop 已经保证在手动模式下填好 DOF。
     */
    bool missing_input_is_error = false;
};

/**
 * @brief 手动模式控制器实现。
 *
 * 生命周期：
 *  - 配置阶段：构造 ManualController(config)；
 *  - 控制循环：每个周期调用 compute(state, ref, out, dt)；
 *  - 若重置模式，可调用 reset()（当前为空实现，保留扩展）。
 */
class ManualController final : public IController {
public:
    explicit ManualController(const ManualControllerConfig& cfg = ManualControllerConfig());
    ~ManualController() override = default;

    /// 控制器名称，用于日志 / ControllerManager 选择。
    const char* name() const noexcept override { return "manual"; }

    /// 所属控制模式：kManual。
    rovctrl::control_core::ControlMode mode() const noexcept override {
        return rovctrl::control_core::ControlMode::kManual;
    }

    /// 状态重置（当前无内部状态，留作未来扩展）。
    void reset() noexcept override {}

    /**
     * @brief 计算手动模式输出。
     *
     * @param state  当前控制状态（Manual 模式下当前实现不使用，仅为接口对齐）。
     * @param ref    控制参考，要求：
     *                  - ref.dof_cmd 已填充（6-DOF ∈ [-1,1]）；
     *                  - 通常 ref.use_dof_cmd = true；
     * @param out    输出结构，将被填充 thruster_command[8] 及标志位。
     * @param dt     控制周期（秒），当前 ManualController 不依赖 dt。
     *
     * @return true  表示计算成功；false 表示配置/输入异常（例如推进器数量不足）。
     *
     * 实际步骤：
     *  1. 从 ref.dof_cmd 中读出 surge/sway/heave/roll/pitch/yaw；
     *  2. 乘对应增益（ManualControllerConfig 中的 *_gain）；
     *  3. 按固定拓扑将 6 DOF 线性组合为 8 路 thruster_command；
     *  4. 调用 clamp_output() 将所有通道裁剪到 [-max_cmd_abs, +max_cmd_abs]；
     *  5. 设置 out.has_thruster_command = true。
     */
    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt) noexcept override;

    const ManualControllerConfig& config() const noexcept { return cfg_; }

private:
    ManualControllerConfig cfg_{};

    // ==== 小工具函数（当前 cpp 实现可能只用 clamp_output，一个个逐步启用） ====

    static double clamp(double v, double lo, double hi) noexcept {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    static bool is_finite(double v) noexcept {
        return std::isfinite(v);
    }

    /// 输出清零：thruster_command / body_wrench 等全部置 0。
    void zero_output(rovctrl::control_core::ControlOutput& out) const noexcept;

    /**
     * @brief 从 6-DOF 指令构建输出。
     *
     * 预期行为（与上面的大文档一致）：
     *  - 根据 cfg_.output_body_wrench 决定是构建 body_wrench 还是 thruster_command；
     *  - 对非法/非有限值做防御；
     *  - 保证输出维度正确。
     *
     * 当前实际项目中，可只实现“直接写 thruster_command”的路径；
     * 若后续希望复用到其他拓扑，可在此处集中修改 DOF→电机的线性组合。
     */
    bool build_from_dof(const rovctrl::control_core::DofCommand& dof,
                        rovctrl::control_core::ControlOutput& out) const noexcept;

    /// 对 thruster_command 做统一限幅。
    void clamp_output(rovctrl::control_core::ControlOutput& out) const noexcept;
};

} // namespace rovctrl::controllers

#endif // ROVCTRL_CONTROLLERS_MANUAL_CONTROLLER_HPP
