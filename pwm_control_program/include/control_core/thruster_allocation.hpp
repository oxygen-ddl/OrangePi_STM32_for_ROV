#pragma once

/**
 * @file   thruster_allocation.hpp
 * @brief  6DOF 力/力矩 → 8 推进器归一化指令 的通用分配模块
 *
 * 模块定位（和控制模式的关系）：
 *   - 本模块只做一件事：把「机体坐标系下的 6D 力/力矩 body_wrench = [Fx, Fy, Fz, Mx, My, Mz]」
 *     通过一个 6×8 分配矩阵，映射为 8 路推进器归一化命令 norm_cmd[0..7]（约在 [-1,1]）；
 *   - 它对“控制模式”（Manual / Auto / MPC / RL ...）本身是无感知的：
 *       * 在 **轨迹跟踪 / MPC / PID** 等“输出 body_wrench 的控制器”中使用本分配器；
 *       * 在 **遥控模式（ManualController）** 下，一般直接在控制器内部做 DOF→8 推进器的线性混合，
 *         不需要调用 ThrusterAllocator；
 *   - 控制模式的切换、哪种控制器被激活，由 ControllerManager 决定；
 *     本模块保持“纯几何 + 伪逆求解”的职责，方便独立单元测试与复用。
 *
 * 典型使用流程（Auto / 轨迹跟踪控制路径）：
 *   1. 从 YAML 解析出 ThrusterAllocationConfig（在 app_main / config_loader 中完成）；
 *   2. ThrusterAllocator::init(config)，内部完成：
 *        - 按 P1..P8 规范顺序重排列（无论 YAML 中顺序如何）；
 *        - 根据 active_dof 选取有效 DOF 行（例如 Fx, Fy, Fz, Mz），形成 A_active；
 *        - 计算伪逆 A_pinv（当前假定激活 DOF 数为 4，得到 8×4 矩阵）；
 *   3. 控制器输出 body_wrench[6]（单位 N / N·m），调用 allocate() 得到 8 路归一化指令 norm_cmd[8]；
 *   4. norm_cmd 再交给 PWM 安全层（pwm_client）转为具体 PWM 目标值。
 */

#include <array>
#include <cstddef>
#include <string>

#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wsign-conversion"
#endif

#include <Eigen/Dense>

#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#endif

// 只做前向声明，避免在所有 include 这个头文件的地方都引入 yaml-cpp 依赖
namespace YAML {
class Node;
}

namespace rovctrl::control_core {

// 一些便于阅读的别名
using BodyWrench6D      = std::array<double, 6>; ///< [Fx, Fy, Fz, Mx, My, Mz]
using ThrusterNormArray = std::array<float, 8>;  ///< P1..P8 对应的归一化命令

// ============================================================================
// 1. 推力模型：单个推进器「力(N) ↔ 指令(norm)」关系
// ============================================================================

/**
 * @brief 推力模型（初期线性，后续可扩展为多项式 / 查表）
 *
 * 当前约定：
 *   - 输入 thrust_N：期望推进器推力，单位 N，正负表示正/反向；
 *   - 输出 norm：归一化指令（[-1,1]），再交给 PWM 安全层；
 *
 * 在本工程中，一般的使用路径为：
 *   控制器输出 body_wrench(N/Nm) → ThrusterAllocator::allocate() 通过 A_pinv
 *   得到每个推进器的推力需求，再统一通过 ThrusterThrustModel 转为 norm。
 *
 * 注意：
 *   - 目前所有推进器共用一组推力模型参数（后续可扩展为 per-thruster 参数）；
 *   - norm_min/norm_max 是「thrust 映射后的归一化范围」，并不是 PWM 的最终范围。
 */
struct ThrusterThrustModel {
    double max_forward_N{100.0};   ///< 正向最大推力（N）
    double max_reverse_N{-80.0};   ///< 反向最大推力（N，通常为负）

    double norm_min{-1.0};
    double norm_max{ 1.0};

    /// thrust(N) → norm([-1,1])，线性 + 限幅
    float thrustToNorm(double thrust_N) const;

    /// 预留：norm([-1,1]) → 估计 thrust(N)，用于诊断或反算
    double normToThrust(float norm_cmd) const;
};

// ============================================================================
// 2. 推力分配配置：从 YAML 映射过来的结构体
// ============================================================================

/**
 * @brief YAML 中 vehicle.thrusters 部分映射到的配置
 *
 * YAML 示例（与 config/control_params.yaml 对齐）：
 *
 *   vehicle:
 *     thrusters:
 *       order: [P5, P6, P7, P8, P1, P2, P3, P4]
 *       allocation_matrix:
 *         rows: [Fx, Fy, Fz, Mx, My, Mz]   # 行含义固定
 *         data: ...                        # 6×8 矩阵，按行展开
 *         active_rows: [Fx, Fy, Fz, Mz]    # 参与伪逆求解的 DOF
 *       limits:
 *         norm_min: -1.0
 *         norm_max:  1.0
 *
 * 说明：
 *   - thruster_order_yaml 用于把 YAML 中随意顺序的推进器，重排到工程内部规范顺序 P1..P8；
 *   - allocation_matrix_yaml 始终保持 [Fx,Fy,Fz,Mx,My,Mz]×[P?] 的语义；
 *   - active_dof 决定哪些 DOF 参与伪逆求解，典型为 {Fx,Fy,Fz,Mz}；
 *   - thrust_model 决定“力(N)”与“归一化指令”之间的线性映射。
 */
struct ThrusterAllocationConfig {
    // YAML 中的推进器顺序，例如：["P5","P6","P7","P8","P1","P2","P3","P4"]
    std::array<std::string, 8> thruster_order_yaml{};

    // 6x8 分配矩阵，按 YAML 的行顺序 & 列顺序存放
    // 行顺序固定为 [Fx, Fy, Fz, Mx, My, Mz]，data[row][col]
    std::array<std::array<double, 8>, 6> allocation_matrix_yaml{};

    // 每个 DOF 是否参与分配，对应 [Fx, Fy, Fz, Mx, My, Mz]
    std::array<bool, 6> active_dof{};  ///< 来自 YAML allocation_matrix.active_rows

    // 归一化指令限幅（在 thrustToNorm 之前的全局限幅）
    double norm_min{-1.0};
    double norm_max{ 1.0};

    // 推力模型参数（所有推进器共享一组，后续可以扩成 per-motor）
    ThrusterThrustModel thrust_model{};
};

// ============================================================================
// 2.1 从 YAML 节点加载 ThrusterAllocationConfig（由 app_main 调用）
// ============================================================================

/**
 * @brief 从 YAML 的 vehicle 节点加载 ThrusterAllocationConfig
 *
 * @param vehicle_node  对应 YAML 中的 root["vehicle"]
 * @param cfg           输出配置
 * @return true  加载成功
 * @return false 加载失败（字段缺失或解析异常）
 *
 * 注意：
 *   - 实现放在 thruster_allocation.cpp 中，这里只做声明；
 *   - 只负责“解析配置”，不负责构建伪逆矩阵，后者由 ThrusterAllocator::init 完成。
 */
bool load_thruster_allocation_from_yaml(const YAML::Node& vehicle_node,
                                        ThrusterAllocationConfig& cfg);

// ============================================================================
// 3. 推力分配类：统一入口（对控制模式无感知）
// ============================================================================

/**
 * @brief ThrusterAllocator
 *
 * 职责：
 *   - 维护一个 6×8 的“规范顺序”分配矩阵 A_body（列为 P1..P8）；
 *   - 根据 active_dof 构造一个有效 DOF 子矩阵 A_active，并计算其伪逆 A_pinv；
 *   - 在运行时接受 body_wrench[6] 作为输入，输出 8 路归一化指令 norm_cmd[8]。
 *
 * 和控制模式的关系：
 *   - 本类不关心当前是 Manual / Auto / MPC，仅仅根据传入的 body_wrench 求解；
 *   - 在 **Auto / MPC / PID** 控制器中，一般是：
 *        controller → body_wrench → ThrusterAllocator → norm_cmd → PWM；
 *   - 在 **Manual（遥控）模式** 下，通常由 ManualController 直接给出 thruster_command[8]，
 *     ControlLoop::build_thruster_command_ 应优先使用控制器直接输出，
 *     没有必要再走一次 ThrusterAllocator。
 */
class ThrusterAllocator {
public:
    ThrusterAllocator()  = default;
    ~ThrusterAllocator() = default;

    /// 使用已经解析好的配置初始化分配器（构造 A_body / A_pinv 等）
    bool init(const ThrusterAllocationConfig& cfg);

    /// 是否初始化成功（配置 & 矩阵均已就绪）
    bool ok() const { return inited_; }

    /**
     * @brief 执行推力分配：body wrench → 8 路归一化指令
     *
     * @param body_wrench  6 维力/力矩：[Fx,Fy,Fz,Mx,My,Mz]（单位 N / N·m）
     * @param norm_cmd_out 8 维推进器归一化指令（输出，范围大致在 [-1,1]）
     *
     * 返回值：
     *   - true  分配成功，norm_cmd_out 填充有效数据；
     *   - false 分配失败（例如矩阵未初始化 / 伪逆维度不合法），调用方应视为
     *           “本轮无法下发有效推力”，可以选择归中或进入 failsafe。
     */
    bool allocate(const BodyWrench6D& body_wrench,
                  ThrusterNormArray&  norm_cmd_out) const;

    /// 便于调试：返回当前启用的 DOF 行（Fx..Mz）
    const std::array<bool, 6>& activeDofs() const { return active_dof_; }

private:
    bool inited_{false};

    // 当前激活的 DOF 标志（Fx..Mz），和 ThrusterAllocationConfig::active_dof 对应
    std::array<bool, 6> active_dof_{};

    // 规范顺序下的 6×8 分配矩阵（列顺序为 P1..P8）
    Eigen::Matrix<double, 6, 8> A_body_{};

    // 伪逆矩阵（8×4），对应当前激活的 4 个 DOF
    //
    // 当前实现假定“激活 DOF 数为 4”，典型为 Fx/Fy/Fz/Mz；
    // 如果未来需要支持任意激活 DOF 数，可扩展为动态矩阵。
    Eigen::Matrix<double, 8, 4> A_pinv_{};

    // 记录激活 DOF 的索引，长度 <= 4，例如 [0,1,2,5]
    std::array<int, 4> active_indices_{};
    int active_dim_{0};

    ThrusterThrustModel thrust_model_{};

    // 内部辅助函数：根据 YAML 配置构造 A_body / active_dof_ / active_indices_
    bool buildInternalMatrices(const ThrusterAllocationConfig& cfg);

    // 内部辅助函数：根据当前 active_indices_ 计算 A_pinv_
    bool computePseudoInverse();
};

} // namespace rovctrl::control_core
