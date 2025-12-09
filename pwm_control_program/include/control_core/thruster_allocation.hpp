#pragma once

/**
 * @file   thruster_allocation.hpp
 * @brief  6DOF 力/力矩 → 8 推进器归一化指令 的通用分配模块
 *
 * 使用流程：
 *   1. 从 YAML 解析出 ThrusterAllocationConfig；
 *   2. ThrusterAllocator::init(config)，内部完成：
 *        - 按 P1..P8 规范顺序重排列（无论 YAML 中顺序如何）；
 *        - 选取 active_rows 对应行，形成 A_active(4x8)；
 *        - 计算伪逆 A_pinv(8x4)；
 *   3. 控制器输出 body_wrench[6]，调用 allocate() 得到 8 路归一化指令。
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

// ============================================================================
// 1. 推力模型：单个推进器「力(N) ↔ 指令(norm)」关系
// ============================================================================

/**
 * @brief 推力模型（初期线性，后续可扩展为多项式 / 查表）
 *
 * 当前约定：
 *   - 输入 thrust_N：期望推进器推力，单位 N，正负表示正/反向；
 *   - 输出 norm：归一化指令（[-1,1]），再交给 PWM 安全层。
 */
struct ThrusterThrustModel {
    double max_forward_N{100.0};   ///< 正向最大推力（N）
    double max_reverse_N{-80.0};   ///< 反向最大推力（N，通常为负）

    double norm_min{-1.0};
    double norm_max{ 1.0};

    /// thrust(N) → norm([-1,1])，线性 + 限幅
    float thrustToNorm(double thrust_N) const;

    /// 预留：norm([-1,1]) → 估计 thrust(N)
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
 *         rows: [Fx, Fy, Fz, Mx, My, Mz]
 *         data: ...
 *         active_rows: [Fx, Fy, Fz, Mz]
 *       limits:
 *         norm_min: -1.0
 *         norm_max:  1.0
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
 * 注意：实现放在 thruster_allocation.cpp 中，这里只做声明。
 */
bool load_thruster_allocation_from_yaml(const YAML::Node& vehicle_node,
                                        ThrusterAllocationConfig& cfg);

// ============================================================================
// 3. 推力分配类：统一入口
// ============================================================================

/**
 * @brief ThrusterAllocator
 *
 * 当前版本假定：
 *   - 激活 DOF 数量为 4（例如 Fx, Fy, Fz, Mz），内部构造 A_pinv(8x4)；
 *   - 更一般情况（任意激活 DOF 数）可以后续扩展为动态矩阵。
 */
class ThrusterAllocator {
public:
    ThrusterAllocator()  = default;
    ~ThrusterAllocator() = default;

    /// 使用已经解析好的配置初始化分配器
    bool init(const ThrusterAllocationConfig& cfg);

    /// 是否初始化成功
    bool ok() const { return inited_; }

    /**
     * @brief 执行推力分配：body wrench → 8 路归一化指令
     *
     * @param body_wrench  6 维力/力矩：[Fx,Fy,Fz,Mx,My,Mz]
     * @param norm_cmd_out 8 维推进器归一化指令（输出，范围大致在 [-1,1]）
     */
    bool allocate(const std::array<double, 6>& body_wrench,
                  std::array<float, 8>&        norm_cmd_out) const;

    /// 便于调试：返回当前启用的 DOF 行（Fx..Mz）
    const std::array<bool, 6>& activeDofs() const { return active_dof_; }

private:
    bool inited_{false};

    // 当前激活的 DOF 标志（Fx..Mz）
    std::array<bool, 6> active_dof_{};

    // 规范顺序下的 6×8 分配矩阵（列顺序为 P1..P8）
    Eigen::Matrix<double, 6, 8> A_body_{};

    // 伪逆矩阵（8×4），对应当前激活的 4 个 DOF
    Eigen::Matrix<double, 8, 4> A_pinv_{};

    // 记录激活 DOF 的索引，长度 <= 4，例如 [0,1,2,5]
    std::array<int, 4> active_indices_{};
    int active_dim_{0};

    ThrusterThrustModel thrust_model_{};

    // 内部辅助函数
    bool buildInternalMatrices(const ThrusterAllocationConfig& cfg);
    bool computePseudoInverse();
};

} // namespace rovctrl::control_core
