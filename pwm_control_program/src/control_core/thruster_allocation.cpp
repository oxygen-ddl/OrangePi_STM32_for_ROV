#include "control_core/thruster_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <Eigen/SVD>
#include <yaml-cpp/yaml.h>

namespace rovctrl::control_core {

// ============================================================================
// ThrusterThrustModel 实现
// ============================================================================

float ThrusterThrustModel::thrustToNorm(double thrust_N) const
{
    double norm = 0.0;

    if (thrust_N >= 0.0) {
        if (max_forward_N > 1e-6) {
            norm = thrust_N / max_forward_N;
        } else {
            norm = 0.0;
        }
    } else {
        // thrust_N < 0
        const double max_rev = std::abs(max_reverse_N);
        if (max_rev > 1e-6) {
            norm = thrust_N / max_rev;  // thrust_N 为负，norm 也为负
        } else {
            norm = 0.0;
        }
    }

    // 限幅到配置范围内
    norm = std::clamp(norm, norm_min, norm_max);
    return static_cast<float>(norm);
}

double ThrusterThrustModel::normToThrust(float norm_cmd) const
{
    const double n = static_cast<double>(norm_cmd);

    if (n >= 0.0) {
        const double n_clamped = std::clamp(n, 0.0, norm_max);
        return n_clamped * max_forward_N;
    } else {
        const double n_clamped = std::clamp(n, norm_min, 0.0);
        return n_clamped * std::abs(max_reverse_N);  // 结果为负
    }
}

// ============================================================================
// 内部辅助函数：YAML 标签与索引映射
// ============================================================================

namespace {

// "P1".."P8" → 0..7
int thrusterLabelToIndex(const std::string& label)
{
    if (label.size() < 2) {
        return -1;
    }
    if (label[0] != 'P' && label[0] != 'p') {
        return -1;
    }

    int id = 0;
    try {
        id = std::stoi(label.substr(1));
    } catch (...) {
        return -1;
    }

    if (id < 1 || id > 8) {
        return -1;
    }
    return id - 1;  // P1 -> 0, ..., P8 -> 7
}

// "Fx","Fy","Fz","Mx","My","Mz" → 0..5
int dofNameToIndex(const std::string& name)
{
    if (name == "Fx") return 0;
    if (name == "Fy") return 1;
    if (name == "Fz") return 2;
    if (name == "Mx") return 3;
    if (name == "My") return 4;
    if (name == "Mz") return 5;
    return -1;
}

} // anonymous namespace

// ============================================================================
//  ThrusterAllocator 初始化：buildInternalMatrices + computePseudoInverse
// ============================================================================

bool ThrusterAllocator::init(const ThrusterAllocationConfig& cfg)
{
    inited_       = false;
    active_dim_   = 0;
    active_dof_   = cfg.active_dof;
    thrust_model_ = cfg.thrust_model;

    // 同步 norm 限幅到 thrust_model，保证一致
    thrust_model_.norm_min = cfg.norm_min;
    thrust_model_.norm_max = cfg.norm_max;

    // 步骤 1：构建规范顺序的 A_body_
    if (!buildInternalMatrices(cfg)) {
        std::cerr << "[ThrusterAllocator] buildInternalMatrices() failed.\n";
        return false;
    }

    // 步骤 2：根据激活 DOF 计算伪逆矩阵
    if (!computePseudoInverse()) {
        std::cerr << "[ThrusterAllocator] computePseudoInverse() failed.\n";
        return false;
    }

    inited_ = true;
    return true;
}

bool ThrusterAllocator::buildInternalMatrices(const ThrusterAllocationConfig& cfg)
{
    // 1) 清零矩阵
    A_body_.setZero();
    active_indices_.fill(-1);
    active_dim_ = 0;

    // 2) 列重排：将 YAML 顺序的列，映射到 P1..P8 规范顺序
    std::array<bool, 8> used{};
    used.fill(false);

    for (int col_yaml = 0; col_yaml < 8; ++col_yaml) {
        const auto col_idx = static_cast<std::size_t>(col_yaml);

        const std::string& label = cfg.thruster_order_yaml[col_idx];
        const int          canon_id = thrusterLabelToIndex(label);

        if (canon_id < 0 || canon_id >= 8) {
            std::cerr << "[ThrusterAllocator] invalid thruster label: "
                      << label << " at col " << col_yaml << "\n";
            return false;
        }

        const auto canon_idx = static_cast<std::size_t>(canon_id);

        if (used[canon_idx]) {
            std::cerr << "[ThrusterAllocator] duplicate thruster label: "
                      << label << "\n";
            return false;
        }
        used[canon_idx] = true;

        // 将该列拷贝到规范列 canon_id
        for (int row = 0; row < 6; ++row) {
            const auto row_idx = static_cast<std::size_t>(row);
            A_body_(row, canon_id) =
                cfg.allocation_matrix_yaml[row_idx][col_idx];
        }
    }

    // 3) 统计激活的 DOF（Fx..Mz）
    int count_active = 0;
    for (int i = 0; i < 6; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        if (active_dof_[idx]) {
            if (count_active < 4) {
                const auto act_idx = static_cast<std::size_t>(count_active);
                active_indices_[act_idx] = i;
            }
            ++count_active;
        }
    }

    if (count_active == 0) {
        std::cerr << "[ThrusterAllocator] no active DOF specified.\n";
        return false;
    }
    if (count_active > 4) {
        std::cerr << "[ThrusterAllocator] currently supports up to 4 active DOFs, "
                  << "but got " << count_active << ".\n";
        return false;
    }

    active_dim_ = count_active;
    return true;
}


bool ThrusterAllocator::computePseudoInverse()
{
    if (active_dim_ <= 0 || active_dim_ > 4) {
        std::cerr << "[ThrusterAllocator] invalid active_dim_ = "
                  << active_dim_ << "\n";
        return false;
    }

    // 1) 构造 A_active (m x 8)，m = active_dim_
    const int m = active_dim_;
    const int n = 8;

    Eigen::MatrixXd A_active(m, n);
    for (int k = 0; k < m; ++k) {
        const auto k_idx  = static_cast<std::size_t>(k);
        const int  dof_idx = active_indices_[k_idx];  // 0..5 → Fx..Mz
        if (dof_idx < 0 || dof_idx >= 6) {
            std::cerr << "[ThrusterAllocator] invalid active_indices_["
                      << k << "] = " << dof_idx << "\n";
            return false;
        }
        A_active.row(k) = A_body_.row(dof_idx);
    }

    // 2) 使用 SVD 计算 Moore-Penrose 伪逆：A_pinv = V * S_pinv * U^T
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        A_active,
        Eigen::ComputeThinU | Eigen::ComputeThinV
    );

    const auto& S = svd.singularValues();
    const auto& U = svd.matrixU();
    const auto& V = svd.matrixV();

    const int r = static_cast<int>(S.size());
    if (r == 0) {
        std::cerr << "[ThrusterAllocator] SVD returned zero singular values.\n";
        return false;
    }

    Eigen::MatrixXd S_pinv = Eigen::MatrixXd::Zero(r, r);
    const double tol = 1e-8;

    for (int i = 0; i < r; ++i) {
        if (S(i) > tol) {
            S_pinv(i, i) = 1.0 / S(i);
        }
    }

    // 伪逆矩阵 X = V * S_pinv * U^T，维度：n x m = 8 x active_dim_
    Eigen::MatrixXd X = V * S_pinv * U.transpose();

    // 3) 将 X 拷贝到固定大小的 A_pinv_ (8x4)，只用前 active_dim_ 列
    A_pinv_.setZero();
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            A_pinv_(i, j) = X(i, j);
        }
    }

    return true;
}

// ============================================================================
// 推力分配：body_wrench → 8 路归一化指令
// ============================================================================

bool ThrusterAllocator::allocate(const std::array<double, 6>& body_wrench,
                                 std::array<float, 8>&        norm_cmd_out) const
{
    if (!inited_) {
        std::cerr << "[ThrusterAllocator] allocate() called before init().\n";
        return false;
    }

    // 1) 提取激活 DOF 的目标 wrench，形成 w_active (active_dim_ x 1)
    Eigen::Matrix<double, 4, 1> w_act;
    w_act.setZero();

    for (int k = 0; k < active_dim_; ++k) {
    const auto k_idx  = static_cast<std::size_t>(k);
    const int  dof_idx = active_indices_[k_idx];  // 0..5

    if (dof_idx < 0 || dof_idx >= 6) {
        continue;
    }
    const auto dof_idx_sz = static_cast<std::size_t>(dof_idx);
    w_act(k, 0) = body_wrench[dof_idx_sz];
    }


    // 2) 线性最小二乘：t = A_pinv_ * w_act，得到每个推进器的推力估计（N）
    Eigen::Matrix<double, 8, 1> t = A_pinv_ * w_act;

    // 3) 推力 → norm 指令，并限幅
    for (int i = 0; i < 8; ++i) {
       const double thrust_i = t(i, 0);
       const auto   idx      = static_cast<std::size_t>(i);
       norm_cmd_out[idx] = thrust_model_.thrustToNorm(thrust_i);
    }

    return true;
}

// ============================================================================
// YAML 解析：vehicle 节点 → ThrusterAllocationConfig
// ============================================================================

bool load_thruster_allocation_from_yaml(const YAML::Node& vehicle_node,
                                        ThrusterAllocationConfig& cfg)
{
    if (!vehicle_node || !vehicle_node["thrusters"]) {
        std::cerr << "[ThrusterAllocator] YAML: missing 'vehicle.thrusters' node.\n";
        return false;
    }

    YAML::Node thrusters = vehicle_node["thrusters"];

    // 1) order: [P5, P6, P7, P8, P1, P2, P3, P4]
    if (thrusters["order"] && thrusters["order"].IsSequence()) {
        auto order_node = thrusters["order"];
        const std::size_t n = std::min<std::size_t>(order_node.size(), 8);
        for (std::size_t i = 0; i < n; ++i) {
            cfg.thruster_order_yaml[i] = order_node[i].as<std::string>();
        }
        // 若数量不足 8 个，剩余填充默认 P1..P8
        for (std::size_t i = n; i < 8; ++i) {
            cfg.thruster_order_yaml[i] = "P" + std::to_string(i + 1);
        }
    } else {
        // 如果缺省，则用规范顺序 P1..P8
        for (int i = 0; i < 8; ++i) {
           const auto idx = static_cast<std::size_t>(i);
           cfg.thruster_order_yaml[idx] = "P" + std::to_string(i + 1);
        }
    }

    // 2) allocation_matrix: rows + data + active_rows
    if (!thrusters["allocation_matrix"]) {
        std::cerr << "[ThrusterAllocator] YAML: missing 'thrusters.allocation_matrix'.\n";
        return false;
    }

    YAML::Node alloc = thrusters["allocation_matrix"];

    // 2.1 data: 行是 DOF（按 [Fx,Fy,Fz,Mx,My,Mz] 或 rows 顺序）
    cfg.allocation_matrix_yaml = {}; // 全部清零

    // 允许有 rows 字段，但当前实现仍假设顺序为 Fx..Mz
    YAML::Node data = alloc["data"];
    if (!data || !data.IsSequence()) {
        std::cerr << "[ThrusterAllocator] YAML: allocation_matrix.data is missing or not sequence.\n";
        return false;
    }

    const std::size_t rows_n = std::min<std::size_t>(data.size(), 6);
    for (std::size_t r = 0; r < rows_n; ++r) {
        YAML::Node row = data[r];
        if (!row.IsSequence()) {
            std::cerr << "[ThrusterAllocator] YAML: allocation_matrix.data row "
                      << r << " is not a sequence.\n";
            return false;
        }
        const std::size_t cols_n = std::min<std::size_t>(row.size(), 8);
        for (std::size_t c = 0; c < cols_n; ++c) {
            cfg.allocation_matrix_yaml[r][c] = row[c].as<double>();
        }
    }

    // 2.2 active_rows: [Fx, Fy, Fz, Mz]
    cfg.active_dof.fill(false);
     if (alloc["active_rows"] && alloc["active_rows"].IsSequence()) {
     YAML::Node ar = alloc["active_rows"];
     for (std::size_t i = 0; i < ar.size(); ++i) {
        const std::string name = ar[i].as<std::string>();
        const int idx = dofNameToIndex(name);
        if (idx >= 0 && idx < 6) {
            const auto idx_sz = static_cast<std::size_t>(idx);
            cfg.active_dof[idx_sz] = true;
        } else {
            std::cerr << "[ThrusterAllocator] YAML: unknown active_rows entry '"
                      << name << "'.\n";
            }
        }
      } else {
       // 若未指定，默认只开启 Fx,Fy,Fz,Mz
       cfg.active_dof[0] = true; // Fx
       cfg.active_dof[1] = true; // Fy
       cfg.active_dof[2] = true; // Fz
       cfg.active_dof[5] = true; // Mz
    }

    // 3) limits: norm_min / norm_max
    if (thrusters["limits"]) {
        YAML::Node limits = thrusters["limits"];
        if (limits["norm_min"]) {
            cfg.norm_min = limits["norm_min"].as<double>();
        }
        if (limits["norm_max"]) {
            cfg.norm_max = limits["norm_max"].as<double>();
        }
        // 其它如 wrench_limits / norm_slew_rate 在此模块暂不使用
    }

    // 4) thrust_model（可选）
    if (thrusters["thrust_model"]) {
        YAML::Node tm = thrusters["thrust_model"];
        if (tm["max_forward_N"]) {
            cfg.thrust_model.max_forward_N = tm["max_forward_N"].as<double>();
        }
        if (tm["max_reverse_N"]) {
            cfg.thrust_model.max_reverse_N = tm["max_reverse_N"].as<double>();
        }
        if (tm["norm_min"]) {
            cfg.thrust_model.norm_min = tm["norm_min"].as<double>();
        }
        if (tm["norm_max"]) {
            cfg.thrust_model.norm_max = tm["norm_max"].as<double>();
        }
    }

    return true;
}

} // namespace rovctrl::control_core
