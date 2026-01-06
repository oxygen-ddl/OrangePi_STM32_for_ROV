// pwm_control_program/include/utils/config_loader.hpp
#pragma once

#include <array>        // <<< NEW: teleop mixer 需要 std::array
#include <filesystem>
#include <initializer_list>
#include <iosfwd>
#include <string>

#include "platform/pwm_client.hpp"

namespace rovctrl {

namespace control_core {
    // 在 control_core/thruster_allocation.hpp 中定义
    struct ThrusterAllocationConfig;

    // 预留：轨迹配置结构体，暂时只做前向声明
    // 你可以在将来的 trajectory_config.hpp 中给出完整定义
    struct TrajectoryConfig;

    /**
     * @brief Teleop 混合器配置：6DOF → 8 推进器
     *
     * 典型使用场景：
     *   - 键盘 / GCS 遥控直接输出 6DOF “归一化 wrench” 指令；
     *   - 本结构体给出将 [surge, sway, heave, roll, pitch, yaw]
     *     混合到 8 路逻辑推进器指令的线性关系。
     *
     * 设计约定：
     *   - DOF 列顺序固定为： [surge, sway, heave, roll, pitch, yaw]
     *   - matrix[thruster][dof]，即第 i 行是第 i 个推进器对 6DOF 的响应系数；
     *   - output_limit_abs 是对混合结果的统一限幅（归一化指令范围）；
     *   - gains 是对输入 DOF 的一阶缩放，便于现场调敏感度；
     *   - input_deadzone 用于抑制小抖动（例如键盘重复、漂移等）。
     */
    struct TeleopMixerConfig;

} // namespace control_core

} // namespace rovctrl

namespace rovctrl::utils {

namespace fs = std::filesystem;

/**
 * 通用配置路径解析工具：
 *
 * 解析优先级：
 *   1) 命令行传入的 cli_opt（如果非空）
 *   2) 环境变量 env_name（如果非空）
 *   3) 依次尝试相对可执行文件目录的若干候选路径：
 *        exe_dir / rel_candidates[0]
 *        exe_dir / rel_candidates[1]
 *        ...
 *
 * 任意一步找到存在的文件，则：
 *   - out_path 由 canonical() 处理为绝对路径
 *   - 在 log 中输出来源信息
 *
 * 返回值：
 *   - true  : 成功找到配置文件
 *   - false : 所有途径均失败
 */
bool resolve_config_path(const std::string& cli_opt,
                         const char*        argv0,
                         const char*        env_name,
                         std::initializer_list<std::string> rel_candidates,
                         fs::path&          out_path,
                         std::ostream&      log);

/**
 * 专门针对 pwm_client.yaml 的路径解析：
 *
 * 约定：
 *   - 命令行参数：--pwm-config <path>  → 传入 cli_opt
 *   - 环境变量： PWM_CLIENT_CONFIG
 *   - 相对 exe_dir 的候选路径：
 *        1) exe_dir / "config/pwm_client.yaml"
 *        2) exe_dir / "../../pwm_control_program/config/pwm_client.yaml"
 *
 * 注意：这里不负责真正加载 YAML，只负责“定位文件路径”。
 */
bool resolve_pwm_client_config_path(const std::string& cli_opt,
                                    const char*        argv0,
                                    fs::path&          out_path,
                                    std::ostream&      log);

/**
 * 专门针对 control_params.yaml 的路径解析：
 *
 * 约定：
 *   - 命令行参数：--control-config <path>  → 传入 cli_opt
 *   - 环境变量： ROV_CONTROL_CONFIG
 *   - 相对 exe_dir 的候选路径：
 *        1) exe_dir / "config/control_params.yaml"
 *        2) exe_dir / "../../pwm_control_program/config/control_params.yaml"
 */
bool resolve_control_config_path(const std::string& cli_opt,
                                 const char*        argv0,
                                 fs::path&          out_path,
                                 std::ostream&      log);

/**
 * 专门针对 trajectory.yaml 的路径解析：
 *
 * 约定：
 *   - 命令行参数：--traj-config <path>  → 传入 cli_opt
 *   - 环境变量： ROV_TRAJECTORY_CONFIG
 *   - 相对 exe_dir 的候选路径：
 *        1) exe_dir / "config/trajectory.yaml"
 *        2) exe_dir / "../../pwm_control_program/config/trajectory.yaml"
 *
 * 当前即便你还没实现轨迹逻辑，也可以先占位，方便后续接入。
 */
bool resolve_trajectory_config_path(const std::string& cli_opt,
                                    const char*        argv0,
                                    fs::path&          out_path,
                                    std::ostream&      log);

/**
 * 专门针对 alloc.yaml（推力分配配置）的路径解析：
 *
 * 约定：
 *   - 命令行参数：--alloc-config <path>  → 传入 cli_opt
 *   - 环境变量： ROV_ALLOC_CONFIG
 *   - 相对 exe_dir 的候选路径：
 *        1) exe_dir / "config/alloc.yaml"
 *        2) exe_dir / "../../pwm_control_program/config/alloc.yaml"
 *
 * 注意：这里不负责真正加载 YAML，只负责“定位文件路径”。
 */
bool resolve_alloc_config_path(const std::string& cli_opt,
                               const char*        argv0,
                               fs::path&          out_path,
                               std::ostream&      log);

/**
 * 专门针对 teleop_mixer.yaml（键盘 6DOF → 8 推进器混合矩阵）的路径解析。
 *
 * 约定：
 *   - 命令行参数：--teleop-mixer-config <path>  → 传入 cli_opt
 *   - 环境变量： ROV_TELEOP_MIXER_CONFIG
 *   - 相对 exe_dir 的候选路径：
 *        1) exe_dir / "config/teleop_mixer.yaml"
 *        2) exe_dir / "../../pwm_control_program/config/teleop_mixer.yaml"
 *
 * 注意：这里只负责“定位文件路径”，不解析 YAML。
 */
bool resolve_teleop_mixer_config_path(const std::string& cli_opt,   // <<< NEW
                                      const char*        argv0,
                                      fs::path&          out_path,
                                      std::ostream&      log);

/**
 * 从 pwm_client.yaml 加载底层通信 / 安全层配置。
 *
 * 参数：
 *   - path : 已经解析好的配置文件路径（建议用上面 resolve_* 得到）
 *   - cfg  : 输出结构体，由该函数填充；结构体中已有的默认值会作为缺省回退
 *   - log  : 日志输出流（一般传 std::cerr）
 *
 * 返回值：
 *   - true  : 加载成功
 *   - false : 加载失败（结构体内容保持调用前或部分更新，调用方应决定是否继续）
 */
bool load_pwm_client_config(const fs::path&                 path,
                            rovctrl::platform::PwmClientConfig& cfg,
                            std::ostream&                   log);

/**
 * 从 alloc.yaml 加载推力分配（Thruster Allocation）配置。
 *
 * YAML 顶层结构建议如下（简化示意）：
 *
 *   version: 1
 *   thrusters:
 *     count: 8
 *     order: [P1, P2, ...]
 *     allocation_matrix:
 *       rows: [Fx, Fy, Fz, Mx, My, Mz]
 *       data: [[...8...], ...]     # 6 行
 *       active_rows: [Fx, Fy, Fz, Mz]
 *     limits:
 *       norm_min: -1.0
 *       norm_max:  1.0
 *       wrench_limits: { Fx_min:..., Fx_max:..., ... }
 *       norm_slew_rate: { enabled: true, max_delta_per_step: 0.05 }
 *     solver:
 *       method: pinv
 *       svd_epsilon: 1e-6
 *       thruster_weights: [1,1,1,1,1,1,1,1]
 *
 * 参数：
 *   - path : 已解析好的 alloc.yaml 路径
 *   - cfg  : 输出 ThrusterAllocationConfig（结构体内部默认值可作为缺省回退）
 *   - log  : 日志输出流
 *
 * 返回值：
 *   - true  : 加载成功
 *   - false : 加载失败（调用方应决定是否退出）
 */
bool load_thruster_allocation_config(
    const fs::path&                                      path,
    rovctrl::control_core::ThrusterAllocationConfig&     cfg,
    std::ostream&                                        log);

/**
 * 从 trajectory.yaml 加载轨迹配置（预留接口）。
 *
 * 你可以在后续定义 rovctrl::control_core::TrajectoryConfig 的具体字段，
 * 然后在 src/utils/config_loader.cpp 中实现加载逻辑。
 *
 * 当前若尚未实现，可以先在 cpp 中返回 false，并在日志中提示“not implemented”。
 */
bool load_trajectory_config(const fs::path&                 path,
                            rovctrl::control_core::TrajectoryConfig& cfg,
                            std::ostream&                   log);

/**
 * 从 teleop_mixer.yaml 加载键盘混合器配置（6DOF → 8 thrusters）。
 * 
 * 校验建议：
 *   - matrix 必须为 8 行，每行 6 列；
 *   - output_limit_abs > 0；
 *   - gains 缺省字段按 1.0 回退；
 *   - enable=false 时由调用方决定回退策略（例如禁用 teleop）。
 *
 * 返回值：
 *   - true  : 加载成功
 *   - false : 加载失败（调用方应决定是否退出或回退默认）
 */
bool load_teleop_mixer_config(const fs::path&                      path,   // <<< NEW
                              rovctrl::control_core::TeleopMixerConfig& cfg,
                              std::ostream&                        log);

} // namespace rovctrl::utils
