// pwm_control_program/include/utils/config_loader.hpp
#pragma once

#include <filesystem>
#include <initializer_list>
#include <iosfwd>
#include <string>

namespace rovctrl {

namespace platform {
    // 在 platform/pwm_client.hpp 中定义
    struct PwmClientConfig;
}

namespace control_core {
    // 在 control_core/thruster_allocation.hpp 中定义
    struct ThrusterAllocationConfig;

    // 预留：轨迹配置结构体，暂时只做前向声明
    // 你可以在将来的 trajectory_config.hpp 中给出完整定义
    struct TrajectoryConfig;
}

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
 *   - 命令行参数：--config <path>  → 传入 cli_opt
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
 *   - 建议未来命令行参数：--traj-config <path>  → 传入 cli_opt
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
bool load_pwm_client_config(const fs::path&         path,
                            rovctrl::platform::PwmClientConfig& cfg,
                            std::ostream&           log);

/**
 * 从 control_params.yaml 加载推力分配 / 控制相关参数。
 *
 * 参数：
 *   - path : 已解析好的 control_params.yaml 路径
 *   - cfg  : 输出 ThrusterAllocationConfig
 *   - log  : 日志输出流
 *
 * 返回值：
 *   - true  : 加载成功
 *   - false : 加载失败
 */
bool load_thruster_allocation_config(
    const fs::path&                         path,
    rovctrl::control_core::ThrusterAllocationConfig& cfg,
    std::ostream&                           log);

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

} // namespace rovctrl::utils
