#pragma once
/**
 * @file app_main.hpp
 * @brief pwm_control_program 的高层入口封装（应用层）
 *
 * 设计原则：
 *   - app_main.hpp 只定义“应用入口 + 运行选项（AppOptions）”，避免包含 IO/Platform 细节类型；
 *   - 初始化与装配逻辑（PwmClient / InputProvider / ControllerManager / ControlLoop / GCS UDP 等）
 *     全部在 app_main.cpp 内完成；
 *   - 配置加载细节（YAML 字段解析）应放在 utils/config_loader.* 或各模块专用 loader 中，
 *     避免 app_main.hpp 变成“配置函数集合”。
 *
 * 典型调用方式：
 *   int main(int argc, char** argv) { return rovctrl::control_core::app_main(argc, argv); }
 *
 * 说明：
 *   - AppOptions 是“命令行参数 + 配置文件路径 + 运行时开关”的聚合；
 *   - 具体默认值可与当前工程默认行为一致，后续可在 app_main.cpp 中覆盖。
 */

#include <cstdint>
#include <string>

namespace rovctrl::control_core {

/**
 * @brief 应用层运行选项集合
 *
 * 约定：
 *  - *_config_path：YAML 配置路径（可为空，表示使用默认值或禁用对应模块）
 *  - 频率单位：Hz
 *  - PWM 单位：us（若在配置中使用）
 *  - max_step_pct：0~1 的比例（如 0.2 表示每步最大变化 20%）
 */
struct AppOptions final {
    // -------------------------
    // 配置文件路径（建议全部可通过 CLI 覆盖）
    // -------------------------

    /// PWM client 与安全层配置（如 pwm_client.yaml）
    std::string pwm_config_path;

    /// 控制参数配置（如 control_params.yaml）
    std::string control_config_path;

    /// 轨迹配置（如 trajectory.yaml）
    std::string trajectory_config_path;

    /// 推力分配配置（thruster allocation），建议单独一份 yaml
    std::string alloc_config_path;

    // -------------------------
    // 运行频率与安全参数
    // -------------------------

    /// 主控制循环频率（ControlLoop 调度频率）
    int loop_hz = 100;

    /// PWM 安全层内部控制频率（若与 loop_hz 分开）
    int pwm_hz = 100;

    /// 每个周期允许的最大输出变化比例（0~1）
    double max_step_pct = 0.2;

    /// 启用 PWM 下发（调试时可关闭：只算不下发）
    bool enable_pwm_output = true;

    /// 启用 PWM 日志
    bool enable_pwm_logging = true;

    // -------------------------
    // UDP / 上位机通信（GCS Link）
    // -------------------------

    /// 是否启用与上位机的 UDP 通信模块
    bool udp_enable = false;

    /// 本机 UDP 监听端口（接收上位机指令/心跳等）
    int udp_listen_port = 14550;

    /// 上位机 IP（可为空：表示仅监听不回发，或等待首次报文自动学习）
    std::string udp_peer_ip;

    /// 上位机端口（0 表示未指定；可配合 udp_peer_ip 使用）
    int udp_peer_port = 0;

    /// UDP 协议/消息版本（可选：例如 "v1", "simple", "gcs_v1" 等）
    std::string udp_protocol = "default";

    // -------------------------
    // NavState 订阅（来自导航进程/共享内存/UDP 等）
    // -------------------------

    /// 是否订阅导航状态（NavState），用于闭环控制
    bool nav_sub_enable = true;

    /// NavState 数据源类型（例如 "shm", "udp", "none"）
    std::string nav_source = "shm";

    /// 若 nav_source=udp：本机监听端口
    int nav_udp_listen_port = 0;

    // -------------------------
    // Teleop 输入
    // -------------------------

    /// 是否启用键盘/遥操作输入
    bool teleop_enable = true;

    /// Teleop 设备类型（例如 "keyboard", "none"）
    std::string teleop_device = "keyboard";

    // -------------------------
    // 诊断与调试
    // -------------------------

    /// 输出更详细日志
    bool verbose = false;

    /// 干跑模式：禁止实际下发（等价于 enable_pwm_output=false），但保持日志/计算链路
    bool dry_run = false;
};

/**
 * @brief 程序主入口（由 main.cpp 调用）
 *
 * 支持命令行参数（建议 app_main.cpp 实现完整解析，并与 docs/config_reference.md 对齐）：
 *
 *  配置文件：
 *   --pwm-config <yaml>         PWM client 与安全层配置（默认：config/pwm_client.yaml）
 *   --control-config <yaml>     控制参数配置（默认：config/control_params.yaml）
 *   --traj-config <yaml>        轨迹配置（默认：config/trajectory.yaml）
 *   --alloc-config <yaml>       推力分配配置（默认：config/control_params.yaml 或单独文件）
 *
 *  频率与限幅：
 *   --loop-hz <Hz>              主控制循环频率
 *   --pwm-hz <Hz>               PWM 安全层内部频率
 *   --max-step <pct>            单步 PWM 最大变化比例（0~1）
 *
 *  运行开关：
 *   --dry-run                   干跑：不实际下发 PWM
 *   --no-pwm-log                关闭 PWM 日志
 *   --no-pwm-out                禁止 PWM 输出（只算不发）
 *   --verbose                   更详细日志
 *
 *  UDP / 上位机：
 *   --udp-enable                启用 GCS UDP 通信
 *   --udp-listen <port>         本机 UDP 监听端口
 *   --udp-peer-ip <ip>          上位机 IP（可选）
 *   --udp-peer-port <port>      上位机端口（可选）
 *   --udp-proto <name>          协议版本/名称（可选）
 *
 *  导航订阅：
 *   --nav-enable / --nav-disable
 *   --nav-source <shm|udp|none>
 *   --nav-udp-listen <port>     若 nav-source=udp
 *
 *  Teleop：
 *   --teleop-enable / --teleop-disable
 *   --teleop-device <keyboard|none>
 *
 * @return 0 成功退出；非 0 表示错误。
 */
int app_main(int argc, char** argv);

} // namespace rovctrl::control_core
