// pwm_control_program/src/utils/config_loader_teleop_mixer.cpp
//
// 负责从 teleop_mixer.yaml 加载键盘 6DOF → 8 推进器的混合矩阵。
// 设计目标：
//   - 配置文件是“增强项”，不是必需项：读不到/解析失败时保留默认配置；
//   - 不访问 TeleopMixerConfig 中不存在的字段（避免结构不一致）；
//   - 捕获所有 std::exception，避免把异常抛到控制栈最外层。

#include "utils/config_loader.hpp"

#include <exception>
#include <string>

#include <yaml-cpp/yaml.h>

#include "control_core/teleop_mixer.hpp"
#include "utils/detail/config_log.hpp"

namespace rovctrl::utils {

namespace fs = std::filesystem;
using detail::LogLevel;
using detail::log_line;

using rovctrl::control_core::TeleopMixerConfig;

bool load_teleop_mixer_config(const fs::path&           path,
                              TeleopMixerConfig&        cfg,
                              std::ostream&             log)
{
    try {
        // 1) 路径为空：不做任何修改，保留默认
        if (path.empty()) {
            log_line(log, "TeleopMix", LogLevel::Warn,
                     "teleop_mixer.yaml path is empty, keep default TeleopMixerConfig.");
            return true;
        }

        log_line(log, "TeleopMix", LogLevel::Info,
                 std::string("Loading teleop_mixer.yaml: ") + path.string());

        // 2) 加载 YAML
        YAML::Node root = YAML::LoadFile(path.string());
        if (!root || !root["teleop_mixer"]) {
            log_line(log, "TeleopMix", LogLevel::Warn,
                     "Missing top-level key 'teleop_mixer'; keep default TeleopMixerConfig.");
            return true;    // 非致命：直接用默认 cfg
        }

        YAML::Node tm = root["teleop_mixer"];

        // 3) enable（可选）
        if (tm["enable"]) {
            cfg.enable = tm["enable"].as<bool>();
        }

        // 4) input_deadzone（可选）
        if (tm["input_deadzone"]) {
            cfg.input_deadzone = tm["input_deadzone"].as<float>();
            if (cfg.input_deadzone < 0.0f) {
                log_line(log, "TeleopMix", LogLevel::Warn,
                         "input_deadzone < 0, clamp to 0.");
                cfg.input_deadzone = 0.0f;
            }
        }

        // 5) output_limit_abs（可选）
        //
        // 注意：根据你的头文件，这个字段是 float 类型，因此用 as<float>()，
        // 避免 double→float 的转换告警。
        if (tm["output_limit_abs"]) {
            cfg.output_limit_abs = tm["output_limit_abs"].as<float>();
            if (!(cfg.output_limit_abs > 0.0f)) {
                log_line(log, "TeleopMix", LogLevel::Warn,
                         "output_limit_abs <= 0, reset to 1.0.");
                cfg.output_limit_abs = 1.0f;
            }
        }

        // 6) mix_matrix: [thruster][dof]，即 8x6
        //
        // 格式示例：
        // teleop_mixer:
        //   mix_matrix:
        //     - [ 1,  1,  0,  0,  0, -1 ]   # thruster 0
        //     - [ 1, -1,  0,  0,  0,  1 ]   # thruster 1
        //     - [ 1, -1,  0,  0,  0, -1 ]   # thruster 2
        //     - [ 1,  1,  0,  0,  0,  1 ]   # thruster 3
        //     - [ 0,  0,  1, -1, -1,  0 ]   # thruster 4
        //     - [ 0,  0,  1,  1, -1,  0 ]   # thruster 5
        //     - [ 0,  0,  1, -1,  1,  0 ]   # thruster 6
        //     - [ 0,  0,  1,  1,  1,  0 ]   # thruster 7
        if (tm["mix_matrix"]) {
            YAML::Node m = tm["mix_matrix"];
            if (!m.IsSequence() || m.size() != TeleopMixerConfig::kNumThrusters) {
                log_line(log, "TeleopMix", LogLevel::Err,
                         "mix_matrix must be a sequence of size " +
                         std::to_string(TeleopMixerConfig::kNumThrusters) + ".");
                return false;   // 配置结构错误：明确返回 false
            }

            for (std::size_t r = 0; r < TeleopMixerConfig::kNumThrusters; ++r) {
                YAML::Node row = m[r];
                if (!row.IsSequence() || row.size() != TeleopMixerConfig::kNumDof) {
                    log_line(log, "TeleopMix", LogLevel::Err,
                             "mix_matrix row " + std::to_string(r) +
                             " must be a sequence of size " +
                             std::to_string(TeleopMixerConfig::kNumDof) + ".");
                    return false;
                }

                for (std::size_t c = 0; c < TeleopMixerConfig::kNumDof; ++c) {
                    cfg.mix_matrix[r][c] = row[c].as<float>();
                }
            }
        } else {
            log_line(log, "TeleopMix", LogLevel::Warn,
                     "teleop_mixer.mix_matrix not provided; keep default mix_matrix.");
        }

        // 7) 简短 summary
        {
            std::string msg = "teleop_mixer loaded: enable=";
            msg += (cfg.enable ? "true" : "false");
            msg += ", input_deadzone=" + std::to_string(cfg.input_deadzone);
            msg += ", output_limit_abs=" + std::to_string(cfg.output_limit_abs);
            log_line(log, "TeleopMix", LogLevel::Info, msg);
        }

        return true;
    } catch (const std::exception& e) {
        log_line(log, "TeleopMix", LogLevel::Err,
                 std::string("Exception while loading teleop_mixer.yaml: ") + e.what());
        // 这里返回 false，让上层决定是否认为“配置损坏”需要特殊处理；
        // 上层如果只是打印一条 [ERR] 然后继续用默认 cfg，也完全可以。
        return false;
    }
}

} // namespace rovctrl::utils
