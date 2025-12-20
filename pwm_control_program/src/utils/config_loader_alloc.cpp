// pwm_control_program/src/utils/config_loader_alloc.cpp

#include "utils/config_loader.hpp"

#include <array>
#include <exception>
#include <string>

#include <yaml-cpp/yaml.h>

#include "control_core/thruster_allocation.hpp"
#include "utils/detail/config_log.hpp"
#include "utils/detail/dof.hpp"
#include "utils/detail/yaml_helpers.hpp"

namespace rovctrl::utils {

namespace fs = std::filesystem;
using detail::LogLevel;
using detail::log_line;

bool load_thruster_allocation_config(const fs::path&                         path,
                                     control_core::ThrusterAllocationConfig& alloc_cfg,
                                     std::ostream&                           log)
{
    try {
        log_line(log, "Alloc", LogLevel::Info,
                 std::string("Loading alloc.yaml: ") + path.string());

        YAML::Node root = YAML::LoadFile(path.string());

        if (!root["thrusters"]) {
            log_line(log, "Alloc", LogLevel::Err, "Missing top-level key: thrusters");
            return false;
        }
        YAML::Node thr = root["thrusters"];

        int count = 0;
        if (!detail::yaml_as<int>(thr["count"], "thrusters.count", count, log, "Alloc", true)) {
            return false;
        }
        if (count != 8) {
            log_line(log, "Alloc", LogLevel::Err,
                     "thrusters.count must be 8 for current build. Got: " + std::to_string(count));
            return false;
        }

        if (!thr["order"] || !thr["order"].IsSequence() || thr["order"].size() != 8) {
            log_line(log, "Alloc", LogLevel::Err,
                     "thrusters.order missing/not sequence/size!=8. Expected 8 strings.");
            return false;
        }
        for (std::size_t i = 0; i < 8; ++i) {
            alloc_cfg.thruster_order_yaml[i] = thr["order"][i].as<std::string>();
        }

        if (!thr["allocation_matrix"]) {
            log_line(log, "Alloc", LogLevel::Err, "Missing key: thrusters.allocation_matrix");
            return false;
        }
        YAML::Node mat = thr["allocation_matrix"];

        if (mat["rows"]) {
            if (!mat["rows"].IsSequence() || mat["rows"].size() != 6) {
                log_line(log, "Alloc", LogLevel::Err,
                         "allocation_matrix.rows must be [Fx,Fy,Fz,Mx,My,Mz].");
                return false;
            }
            static const std::array<std::string, 6> kExpected = {"Fx","Fy","Fz","Mx","My","Mz"};
            for (std::size_t i = 0; i < 6; ++i) {
                const std::string s = mat["rows"][i].as<std::string>();
                if (s != kExpected[i]) {
                    log_line(log, "Alloc", LogLevel::Err,
                             "allocation_matrix.rows mismatch at index " + std::to_string(i) +
                             ": got '" + s + "', expected '" + kExpected[i] + "'.");
                    return false;
                }
            }
        } else {
            log_line(log, "Alloc", LogLevel::Debug,
                     "allocation_matrix.rows not provided; assume fixed order [Fx,Fy,Fz,Mx,My,Mz].");
        }

        if (!mat["data"] || !mat["data"].IsSequence() || mat["data"].size() != 6) {
            log_line(log, "Alloc", LogLevel::Err,
                     "allocation_matrix.data must be a 6-row sequence (6x8).");
            return false;
        }
        for (std::size_t r = 0; r < 6; ++r) {
            YAML::Node row = mat["data"][r];
            if (!row.IsSequence() || row.size() != 8) {
                log_line(log, "Alloc", LogLevel::Err,
                         "allocation_matrix.data row " + std::to_string(r) + " must have 8 elements.");
                return false;
            }
            for (std::size_t c = 0; c < 8; ++c) {
                alloc_cfg.allocation_matrix_yaml[r][c] = row[c].as<double>();
            }
        }

        alloc_cfg.active_dof = {false, false, false, false, false, false};
        if (mat["active_rows"]) {
            if (!mat["active_rows"].IsSequence()) {
                log_line(log, "Alloc", LogLevel::Err,
                         "allocation_matrix.active_rows must be a sequence, e.g. [Fx,Fy,Fz,Mz].");
                return false;
            }
            for (std::size_t i = 0; i < mat["active_rows"].size(); ++i) {
                const std::string name = mat["active_rows"][i].as<std::string>();
                const int idx = detail::dof_index(name);
                if (idx < 0) {
                    log_line(log, "Alloc", LogLevel::Warn,
                             "Unknown DOF in active_rows: '" + name + "' (ignored)");
                    continue;
                }
                alloc_cfg.active_dof[static_cast<std::size_t>(idx)] = true;
            }
        } else {
            log_line(log, "Alloc", LogLevel::Warn,
                     "allocation_matrix.active_rows missing; all DOF inactive (may output zeros).");
        }

        if (!detail::any_active(alloc_cfg.active_dof)) {
            log_line(log, "Alloc", LogLevel::Warn,
                     "No active DOF enabled. Verify allocation_matrix.active_rows.");
        }

        if (thr["limits"]) {
            YAML::Node lim = thr["limits"];
            if (lim["norm_min"]) alloc_cfg.norm_min = lim["norm_min"].as<double>();
            if (lim["norm_max"]) alloc_cfg.norm_max = lim["norm_max"].as<double>();
        } else {
            log_line(log, "Alloc", LogLevel::Warn,
                     "thrusters.limits missing; keep defaults norm_min/norm_max.");
        }

        if (!(alloc_cfg.norm_min < alloc_cfg.norm_max)) {
            log_line(log, "Alloc", LogLevel::Err,
                     "Invalid norm range: norm_min=" + std::to_string(alloc_cfg.norm_min) +
                     ", norm_max=" + std::to_string(alloc_cfg.norm_max));
            return false;
        }

        alloc_cfg.thrust_model.norm_min = alloc_cfg.norm_min;
        alloc_cfg.thrust_model.norm_max = alloc_cfg.norm_max;

        {
            std::string order = "order=[";
            for (std::size_t i = 0; i < 8; ++i) {
                order += alloc_cfg.thruster_order_yaml[i];
                if (i + 1 < 8) order += ",";
            }
            order += "]";
            log_line(log, "Alloc", LogLevel::Info,
                     "alloc.yaml loaded OK: " + order +
                     ", norm=[" + std::to_string(alloc_cfg.norm_min) + "," +
                     std::to_string(alloc_cfg.norm_max) + "]");
        }

        return true;
    } catch (const std::exception& e) {
        log_line(log, "Alloc", LogLevel::Err,
                 std::string("Failed to load alloc.yaml: ") + e.what());
        return false;
    }
}

} // namespace rovctrl::utils
