// src/controllers/manual_controller.cpp

#include "controllers/manual_controller.hpp"

#include <algorithm>
#include <iostream>

namespace rovctrl::controllers {

using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlOutput;

ManualController::ManualController(const ManualControllerConfig& cfg)
    : cfg_(cfg)
{}

// 统一限幅到 [-max_cmd_abs, max_cmd_abs]
void ManualController::clamp_output(ControlOutput& out) const
{
    const double limit = (cfg_.max_cmd_abs > 0.0) ? cfg_.max_cmd_abs : 1.0;
    for (auto& v : out.thruster_command) {
        v = static_cast<float>(std::clamp(static_cast<double>(v), -limit, limit));
    }
}

bool ManualController::compute(const ControlState&     /*state*/,
                               const ControlReference& ref,
                               ControlOutput&          out,
                               double                  /*dt*/)
{
    // 读取 DOF 命令并施加增益（这里 ref.* 是 TeleopInput 写进来的 [-1,1]）
    const double surge = static_cast<double>(ref.surge) * cfg_.surge_gain;
    const double sway  = static_cast<double>(ref.sway)  * cfg_.sway_gain;
    const double heave = static_cast<double>(ref.heave) * cfg_.heave_gain;
    const double yaw   = static_cast<double>(ref.yaw)   * cfg_.yaw_gain;
    const double roll  = static_cast<double>(ref.roll)  * cfg_.roll_gain;
    const double pitch = static_cast<double>(ref.pitch) * cfg_.pitch_gain;

    // 清空输出
    out = ControlOutput{};
    out.has_thruster_command = true;

    auto& u = out.thruster_command;
    if (u.size() < 8) {
        std::cerr << "[ManualController] thruster_command size < 8, got "
                  << u.size() << "\n";
        return false;
    }

    // ===========================
    //  水平 1–4 逻辑电机分配
    // ===========================
    //
    // 约定：
    //  - u[0]..u[3] = 逻辑电机 1..4，对应你说的“水平推进器”
    //  - surge>0: 1~4 同向前进
    //  - sway>0: 1,4 正向；2,3 反向（右移）
    //  - yaw>0 : 左转：1 反、2 正、3 反、4 正

    // 先全部置零
    for (int i = 0; i < 8; ++i) {
        u[i] = 0.0f;
    }

    // surge 分量： [1,1,1,1]
    u[0] += static_cast<float>(surge);
    u[1] += static_cast<float>(surge);
    u[2] += static_cast<float>(surge);
    u[3] += static_cast<float>(surge);

    // sway 分量：右移模式 [1,-1,-1,1]
    u[0] += static_cast<float>(sway);
    u[1] += static_cast<float>(-sway);
    u[2] += static_cast<float>(-sway);
    u[3] += static_cast<float>(sway);

    // yaw 分量：左转模式 [-1,+1,-1,+1]
    u[0] += static_cast<float>(-yaw);
    u[1] += static_cast<float>(+yaw);
    u[2] += static_cast<float>(-yaw);
    u[3] += static_cast<float>(+yaw);

    // ===========================
    //  垂向 5–8 逻辑电机分配（示例）
    // ===========================
    //
    // 这里还是给一个常见写法，你可以根据实际机体布局改：
    //  - heave: 所有垂向推进器同向
    //  - roll : 左右差动
    //  - pitch: 前后差动
    //
    // 假设：
    //  4: 前左垂向 (逻辑 5 → u[4])
    //  5: 前右垂向 (逻辑 6 → u[5])
    //  6: 后左垂向 (逻辑 7 → u[6])
    //  7: 后右垂向 (逻辑 8 → u[7])

    u[4] += static_cast<float>(heave - roll - pitch); // 前左
    u[5] += static_cast<float>(heave + roll - pitch); // 前右
    u[6] += static_cast<float>(heave - roll + pitch); // 后左
    u[7] += static_cast<float>(heave + roll + pitch); // 后右

    // 统一限幅
    clamp_output(out);
    return true;
}

} // namespace rovctrl::controllers
