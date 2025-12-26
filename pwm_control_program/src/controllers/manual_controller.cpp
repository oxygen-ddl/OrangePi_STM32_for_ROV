#include "controllers/manual_controller.hpp"

#include <algorithm>
#include <iostream>

namespace rovctrl::controllers {

using rovctrl::control_core::ControlState;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlOutput;

// 构造函数
ManualController::ManualController(const ManualControllerConfig& cfg)
    : cfg_(cfg)
{}

// 统一限幅到 [-max_cmd_abs, max_cmd_abs]
void ManualController::clamp_output(rovctrl::control_core::ControlOutput& out) const noexcept
{
    const double limit = (cfg_.max_cmd_abs > 0.0) ? cfg_.max_cmd_abs : 1.0;
    for (auto& v : out.thruster_command) {
        v = static_cast<float>(
            std::clamp(static_cast<double>(v), -limit, limit)
        );
    }
}

bool ManualController::compute(const ControlState&     /*state*/,
                               const ControlReference& ref,
                               ControlOutput&          out,
                               double                  /*dt*/) noexcept
{
    // ============================================================
    // 1. 读取 Teleop 写入的 6-DOF 归一化指令 [-1,1]
    //
    // 约定（与 ControlTypes 中 DofCommand 对应）：
    //   surge : X 轴前后   (+ 前进,  - 后退)
    //   sway  : Y 轴左右   (+ 右移,  - 左移)
    //   heave : Z 轴上下   (+ 上浮,  - 下潜)
    //   roll  : 横滚      (+ 右侧下沉, - 左侧下沉)
    //   pitch : 俯仰      (+ 头上尾下, - 头下尾上)
    //   yaw   : 偏航      (+ 左转,    - 右转)
    //
    // 这里手动模式做的事情很简单：
    //   - 对每个 DOF 乘一个增益系数（cfg_.xxx_gain）
    //   - 再按推进器拓扑（电机布局）把 6 个 DOF 映射到 8 个电机输出上
    // ============================================================
    const double surge = static_cast<double>(ref.dof_cmd.surge) * cfg_.surge_gain;
    const double sway  = static_cast<double>(ref.dof_cmd.sway)  * cfg_.sway_gain;
    const double heave = static_cast<double>(ref.dof_cmd.heave) * cfg_.heave_gain;
    const double yaw   = static_cast<double>(ref.dof_cmd.yaw)   * cfg_.yaw_gain;
    const double roll  = static_cast<double>(ref.dof_cmd.roll)  * cfg_.roll_gain;
    const double pitch = static_cast<double>(ref.dof_cmd.pitch) * cfg_.pitch_gain;

    // ============================================================
    // 2. 清空输出，准备写入 thruster_command[8]
    // ============================================================
    out = ControlOutput{};
    out.has_thruster_command = true;

    auto& u = out.thruster_command;
    if (u.size() < 8) {
        std::cerr << "[ManualController] thruster_command size < 8, got "
                  << u.size() << "\n";
        return false;
    }

    // 全部置零（逻辑上等价于“当前帧无任何输入”）
    for (std::size_t i = 0; i < u.size(); ++i) {
        u[i] = 0.0f;
    }

    // ============================================================
    // 3. 推进器拓扑说明（当前假定布局）
    //
    // 下文中假定 8 个推进器逻辑编号如下：
    //
    //   水平推进器（1–4）俯视图（机体指向 ↑ 前方）：
    //
    //          前方
    //           ↑
    //       [1]   [2]
    //
    //       [3]   [0]
    //           ↓ 后方
    //
    //   垂向推进器（5–8）俯视图：
    //
    //          前方
    //           ↑
    //       [4]   [5]
    //
    //       [6]   [7]
    //           ↓ 后方
    //
    // 说明：
    //   - u[i] > 0 表示“正向转动”（具体方向由接线决定）
    //   - u[i] < 0 表示“反向转动”
    //
    // 如果日后推进器重新布线 / 更换布置，
    //   只需要修改下面的“加减关系”，不需要改上层 Teleop / ControlLoop。
    // ============================================================

    // ============================================================
    // 4. 水平推进器 0–3：根据 surge / sway / yaw 做合成
    // ============================================================

    // 4.1 surge 分量：前进/后退
    //
    // 设计思路：
    //   - 前进时：4 个水平推进器同向输出 → 比如全为 +surge
    //   - 后退时：4 个水平推进器同向输出 → 全为 -|surge|
    //
    // 这里简单采用向量和： u[i] += surge
    u[0] += static_cast<float>(surge);
    u[1] += static_cast<float>(surge);
    u[2] += static_cast<float>(surge);
    u[3] += static_cast<float>(surge);

    // 4.2 sway 分量：左右平移
    //
    // 约定：sway > 0 表示“向右平移”
    //
    // 一个常见的差动分配方案：
    //   - 右移时：右侧推进器推前，左侧推进器推后（或反之，取决于安装）
    //   - 这里采用符号模式 [ +1, -1, -1, +1 ]
    //
    // 具体物理意义：
    //   - u[0] (右后) += +sway
    //   - u[1] (右前) += -sway
    //   - u[2] (左前) += -sway
    //   - u[3] (左后) += +sway
    //
    // 如果将来调试发现“按 D 结果是向左”，
    //   就可以在这里整体翻转符号或者调换若干电机位置。
    u[0] += static_cast<float>(sway);
    u[1] += static_cast<float>(-sway);
    u[2] += static_cast<float>(-sway);
    u[3] += static_cast<float>(sway);

    // 4.3 yaw 分量：平面偏航（左转 / 右转）
    //
    // 约定：yaw > 0 表示“左转”。
    //
    // 为了产生“绕 Z 轴的力矩”，需要左右两侧推力方向不同：
    //   - 这里采用符号模式 [ -1, +1, -1, +1 ]
    //
    // 直观理解（示例）：
    //   - 左转时：右侧电机略向前推、左侧略向后推，
    //     在机体上形成一个“逆时针”的合力矩。
    u[0] += static_cast<float>(-yaw);
    u[1] += static_cast<float>(+yaw);
    u[2] += static_cast<float>(-yaw);
    u[3] += static_cast<float>(+yaw);

    // ============================================================
    // 5. 垂向推进器 4–7：根据 heave / roll / pitch 做合成
    // ============================================================
    //
    // 布局回顾（俯视）：
    //
    //         前方
    //          ↑
    //     [4] FL   FR [5]
    //
    //     [6] RL   RR [7]
    //          ↓ 后方
    //
    // 设计思路：
    //   - heave：整体上升/下降 → 所有垂向推进器同向输出
    //   - roll ：左右差动       → 左右两侧推力不同
    //   - pitch：前后差动       → 前后两侧推力不同
    //
    // 组合规则（可当作“线性叠加模板”）：
    //
    //   u[4] (前左 FL) =  heave  - roll  - pitch
    //   u[5] (前右 FR) =  heave  + roll  - pitch
    //   u[6] (后左 RL) =  heave  - roll  + pitch
    //   u[7] (后右 RR) =  heave  + roll  + pitch
    //
    // 这样可以检查三个 DOF 的效果：
    //
    // 1) 纯 heave（roll=pitch=0）：
    //      所有电机 = heave → 纯上浮/下潜
    //
    // 2) 纯 roll（heave=pitch=0）：
    //      左侧电机：heave - roll → -roll
    //      右侧电机：heave + roll → +roll
    //    → 左右差动，产生 “绕 X 轴” 的力矩
    //
    // 3) 纯 pitch（heave=roll=0）：
    //      前方：heave - pitch → -pitch
    //      后方：heave + pitch → +pitch
    //    → 前后差动，产生 “绕 Y 轴” 的力矩
    //
    // 将来如果拓扑变化（例如 6 推进器、推力方向不同）：
    //   - 只需要在这里重新写 4~7 的线性组合；
    //   - 建议保持 “heave 一致 + roll 左右差动 + pitch 前后差动” 的思路不变。
    //
    u[4] += static_cast<float>(heave - roll - pitch); // 前左 FL
    u[5] += static_cast<float>(heave + roll - pitch); // 前右 FR
    u[6] += static_cast<float>(heave - roll + pitch); // 后左 RL
    u[7] += static_cast<float>(heave + roll + pitch); // 后右 RR

    // ============================================================
    // 6. 统一限幅（保护推进器 / 保证 [-1,1] 约束）
    //
    // clamp_output() 内部通常会：
    //   - 对 thruster_command[i] 做 [-max_cmd_abs, +max_cmd_abs] 的裁剪；
    //   - max_cmd_abs 由 cfg_.max_cmd_abs 给出（默认 1.0）；
    //   - 可以在将来加入软限幅/非线性放大等更复杂逻辑。
    // ============================================================
    clamp_output(out);
    return true;
}

}