#pragma once
#ifndef GATEWAY_IPC_KEYS_KEYBOARD_MAPPER_HPP
#define GATEWAY_IPC_KEYS_KEYBOARD_MAPPER_HPP

#include <cstdint>

#include "shared/msg/key_event.hpp"

namespace comm_gcs::ipc::keys {

/**
 * @brief 键盘事件语义映射（KeyEvent -> TeleopAction）
 *
 * 业务目标（给新人看的）：
 *  1) 本模块只做“输入设备语义层”的工作：把低层 KeyEvent（按键码/修饰键/动作）归一化，
 *     映射为上层可复用的控制语义 TeleopAction。
 *  2) 本模块不做任何“时间逻辑/安全逻辑/执行逻辑”：
 *     - 不实现 hold-to-run 超时归零（属于 TeleopStateMachine / IntentKeyboardSource）
 *     - 不实现 throttle 平滑/限速（属于 TeleopStateMachine / IntentKeyboardSource）
 *     - 不实现 E-STOP 锁存与解除门槛（属于 ControlGuard）
 *     - 不做 PWM/推进器分配等执行（属于 pwm_control / control_core）
 *     - 不做“阻塞式单电机测试”的计时与停止（属于上层执行逻辑）
 *  3) 通过这一层抽象，未来无论输入来自本地键盘、GCS UI、手柄或脚本，
 *     都可以统一转换为 TeleopAction，再由同一套状态机与安全层处理，保证行为一致。
 *
 * 映射原则（迁移 teleop_keyboard.cpp 的行为）：
 *  - 连续 DOF（WASD/QE/HG/RT/FV）：按住有效；终端常见只有 Press/Repeat，
 *    因此 Press/Repeat 都视为“刷新 hold”的动作。
 *  - 离散命令（E-STOP/ClearEStop/Arm/Disarm/Exit/Center/Help/ThrottleStep/单电机测试）：
 *    默认只在 Press 触发（edge-trigger），避免 Repeat 导致重复触发。
 *    油门键是“-”和“+”，也视为离散命令以避免长按跑飞。
 *
 * 安全/系统命令：
 *  - Ctrl + Space：E-STOP 请求（真正锁存由 ControlGuard 决定）
 *  - Ctrl + M/m：解除急停请求（真正解除需 ControlGuard 门槛）
 *  - ',' 或 '，'：Arm 请求（全角/半角兼容）
 *  - '.' 或 '。'：Disarm 请求（全角/半角兼容）
 *  - 'Z'：Help（输出 TeleopAction::Help，由上层决定打印帮助/诊断信息）
 *
 * 单电机测试（阻塞式输入语义）：
 *  - 数字键 '1'..'8'：请求对对应“逻辑电机”进行单电机测试。
 *    上层执行语义为：收到该动作后，进入“单电机测试”模式，电机转动 2s 后自动停止（归中）。
 *    注意：这里仅发出“开始测试”的语义，不负责计时与停止；计时与归中由上层实现。
 *  - '0'：请求立即停止单电机测试并归中（可选但推荐，用于快速中止测试）。
 *  - 约束建议（由上层落实）：仅在系统 idle（无 DOF hold）时允许进入单电机测试；否则忽略并告警。
 */
enum class TeleopAction : std::uint16_t {
    None = 0,

    // ------------------------------
    // 连续运动意图（Hold-to-run 语义）
    // ------------------------------
    SurgePos,   // W
    SurgeNeg,   // S

    SwayNeg,    // A  (注意：与旧 teleop_keyboard.cpp 一致：A = sway-)
    SwayPos,    // D

    YawPos,     // Q
    YawNeg,     // E

    HeavePos,   // H
    HeaveNeg,   // G

    RollPos,    // R  (特殊姿态：通常会清空其他 DOF，由状态机实现)
    RollNeg,    // T

    PitchPos,   // F  (特殊姿态：通常会清空其他 DOF，由状态机实现)
    PitchNeg,   // V

    // ------------------------------
    // 离散系统命令（Edge-trigger 语义）
    // ------------------------------
    Center,        // M：清空 DOF（不直接操作 PWM）
    Exit,          // ESC：请求退出（由上层执行停机流程）
    Help,          // Z：显示帮助/诊断信息（由上层决定如何展示）

    EStop,         // Ctrl+Space：急停请求（锁存由 ControlGuard 实现）
    ClearEStop,    // Ctrl+M/m：解除急停请求（门槛由 ControlGuard 实现）

    Arm,           // ',' 或 '，'
    Disarm,        // '.' 或 '。'

    ThrottleUp,    // '=' 或 '+'
    ThrottleDown,  // '-' 或 '_'

    // ------------------------------
    // 单电机测试（Edge-trigger 语义；执行层阻塞 2s 自动停止）
    // ------------------------------
    MotorTest1,     // '1'：测试逻辑电机 1（上层：转动 2s 后归中）
    MotorTest2,     // '2'
    MotorTest3,     // '3'
    MotorTest4,     // '4'
    MotorTest5,     // '5'
    MotorTest6,     // '6'
    MotorTest7,     // '7'
    MotorTest8,     // '8'
    MotorTestStop,  // '0'：立即停止单电机测试并归中（推荐）
};

/**
 * @brief Mapper 行为配置（默认值匹配典型终端行为）
 *
 * 说明：
 *  - repeat_as_press：终端常见只会产生 Press/Repeat；DOF hold 需要 Repeat 刷新。
 *  - discrete_on_press_only：离散命令默认只在 Press 触发，避免长按/自动 repeat 造成多次触发。
 */
struct KeyboardMapperConfig final {
    bool repeat_as_press = true;
    bool discrete_on_press_only = true;
};

/**
 * @brief 将 KeyEvent 映射为 TeleopAction
 *
 * @param ev  输入的低层按键事件（KeyEvent）
 * @param cfg 行为配置（通常使用默认值即可）
 * @return    映射得到的控制语义动作；若无匹配/事件不应触发，则返回 TeleopAction::None
 *
 * 注意：
 *  - 本函数不改变任何系统状态，不持有时间，不做节拍。
 *  - 对大小写不敏感（字母键会归一化为大写）。
 *  - 对 ','/'.' 支持全角/半角兼容（若上游能提供对应码点）。
 *  - 单电机测试在这里仅产生语义动作，阻塞 2s 与归中由上层实现。
 */
TeleopAction map_key_event_to_action(const shared::msg::KeyEvent& ev,
                                     const KeyboardMapperConfig& cfg = {}) noexcept;

} // namespace comm_gcs::ipc::keys

#endif // GATEWAY_IPC_KEYS_KEYBOARD_MAPPER_HPP
