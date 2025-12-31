#include "gateway/IPC/keys/keyboard_mapper.hpp"

namespace comm_gcs::ipc::keys {

namespace {

// ---------- small helpers (local to this TU) ----------

inline bool has_ctrl(std::uint16_t mods) noexcept
{
    return (mods & shared::msg::kModCtrl) != 0;
}

// Continuous DOF keys: Press/Repeat refresh “hold-to-run” semantics.
inline bool is_hold_refresh(const shared::msg::KeyEvent& ev,
                            const KeyboardMapperConfig& cfg) noexcept
{
    if (ev.action == shared::msg::KeyAction::kPress) return true;
    if (cfg.repeat_as_press && ev.action == shared::msg::KeyAction::kRepeat) return true;
    return false;
}

// Discrete keys: edge-trigger by default (Press only).
inline bool is_discrete_trigger(const shared::msg::KeyEvent& ev,
                                const KeyboardMapperConfig& cfg) noexcept
{
    if (!cfg.discrete_on_press_only) {
        return is_hold_refresh(ev, cfg);
    }
    return ev.action == shared::msg::KeyAction::kPress;
}

// Normalize ASCII letters to upper-case; non-ASCII is returned unchanged.
// This keeps mapping tables compact and makes 'm' and 'M' equivalent.
inline std::int32_t normalize_key(std::int32_t k) noexcept
{
    if (k >= 'a' && k <= 'z') return k - ('a' - 'A');
    return k;
}

// Full-width/half-width punctuation compatibility (if upstream provides such codepoints).
inline bool is_comma_key(std::int32_t k) noexcept
{
    // ',' (0x2C) or '，' (U+FF0C = 65292)
    return (k == ',') || (k == 65292);
}

inline bool is_dot_key(std::int32_t k) noexcept
{
    // '.' (0x2E) or '。' (U+3002 = 12290)
    return (k == '.') || (k == 12290);
}

} // namespace

TeleopAction map_key_event_to_action(const shared::msg::KeyEvent& ev,
                                     const KeyboardMapperConfig& cfg) noexcept
{
    const std::int32_t key  = normalize_key(ev.key);
    const bool         ctrl = has_ctrl(ev.mods);

    // Ctrl-combos take precedence to avoid semantic ambiguity (e.g., Ctrl+M vs 'M' center).
    if (ctrl && is_discrete_trigger(ev, cfg)) {
        if (key == ' ') return TeleopAction::EStop;       // Ctrl + Space
        if (key == 'M') return TeleopAction::ClearEStop;  // Ctrl + M/m
    }

    // Arm/Disarm: full-width/half-width compatible punctuation.
    if (is_discrete_trigger(ev, cfg)) {
        if (is_comma_key(key)) return TeleopAction::Arm;     // ',' / '，'
        if (is_dot_key(key))   return TeleopAction::Disarm;  // '.' / '。'
    }

    // Other discrete actions (edge-trigger by default).
    if (is_discrete_trigger(ev, cfg)) {
        if (key == 27)  return TeleopAction::Exit;       // ESC
        if (key == 'Z') return TeleopAction::Help;       // Help (print usage)
        if (key == 'M') return TeleopAction::Center;     // Center DOF (does not touch PWM)

        // Throttle step (global scale) is treated as discrete to avoid runaway on repeat.
        if (key == '=' || key == '+') return TeleopAction::ThrottleUp;
        if (key == '-' || key == '_') return TeleopAction::ThrottleDown;

        // ------------------------------
        // Single-motor test (discrete)
        // ------------------------------
        // NOTE: execution is handled by upper layer (2s blocking + auto stop/center).
        switch (key) {
            case '1': return TeleopAction::MotorTest1;
            case '2': return TeleopAction::MotorTest2;
            case '3': return TeleopAction::MotorTest3;
            case '4': return TeleopAction::MotorTest4;
            case '5': return TeleopAction::MotorTest5;
            case '6': return TeleopAction::MotorTest6;
            case '7': return TeleopAction::MotorTest7;
            case '8': return TeleopAction::MotorTest8;
            case '0': return TeleopAction::MotorTestStop; // recommended
            default: break;
        }
    }

    // Continuous DOF: Press/Repeat refresh; timeout/zeroing is handled by the state machine.
    if (!is_hold_refresh(ev, cfg)) return TeleopAction::None;

    // Mapping aligned with legacy teleop_keyboard.cpp help:
    //   W/S: surge +/-
    //   A/D: sway  -/+
    //   Q/E: yaw   +/-
    //   H/G: heave +/-
    //   R/T: roll  +/-
    //   F/V: pitch +/-
    switch (key) {
        case 'W': return TeleopAction::SurgePos;
        case 'S': return TeleopAction::SurgeNeg;

        case 'A': return TeleopAction::SwayNeg;
        case 'D': return TeleopAction::SwayPos;

        case 'Q': return TeleopAction::YawPos;
        case 'E': return TeleopAction::YawNeg;

        case 'H': return TeleopAction::HeavePos;
        case 'G': return TeleopAction::HeaveNeg;

        case 'R': return TeleopAction::RollPos;
        case 'T': return TeleopAction::RollNeg;

        case 'F': return TeleopAction::PitchPos;
        case 'V': return TeleopAction::PitchNeg;

        default: break;
    }

    return TeleopAction::None;
}


} // namespace comm_gcs::ipc::keys
