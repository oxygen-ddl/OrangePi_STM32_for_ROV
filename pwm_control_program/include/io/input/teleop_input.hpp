#pragma once
#ifndef ROVCTRL_IO_TELEOP_INPUT_HPP
#define ROVCTRL_IO_TELEOP_INPUT_HPP

#include <cstdint>

#include "io/input/input_provider.hpp"              // IInputProvider
#include "control_core/control_intent.hpp"    // control_core::ControlIntent
#include "control_core/control_types.hpp"     // ControlState
#include "control_core/control_mode.hpp"      // ControlMode (single source)

namespace rovctrl::io {

/**
 * @brief Keyboard-based teleoperation input provider.
 *
 * Role:
 *   - Operator-driven input source.
 *   - Translates keyboard events into structured ControlIntent.
 *   - No control law / no safety arbitration.
 */
class TeleopInputProvider final : public IInputProvider {
public:
    TeleopInputProvider() = default;
    ~TeleopInputProvider() override = default;

    bool init() override;

    // Must match IInputProvider::poll(ControlState&, ControlIntent&)
    bool poll(rovctrl::control_core::ControlState&  state,
              rovctrl::control_core::ControlIntent& intent) override;

    void reset() override;

private:
    using ControlIntent = rovctrl::control_core::ControlIntent;

    void fill_teleop_dof(ControlIntent& intent);
    void handle_mode_keys(int key, ControlIntent& intent);
    void handle_safety_keys(int key, ControlIntent& intent);
    void handle_ref_delta_keys(int key, ControlIntent& intent);
    void finalize_intent(ControlIntent& intent);

    // low-level nonblocking key read (implemented in .cpp)
    int read_key_nonblock();

private:
    std::uint64_t seq_{0};

    // these are referenced by your current teleop_input.cpp errors
    bool initialized_{false};
    bool raw_mode_{false};          // legacy behavior toggle (if you still use it in .cpp)
    bool exit_requested_{false};    // legacy exit flag (if you still use it in .cpp)

    std::int64_t last_event_ns_{0}; // used in finalize_intent()
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_TELEOP_INPUT_HPP
