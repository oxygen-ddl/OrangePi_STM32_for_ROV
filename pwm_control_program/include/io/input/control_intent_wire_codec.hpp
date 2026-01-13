#pragma once
#ifndef ROVCTRL_IO_INPUT_CONTROL_INTENT_WIRE_CODEC_HPP
#define ROVCTRL_IO_INPUT_CONTROL_INTENT_WIRE_CODEC_HPP

#include <cstdint>

#include "control_core/control_intent.hpp"
#include "shared/msg/control_intent.hpp"

namespace rovctrl::io::input {

// Return false if wire version mismatch / invalid.
bool decode_control_intent(const shared::msg::ControlIntent& w,
                           rovctrl::control_core::ControlIntent& c) noexcept;

// Encode core intent into wire intent (for publishing to SHM / gateway).
// Note: source_id/source_prio can be set by caller; this function focuses on payload+flags.
bool encode_control_intent(const rovctrl::control_core::ControlIntent& c,
                           shared::msg::ControlIntent& w) noexcept;

} // namespace rovctrl::io::input

#endif
