#pragma once

#include <cstdint>

#include "shared/msg/control_intent.hpp"  // wire type

namespace rovctrl::control_core {
struct ControlIntent;  // forward decl OK
} // namespace rovctrl::control_core

namespace rovctrl::io {

/**
 * @brief Convert wire intent (shared::msg) to internal control_core::ControlIntent.
 *
 * @return true  Conversion succeeded; out is written (payload only, or full).
 * @return false Unsupported wire version or invalid payload; out is untouched.
 *
 * Notes:
 * - Keep this header lightweight: no inclusion of control_core headers.
 * - Implementation file includes control_core/control_intent.hpp.
 */
bool to_internal_intent(const shared::msg::ControlIntent& w,
                        rovctrl::control_core::ControlIntent& out) noexcept;

} // namespace rovctrl::io
