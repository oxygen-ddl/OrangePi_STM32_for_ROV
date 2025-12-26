#pragma once
#ifndef ROVCTRL_IO_CONTROL_INTENT_HPP
#define ROVCTRL_IO_CONTROL_INTENT_HPP

/**
 * @file  io/control_intent.hpp
 * @brief 兼容层：历史路径下的 ControlIntent 别名
 *
 * 说明：
 * - ControlIntent 的唯一真源在 control_core/control_intent.hpp
 * - 本文件仅提供 rovctrl::io::ControlIntent -> rovctrl::control_core::ControlIntent 的别名，
 *   以兼容旧 include 路径与旧命名空间写法。
 */

#include "control_core/control_intent.hpp"

namespace rovctrl::io {

namespace cc = rovctrl::control_core;

// 兼容别名：不再定义第二份结构体，避免类型不一致
using ControlIntent = cc::ControlIntent;

} // namespace rovctrl::io

#endif // ROVCTRL_IO_CONTROL_INTENT_HPP
