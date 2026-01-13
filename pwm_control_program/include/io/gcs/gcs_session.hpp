#pragma once
#ifndef ROVCTRL_IO_GCS_SESSION_BRIDGE_HPP
#define ROVCTRL_IO_GCS_SESSION_BRIDGE_HPP

// pwm_control_program 内部统一使用该桥接头，避免直接散落引用 gateway 头文件。
// 后期如果 gateway 目录/命名空间调整，只需修改这里。

#include "gateway/session/gcs_session.hpp"

// 可选：如果你希望在 pwm_control_program 命名空间下提供别名，降低改动面
namespace rovctrl::io::gcs_bridge {

using GcsSession = comm_gcs::session::GcsSession;

// 如果后面还会用到这些类型，也建议一起桥接出来：
// using PacketHeader = ...;
// using SessionStats = ...;

} // namespace rovctrl::io::gcs_bridge

#endif // ROVCTRL_IO_GCS_SESSION_BRIDGE_HPP
