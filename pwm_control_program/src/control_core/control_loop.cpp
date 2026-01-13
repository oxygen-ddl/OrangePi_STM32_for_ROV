/**
 * @file   control_loop.cpp
 * @brief  控制主循环实现（ControlIntent + Guard 仲裁版 / Path A）- glue
 */

#include "control_core/control_loop.hpp"

#include <utility>

namespace rovctrl::control_core {

// NavSub / PwmLogImpl 在其他 cpp 内实现，这里不需要 include 其依赖
ControlLoop::ControlLoop(const Config&                  cfg,
                         rovctrl::platform::PwmClient&  pwm,
                         rovctrl::io::InputProviderPtr  input,
                         ControllerManager&&            ctrl_mgr,
                         std::atomic_bool*              external_stop_flag)
    : cfg_(cfg)
    , pwm_(pwm)
    , input_(std::move(input))
    , ctrl_mgr_(std::move(ctrl_mgr))
    , external_stop_(external_stop_flag)
    , guard_(cfg.guard_cfg)
{
    // nav_sub_ 在 update_nav_feedback_() 中懒初始化
    // pwm_logger_ 在 run() 中按 enable 开关创建
}

} // namespace rovctrl::control_core
