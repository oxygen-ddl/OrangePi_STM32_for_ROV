#ifndef ROVCTRL_IO_INPUT_INTENT_SINK_HPP
#define ROVCTRL_IO_INPUT_INTENT_SINK_HPP

#include "control_core/control_intent.hpp"

namespace rovctrl::io {

class IIntentSink {
public:
    virtual ~IIntentSink() = default;
    virtual void submit_gcs_intent(const rovctrl::control_core::ControlIntent& intent) = 0;
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_INPUT_INTENT_SINK_HPP
