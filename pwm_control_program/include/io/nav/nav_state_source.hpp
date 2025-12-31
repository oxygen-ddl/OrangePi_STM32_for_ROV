#pragma once
#ifndef ROVCTRL_IO_NAV_STATE_SOURCE_HPP
#define ROVCTRL_IO_NAV_STATE_SOURCE_HPP

#include "io/nav/nav_state_view.hpp"

namespace rovctrl::io {

class INavStateSource {
public:
    virtual ~INavStateSource() noexcept = default;

    virtual bool init() = 0;
    virtual bool ok() const noexcept = 0;
    virtual bool read_latest(NavStateView& out) = 0; // struct version
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_NAV_STATE_SOURCE_HPP
