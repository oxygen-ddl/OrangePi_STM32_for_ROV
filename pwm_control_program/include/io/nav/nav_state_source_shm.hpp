#pragma once
#ifndef ROVCTRL_IO_NAV_NAV_STATE_SOURCE_SHM_HPP
#define ROVCTRL_IO_NAV_NAV_STATE_SOURCE_SHM_HPP

#include <memory>
#include <string>

#include "io/nav/nav_state_source.hpp"
#include "io/nav/nav_state_view.hpp"

namespace rovctrl::io {

class NavStateSourceShm final : public INavStateSource {
public:
    explicit NavStateSourceShm(const std::string& shm_name);
    ~NavStateSourceShm() override;

    bool init() override;
    bool ok() const noexcept override;
    bool read_latest(NavStateView& out) override;
    void reset() noexcept;

private:
    struct Impl;
    std::string shm_name_;
    std::unique_ptr<Impl> impl_;
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_NAV_NAV_STATE_SOURCE_SHM_HPP
