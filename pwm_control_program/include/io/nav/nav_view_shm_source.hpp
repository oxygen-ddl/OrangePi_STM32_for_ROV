#pragma once
#ifndef ROVCTRL_IO_NAV_VIEW_SHM_SOURCE_HPP
#define ROVCTRL_IO_NAV_VIEW_SHM_SOURCE_HPP

#include <cstdint>
#include <string>

#include "io/nav/nav_state_view.hpp"     // 提供 rovctrl::io::NavStateView struct
#include "io/nav/nav_state_source.hpp"   // 提供 INavStateSource 接口
#include "io/nav/nav_view_subscriber_shm.hpp"


namespace rovctrl::io::nav {

/**
 * @brief NavStateView source backed by shared memory.
 *
 * Role:
 *  - Adapter from NavViewSubscriberShm (IPC / SHM)
 *    to INavStateSource (control-side interface).
 *
 * Policy:
 *  - Optional max-age filtering
 *  - Optional validity check
 *  - Lazy init supported
 */
class NavViewShmSource final : public rovctrl::io::INavStateSource {
public:
    struct Config {
        bool          enable        = true;
        std::string   shm_name      = "/rovctrl_nav_view_v1";
        std::uint32_t max_age_ms    = 250;
        bool          require_valid = true;
        bool          lazy_init     = true;
    };

    NavViewShmSource();
    ~NavViewShmSource() noexcept override;

    // ---------- configuration ----------
    bool init(const Config& cfg);

    // ---------- INavStateSource ----------
    bool init() override;   // uses current cfg_
    bool ok() const noexcept override;
    bool read_latest(rovctrl::io::NavStateView& out) override;

    // ---------- lifecycle ----------
    void shutdown() noexcept;

private:
    static std::uint64_t now_mono_ns_() noexcept;

private:
    bool enabled_{false};
    bool ok_{false};
    Config cfg_{};

    NavViewSubscriberShm sub_{};
};

} // namespace rovctrl::io::nav

#endif // ROVCTRL_IO_NAV_VIEW_SHM_SOURCE_HPP
