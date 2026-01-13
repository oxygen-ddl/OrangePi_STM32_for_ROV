#pragma once

#include "io/input/input_provider.hpp"  // IInputProvider / InputProviderPtr

#include <cstddef>
#include <memory>
#include <string>

namespace rovctrl::io::input {

/**
 * @brief InputProvider implementation: subscribe GCS intent from shared memory
 * and convert wire intent (shared::msg::ControlIntent) to internal intent
 * (control_core::ControlIntent).
 *
 * Integration intent:
 *  - comm_gcs process publishes shared::msg::ControlIntent into shm
 *  - pwm_control_program reads shm here and participates as one input source
 *    in MultiInputProvider.
 *
 * Polling policy:
 *  - Non-fatal on read failures (comm_gcs not running, shm missing, retry, etc.)
 *  - On failure: output is cleared payload; return true.
 */
class GcsShmInputProvider final : public rovctrl::io::IInputProvider {
public:
    struct Config {
        bool        enable    = true;

        // Must match comm_gcs publisher shm name (default as requested)
        std::string shm_name  = "/rovctrl_gcs_intent_v1";

        // 0 => use sizeof(layout) / publisher size (subscriber uses fstat)
        std::size_t shm_size  = 0;

        // allow pwm_control_program start before comm_gcs
        bool        lazy_init = true;

        // If wire intent stamp_ns==0, provider will fill with local now_ns().
        bool        fill_local_stamp_if_missing = true;
    };

    explicit GcsShmInputProvider(Config cfg);
    ~GcsShmInputProvider() override;

    GcsShmInputProvider(const GcsShmInputProvider&)            = delete;
    GcsShmInputProvider& operator=(const GcsShmInputProvider&) = delete;

    GcsShmInputProvider(GcsShmInputProvider&&) noexcept;
    GcsShmInputProvider& operator=(GcsShmInputProvider&&) noexcept;

    bool init() override;
    void reset() override;

    bool poll(rovctrl::control_core::ControlState& state,
              rovctrl::control_core::ControlIntent& out) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rovctrl::io::input
