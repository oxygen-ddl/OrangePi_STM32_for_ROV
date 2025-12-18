#pragma once
#ifndef ROVCTRL_IO_MULTI_INPUT_PROVIDER_HPP
#define ROVCTRL_IO_MULTI_INPUT_PROVIDER_HPP

#include <cstdint>
#include <memory>

#include "io/input/input_provider.hpp"  // IInputProvider, InputProviderPtr

namespace rovctrl::io {

namespace cc = rovctrl::control_core;

class MultiInputProvider final : public IInputProvider {
public:
    struct Config {
        bool          gcs_priority   = true;
        std::uint32_t default_ttl_ms = 200;
        Config() = default;
    };

    MultiInputProvider(InputProviderPtr teleop,
                       InputProviderPtr gcs);

    MultiInputProvider(InputProviderPtr teleop,
                       InputProviderPtr gcs,
                       Config cfg);

    bool init() override;
    bool poll(cc::ControlState&  state,
              cc::ControlIntent& intent) override;
    void reset() override;

private:
    // 判断一个 intent 是否“携带有效载荷”（不看 seq/stamp/ttl）
    static bool has_payload_(const cc::ControlIntent& in) noexcept;

private:
    InputProviderPtr teleop_;
    InputProviderPtr gcs_;
    Config cfg_{};

    std::uint64_t seq_{0};
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_MULTI_INPUT_PROVIDER_HPP
