#pragma once
#ifndef ROVCTRL_IO_INPUT_INTENT_SUBSCRIBER_SHM_HPP
#define ROVCTRL_IO_INPUT_INTENT_SUBSCRIBER_SHM_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "shared/msg/control_intent.hpp"
#include "shared/shm/control_intent_shm.hpp"

namespace rovctrl::io::input {

class IntentSubscriberShm final {
public:
    struct Config {
        bool        enable    = true;
        std::string shm_name  = "/rovctrl_intent_mux_v1"; // 由你们约定的默认值（例如 mux 输出）
        bool        lazy_init = true; // SHM 不存在：不报错，后续轮询重试 open
        std::size_t shm_size  = 0;    // 0 => sizeof(shared::shm::IntentShmLayout)
        bool        auto_recover = true; // ABI mismatch / mmap fail 后是否允许后续重试
    };

    IntentSubscriberShm() = default;
    ~IntentSubscriberShm() noexcept;

    IntentSubscriberShm(const IntentSubscriberShm&) = delete;
    IntentSubscriberShm& operator=(const IntentSubscriberShm&) = delete;

    IntentSubscriberShm(IntentSubscriberShm&&) noexcept;
    IntentSubscriberShm& operator=(IntentSubscriberShm&&) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }
    bool healthy() const noexcept { return initialized_ && !error_flag_; }

    // One-shot poll: returns latest intent snapshot if available (stable).
    // Optionally returns publisher timestamps from header.
    std::optional<shared::msg::ControlIntent> poll(std::uint64_t* out_mono_ns = nullptr,
                                                  std::uint64_t* out_wall_ns = nullptr) noexcept;
    std::optional<shared::msg::ControlIntent> poll_wire(std::uint64_t* out_mono_ns = nullptr,
                                                    std::uint64_t* out_wall_ns = nullptr) noexcept
    {
        return poll(out_mono_ns, out_wall_ns);
    }

private:
    bool try_open_() noexcept;
    bool map_() noexcept;
    void close_() noexcept;

    bool validate_contract_() noexcept;

    // seqlock read
    bool read_consistent_(shared::msg::ControlIntent& out,
                          std::uint64_t* out_mono_ns,
                          std::uint64_t* out_wall_ns) noexcept;

private:
    Config cfg_{};

    bool enabled_{false};
    bool initialized_{false};
    bool error_flag_{false};

#ifndef _WIN32
    int shm_fd_{-1};
#else
    void* shm_handle_{nullptr};
#endif

    std::size_t shm_size_{0};
    void* map_ptr_{nullptr};

    shared::shm::IntentShmLayout* shm_{nullptr};
};

} // namespace rovctrl::io::input

#endif // ROVCTRL_IO_INPUT_INTENT_SUBSCRIBER_SHM_HPP
