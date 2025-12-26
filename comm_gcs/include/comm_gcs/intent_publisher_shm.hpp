#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

namespace shared::msg {
struct ControlIntent; // forward decl; .cpp includes the full header
} // namespace shared::msg

namespace comm_gcs {

/**
 * @brief Publish shared::msg::ControlIntent to shared memory (POSIX shm).
 *
 * Design goals:
 *  - Writer is robust and low-latency (seqlock).
 *  - Payload is POD / trivially-copyable and written via memcpy.
 *  - This publisher contains NO session logic, NO UDP logic, NO protocol logic.
 */
class IntentPublisherShm final {
public:
    // ----------------------------------------------------------------------------
    // Config
    // ----------------------------------------------------------------------------
    struct Config {
        bool        enable   = true;
        std::string shm_name = "/rovctrl_gcs_intent_v1";
        std::size_t shm_size = 0;  // 0 => sizeof(ShmLayout) in .cpp
    };

    // ----------------------------------------------------------------------------
    // Seqlock header (shared memory header)
    // ----------------------------------------------------------------------------
    struct ShmHeader {
        // writer seq (seqlock): odd => writing, even => stable
        std::atomic<std::uint64_t> seq{0};

        // timestamps: mono=steady_clock ns, wall=system_clock ns
        std::uint64_t mono_ns = 0;
        std::uint64_t wall_ns = 0;

        // protocol tagging
        std::uint32_t magic   = 0;
        std::uint32_t version = 0;
    };

    // ----------------------------------------------------------------------------
    // Lifecycle / move-only
    // ----------------------------------------------------------------------------
    IntentPublisherShm() = default;
    ~IntentPublisherShm() noexcept;

    IntentPublisherShm(const IntentPublisherShm&)            = delete;
    IntentPublisherShm& operator=(const IntentPublisherShm&) = delete;

    IntentPublisherShm(IntentPublisherShm&& other) noexcept;
    IntentPublisherShm& operator=(IntentPublisherShm&& other) noexcept;

    // ----------------------------------------------------------------------------
    // Public API
    // ----------------------------------------------------------------------------
    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    /**
     * @brief Publish a ControlIntent snapshot to shared memory.
     * @return true if published (or publisher disabled); false if not initialized/error.
     */
    bool publish(const shared::msg::ControlIntent& intent);

private:
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;

private:
    bool enabled_     = false;
    bool initialized_ = false;
    bool error_flag_  = false;

    std::string shm_name_;
    std::size_t shm_size_ = 0;

#ifndef _WIN32
    int shm_fd_ = -1;

    // defined in .cpp:
    // struct ShmLayout { ShmHeader hdr; shared::msg::ControlIntent intent; };
    struct ShmLayout;
    ShmLayout* shm_ptr_ = nullptr;
#else
    void* shm_handle_ = nullptr;
    void* shm_ptr_    = nullptr;
#endif
};

} // namespace comm_gcs
