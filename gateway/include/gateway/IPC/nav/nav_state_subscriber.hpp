#pragma once
/**
 * @file   nav_state_subscriber.hpp
 * @brief  从导航进程共享内存读取导航状态（NavState）的订阅器。
 *
 * 设计目标：
 *  - 仅依赖标准库 + shared/msg/nav_state.hpp；
 *  - 读端为只读、无锁，多读者安全；
 *  - 使用 seq 奇偶机制，避免读取到“写入中”的中间状态；
 *  - 面向控制循环提供简单接口：read_latest(NavState&).
 */

#include <cstdint>
#include <cstddef>
#include <string>

#include "shared/msg/nav_state.hpp"   // shared::msg::NavState

namespace rovctrl::io {

/**
 * @class NavStateSubscriber
 * @brief 通过 POSIX 共享内存从导航进程订阅 NavState。
 *
 * 典型用法：
 * @code
 *   rovctrl::io::NavStateSubscriber sub;
 *   if (!sub.init("/rov_nav_state_v1")) {
 *       // 记录警告，但不强制退出控制程序
 *   }
 *
 *   shared::msg::NavState nav{};
 *   if (sub.ok() && sub.read_latest(nav)) {
 *       // 使用 nav 作为当前导航状态
 *   }
 * @endcode
 */
class NavStateSubscriber {
public:
    NavStateSubscriber() = default;
    ~NavStateSubscriber() noexcept;

    NavStateSubscriber(const NavStateSubscriber&)            = delete;
    NavStateSubscriber& operator=(const NavStateSubscriber&) = delete;

    NavStateSubscriber(NavStateSubscriber&& other) noexcept;
    NavStateSubscriber& operator=(NavStateSubscriber&& other) noexcept;

    /**
     * @brief 初始化订阅器并映射共享内存。
     *
     * @param shm_name 共享内存名称，默认 "/rov_nav_state_v1"。
     *                 必须与导航进程的 NavStatePublisher 使用的名称一致。
     *
     * @return true  初始化成功；
     * @return false 初始化失败（可通过 ok()/has_error() 进一步判断）。
     *
     * 注意：
     *  - 这里只尝试以只读方式打开既有的共享内存；
     *  - 如果导航进程尚未创建共享内存，则 init 会失败。
     */
    bool init(const std::string& shm_name = "/rov_nav_state_v1");

    /// 关闭共享内存映射并重置内部状态。
    void shutdown() noexcept;

    /// 是否初始化成功且当前无致命错误。
    bool ok() const noexcept { return initialized_ && !error_flag_; }

    /// 是否已经检测到错误（例如 shm_open/mmap 失败）。
    bool has_error() const noexcept { return error_flag_; }

    /**
     * @brief 读取一帧最新的导航状态。
     *
     * 读取过程使用 seq 奇偶机制来保证一致性：
     *  - 读之前和读之后各检查一次 seq；
     *  - 期间若 seq 发生变化或为奇数（写入中），认为数据不稳定，返回 false；
     *  - 只有在“读前后 seq 相同且为偶数”时，才拷贝 NavState 并返回 true。
     *
     * @param[out] out_state 若成功，填充为最新 NavState。
     *
     * @return true  成功读取到一帧稳定数据；
     * @return false 当前没有稳定数据（可在下一周期重试）。
     */
    bool read_latest(shared::msg::NavState& out_state) const;

private:
    /// 与 nav_core 端保持一致的共享内存头部格式。
    struct ShmHeader {
        std::uint32_t magic;    ///< 魔数：'NAV1'
        std::uint32_t version;  ///< 版本号：当前为 1
        std::uint64_t seq;      ///< 序号：奇数=写入中，偶数=稳定
        std::int64_t  mono_ns;  ///< 单调时间（ns）
        std::int64_t  wall_ns;  ///< 墙钟时间（ns）
    };

    /// 共享内存整体布局：头部 + 导航状态。
    struct ShmLayout {
        ShmHeader             hdr;
        shared::msg::NavState state;
    };

    /// 打开并映射共享内存（内部使用）。
    bool open_shm(const std::string& shm_name);

    /// 取消映射并关闭共享内存（内部使用）。
    void close_shm() noexcept;

private:
    bool        initialized_{false};
    bool        error_flag_{false};
    std::string shm_name_;
    std::size_t shm_size_{0};

#ifndef _WIN32
    int         shm_fd_{-1};
    ShmLayout*  shm_ptr_{nullptr};
#else
    void*       shm_handle_{nullptr};
    ShmLayout*  shm_ptr_{nullptr};
#endif
};

} // namespace rovctrl::io
