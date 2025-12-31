#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string_view>

namespace comm_gcs {

using Byte = std::uint8_t;

struct BytesView {
    const Byte* data{nullptr};
    std::size_t size{0};

    constexpr bool empty() const noexcept { return size == 0; }
};

inline BytesView as_bytes(std::string_view sv) noexcept {
    return BytesView{reinterpret_cast<const Byte*>(sv.data()), sv.size()};
}

inline std::vector<Byte> to_vec(BytesView v) {
    return std::vector<Byte>(v.data, v.data + v.size);
}

} // namespace comm_gcs
