#include <cstdint>
#include <cstddef>

namespace rovctrl::io::gcs {

static inline std::uint32_t crc32c_sw(const std::uint8_t* data, std::size_t len) noexcept
{
    // CRC-32C (Castagnoli) reflected polynomial: 0x82F63B78
    static std::uint32_t table[256];
    static bool inited = false;

    if (!inited) {
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1) ? (0x82F63B78u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        inited = true;
    }

    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

std::uint32_t crc32c(const void* data, std::size_t len) noexcept
{
    if (!data || len == 0) return 0;
    return crc32c_sw(reinterpret_cast<const std::uint8_t*>(data), len);
}

} // namespace rovctrl::io::gcs
