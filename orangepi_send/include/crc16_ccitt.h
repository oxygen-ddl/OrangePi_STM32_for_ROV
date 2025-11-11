#ifndef PROTOCOL_CRC16_CCITT_H_
#define PROTOCOL_CRC16_CCITT_H_

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string_view>

namespace proto {

// CRC-16/CCITT-FALSE parameters:
// poly = 0x1021, init = 0xFFFF, xorout = 0x0000, refin=false, refout=false.
struct Crc16Ccitt {
    static constexpr std::uint16_t kInit   = 0xFFFF;
    static constexpr std::uint16_t kXorOut = 0x0000;

    // 一次性计算
    static std::uint16_t compute(const std::uint8_t* data, std::size_t len);
    static std::uint16_t compute(const std::vector<std::uint8_t>& buf) {
        return compute(buf.data(), buf.size());
    }
    static std::uint16_t compute(std::string_view sv) {
        return compute(reinterpret_cast<const std::uint8_t*>(sv.data()), sv.size());
    }

    // 增量更新：可用于“先放头部，再追加payload”的场景
    static std::uint16_t update(std::uint16_t crc, const std::uint8_t* data, std::size_t len);
    static std::uint16_t update(std::uint16_t crc, const std::vector<std::uint8_t>& buf) {
        return update(crc, buf.data(), buf.size());
    }
    static std::uint16_t update(std::uint16_t crc, std::string_view sv) {
        return update(crc, reinterpret_cast<const std::uint8_t*>(sv.data()), sv.size());
    }

    // 初始化与收尾（这里 xorout=0）
    static constexpr std::uint16_t init()    { return kInit; }
    static constexpr std::uint16_t finalize(std::uint16_t crc) { return static_cast<std::uint16_t>(crc ^ kXorOut); }
};

} // namespace proto

#endif // PROTOCOL_CRC16_CCITT_H_
