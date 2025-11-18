#ifndef PWMFRAMEBUILDER_H
#define PWMFRAMEBUILDER_H

/**
 * @file    PwmFrameBuilder.h
 * @brief   构建与解析“香橙派 ↔ STM32”控制链路的数据帧（升级版协议）。
 *
 * 协议版本：v1（推荐）
 *   线缆格式（所有多字节字段均为网络序：大端 big-endian）
 *   ┌────────┬────────┬────────┬────────────┬───────────┬───────────┬──────────┐
 *   │  SOF   │  VER   │ MSG_ID │   SEQ(u16) │ TICKS(u32)│ LEN(u16)  │ PAYLOAD  │
 *   │ 0xAA55 │  0x01  │ 见枚举 │ 递增序列号 │ 发送毫秒  │ 负载字节数 │ LEN字节   │
 *   ├────────┴────────┴────────┴────────────┴───────────┴───────────┼──────────┤
 *   │                               CRC16-CCITT (u16)               │
 *   └────────────────────────────────────────────────────────────────┴──────────┘
 *   * CRC16-CCITT/FALSE: poly=0x1021, init=0xFFFF, xorout=0x0000, refin/refout=false
 *   * CRC 计算范围：从 VER 起，到 PAYLOAD 末；不含 SOF 与 CRC 字段本身。
 *
 * 常用 MSG_ID 与负载：
 *   - PWM_CMD (0x01)：负载为 8×uint16（大端），每通道 0..10000，语义：[-1..+1] 映射→[1000..2000us]
 *   - HEARTBEAT (0x10)：负载为空；对端应回 HEARTBEAT_ACK（0x11），负载为空
 *   - STATUS (0x20)：预留为状态上报结构（建议 JSON/定长结构均可，后续迭代）
 *
 * 旧版协议（v0，短期兼容，可选）
 *   SOF=0xAA55, frame_id=0x01, data_length=16, 8×uint16, 8-bit sum 校验；
 *   心跳帧 SOF=0x55AA, 时间戳(u32, 秒)，8-bit sum 校验。
 *
 * 协作规范：
 *   1) 严格遵守本头文件里对字节序与字段含义的约定；
 *   2) `.cpp` 中实现必须对所有输入做边界检查（LEN上限、CRC、VER/MSG_ID有效性）；
 *   3) 如需扩展，请新增 MSG_ID 或扩展 PAYLOAD，避免改变头字段含义。
 */

#include <cstdint>
#include <vector>
#include <string_view>
#include <array>
#include <optional>

// 升级协议需要 CRC16-CCITT 实现（见你新增的 crc16_ccitt.{h,cpp}）
#include "crc16_ccitt.h"

// === 如需短期兼容旧协议 v0，请保留该宏；完全迁移后可删除 ===
// #define PWM_PROTO_ENABLE_V0_COMPAT 1

class PwmFrameBuilder {
public:
    // ========================= 公共常量与类型 =========================

    /// 协议版本号（头字段 VER）
    static constexpr std::uint8_t kProtoVerV1 = 0x01;

    /// 固定的帧头（SOF）
    static constexpr std::uint16_t kSof = 0xAA55;

    /// PWM 通道与取值范围（控制语义层）
    static constexpr std::size_t  kPwmChannelCount = 8;
    static constexpr std::uint16_t kPwmMaxValue    = 10000; // 映射 [-1..+1]→[-5000..+5000] + 5000

    /// 消息类型（MSG_ID）
    enum class MsgId : std::uint8_t {
        PWM_CMD        = 0x01, ///< 负载=8×u16（大端），每通道 0..10000
        HEARTBEAT      = 0x10, ///< 负载为空；对端需回 HEARTBEAT_ACK
        HEARTBEAT_ACK  = 0x11, ///< 负载为空；心跳确认
        STATUS         = 0x20, ///< 预留：状态上报（负载结构待定）
    };

    // ========================= 线缆结构（打包） =========================
    // 说明：结构体只用于内存布局参考，实际构帧/解析请按字节序逐字节处理，避免未定义对齐问题。

#pragma pack(push, 1)
    struct HeaderV1 {
        std::uint16_t sof_be;   ///< 固定 0xAA55（大端）
        std::uint8_t  ver;      ///< 协议版本，固定 0x01
        std::uint8_t  msg_id;   ///< 见 MsgId
        std::uint16_t seq_be;   ///< 发送方递增序列号（大端）
        std::uint32_t ticks_be; ///< 发送方毫秒时间戳（大端，since boot）
        std::uint16_t len_be;   ///< 负载字节数（大端）
        // 紧随其后：payload[len] + crc16_be
    };
#pragma pack(pop)

    static_assert(sizeof(HeaderV1) == 2 + 1 + 1 + 2 + 4 + 2, "HeaderV1 layout changed!");

    // ========================= 构帧（v1） =========================

    /**
     * @brief 构建 PWM 指令帧（v1）
     * @param pwm_values 8 通道，每通道 0..10000（将被裁剪）
     * @param seq        序列号（建议每次发送自增）
     * @param ticks_ms   本地毫秒时间戳（建议 steady_clock）
     * @return 已打包好的字节数组（含 SOF、头、payload、CRC）
     */
    static std::vector<std::uint8_t>
    buildPwmCmdFrameV1(const std::array<std::uint16_t, kPwmChannelCount>& pwm_values,
                       std::uint16_t seq,
                       std::uint32_t ticks_ms);

    /**
     * @brief 构建心跳帧（v1）
     * @param seq      序列号（可自增）
     * @param ticks_ms 毫秒时间戳
     */
    static std::vector<std::uint8_t>
    buildHeartbeatFrameV1(std::uint16_t seq, std::uint32_t ticks_ms);

    // ========================= 解析（v1） =========================

    /**
     * @brief 判定并解析心跳 ACK（v1）
     * @param frame   原始帧
     * @param seq_rx  [输出] 对端返回的 SEQ
     * @param ticks_rx[输出] 对端填入的 TICKS32（毫秒）
     * @return true=是合法的 HEARTBEAT_ACK；false=不是或校验失败
     */
    static bool parseHeartbeatAckV1(std::string_view frame,
                                    std::uint16_t& seq_rx,
                                    std::uint32_t& ticks_rx);

    /**
     * @brief 判定并解析 STATUS（v1，预留扩展）
     * @param frame 原始帧
     * @return payload（若合法且为 STATUS），否则 std::nullopt
     */
    static std::optional<std::string_view> parseStatusV1(std::string_view frame);

    // ========================= 轻量工具 =========================

    /// @return 8 通道个数（编译期常量）
    static constexpr std::size_t getPwmChannelCount() { return kPwmChannelCount; }

    /// @return 控制层定义的最大 PWM 值（0..10000）
    static constexpr std::uint16_t getMaxPwmValue() { return kPwmMaxValue; }

    /// 快速判断是否为 v1 帧（仅看 SOF 与 VER，不做 CRC）
    static bool looksLikeV1Frame(std::string_view frame);

#ifdef PWM_PROTO_ENABLE_V0_COMPAT
    // ========================= 旧协议（v0，可选兼容） =========================
#pragma pack(push, 1)
    struct PwmDataFrameV0 {
        std::uint16_t frame_header;   ///< 0xAA55
        std::uint8_t  frame_id;       ///< 0x01
        std::uint8_t  data_length;    ///< 16
        std::uint16_t pwm_data[8];    ///< 8×uint16，大端
        std::uint8_t  checksum;       ///< 8-bit 累加和
    };
    struct HeartbeatFrameV0 {
        std::uint16_t frame_header;   ///< 0x55AA
        std::uint32_t timestamp_s;    ///< 秒级时间戳（本地语义，不带时区）
        std::uint8_t  checksum;       ///< 8-bit 累加和
    };
#pragma pack(pop)

    static std::vector<std::uint8_t>
    buildPwmCmdFrameV0(const std::array<std::uint16_t, kPwmChannelCount>& pwm_values);

    static std::vector<std::uint8_t>
    buildHeartbeatFrameV0(std::uint32_t timestamp_s);

    static bool parseHeartbeatFrameV0(std::string_view data, std::uint32_t& timestamp_s);

    static bool looksLikeV0PwmFrame(std::string_view frame);
#endif // PWM_PROTO_ENABLE_V0_COMPAT

private:
    // ========================= 内部工具（仅声明，.cpp 实现） =========================

    /// 将 u16 写为大端追加到缓冲区
    static void appendU16BE(std::vector<std::uint8_t>& buf, std::uint16_t v);

    /// 将 u32 写为大端追加到缓冲区
    static void appendU32BE(std::vector<std::uint8_t>& buf, std::uint32_t v);

    /// 从大端字节序读取 u16（调用前确保长度）
    static std::uint16_t readU16BE(const std::uint8_t* p);

    /// 从大端字节序读取 u32（调用前确保长度）
    static std::uint32_t readU32BE(const std::uint8_t* p);

    /// 将 PWM 值裁剪到 [0..kPwmMaxValue]
    static std::uint16_t clampPwm(std::uint16_t v);

    /// 校验 8 通道向量尺寸并裁剪（若尺寸不对，.cpp 中应抛出或返回空帧）
    static bool validatePwmArray(const std::array<std::uint16_t, kPwmChannelCount>& pwm_values);

#ifdef PWM_PROTO_ENABLE_V0_COMPAT
    /// 旧协议 v0：8-bit 求和校验
    static std::uint8_t sum8(const void* data, std::size_t len);
#endif
};

/* ========================= 使用示例 =========================
#include "PwmFrameBuilder.h"
#include <chrono>

void example_send_pwm(int sock, const sockaddr_in& peer) {
    using Clock = std::chrono::steady_clock;
    static std::uint16_t seq = 0;

    std::array<std::uint16_t, PwmFrameBuilder::getPwmChannelCount()> pwm{};
    // 填充你的 8 通道控制量（0..10000）
    pwm[0] = 5000; // 中位
    // ...

    const auto ticks_ms = static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count()
    );

    auto frame = PwmFrameBuilder::buildPwmCmdFrameV1(pwm, ++seq, ticks_ms);
    // sendto(sock, frame.data(), frame.size(), 0, (sockaddr*)&peer, sizeof(peer));
}

bool example_parse_hb_ack(std::string_view rx) {
    std::uint16_t seq_rx = 0;
    std::uint32_t ticks_rx = 0;
    if (PwmFrameBuilder::parseHeartbeatAckV1(rx, seq_rx, ticks_rx)) {
        // 计算 RTT 等
        return true;
    }
    return false;
}
================================================================ */

#endif // PWMFRAMEBUILDER_H
