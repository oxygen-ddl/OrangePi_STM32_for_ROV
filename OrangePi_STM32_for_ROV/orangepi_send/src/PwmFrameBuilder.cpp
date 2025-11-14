#include "PwmFrameBuilder.h"
#include <algorithm>
#include <cstring>
#include <stdexcept>

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;
using std::size_t;
using std::string_view;
using std::vector;
using std::array;

namespace {

// 最小帧长（v1，无负载时）：SOF(2)+VER(1)+MSG(1)+SEQ(2)+TICKS(4)+LEN(2)+CRC(2) = 14
constexpr size_t kMinFrameLenV1 = 14;

// 工具：检查 frame 是否至少包含一个完整的 v1 头+CRC（payload 可为 0）
inline bool hasMinHeaderV1(string_view frame) {
    return frame.size() >= kMinFrameLenV1;
}

} // namespace

// ========================= private helpers =========================
void PwmFrameBuilder::appendU16BE(vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
}

void PwmFrameBuilder::appendU32BE(vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
}

uint16_t PwmFrameBuilder::readU16BE(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t PwmFrameBuilder::readU32BE(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8)  |
            static_cast<uint32_t>(p[3]);
}

uint16_t PwmFrameBuilder::clampPwm(uint16_t v) {
    return (v > kPwmMaxValue) ? kPwmMaxValue : v;
}

bool PwmFrameBuilder::validatePwmArray(const array<uint16_t, kPwmChannelCount>& pwm_values) {
    // 固定 8 通道；仅裁剪，不判错误（更友好）
    (void)pwm_values;
    return true;
}

// ========================= v1 构帧 =========================
vector<uint8_t>
PwmFrameBuilder::buildPwmCmdFrameV1(const array<uint16_t, kPwmChannelCount>& pwm_values,
                                    uint16_t seq,
                                    uint32_t ticks_ms) {
    if (!validatePwmArray(pwm_values)) {
        throw std::invalid_argument("PWM array invalid");
    }

    // 预估容量：SOF(2)+hdr(1+1+2+4+2)+payload(16)+CRC(2) = 28
    vector<uint8_t> buf;
    buf.reserve(28);

    // 1) SOF
    appendU16BE(buf, kSof);

    // 2) 头部（从 VER 开始要参与 CRC）
    const size_t ver_offset = buf.size();
    buf.push_back(kProtoVerV1);                             // VER
    buf.push_back(static_cast<uint8_t>(MsgId::PWM_CMD));    // MSG_ID
    appendU16BE(buf, static_cast<uint16_t>(seq));           // SEQ (BE)
    appendU32BE(buf, static_cast<uint32_t>(ticks_ms));      // TICKS32 (BE)
    appendU16BE(buf, static_cast<uint16_t>(kPwmChannelCount * 2)); // LEN=16 (BE)

    // 3) PAYLOAD：8×u16（大端），每通道 0..10000（裁剪）
    for (size_t i = 0; i < kPwmChannelCount; ++i) {
        appendU16BE(buf, clampPwm(pwm_values[i]));
    }

    // 4) CRC16：覆盖 [VER..PAYLOAD_END]
    const uint16_t crc = proto::Crc16Ccitt::compute(&buf[ver_offset], buf.size() - ver_offset);
    appendU16BE(buf, crc);

    return buf;
}

vector<uint8_t>
PwmFrameBuilder::buildHeartbeatFrameV1(uint16_t seq, uint32_t ticks_ms) {
    vector<uint8_t> buf;
    buf.reserve(kMinFrameLenV1);

    appendU16BE(buf, kSof);                         // SOF
    const size_t ver_offset = buf.size();
    buf.push_back(kProtoVerV1);                     // VER
    buf.push_back(static_cast<uint8_t>(MsgId::HEARTBEAT)); // MSG_ID
    appendU16BE(buf, seq);                          // SEQ
    appendU32BE(buf, ticks_ms);                     // TICKS
    appendU16BE(buf, 0);                            // LEN=0（无负载）

    const uint16_t crc = proto::Crc16Ccitt::compute(&buf[ver_offset], buf.size() - ver_offset);
    appendU16BE(buf, crc);

    return buf;
}

// ========================= v1 解析 =========================
bool PwmFrameBuilder::looksLikeV1Frame(string_view frame) {
    if (!hasMinHeaderV1(frame)) return false;
    const auto* p = reinterpret_cast<const uint8_t*>(frame.data());
    const uint16_t sof = readU16BE(p);
    if (sof != kSof) return false;
    const uint8_t ver = p[2];
    return ver == kProtoVerV1;
}

bool PwmFrameBuilder::parseHeartbeatAckV1(string_view frame,
                                          uint16_t& seq_rx,
                                          uint32_t& ticks_rx) {
    seq_rx = 0;
    ticks_rx = 0;

    if (!hasMinHeaderV1(frame)) return false;

    const auto* p = reinterpret_cast<const uint8_t*>(frame.data());
    const size_t n = frame.size();

    // SOF
    if (readU16BE(p) != kSof) return false;

    // 基本头字段
    const uint8_t  ver    = p[2];
    const uint8_t  msg_id = p[3];
    if (ver != kProtoVerV1) return false;
    if (msg_id != static_cast<uint8_t>(MsgId::HEARTBEAT_ACK)) return false;

    const uint16_t seq   = readU16BE(p + 4);
    const uint32_t ticks = readU32BE(p + 6);
    const uint16_t len   = readU16BE(p + 10);

    // 长度合理性：总长 = 2(SOF)+10(VER..LEN)+LEN+2(CRC)
    const size_t expected = 2 + 10 + static_cast<size_t>(len) + 2;
    if (n != expected) return false;

    const size_t ver_offset = 2; // CRC 范围从 VER 起
    const uint16_t crc_calc = proto::Crc16Ccitt::compute(p + ver_offset, 10 + len);
    const uint16_t crc_in   = readU16BE(p + 2 + 10 + len);
    if (crc_calc != crc_in) return false;

    // 对 HB_ACK，建议 len==0；若后续扩展，也允许 len!=0，这里仅做宽松判定
    seq_rx   = seq;
    ticks_rx = ticks;
    return true;
}

std::optional<std::string_view>
PwmFrameBuilder::parseStatusV1(string_view frame) {
    if (!hasMinHeaderV1(frame)) return std::nullopt;

    const auto* p = reinterpret_cast<const uint8_t*>(frame.data());
    const size_t n = frame.size();

    if (readU16BE(p) != kSof) return std::nullopt;
    if (p[2] != kProtoVerV1)  return std::nullopt;
    if (p[3] != static_cast<uint8_t>(MsgId::STATUS)) return std::nullopt;

    const uint16_t len = readU16BE(p + 10);
    const size_t expected = 2 + 10 + static_cast<size_t>(len) + 2;
    if (n != expected) return std::nullopt;

    const size_t ver_offset = 2;
    const uint16_t crc_calc = proto::Crc16Ccitt::compute(p + ver_offset, 10 + len);
    const uint16_t crc_in   = readU16BE(p + 2 + 10 + len);
    if (crc_calc != crc_in) return std::nullopt;

    // 返回 payload 视图（不拷贝）
    const char* payload_ptr = reinterpret_cast<const char*>(p + 12);
    return std::string_view(payload_ptr, len);
}

// ========================= v0 兼容（可选） =========================
#ifdef PWM_PROTO_ENABLE_V0_COMPAT

static inline uint8_t sum8_impl(const uint8_t* data, size_t len) {
    uint8_t s = 0;
    for (size_t i = 0; i < len; ++i) s = static_cast<uint8_t>(s + data[i]);
    return s;
}

uint8_t PwmFrameBuilder::sum8(const void* data, size_t len) {
    return sum8_impl(reinterpret_cast<const uint8_t*>(data), len);
}

vector<uint8_t>
PwmFrameBuilder::buildPwmCmdFrameV0(const array<uint16_t, kPwmChannelCount>& pwm_values) {
    PwmDataFrameV0 f{};
    f.frame_header = 0xAA55;           // 这里不做主机序/网络序区分，直接按字节拷贝在 STM32 端解析；
    f.frame_id     = 0x01;
    f.data_length  = 16;

    for (size_t i = 0; i < kPwmChannelCount; ++i) {
        const uint16_t v = clampPwm(pwm_values[i]);
        f.pwm_data[i] = static_cast<uint16_t>((v >> 8) & 0xFF) << 8 | (v & 0xFF); // 大端序布局
    }

    f.checksum = sum8(&f, sizeof(f) - 1);

    vector<uint8_t> buf(sizeof(f));
    std::memcpy(buf.data(), &f, sizeof(f));
    return buf;
}

vector<uint8_t>
PwmFrameBuilder::buildHeartbeatFrameV0(uint32_t timestamp_s) {
    HeartbeatFrameV0 f{};
    f.frame_header = 0x55AA;
    // 秒级时间戳（大端布局）
    f.timestamp_s = ((timestamp_s >> 24) & 0xFF) << 24 |
                    ((timestamp_s >> 16) & 0xFF) << 16 |
                    ((timestamp_s >> 8)  & 0xFF) << 8  |
                     (timestamp_s        & 0xFF);
    f.checksum = sum8(&f, sizeof(f) - 1);

    vector<uint8_t> buf(sizeof(f));
    std::memcpy(buf.data(), &f, sizeof(f));
    return buf;
}

bool PwmFrameBuilder::parseHeartbeatFrameV0(string_view data, uint32_t& timestamp_s) {
    timestamp_s = 0;
    if (data.size() != sizeof(HeartbeatFrameV0)) return false;

    HeartbeatFrameV0 tmp{};
    std::memcpy(&tmp, data.data(), sizeof(tmp));

    // 校验头与校验和
    if (tmp.frame_header != 0x55AA) return false;
    const uint8_t csum = sum8(&tmp, sizeof(tmp) - 1);
    if (csum != tmp.checksum) return false;

    // 按大端还原
    const auto* p = reinterpret_cast<const uint8_t*>(&tmp.timestamp_s);
    timestamp_s = readU32BE(p);
    return true;
}

bool PwmFrameBuilder::looksLikeV0PwmFrame(string_view frame) {
    if (frame.size() != sizeof(PwmDataFrameV0)) return false;
    const auto* p = reinterpret_cast<const uint8_t*>(frame.data());
    // 前两字节是否为 0xAA55（大端布局）
    return (p[0] == 0xAA && p[1] == 0x55);
}
#endif // PWM_PROTO_ENABLE_V0_COMPAT
