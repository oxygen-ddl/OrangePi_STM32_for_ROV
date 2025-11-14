#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>
#include <unordered_map>
#include <vector>
#include <array>

#include "UdpSender.h"
#include "protocol_pack.hpp" // 我们的上位机轻量打包封装

namespace
{
    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::milliseconds;

    std::atomic<bool> g_running{true};
    void handle_sigint(int) { g_running = false; }

    // 进程启动以来的毫秒（单调时钟）
    static inline std::uint32_t now_ticks_ms()
    {
        return static_cast<std::uint32_t>(
            std::chrono::duration_cast<Ms>(Clock::now().time_since_epoch()).count());
    }

    struct Stats
    {
        uint64_t sent_pwm = 0;
        uint64_t sent_hb = 0;
        uint64_t rx_hb_ack = 0;
        double rtt_ms_avg = 0.0; // 简单EMA
        void add_rtt(double rtt_ms)
        {
            const double alpha = 0.1;
            rtt_ms_avg = (sent_hb == 0) ? rtt_ms : (1.0 - alpha) * rtt_ms_avg + alpha * rtt_ms;
        }
    };

    /* ===================== 轻量 HB_ACK 解析器 ===================== */
    /* 格式：SOF(AA55) VER(01) MSG(11) SEQ(2) TICKS(4) LEN(0000) CRC(2)，总长14 */
    /* 与协议一致：CRC16-CCITT-FALSE 覆盖 VER..LEN (10字节) */
    static inline uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
    static inline uint32_t be32(const uint8_t *p)
    {
        return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3];
    }
    static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
    {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < len; i++)
        {
            crc ^= (uint16_t)((uint16_t)data[i] << 8);
            for (uint8_t j = 0; j < 8; j++)
            {
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
            }
        }
        return crc;
    }
    static bool try_parse_hb_ack(const std::vector<std::uint8_t> &rx, uint16_t &seq, uint32_t &ticks_ms)
    {
        if (rx.size() != 14)
            return false;
        const uint8_t *p = rx.data();
        if (p[0] != 0xAA || p[1] != 0x55)
            return false; // SOF
        if (p[2] != 0x01)
            return false; // VER
        if (p[3] != 0x11)
            return false; // MSG_HB_ACK
        seq = be16(p + 4);
        ticks_ms = be32(p + 6);
        if (be16(p + 10) != 0x0000)
            return false;                                 // LEN=0
        const uint16_t crc_calc = crc16_ccitt(p + 2, 10); // VER..LEN
        const uint16_t crc_rx = be16(p + 12);
        return crc_calc == crc_rx;
    }

} // namespace

int main(int argc, char **argv)
{
    // ====== 0) 运行参数 ======
    const char *target_ip = (argc > 1) ? argv[1] : "192.168.2.16";
    const int target_port = (argc > 2) ? std::stoi(argv[2]) : 8000;
    const int control_hz = (argc > 3) ? std::stoi(argv[3]) : 51; // 51Hz
    const int hb_hz = (argc > 4) ? std::stoi(argv[4]) : 1;       // 1Hz

    const Ms control_period_ms(1000 / std::max(1, control_hz));
    const Ms hb_period_ms(1000 / std::max(1, hb_hz));

    std::signal(SIGINT, handle_sigint);

    std::cout << "[INFO] Target=" << target_ip << ":" << target_port
              << " ctrl=" << control_hz << "Hz hb=" << hb_hz << "Hz\n";

    // ====== 1) 初始化 UDP ======
    UdpSender udp;
    if (!udp.initialize(target_ip, static_cast<uint16_t>(target_port)))
    {
        std::cerr << "[ERROR] UdpSender.initialize failed\n";
        return 1;
    }

    // ====== 2) 控制数据（示例固定 8 路；实际可接入你的上层指令源） ======
    std::array<std::uint16_t, 8> pwm = {500, 2000, 3000, 4500, 5000, 6000, 7000, 9999};

    // ====== 3) 循环辅助变量 ======
    auto t_next_ctrl = Clock::now();
    auto t_next_hb = Clock::now();

    // 保存“心跳 seq → 发送时刻”，用于 RTT 计算（最多保存最近 256 条）
    std::unordered_map<std::uint16_t, Clock::time_point> hb_send_times;
    hb_send_times.reserve(256);

    Stats stats;
    protocol_pack_init(); // 重置 packer 的本地序列号（从 0 开始）

    // ====== 4) 主循环：固定频率发送 PWM + 低频心跳；短超时接收 ======
    while (g_running.load())
    {
        const auto t_now = Clock::now();

        // 4.1 发送 PWM（固定频率）
        if (t_now >= t_next_ctrl)
        {
            t_next_ctrl += control_period_ms;//实现可调节的周期性

            auto frame = ProtocolV1Packer::packPWM(pwm);
            if (frame.empty() || !udp.sendHexData(frame))
            {
                std::cerr << "[WARN] send PWM failed\n";
            }
            else
            {
                ++stats.sent_pwm;
            }
        }

        // 4.2 发送心跳（1Hz）
        if (t_now >= t_next_hb)
        {
            t_next_hb += hb_period_ms;

            auto frame = ProtocolV1Packer::packHeartbeat();
            // 取 packer 内的当前序列号-1 作为本次心跳的 seq（因为 packHeartbeat 内部已自增）
            uint16_t seq_sent = (uint16_t)(protocol_pack_get_seq() - 1);

            hb_send_times[seq_sent] = Clock::now();
            if (hb_send_times.size() > 256)
            {
                hb_send_times.erase(hb_send_times.begin()); // 简洁清理
            }

            if (frame.empty() || !udp.sendHexData(frame))
            {
                std::cerr << "[WARN] send HB failed\n";
            }
            else
            {
                ++stats.sent_hb;
            }
        }

        // 4.3 短超时接收与解析（例如 5ms 轮询）
        std::vector<std::uint8_t> rx;
        if (udp.receiveData(rx, /*timeout_ms=*/5))
        {
            uint16_t seq_rx = 0;
            uint32_t ticks_rx = 0;

            if (try_parse_hb_ack(rx, seq_rx, ticks_rx))
            {
                ++stats.rx_hb_ack;
                auto it = hb_send_times.find(seq_rx);
                if (it != hb_send_times.end())
                {
                    const double rtt_ms =
                        std::chrono::duration<double, std::milli>(Clock::now() - it->second).count();
                    stats.add_rtt(rtt_ms);
                    hb_send_times.erase(it);
                    std::cout << "[HB_ACK] seq=" << seq_rx
                              << " rtt=" << std::fixed << std::setprecision(2) << rtt_ms << " ms"
                              << " ticks_remote=" << ticks_rx << "\n";
                }
                else
                {
                    std::cout << "[HB_ACK] seq=" << seq_rx << " (no send record)\n";
                }
            }
            else
            {
                // 未知帧：按需打印/忽略
                // std::cout << "[INFO] rx " << rx.size() << " bytes (unknown)\n";
            }
        }

        // 4.4 小睡一会，避免空转占满CPU（不影响固定频率调度）
        std::this_thread::sleep_for(Ms(1));

        // 4.5 每秒打印简报
        static auto t_last_report = Clock::now();
        if (t_now - t_last_report >= Ms(1000))
        {
            t_last_report = t_now;
            std::cout << "[STAT] sent_pwm=" << stats.sent_pwm
                      << " sent_hb=" << stats.sent_hb
                      << " rx_hb_ack=" << stats.rx_hb_ack
                      << " rtt_avg=" << std::fixed << std::setprecision(2) << stats.rtt_ms_avg << " ms\n";
        }
    }

    udp.close();
    std::cout << "[INFO] exit.\n";
    return 0;
}
