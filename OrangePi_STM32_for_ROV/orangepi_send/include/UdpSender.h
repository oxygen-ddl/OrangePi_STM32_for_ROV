#ifndef UDPSENDER_H
#define UDPSENDER_H

#include <string>
#include <vector>
#include <cstdint>

/**
 * @brief 轻量级 UDP 发送/接收封装（IPv4）
 * 设计目标：
 *  - 与现有代码向下兼容（不改签名）
 *  - 可在实时控制循环中短超时轮询接收（poll 实现）
 *  - 统一错误报告（携带 errno 文本）
 *
 * 线程模型：
 *  - 实例非线程安全；若多线程访问，请在调用侧加锁。
 */
class UdpSender {
public:
    UdpSender();
    ~UdpSender();

    /**
     * @brief 初始化并连接目标端（可选绑定本地地址/端口）
     * @param target_ip   目标 IPv4（如 "192.168.2.16"）
     * @param target_port 目标端口
     * @param timeout_ms  读写超时（用于 setsockopt & poll 缺省值）
     * @return true=成功
     */
    bool initialize(const std::string& target_ip, uint16_t target_port, int timeout_ms = 1000);

    /// 发送功能（与现有接口保持一致）
    bool sendHexData(const std::vector<uint8_t>& data);
    bool sendStringData(const std::string& data);
    bool sendRawData(const void* data, size_t data_size);

    /// 接收功能（短超时轮询）
    bool receiveData(std::vector<uint8_t>& buffer, int timeout_ms = 1000);

    /// 接收固定长度（若长度不符返回 false）
    bool receiveDataWithSize(std::vector<uint8_t>& buffer, size_t expected_size, int timeout_ms = 1000);

    /// 接收并返回对端地址（可选用，将来做来源检查）
    bool receiveFrom(std::vector<uint8_t>& buffer, std::string& src_ip, uint16_t& src_port, int timeout_ms = 1000);

    /// 主动关闭
    void close();

    /// 最近一次错误说明（含 errno 文本）
    std::string getLastError() const;

    /// 可选：绑定本地地址与端口（在 initialize 前调用）
    void setLocalBind(const std::string& local_ip, uint16_t local_port);

    /// 可选：设置接收/发送缓冲大小（字节）
    void setSocketBuffers(int rcvbuf_bytes, int sndbuf_bytes);

    /// 可选：切换是否非阻塞（默认初始化后即为非阻塞）
    bool setNonBlocking(bool nb);

private:
    bool applySocketOptions(int timeout_ms);
    bool bindLocalIfNeeded();

private:
    int         sockfd_;
    std::string target_ip_;
    uint16_t    target_port_;
    bool        is_initialized_;
    std::string last_error_;

    // 目标地址缓存
    struct sockaddr_in* peer_; // 前向声明用指针避免在头文件里包含 <netinet/in.h>

    // 可选本地绑定
    std::string local_ip_;
    uint16_t    local_port_;

    // 可选缓冲设置（<=0 表示不修改）
    int rcvbuf_bytes_;
    int sndbuf_bytes_;
};

#endif // UDPSENDER_H
