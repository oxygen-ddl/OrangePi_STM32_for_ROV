#ifndef LIBPWM_HOST_H
#define LIBPWM_HOST_H



/**
 * @file      libpwm_host.h
 * @brief     上位机（香橙派）控制 STM32 PWM 的最小可复用 C 接口
 * @version   1.1.0
 *
 * 设计目标：
 *  - 作为“底层驱动库”供 C/C++ 直接链接，Python 可通过 ctypes/cffi 调用
 *  - 稳定的纯 C API（无异常），多语言友好
 *  - 将“打包 / CRC / UDP 发送 / 简单渐变 / 心跳-RTT-统计”统一封装
 *
 * 协议假设（与 STM32 一致）：
 *  - 帧头 0xAA55，VER=0x01，MSG_PWM=0x01
 *  - 8x uint16（大端）范围 0..10000（5000 = 7.5% 中位）
 *  - CRC16-CCITT(False), poly=0x1021, init=0xFFFF, xorout=0x0000
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ----------------------------- 可见性与版本 ----------------------------- */
#if defined(_WIN32) || defined(__CYGWIN__)
  #ifdef PWMH_BUILD_DLL
    #define PWMH_API __declspec(dllexport)
  #else
    #define PWMH_API __declspec(dllimport)
  #endif
#else
  #define PWMH_API __attribute__((visibility("default")))
#endif

/** 库语义版本（供运行时查询） */
#define PWM_HOST_SEMVER "1.1.0"

/** 协议固定参数（与 STM32 端保持一致） */
enum {
    PWM_HOST_PROTO_VER   = 0x01,   /**< protocol_v1 版本号 */
    PWM_HOST_MSG_PWM     = 0x01,   /**< PWM 指令消息 ID */
    PWM_HOST_MSG_HB      = 0x10,   /**< 心跳（Host -> STM32） */
    PWM_HOST_MSG_HB_ACK  = 0x11,   /**< 心跳 ACK（STM32 -> Host） */
    PWM_HOST_SOF_BE      = 0xAA55, /**< 帧头（大端） */
    PWM_HOST_CH_NUM      = 8,      /**< 通道数固定 8 */
    PWM_HOST_VAL_MIN     = 0,      /**< 协议值最小 */
    PWM_HOST_VAL_MID     = 5000,   /**< 7.5% 中位 */
    PWM_HOST_VAL_MAX     = 10000   /**< 协议值最大 */
};

/** 建议占空比范围（单位：%） */
#define PWM_HOST_PCT_MIN 5.0f
#define PWM_HOST_PCT_MID 7.5f
#define PWM_HOST_PCT_MAX 10.0f

/* ----------------------------- 返回码与错误 ----------------------------- */

/**
 * @brief 函数返回码（避免抛异常，便于多语言绑定）
 *
 * 约定：
 *  - 大多数 API 返回 pwmh_result_t；
 *  - 个别 API（如 pwm_host_poll）可能返回 >=0 表示“数量”，<0 表示 -PWMH_xxx。
 */
typedef enum {
    PWMH_OK = 0,
    PWMH_EINVAL,      /**< 参数非法 */
    PWMH_ENOTINIT,    /**< 未初始化 */
    PWMH_ESYS,        /**< 系统调用错误（socket等） */
    PWMH_EBUSY,       /**< 正在进行阻塞操作（如内部渐变） */
    PWMH_EINTERNAL    /**< 其他内部错误 */
} pwmh_result_t;

/**
 * @brief 错误码 → 短字符串
 * @return 指向静态字符串的指针，**线程安全，无需释放**
 */
PWMH_API const char* pwm_host_strerror(pwmh_result_t rc);

/* ----------------------------- 配置结构体 ----------------------------- */

/**
 * @brief 初始化配置
 * 所有字段若为 0/NULL 将使用合理默认值：
 *  - stm32_ip:       "192.168.2.16"
 *  - stm32_port:     8000
 *  - send_hz:        50
 *  - socket_sndbuf:  不修改
 *  - nonblock_send:  0（阻塞发送）
 */
typedef struct {
    const char* stm32_ip;       /**< 目标 STM32 IP（默认 "192.168.2.16"） */
    uint16_t    stm32_port;     /**< 目标端口（默认 8000） */
    int         send_hz;        /**< 建议发送频率（默认 50） */
    int         socket_sndbuf;  /**< socket 发送缓冲（字节），0=不修改 */
    int         nonblock_send;  /**< 非零则使用非阻塞 sendto（默认 0 阻塞） */
} pwm_host_config_t;

/**
 * @brief 填充默认配置
 * @param cfg 不可为 NULL
 */
PWMH_API void pwm_host_default_config(pwm_host_config_t* cfg);

/* ----------------------------- 统计与心跳状态 ----------------------------- */

typedef struct {
    uint64_t tx_pwm;        /**< 已发送 PWM 帧计数 */
    uint64_t tx_hb;         /**< 已发送心跳帧计数 */
    uint64_t rx_hb_ack;     /**< 收到心跳 ACK 计数 */
    uint64_t tx_err;        /**< 发送错误计数（系统调用失败等） */
    uint64_t rx_err;        /**< 接收/解析错误计数（CRC/长度等） */
} pwm_host_stats_t;

/* ----------------------------- 基础生命周期 ----------------------------- */

/**
 * @brief 初始化库（创建/配置 UDP socket，重置序列号/影子值）
 * @param cfg 若为 NULL 则采用 pwm_host_default_config() 的默认配置
 * @return PWMH_OK / 错误码
 */
PWMH_API pwmh_result_t pwm_host_init(const pwm_host_config_t* cfg);

/**
 * @brief 关闭库（关闭 socket），可重入多次调用
 */
PWMH_API void pwm_host_close(void);

/**
 * @brief 获取库版本（语义化）
 * @return 静态字符串，例如 "1.1.0"
 */
PWMH_API const char* pwm_host_version(void);

/* ----------------------------- 值映射工具 ----------------------------- */

/**
 * @brief 将占空比（百分比 5.0..10.0）转换为协议值（0..10000）
 * @note 超出范围将自动裁剪；7.5% → 5000
 */
PWMH_API uint16_t pwm_host_percent_to_u16(float pct);

/**
 * @brief 将协议值（0..10000）映射回占空比（百分比）
 */
PWMH_API float pwm_host_u16_to_percent(uint16_t v);

/* ----------------------------- 发送接口（原子帧） ----------------------------- */

/**
 * @brief 直接下发 8 路协议值（0..10000）
 * @param v 8个通道值数组，长度须为 8
 * @return PWMH_OK / 错误码
 */
PWMH_API pwmh_result_t pwm_host_set_all_u16(const uint16_t v[PWM_HOST_CH_NUM]);

/**
 * @brief 直接下发 8 路占空比（百分比，5.0..10.0；传负值表示“使用中位 7.5%”）
 * @param pct 8个通道百分比，长度须为 8；负值会被替换为 7.5%
 * @return PWMH_OK / 错误码
 */
PWMH_API pwmh_result_t pwm_host_set_all_pct(const float pct[PWM_HOST_CH_NUM]);

/**
 * @brief 设置单通道占空比（百分比，5.0..10.0；-1 表示中位）
 * @param ch  1..8
 * @param pct 百分比或 -1
 * @return PWMH_OK / 错误码
 *
 * 注意：本函数内部会先读取当前影子缓存的 8 路值，替换 ch，再整体下发一次。
 */
PWMH_API pwmh_result_t pwm_host_set_ch_pct(int ch, float pct);

/* ----------------------------- 心跳与轮询 ----------------------------- */

/**
 * @brief 发送心跳（立即打包并发送一帧 MSG_HB）
 * @return PWMH_OK / 错误码
 */
PWMH_API pwmh_result_t pwm_host_send_heartbeat(void);

/**
 * @brief 轮询收包/处理（解析 HB_ACK / 统计 / RTT 计算）
 * @param timeout_ms 轮询超时（毫秒）。0=非阻塞，>0=阻塞等待至多 timeout_ms。
 * @return
 *   - >= 0 : 本次处理的帧数
 *   - <  0 : 发生错误，对应 -PWMH_xxx（例如返回 -PWMH_ESYS）
 */
PWMH_API int pwm_host_poll(int timeout_ms);

/**
 * @brief 最近一次心跳 RTT（毫秒）
 * @return 若尚无有效 RTT，返回负数（如 -1.0）
 */
PWMH_API double pwm_host_last_rtt_ms(void);

/**
 * @brief 获取统计数据（快照）
 */
PWMH_API void pwm_host_get_stats(pwm_host_stats_t* out);

/* ----------------------------- 阻塞式渐变（简易） ----------------------------- */

/**
 * @brief 单通道线性渐变（阻塞执行）
 * @param ch        1..8
 * @param start_pct 起始占空比（%）
 * @param end_pct   结束占空比（%）
 * @param seconds   渐变总时长（秒），>0
 * @param hz        发送频率（建议 50Hz）
 * @return PWMH_OK / 错误码
 *
 * 说明：
 *  - 内部以固定步长插值 + 周期发送，直到完成；
 *  - 其他 7 路保持当前值不变；
 *  - 若需非阻塞 / 多路同时渐变，建议上移到“daemon 服务层”统一调度。
 */
PWMH_API pwmh_result_t pwm_host_ramp_pct(int ch, float start_pct, float end_pct, float seconds, int hz);

/* ----------------------------- 线程安全性说明 ----------------------------- */
/*
 * - 本库内部维护一个 UDP socket 与“上次下发的 8 路影子值”，默认实现非线程安全；
 * - 若需要多线程调用，请在外部加互斥锁，或在 .c 实现中加入 pthread_mutex 保护；
 * - 建议：进程内统一通过单线程调度发送，或改用“pwm-daemon 服务层”。
 */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIBPWM_HOST_H */
