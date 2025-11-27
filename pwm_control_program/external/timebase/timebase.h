#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// 获取当前 Unix 时间戳（秒，double）和单调时间（纳秒，int64_t）
void timebase_now(double* t_epoch_s, int64_t* t_mono_ns);

// 初始化 timebase（用于记录相对 steady 起点，非必须跨项目共享）
void timebase_init(void);

#ifdef __cplusplus
}
#endif

#endif // TIMEBASE_H
