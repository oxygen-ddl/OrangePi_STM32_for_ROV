#include "timebase.h"
#include <chrono>
#include <sys/time.h>
#include <iostream>
#include <atomic>
#include <cstring>

using MonoClock = std::chrono::steady_clock;
static std::atomic<bool> s_tb_inited{false};
static MonoClock::time_point s_t0;

void timebase_init(void)
{
    s_t0 = MonoClock::now();
    s_tb_inited = true;
    s_tb_inited.store(true);
    std::cout << "[TB] timebase initialized.\n";
}

void timebase_now(double* t_epoch_s, int64_t* t_mono_ns)
{
    if (!s_tb_inited.load()) {
        std::cerr << "[WARN] timebase not inited, auto init now.\n";
        timebase_init();
    }

    // epoch 秒
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    *t_epoch_s = tv.tv_sec + tv.tv_usec / 1e6;

    // monotonic ns
    auto now_mono = MonoClock::now();
    *t_mono_ns = (int64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(now_mono - s_t0).count();
}
