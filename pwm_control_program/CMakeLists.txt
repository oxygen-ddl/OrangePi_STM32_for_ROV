cmake_minimum_required(VERSION 3.10)
project(pwm_control_program LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
  add_compile_options(-Wshadow -Wformat=2 -Wundef)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
  endif()
endif()

# ---------- 路径：改成你 OrangePi 上实际的路径 ----------
# 这里假设你在 orangepi 上的路径是：
#   /home/orangepi/ROV/orangepi_send/build
#   /home/orangepi/ROV/orangepi_send/include
# 如果不一样，就按实际情况调整
set(PWM_LIB_DIR   "/home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/build")
set(PWM_INCLUDE_DIR "/home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/include")

# ============ 可执行程序 ============
add_executable(pwm_control_program
  src/main.cpp
  src/pwm_teleop_keys.cpp    # ← 把 teleop 的实现文件加进来
  src/timebase.cpp
)

target_include_directories(pwm_control_program PRIVATE
  ${PWM_INCLUDE_DIR}             # libpwm_host / pwm_control 的头文件
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # 本工程自己的头文件（pwm_teleop_keys.h）
)

# 线程 + rt（clock_gettime 在某些 Linux 上需要）
find_package(Threads REQUIRED)

target_link_libraries(pwm_control_program PRIVATE
  ${PWM_LIB_DIR}/libpwm_host.a
  Threads::Threads
  rt
)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# # ---------- 路径按你的实际位置 ----------
# set(PWM_LIB_DIR   "/home/orangepi/ROV/OrangePi_STM32_for_ROV/orangepi_send/build")
# set(PWM_INCLUDE_DIR "/home/orangepi/ROV/OrangePi_STM32_for_ROV/orangepi_send/include")