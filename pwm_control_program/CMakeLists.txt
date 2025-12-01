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

# ---------- libpwm_host 静态库路径（按你在 OrangePi 上的路径） ----------
# 注意：这里假定目录结构与 PC 上一致：
#   /home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/build
#   /home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/include
# 如有变动，只需要改下面两个变量
set(PWM_LIB_DIR
    "/home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/build"
)
set(PWM_INCLUDE_DIR
    "/home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/include"
)

# ---------- 源码列表：按子目录组织 ----------
set(PWMCTRL_SOURCES
  src/main.cpp

  # 平台封装层：时间基准 + PWM 客户端
  src/platform/timebase.cpp
  src/platform/pwm_client.cpp

  # 输入层：键盘 Teleop
  src/platform/io/teleop_keyboard.cpp
)

add_executable(pwm_control_program
  ${PWMCTRL_SOURCES}
)

# ---------- 头文件搜索路径 ----------
target_include_directories(pwm_control_program PRIVATE
  ${PWM_INCLUDE_DIR}                    # libpwm_host.h / pwm_control.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include   # 本工程头文件：platform/*, platform/io/*
)

# ---------- 线程 + rt（部分 Linux 上 clock_gettime 需要） ----------
find_package(Threads REQUIRED)

target_link_libraries(pwm_control_program PRIVATE
  ${PWM_LIB_DIR}/libpwm_host.a
  Threads::Threads
  rt
)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Using PWM include dir: ${PWM_INCLUDE_DIR}")
message(STATUS "Using PWM lib dir:     ${PWM_LIB_DIR}")
