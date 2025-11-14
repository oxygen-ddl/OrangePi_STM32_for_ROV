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

# ---------- 路径按你的实际位置 ----------
set(PWM_LIB_DIR   "/home/orangepi/ROV/OrangePi_STM32_for_ROV/orangepi_send/build")
set(PWM_INCLUDE_DIR "/home/orangepi/ROV/OrangePi_STM32_for_ROV/orangepi_send/include")

add_executable(pwm_control_program
  src/main.cpp
)

target_include_directories(pwm_control_program PRIVATE
  ${PWM_INCLUDE_DIR}
)

# 链接我们的静态库
target_link_libraries(pwm_control_program PRIVATE
  ${PWM_LIB_DIR}/libpwm_host.a
)

# 线程 + rt（clock_gettime）
find_package(Threads REQUIRED)
target_link_libraries(pwm_control_program PRIVATE Threads::Threads rt)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
add_executable(pwm_teleop
  src/pwm_teleop.cpp
)

target_include_directories(pwm_teleop PRIVATE
  ${PWM_INCLUDE_DIR}
)

target_link_libraries(pwm_teleop PRIVATE
  ${PWM_LIB_DIR}/libpwm_host.a
)

find_package(Threads REQUIRED)
target_link_libraries(pwm_teleop PRIVATE Threads::Threads rt)
