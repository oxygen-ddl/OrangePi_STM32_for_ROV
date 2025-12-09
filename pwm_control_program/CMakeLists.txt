cmake_minimum_required(VERSION 3.14)
project(pwm_control_program LANGUAGES CXX)

# ==================== 编译选项 ====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall -Wextra -Wpedantic
        -Wconversion -Wsign-conversion
        -Wshadow -Wformat=2 -Wundef
    )
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
    endif()
endif()

# ==================== 路径约定 ====================
# 当前目录：.../UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program
# 上一层：  .../UnderwaterRobotSystem/OrangePi_STM32_for_ROV
# 同级目录：
#   - orangepi_send
#   - pwm_control_program
# 另一项目：
#   .../UnderwaterRobotSystem/Underwater-robot-navigation

# libpwm_host 头文件所在目录：
#   <repo_root>/OrangePi_STM32_for_ROV/orangepi_send/include
set(PWM_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../orangepi_send/include"
    CACHE PATH "Directory containing libpwm_host.h / pwm_control.h"
)

# 共享导航消息头文件所在项目根目录：
#   <repo_root>/Underwater-robot-navigation
# 该项目下存在 shared/msg/nav_state.hpp
set(NAV_PROJECT_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Underwater-robot-navigation"
    CACHE PATH "Directory containing shared/msg/nav_state.hpp"
)

# 预编译 libpwm_host.a 所在目录：
#   <repo_root>/OrangePi_STM32_for_ROV/orangepi_send/build
set(PWM_LIB_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../orangepi_send/build"
    CACHE PATH "Directory containing prebuilt libpwm_host.a"
)

# 手动指定 Eigen3 的头文件目录（通过 apt 安装 libeigen3-dev 后默认路径）
# 注意：这个必须放在 target_include_directories 之前！！
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

# ==================== 源码列表 ====================

set(PWMCTRL_SOURCES
    # 顶层入口
    src/main.cpp

    # control_core
    src/control_core/app_main.cpp
    src/control_core/control_loop.cpp
    src/control_core/thruster_allocation.cpp
    src/control_core/trajectory_tracking.cpp

    # controllers
    src/controllers/manual_controller.cpp
    src/controllers/pid_controller.cpp
    src/controllers/controller_manager.cpp

    # platform 层
    src/platform/pwm_client.cpp
    src/platform/timebase.cpp

    # IO 层
    src/io/teleop_keyboard.cpp
    src/io/teleop_input.cpp
    src/io/pwm_logger.cpp           # PWM 日志
    src/io/nav_state_subscriber.cpp # 读取定位算法消息（共享内存）
)

add_executable(pwm_control_program
    ${PWMCTRL_SOURCES}
)

# ==================== 头文件搜索路径 ====================
# 统一用一次 target_include_directories，把所有 include 根目录写清楚
target_include_directories(pwm_control_program
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # 本工程头文件（control_core/controllers/io/platform）
        ${PWM_INCLUDE_DIR}                  # libpwm_host.h / pwm_control.h
        ${NAV_PROJECT_INCLUDE_DIR}          # shared/msg/nav_state.hpp 所在项目根目录
        ${EIGEN3_INCLUDE_DIR}              # /usr/include/eigen3, 提供 <Eigen/Dense>
)

# ==================== 依赖库查找 ====================

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# ==================== 链接库 ====================
# 关键逻辑：优先用子项目 target，其次用预编译静态库文件
if (TARGET pwm_host)
    message(STATUS "[pwm_control_program] Linking against CMake target 'pwm_host' from orangepi_send")
    target_link_libraries(pwm_control_program PRIVATE
        pwm_host          # orangepi_send 生成的静态库 target
        yaml-cpp          # CMake target
        Threads::Threads
    )
else()
    if (NOT EXISTS "${PWM_LIB_DIR}/libpwm_host.a")
        message(WARNING
            "[pwm_control_program] prebuilt libpwm_host.a not found in '${PWM_LIB_DIR}'. "
            "You may need to build orangepi_send separately."
        )
    else()
        message(STATUS "[pwm_control_program] Linking against prebuilt ${PWM_LIB_DIR}/libpwm_host.a")
    endif()

    target_link_libraries(pwm_control_program PRIVATE
        "${PWM_LIB_DIR}/libpwm_host.a"
        yaml-cpp
        Threads::Threads
    )
endif()

# Linux 上可能需要 librt
if (UNIX AND NOT APPLE)
    target_link_libraries(pwm_control_program PRIVATE rt)
endif()

message(STATUS "[pwm_control_program] PWM_INCLUDE_DIR        = ${PWM_INCLUDE_DIR}")
message(STATUS "[pwm_control_program] NAV_PROJECT_INCLUDE_DIR = ${NAV_PROJECT_INCLUDE_DIR}")
message(STATUS "[pwm_control_program] PWM_LIB_DIR            = ${PWM_LIB_DIR}")
message(STATUS "[pwm_control_program] EIGEN3_INCLUDE_DIR     = ${EIGEN3_INCLUDE_DIR}")
