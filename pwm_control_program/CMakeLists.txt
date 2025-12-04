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

# ==================== libpwm_host 头文件路径 ====================
# 头文件所在目录保持原来的约定：
#   <repo_root>/OrangePi_STM32_for_ROV/orangepi_send/include
#
# 顶层 CMake 已经用 CACHE + FORCE 传进来一份 PWM_INCLUDE_DIR，
# 如果没有（单独编译 pwm_control_program），就使用默认值。

set(PWM_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../OrangePi_STM32_for_ROV/orangepi_send/include"
    CACHE PATH "Directory containing libpwm_host.h / pwm_control.h"
)

# ==================== 源码列表 ====================

set(PWMCTRL_SOURCES
    # 顶层入口
    src/main.cpp

    # control_core
    src/control_core/app_main.cpp
    src/control_core/control_loop.cpp

    # controllers
    src/controllers/manual_controller.cpp

    # platform 层
    src/platform/pwm_client.cpp
    src/platform/timebase.cpp

    # IO 层
    src/io/teleop_keyboard.cpp
    src/io/teleop_input.cpp
    src/io/pwm_logger.cpp # ← 新增这一行
)

add_executable(pwm_control_program
    ${PWMCTRL_SOURCES}
)

# ==================== 头文件搜索路径 ====================

target_include_directories(pwm_control_program PRIVATE
    ${PWM_INCLUDE_DIR}                    # libpwm_host.h / pwm_control.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include   # 本工程头文件
)

# ==================== 链接库 ====================
find_package(Threads REQUIRED)

# 尽量用官方推荐写法：yaml-cpp
# libyaml-cpp-dev 在大多数系统上会提供一个 'yaml-cpp' 的 CMake target
find_package(yaml-cpp REQUIRED)

# 关键逻辑：优先用子项目 target，其次用预编译静态库文件
if (TARGET pwm_host)
    message(STATUS "[pwm_control_program] Linking against CMake target 'pwm_host' from orangepi_send")
    target_link_libraries(pwm_control_program PRIVATE
        pwm_host          # orangepi_send 生成的静态库 target
        yaml-cpp          # CMake target，一般就叫这个名字
        Threads::Threads
    )
else()
    set(PWM_LIB_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../OrangePi_STM32_for_ROV/orangepi_send/build"
        CACHE PATH "Directory containing prebuilt libpwm_host.a" FORCE
    )

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
        yaml-cpp          # 同样用 target 名
        Threads::Threads
    )
endif()


# Linux 上可能需要 librt
if (UNIX AND NOT APPLE)
    target_link_libraries(pwm_control_program PRIVATE rt)
endif()

message(STATUS "[pwm_control_program] PWM_INCLUDE_DIR = ${PWM_INCLUDE_DIR}")
