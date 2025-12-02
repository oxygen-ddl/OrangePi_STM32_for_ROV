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

# ==================== 外部库：libpwm_host ====================
# 通过 CACHE 变量暴露给用户，可以在命令行 -D 覆盖
# 例如：
#   cmake -DPWM_LIB_DIR=/path/to/build -DPWM_INCLUDE_DIR=/path/to/include ..

set(PWM_LIB_DIR
    "/home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/build"
    CACHE PATH "Directory containing libpwm_host.a"
)

set(PWM_INCLUDE_DIR
    "/home/orangepi/ROV/OrangePi_STM32_for_ROV/OrangePi_STM32_for_ROV/orangepi_send/include"
    CACHE PATH "Directory containing libpwm_host.h / pwm_control.h"
)

# 简单存在性检查（可选）
if (NOT EXISTS "${PWM_LIB_DIR}/libpwm_host.a")
    message(WARNING "libpwm_host.a not found in PWM_LIB_DIR='${PWM_LIB_DIR}'")
endif()
if (NOT EXISTS "${PWM_INCLUDE_DIR}/libpwm_host.h")
    message(WARNING "libpwm_host.h not found in PWM_INCLUDE_DIR='${PWM_INCLUDE_DIR}'")
endif()

# ==================== 外部库：yaml-cpp ====================
# 在 OrangePi 上建议 apt 安装：
#   sudo apt install libyaml-cpp-dev
find_package(YAML-CPP REQUIRED)

# ==================== 源码列表 ====================
# 目录结构：
#  src/
#    main.cpp
#    control_core/
#    controllers/
#    allocation/   (暂时可能为空)
#    platform/
#    io/
#    utils/        (将来放 config_loader 等)

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

    # 如果将来有 utils/config_loader.cpp 等，可以继续往这里加
    # src/utils/config_loader.cpp
)

add_executable(pwm_control_program
    ${PWMCTRL_SOURCES}
)

# ==================== 头文件搜索路径 ====================
target_include_directories(pwm_control_program PRIVATE
    ${PWM_INCLUDE_DIR}                    # libpwm_host.h / pwm_control.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include   # 本工程头文件：control_core/, controllers/, io/, platform/, utils/
)

# ==================== 链接库 ====================
find_package(Threads REQUIRED)

target_link_libraries(pwm_control_program PRIVATE
    ${PWM_LIB_DIR}/libpwm_host.a   # 底层 UDP + 安全层 C 库
    YAML::YAML_CPP                 # yaml-cpp
    Threads::Threads
)

# Linux 上可能需要 librt（clock_gettime 等），在部分系统上是隐式包含
if (UNIX AND NOT APPLE)
    target_link_libraries(pwm_control_program PRIVATE rt)
endif()

# ==================== 编译定义/特性（如有需要可以再扩展） ====================

# 例如：
# target_compile_definitions(pwm_control_program PRIVATE
#     PWMCTRL_VERSION=\"0.1.0\"
# )

message(STATUS "Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "Using PWM include dir: ${PWM_INCLUDE_DIR}")
message(STATUS "Using PWM lib dir:     ${PWM_LIB_DIR}")
