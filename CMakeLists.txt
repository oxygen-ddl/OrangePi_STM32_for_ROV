cmake_minimum_required(VERSION 3.14)
project(ROV_PWM_STACK LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =============================================================================
# shared 模块（仓库根目录下的 shared）
# 当前位置：<repo_root>/OrangePi_STM32_for_ROV
# shared 位置：<repo_root>/shared
# =============================================================================
set(UWSYS_SHARED_DIR "${CMAKE_CURRENT_LIST_DIR}/../shared")

if (EXISTS "${UWSYS_SHARED_DIR}/CMakeLists.txt")
    add_subdirectory(
        ${UWSYS_SHARED_DIR}
        ${CMAKE_BINARY_DIR}/shared_build
    )
else()
    message(FATAL_ERROR "Shared module not found at: ${UWSYS_SHARED_DIR}")
endif()

# =============================================================================
# 子项目：proto_gcs（当前目录下）
# 统一管理 GCS 协议头 + CRC32C 等协议相关实现
# =============================================================================
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/proto_gcs/CMakeLists.txt")
    add_subdirectory(
        ${CMAKE_CURRENT_LIST_DIR}/proto_gcs
        ${CMAKE_BINARY_DIR}/proto_gcs_build
    )
else()
    message(FATAL_ERROR "proto_gcs module not found at: ${CMAKE_CURRENT_LIST_DIR}/proto_gcs")
endif()

# =============================================================================
# 子项目：comm_gcs（当前目录下，可选：如果目录存在就编）
# =============================================================================
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/comm_gcs/CMakeLists.txt")
    add_subdirectory(
        ${CMAKE_CURRENT_LIST_DIR}/comm_gcs
        ${CMAKE_BINARY_DIR}/comm_gcs_build
    )
else()
    message(STATUS "comm_gcs module not found at: ${CMAKE_CURRENT_LIST_DIR}/comm_gcs (skip)")
endif()

# =============================================================================
# 子项目：orangepi_send（当前目录下）
# =============================================================================
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/orangepi_send/CMakeLists.txt")
    add_subdirectory(
        ${CMAKE_CURRENT_LIST_DIR}/orangepi_send
        ${CMAKE_BINARY_DIR}/orangepi_send_build
    )
else()
    message(FATAL_ERROR "orangepi_send module not found at: ${CMAKE_CURRENT_LIST_DIR}/orangepi_send")
endif()

# =============================================================================
# 子项目：pwm_control_program（当前目录下）
# =============================================================================
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/pwm_control_program/CMakeLists.txt")
    add_subdirectory(
        ${CMAKE_CURRENT_LIST_DIR}/pwm_control_program
        ${CMAKE_BINARY_DIR}/pwm_control_program_build
    )
else()
    message(FATAL_ERROR "pwm_control_program module not found at: ${CMAKE_CURRENT_LIST_DIR}/pwm_control_program")
endif()
