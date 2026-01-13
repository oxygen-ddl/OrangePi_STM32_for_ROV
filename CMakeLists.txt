cmake_minimum_required(VERSION 3.14)
project(ROV_PWM_STACK LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- unify roots ----
get_filename_component(UWSYS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(UWSYS_ROOT "${UWSYS_ROOT}" CACHE PATH "UnderwaterRobotSystem root")

set(UWSYS_SHARED_DIR "${UWSYS_ROOT}/shared" CACHE PATH "shared module dir")

# ---- unify outputs ----
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ---- global options ----
option(ROV_BUILD_TESTS "Build all tests" OFF)
set(GATEWAY_BUILD_TESTS ${ROV_BUILD_TESTS} CACHE BOOL "" FORCE)

find_package(Threads REQUIRED)

# =============================================================================
# shared
# =============================================================================
if (EXISTS "${UWSYS_SHARED_DIR}/CMakeLists.txt")
    add_subdirectory(
        ${UWSYS_SHARED_DIR}
        ${CMAKE_BINARY_DIR}/shared_build
    )
else()
    message(FATAL_ERROR "Shared module not found at: ${UWSYS_SHARED_DIR}")
endif()

# =============================================================================
# proto_gcs
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
# gateway (optional)
# =============================================================================
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/gateway/CMakeLists.txt")
    add_subdirectory(
        ${CMAKE_CURRENT_LIST_DIR}/gateway
        ${CMAKE_BINARY_DIR}/gateway_build
    )
else()
    message(STATUS "gateway module not found at: ${CMAKE_CURRENT_LIST_DIR}/gateway (skip)")
endif()

# =============================================================================
# orangepi_send
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
# pwm_control_program
# =============================================================================
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/pwm_control_program/CMakeLists.txt")
    add_subdirectory(
        ${CMAKE_CURRENT_LIST_DIR}/pwm_control_program
        ${CMAKE_BINARY_DIR}/pwm_control_program_build
    )
else()
    message(FATAL_ERROR "pwm_control_program module not found at: ${CMAKE_CURRENT_LIST_DIR}/pwm_control_program")
endif()
