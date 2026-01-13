cmake_minimum_required(VERSION 3.14)
project(pwm_control_program LANGUAGES CXX)

# =============================================================================
# Build defaults
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# =============================================================================
# Options
# =============================================================================
option(PWMCTRL_STRICT_WARNINGS "Enable strict warnings (-Wall/-Wextra/...)" ON)
option(PWMCTRL_ASAN           "Enable AddressSanitizer/UBSan (Debug only recommended)" OFF)
option(PWMCTRL_FORCE_STDCXXFS "Force link stdc++fs for old toolchains" OFF)

# Route B: GCS comm is external process via shm; pwm_control_program reads shm.
option(PWMCTRL_ENABLE_GCS_SHM "Enable GCS input via shared memory subscriber" ON)

# If you include or link gateway (for telemetry/session bridge)
option(PWMCTRL_ENABLE_GATEWAY "Enable gateway dependency (headers and/or link)" ON)

# =============================================================================
# Repo paths
# =============================================================================
# pwm_control_program/ is at: UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program
# ROV_ROOT   = OrangePi_STM32_for_ROV
# UWSYS_ROOT = UnderwaterRobotSystem
get_filename_component(ROV_ROOT    "${CMAKE_CURRENT_LIST_DIR}/.."    ABSOLUTE)
get_filename_component(UWSYS_ROOT  "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(UWSYS_SHARED_DIR "${UWSYS_ROOT}/shared")

get_filename_component(PWM_SEND_DIR "${ROV_ROOT}/orangepi_send" ABSOLUTE)
set(PWM_INCLUDE_DIR "${PWM_SEND_DIR}/include" CACHE PATH "orangepi_send include dir")
set(PWM_LIB_DIR     "${PWM_SEND_DIR}/build"   CACHE PATH "orangepi_send build dir (fallback)")

set(PROTO_GCS_DIR "${ROV_ROOT}/proto_gcs")
set(GATEWAY_DIR   "${ROV_ROOT}/gateway")

if (NOT EXISTS "${UWSYS_SHARED_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "[pwm_control_program] shared/ not found at: ${UWSYS_SHARED_DIR}")
endif()
if (NOT EXISTS "${PWM_INCLUDE_DIR}")
    message(FATAL_ERROR "[pwm_control_program] orangepi_send include dir not found: ${PWM_INCLUDE_DIR}")
endif()
if (NOT EXISTS "${PROTO_GCS_DIR}/include/proto_gcs/gcs_protocol.hpp")
    message(FATAL_ERROR "[pwm_control_program] proto_gcs header not found: ${PROTO_GCS_DIR}/include/proto_gcs/gcs_protocol.hpp")
endif()

if (PWMCTRL_ENABLE_GATEWAY)
    if (NOT EXISTS "${GATEWAY_DIR}/include/gateway/session/gcs_session.hpp")
        message(FATAL_ERROR
            "[pwm_control_program] gateway header not found: "
            "${GATEWAY_DIR}/include/gateway/session/gcs_session.hpp"
        )
    endif()
endif()

# =============================================================================
# Dependencies
# =============================================================================
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

set(YAMLCPP_TARGET yaml-cpp)
if (TARGET yaml-cpp::yaml-cpp)
    set(YAMLCPP_TARGET yaml-cpp::yaml-cpp)
elseif (TARGET yaml-cpp)
    set(YAMLCPP_TARGET yaml-cpp)
endif()

# ---- Eigen (header-only) ----
if (NOT DEFINED EIGEN3_INCLUDE_DIR)
    find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Dense
        PATHS
            /usr/include/eigen3
            /usr/local/include/eigen3
            /usr/include
            /usr/local/include
    )
endif()

if (NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR
        "[pwm_control_program] Eigen headers not found.\n"
        "Install with: sudo apt-get install -y libeigen3-dev\n"
        "Or set -DEIGEN3_INCLUDE_DIR=/path/to/eigen3"
    )
endif()

message(STATUS "[pwm_control_program] EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")

# =============================================================================
# Subprojects / exported targets (DO NOT re-create if already defined)
# =============================================================================

# ---- shared_msgs (reuse if exists; fallback add_subdirectory) ----
if (NOT TARGET shared_msgs)
    add_subdirectory(
        "${UWSYS_SHARED_DIR}"
        "${CMAKE_BINARY_DIR}/_deps/shared_msgs-build"
    )
endif()

if (NOT TARGET shared_msgs)
    message(FATAL_ERROR
        "[pwm_control_program] shared target 'shared_msgs' not found after add_subdirectory().\n"
        "Please check ${UWSYS_SHARED_DIR}/CMakeLists.txt exports a target named 'shared_msgs'."
    )
endif()

# ---- proto_gcs (reuse if exists; fallback INTERFACE include-only) ----
if (NOT TARGET proto_gcs)
    add_library(proto_gcs INTERFACE)
    target_include_directories(proto_gcs INTERFACE "${PROTO_GCS_DIR}/include")
else()
    # Ensure downstream can include "proto_gcs/gcs_protocol.hpp"
    get_target_property(_proto_gcs_inc proto_gcs INTERFACE_INCLUDE_DIRECTORIES)
    if (NOT _proto_gcs_inc)
        target_include_directories(proto_gcs INTERFACE "${PROTO_GCS_DIR}/include")
    endif()
endif()

# ---- gateway (reuse if exists; fallback add_subdirectory) ----
# We want:
#   - headers: ${GATEWAY_DIR}/include
#   - link: gateway_core (if used by pwm_control_program)
set(PWMCTRL_GATEWAY_LIB "")
if (PWMCTRL_ENABLE_GATEWAY)
    # If gateway_core is already defined by the top-level build, reuse it.
    if (TARGET gateway_core)
        set(PWMCTRL_GATEWAY_LIB gateway_core)
    else()
        # If gateway is part of this superbuild, bring it in safely.
        if (EXISTS "${GATEWAY_DIR}/CMakeLists.txt")
            add_subdirectory(
                "${GATEWAY_DIR}"
                "${CMAKE_BINARY_DIR}/_deps/gateway-build"
            )
        else()
            message(FATAL_ERROR "[pwm_control_program] gateway/CMakeLists.txt not found at: ${GATEWAY_DIR}")
        endif()

        if (TARGET gateway_core)
            set(PWMCTRL_GATEWAY_LIB gateway_core)
        else()
            # Still allow header-only usage if you insist, but warn early.
            message(WARNING
                "[pwm_control_program] gateway_core target not found after add_subdirectory(gateway).\n"
                "Will only add gateway include dirs; linking may fail if you call non-inline gateway code."
            )
        endif()
    endif()
endif()

# =============================================================================
# Sources (match your repo tree exactly)
# =============================================================================
set(SRC_APP
    src/main.cpp
    src/control_core/app_main.cpp
)

set(SRC_CONTROL_CORE
    src/control_core/control_loop.cpp
    src/control_core/control_guard.cpp
    src/control_core/thruster_allocation.cpp
    src/control_core/trajectory_tracking.cpp
    src/control_core/app_context.cpp

    src/control_core/loop/control_loop_run.cpp
    src/control_core/loop/control_loop_nav.cpp
    src/control_core/loop/control_loop_pwm_log.cpp
    src/control_core/loop/control_loop_helpers.cpp
)

set(SRC_CONTROLLERS
    src/controllers/manual_controller.cpp
    src/controllers/pid_controller.cpp
    src/controllers/controller_manager.cpp
)

set(SRC_PLATFORM
    src/platform/pwm_client.cpp
    src/platform/timebase.cpp
)

set(SRC_IO_BASE
    
    src/io/input/teleop_input.cpp
    src/io/input/telemetry_frame.cpp

    src/io/input/control_intent_wire_adapter.cpp
    src/io/input/control_intent_wire_codec.cpp
    src/io/input/multi_input_provider.cpp

    src/io/log/pwm_logger.cpp

    # RouteB can still include a telemetry adapter (for outgoing telemetry)
    
)

set(SRC_IO_SHM
    src/io/input/intent_subscriber_shm.cpp
    src/io/input/key_event_subscriber_shm.cpp
    src/io/input/gcs_shm_input_provider.cpp
    src/io/nav/nav_view_subscriber_shm.cpp
    src/io/nav/nav_view_shm_source.cpp
)

set(SRC_UTILS
    src/utils/config_loader.cpp
    src/utils/config_loader_pwm_client.cpp
    src/utils/config_loader_alloc.cpp
    src/utils/config_loader_trajectory.cpp
    src/utils/config_loader_teleop_mixer.cpp

)

set(SRC_IO ${SRC_IO_BASE})
if (PWMCTRL_ENABLE_GCS_SHM)
    list(APPEND SRC_IO ${SRC_IO_SHM})
endif()

# =============================================================================
# Target
# =============================================================================
add_executable(pwm_control_program
    ${SRC_APP}
    ${SRC_CONTROL_CORE}
    ${SRC_CONTROLLERS}
    ${SRC_PLATFORM}
    ${SRC_IO}
    ${SRC_UTILS}
)

target_include_directories(pwm_control_program
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${PWM_INCLUDE_DIR}"
        "${EIGEN3_INCLUDE_DIR}"
)

# Add gateway include dir only when enabled (so bridge headers compile)
if (PWMCTRL_ENABLE_GATEWAY)
    target_include_directories(pwm_control_program PRIVATE
        "${GATEWAY_DIR}/include"
    )
endif()

# =============================================================================
# Compile defs (feature gates)
# =============================================================================
target_compile_definitions(pwm_control_program PRIVATE
    PWMCTRL_ENABLE_GCS_SHM=$<BOOL:${PWMCTRL_ENABLE_GCS_SHM}>
    PWMCTRL_ENABLE_GATEWAY=$<BOOL:${PWMCTRL_ENABLE_GATEWAY}>
)

# =============================================================================
# Link libraries
# =============================================================================
target_link_libraries(pwm_control_program
    PRIVATE
        ${YAMLCPP_TARGET}
        Threads::Threads
        shared_msgs
        proto_gcs
)

# Link gateway_core if available and enabled
if (PWMCTRL_ENABLE_GATEWAY AND PWMCTRL_GATEWAY_LIB)
    target_link_libraries(pwm_control_program PRIVATE ${PWMCTRL_GATEWAY_LIB})
endif()

# =============================================================================
# Link libpwm_host (prefer target, fallback to prebuilt)
# =============================================================================
if (TARGET pwm_host)
    target_link_libraries(pwm_control_program PRIVATE pwm_host)
else()
    if (EXISTS "${PWM_LIB_DIR}/libpwm_host.a")
        target_link_libraries(pwm_control_program PRIVATE "${PWM_LIB_DIR}/libpwm_host.a")
    else()
        message(FATAL_ERROR
            "[pwm_control_program] Neither target 'pwm_host' nor prebuilt libpwm_host.a found.\n"
            "PWM_LIB_DIR=${PWM_LIB_DIR}\nPWM_INCLUDE_DIR=${PWM_INCLUDE_DIR}"
        )
    endif()
endif()

# =============================================================================
# Warnings / sanitizers (per-target)
# =============================================================================
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    if (PWMCTRL_STRICT_WARNINGS)
        target_compile_options(pwm_control_program PRIVATE
            -Wall -Wextra -Wpedantic
            -Wshadow -Wformat=2 -Wundef
            -Wconversion -Wsign-conversion
        )
    endif()

    if (PWMCTRL_ASAN)
        target_compile_options(pwm_control_program PRIVATE
            -fsanitize=address,undefined -fno-omit-frame-pointer
        )
        target_link_options(pwm_control_program PRIVATE
            -fsanitize=address,undefined
        )
    endif()
endif()

if (UNIX AND NOT APPLE)
    find_library(RT_LIB rt)
    if (RT_LIB)
        target_link_libraries(pwm_control_program PRIVATE ${RT_LIB})
    endif()
endif()

if (PWMCTRL_FORCE_STDCXXFS)
    target_link_libraries(pwm_control_program PRIVATE stdc++fs)
endif()

# =============================================================================
# Hard guard: forbid compiling the old internal GCS input stack (RouteB policy)
# =============================================================================
get_target_property(_pwm_srcs pwm_control_program SOURCES)
foreach(s IN LISTS _pwm_srcs)
    if (s MATCHES "/src/io/gcs/.*(link|session|udp|packet|client|server|input).*\\.cpp$")
        message(FATAL_ERROR
            "[pwm_control_program] RouteB forbids compiling old internal GCS input stack: ${s}\n"
            "Keep only gcs_telemetry_adapter.cpp (telemetry out). Input must come from SHM subscriber.")
    endif()
endforeach()

# =============================================================================
# Diagnostics
# =============================================================================
message(STATUS "[pwm_control_program] ROV_ROOT               = ${ROV_ROOT}")
message(STATUS "[pwm_control_program] UWSYS_ROOT             = ${UWSYS_ROOT}")
message(STATUS "[pwm_control_program] UWSYS_SHARED_DIR       = ${UWSYS_SHARED_DIR}")
message(STATUS "[pwm_control_program] PROTO_GCS_DIR          = ${PROTO_GCS_DIR}")
message(STATUS "[pwm_control_program] GATEWAY_DIR            = ${GATEWAY_DIR}")
message(STATUS "[pwm_control_program] PWM_INCLUDE_DIR        = ${PWM_INCLUDE_DIR}")
message(STATUS "[pwm_control_program] PWM_LIB_DIR            = ${PWM_LIB_DIR}")
message(STATUS "[pwm_control_program] YAMLCPP_TARGET         = ${YAMLCPP_TARGET}")
message(STATUS "[pwm_control_program] BUILD_TYPE             = ${CMAKE_BUILD_TYPE}")
message(STATUS "[pwm_control_program] PWMCTRL_ENABLE_GCS_SHM  = ${PWMCTRL_ENABLE_GCS_SHM}")
message(STATUS "[pwm_control_program] PWMCTRL_ENABLE_GATEWAY  = ${PWMCTRL_ENABLE_GATEWAY}")
message(STATUS "[pwm_control_program] PWMCTRL_GATEWAY_LIB     = ${PWMCTRL_GATEWAY_LIB}")
