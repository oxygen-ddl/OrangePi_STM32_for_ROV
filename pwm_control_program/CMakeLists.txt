cmake_minimum_required(VERSION 3.14)
project(pwm_control_program LANGUAGES CXX)

# =============================================================================
# Build defaults
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# =============================================================================
# Repo paths
# =============================================================================
get_filename_component(UWSYS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(UWSYS_SHARED_DIR "${UWSYS_ROOT}/shared")

get_filename_component(PWM_SEND_DIR "${CMAKE_CURRENT_LIST_DIR}/../orangepi_send" ABSOLUTE)
set(PWM_INCLUDE_DIR "${PWM_SEND_DIR}/include" CACHE PATH "orangepi_send include dir")
set(PWM_LIB_DIR     "${PWM_SEND_DIR}/build"   CACHE PATH "orangepi_send build dir (fallback)")

if (NOT EXISTS "${UWSYS_SHARED_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "[pwm_control_program] shared/ not found at: ${UWSYS_SHARED_DIR}")
endif()
if (NOT EXISTS "${PWM_INCLUDE_DIR}")
    message(FATAL_ERROR "[pwm_control_program] orangepi_send include dir not found: ${PWM_INCLUDE_DIR}")
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

# ---- shared_msgs (fallback add_subdirectory) ----
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

# =============================================================================
# Sources (grouped)
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

# IO: Route B -> NO internal src/io/gcs/*
set(SRC_IO
    src/io/teleop/teleop_keyboard.cpp
    src/io/input/teleop_input.cpp

    src/io/input/telemetry_frame.cpp

    # wire adapter (internal ControlIntent <-> shared/msg ControlIntent)
    src/io/input/control_intent_wire_adapter.cpp

    # multi input arbitration
    src/io/input/multi_input_provider.cpp

    # shm-based GCS input provider (enabled by option)
    src/io/input/gcs_intent_subscriber_shm.cpp
    src/io/input/gcs_shm_input_provider.cpp

    # NOTE: keep gcs_intent_wire.cpp ONLY if it is your "wire codec" file
    # and does NOT include/compile old io/gcs/* stack.
    # If you already renamed it or it conflicts, remove this line accordingly.
    

    src/io/nav/nav_state_source_shm.cpp
    src/io/nav/nav_state_subscriber.cpp

    src/io/log/pwm_logger.cpp
)

set(SRC_UTILS
    src/utils/config_loader.cpp
    src/utils/config_loader_pwm_client.cpp
    src/utils/config_loader_alloc.cpp
    src/utils/config_loader_trajectory.cpp
)

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

# =============================================================================
# Compile defs (feature gates)
# =============================================================================
if (PWMCTRL_ENABLE_GCS_SHM)
    target_compile_definitions(pwm_control_program PRIVATE PWMCTRL_ENABLE_GCS_SHM=1)
else()
    target_compile_definitions(pwm_control_program PRIVATE PWMCTRL_ENABLE_GCS_SHM=0)
endif()

# =============================================================================
# Link libraries
# =============================================================================
target_link_libraries(pwm_control_program
    PRIVATE
        ${YAMLCPP_TARGET}
        Threads::Threads
        shared_msgs
)

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
# Hard guard: forbid internal src/io/gcs/*
# =============================================================================
get_target_property(_pwm_srcs pwm_control_program SOURCES)
foreach(s IN LISTS _pwm_srcs)
    if (s MATCHES "/src/io/gcs/")
        message(FATAL_ERROR
            "[pwm_control_program] Route B forbids compiling internal src/io/gcs/* : ${s}\n"
            "Remove it from sources; use shm-based GCS input instead.")
    endif()
endforeach()

# =============================================================================
# Diagnostics
# =============================================================================
message(STATUS "[pwm_control_program] UWSYS_ROOT           = ${UWSYS_ROOT}")
message(STATUS "[pwm_control_program] UWSYS_SHARED_DIR     = ${UWSYS_SHARED_DIR}")
message(STATUS "[pwm_control_program] PWM_INCLUDE_DIR      = ${PWM_INCLUDE_DIR}")
message(STATUS "[pwm_control_program] PWM_LIB_DIR          = ${PWM_LIB_DIR}")
message(STATUS "[pwm_control_program] YAMLCPP_TARGET       = ${YAMLCPP_TARGET}")
message(STATUS "[pwm_control_program] BUILD_TYPE           = ${CMAKE_BUILD_TYPE}")
message(STATUS "[pwm_control_program] ENABLE_GCS_SHM(RouteB)= ${PWMCTRL_ENABLE_GCS_SHM}")
