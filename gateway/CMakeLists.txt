cmake_minimum_required(VERSION 3.14)
project(gateway LANGUAGES C CXX)

# ================== Global ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default build type (single-config generators)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(GATEWAY_BUILD_TESTS "Build gateway tests" ON)

# Helpful warnings (soft; avoid being too aggressive for embedded toolchains)
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra)
endif()

# ================== Locate roots ==================
# gateway/ is at: UnderwaterRobotSystem/OrangePi_STM32_for_ROV/gateway
#  - ROV_ROOT   = OrangePi_STM32_for_ROV
#  - UWSYS_ROOT = UnderwaterRobotSystem
get_filename_component(ROV_ROOT   "${CMAKE_CURRENT_LIST_DIR}/.."   ABSOLUTE)
get_filename_component(UWSYS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(SHARED_ROOT   "${UWSYS_ROOT}/shared")
set(PROTO_GCS_DIR "${ROV_ROOT}/proto_gcs")

# ---- sanity checks (filesystem) ----
if (NOT EXISTS "${SHARED_ROOT}/msg" OR NOT EXISTS "${SHARED_ROOT}/shm")
    message(FATAL_ERROR
        "shared not found at ${SHARED_ROOT} "
        "(expected ${SHARED_ROOT}/msg and ${SHARED_ROOT}/shm)")
endif()

if (NOT EXISTS "${PROTO_GCS_DIR}/include/proto_gcs/gcs_protocol.hpp")
    message(FATAL_ERROR
        "proto_gcs header not found: ${PROTO_GCS_DIR}/include/proto_gcs/gcs_protocol.hpp")
endif()

# ================== Dependencies targets (REUSE if already defined) ==================
# IMPORTANT:
# In your top-level build, these targets already exist:
#   - shared_msgs (from ${UWSYS_ROOT}/shared)
#   - proto_gcs   (from ${ROV_ROOT}/proto_gcs)  [STATIC library in your build]
#
# For gateway standalone build, we provide minimal fallback INTERFACE targets.

if (NOT TARGET shared_msgs)
    add_library(shared_msgs INTERFACE)
    # allow includes like: #include "shared/msg/xxx.hpp"
    target_include_directories(shared_msgs INTERFACE "${UWSYS_ROOT}")
endif()

if (NOT TARGET proto_gcs)
    # Fallback as header-only if proto_gcs subdir is not added in this build.
    add_library(proto_gcs INTERFACE)
    target_include_directories(proto_gcs INTERFACE "${PROTO_GCS_DIR}/include")
else()
    # If proto_gcs exists but doesn't export include dirs (STATIC lib case),
    # ensure it can still be found by dependents.
    # This is harmless if include dirs are already set.
    get_target_property(_proto_gcs_inc proto_gcs INTERFACE_INCLUDE_DIRECTORIES)
    if (NOT _proto_gcs_inc)
        target_include_directories(proto_gcs INTERFACE "${PROTO_GCS_DIR}/include")
    endif()
endif()

# ================== Interface: gateway_proto ==================
add_library(gateway_proto INTERFACE)
target_include_directories(gateway_proto INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/include"
)
target_link_libraries(gateway_proto INTERFACE
    shared_msgs
    proto_gcs
)

# ================== Library: gateway_core ==================
add_library(gateway_core
    # core shm + protocol
    src/intent_publisher_shm.cpp
    src/IPC/intent_subscriber_shm.cpp

    # intent arbiter/mux
    src/IPC/intent/intent_arbiter.cpp
    src/IPC/intent/intent_mux.cpp

    # key event publisher
    src/IPC/keys/key_event_publisher_shm.cpp
    src/IPC/keys/key_event_subscriber_shm.cpp

    # nav bridge
    src/IPC/nav/nav_state_subscriber.cpp
    src/IPC/nav/nav_state_subscriber_shm.cpp
    src/IPC/nav/nav_view_builder.cpp
    src/IPC/nav/nav_view_publisher_shm.cpp

    # udp + session
    src/udp/udp_client.cpp
    src/udp/udp_endpoint.cpp
    src/udp/udp_server.cpp
    src/session/gcs_session.cpp
)

target_link_libraries(gateway_core
    PUBLIC gateway_proto
)

# Prefer Threads::Threads over raw pthread
find_package(Threads REQUIRED)
target_link_libraries(gateway_core PUBLIC Threads::Threads)

# Optional: math library (mostly relevant when you have C files using libm APIs)
if (UNIX AND NOT APPLE)
    target_link_libraries(gateway_core PUBLIC m)
endif()

# POSIX shm_open/shm_unlink are in librt on some Linux/glibc setups.
# On modern glibc they may be in libc; so link rt only if it exists.
if (UNIX AND NOT APPLE)
    find_library(RT_LIB rt)
    if (RT_LIB)
        target_link_libraries(gateway_core PUBLIC ${RT_LIB})
    endif()
endif()

# ================== Apps ==================
function(gateway_add_app app_name app_src)
    add_executable(${app_name} ${app_src})
    target_link_libraries(${app_name} PRIVATE gateway_core)
endfunction()

gateway_add_app(gcs_server    apps/gcs_server.cpp)
gateway_add_app(gcs_client    apps/gcs_client.cpp)
gateway_add_app(teleop_local  apps/teleop_local.cpp)
gateway_add_app(nav_viewd     apps/nav_viewd.cpp)
gateway_add_app(nav_view_dump apps/nav_view_dump.cpp)
gateway_add_app(intentd       apps/intentd.cpp)
gateway_add_app(intent_dump   apps/intent_dump.cpp)

# ================== Tests ==================
if (GATEWAY_BUILD_TESTS)
    enable_testing()

    add_executable(test_codec tests/test_codec.cpp)
    target_link_libraries(test_codec PRIVATE gateway_core)
    add_test(NAME gateway_test_codec COMMAND test_codec)

    add_executable(test_session tests/test_session.cpp)
    target_link_libraries(test_session PRIVATE gateway_core)
    add_test(NAME gateway_test_session COMMAND test_session)
endif()
