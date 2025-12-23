cmake_minimum_required(VERSION 3.14)
project(comm_gcs LANGUAGES CXX)

# =============================================================================
# Repo paths (for shared/msg/* includes)
# =============================================================================
get_filename_component(UWSYS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(UWSYS_SHARED_DIR "${UWSYS_ROOT}/shared")

message(STATUS "[comm_gcs] UWSYS_ROOT      = ${UWSYS_ROOT}")
message(STATUS "[comm_gcs] UWSYS_SHARED_DIR= ${UWSYS_SHARED_DIR}")

# pthread / std::thread
find_package(Threads REQUIRED)

# =============================================================================
# Common include dirs
#   - ${UWSYS_ROOT} is REQUIRED for: #include "shared/msg/xxx.hpp"
# =============================================================================
set(COMM_GCS_PUBLIC_INCLUDES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${UWSYS_ROOT}
)

# =============================================================================
# 1) Base UDP transport + shm publisher: comm_gcs
# =============================================================================
add_library(comm_gcs
    src/udp_client.cpp
    src/udp_endpoint.cpp
    src/udp_server.cpp
    src/intent_publisher_shm.cpp
)

target_include_directories(comm_gcs
    PUBLIC
        ${COMM_GCS_PUBLIC_INCLUDES}
)

target_compile_features(comm_gcs PUBLIC cxx_std_17)
target_compile_options(comm_gcs PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(comm_gcs
    PUBLIC
        Threads::Threads
        proto_gcs
)

# =============================================================================
# 2) GCS protocol/session/codec: comm_gcs_gcs
# =============================================================================
add_library(comm_gcs_gcs
    src/session/gcs_session.cpp
    # src/codec/gcs_codec.cpp   # 如果后续有 .cpp，就加在这里
)

target_include_directories(comm_gcs_gcs
    PUBLIC
        ${COMM_GCS_PUBLIC_INCLUDES}
)

target_compile_features(comm_gcs_gcs PUBLIC cxx_std_17)
target_compile_options(comm_gcs_gcs PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(comm_gcs_gcs
    PUBLIC
        comm_gcs
        proto_gcs
)

# =============================================================================
# 3) Apps (optional)
# =============================================================================
option(COMM_GCS_BUILD_APPS "Build comm_gcs apps" ON)

if (COMM_GCS_BUILD_APPS)
    add_executable(gcs_server
        apps/gcs_server.cpp
    )
    target_compile_features(gcs_server PRIVATE cxx_std_17)
    target_link_libraries(gcs_server PRIVATE comm_gcs_gcs)
    target_compile_options(gcs_server PRIVATE -Wall -Wextra -Wpedantic)

    add_executable(gcs_client
        apps/gcs_client.cpp
    )
    target_compile_features(gcs_client PRIVATE cxx_std_17)
    target_link_libraries(gcs_client PRIVATE comm_gcs_gcs)
    target_compile_options(gcs_client PRIVATE -Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# 4) Tests (optional)
# =============================================================================
option(COMM_GCS_BUILD_TESTS "Build comm_gcs tests" ON)

if (COMM_GCS_BUILD_TESTS)
    enable_testing()

    add_executable(comm_gcs_test_codec
        tests/test_codec.cpp
    )
    target_compile_features(comm_gcs_test_codec PRIVATE cxx_std_17)
    target_link_libraries(comm_gcs_test_codec PRIVATE comm_gcs_gcs)
    add_test(NAME comm_gcs_test_codec COMMAND comm_gcs_test_codec)

    add_executable(comm_gcs_test_session
        tests/test_session.cpp
    )
    target_compile_features(comm_gcs_test_session PRIVATE cxx_std_17)
    target_link_libraries(comm_gcs_test_session PRIVATE comm_gcs_gcs)
    add_test(NAME comm_gcs_test_session COMMAND comm_gcs_test_session)
endif()
