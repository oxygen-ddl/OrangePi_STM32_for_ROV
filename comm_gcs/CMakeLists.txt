cmake_minimum_required(VERSION 3.16)

project(comm_gcs LANGUAGES CXX)

# ----------------------------
# Library target
# ----------------------------
add_library(comm_gcs STATIC
    src/udp_endpoint.cpp
    src/udp_server.cpp
    src/udp_client.cpp
)

add_library(comm_gcs::comm_gcs ALIAS comm_gcs)

target_compile_features(comm_gcs PUBLIC cxx_std_17)

target_include_directories(comm_gcs
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# ----------------------------
# Threads (pthreads) - important for Linux
# ----------------------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(comm_gcs PUBLIC Threads::Threads)

# ----------------------------
# Warnings (optional, keep conservative)
# ----------------------------
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(comm_gcs PRIVATE
        -Wall -Wextra -Wpedantic
    )
endif()

# ----------------------------
# Platform notes
# ----------------------------
# On Linux, sockets are in libc (no extra link libs needed).
# If you later add epoll, you still typically don't need extra libs.
