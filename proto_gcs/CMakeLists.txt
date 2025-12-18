cmake_minimum_required(VERSION 3.16)
project(proto_gcs LANGUAGES CXX)

add_library(proto_gcs STATIC
    src/crc32c.cpp
)

target_compile_features(proto_gcs PUBLIC cxx_std_17)

# 导出 include/proto_gcs/... 这条路径
target_include_directories(proto_gcs
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 可选：warning（协议库一般很稳定，也可以不加）
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(proto_gcs PRIVATE -Wall -Wextra -Wpedantic)
endif()
