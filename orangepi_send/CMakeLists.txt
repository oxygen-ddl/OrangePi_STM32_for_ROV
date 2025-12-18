cmake_minimum_required(VERSION 3.10)
project(orangepi_send LANGUAGES C CXX)

# --- Standards ---
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Common compile defs ---
add_compile_definitions(_POSIX_C_SOURCE=200809L)

# --- Warnings: 建议放到 target_compile_options，不要全局 add_compile_options ---
function(enable_strict_warnings tgt)
  if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${tgt} PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
      -Wshadow -Wformat=2 -Wundef
    )
  endif()
endfunction()

find_package(Threads REQUIRED)

include(CheckLibraryExists)
check_library_exists(rt clock_gettime "" HAVE_LIBRT)

# ============================================================
# 1) pwm_host: 纯 C “底层 PWM host”
# ============================================================
add_library(pwm_host STATIC
  src/libpwm_host.c
  src/pwm_control.c
)

target_include_directories(pwm_host PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pwm_host PUBLIC Threads::Threads)
if (HAVE_LIBRT)
  target_link_libraries(pwm_host PUBLIC rt)
endif()
set_property(TARGET pwm_host PROPERTY POSITION_INDEPENDENT_CODE ON)
enable_strict_warnings(pwm_host)

# ============================================================
# 2) pwm_proto: 协议/CRC/UDP/FrameBuilder（供上层 pwm_client 调用）
#    注意：这里不要把 main.cpp 编进来
# ============================================================
add_library(pwm_proto STATIC
  src/crc16_ccitt.cpp
  src/protocol_pack.c
  src/PwmFrameBuilder.cpp
  src/UdpSender.cpp
)

target_include_directories(pwm_proto PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pwm_proto PUBLIC Threads::Threads)
if (HAVE_LIBRT)
  target_link_libraries(pwm_proto PUBLIC rt)
endif()
set_property(TARGET pwm_proto PROPERTY POSITION_INDEPENDENT_CODE ON)
enable_strict_warnings(pwm_proto)

# ============================================================
# 3) demo（可选）
# ============================================================
option(BUILD_PWM_PROTO_DEMO "Build orangepi_send demo executable" OFF)
if (BUILD_PWM_PROTO_DEMO)
  add_executable(pwm_proto_demo src/main.cpp)
  target_link_libraries(pwm_proto_demo PRIVATE pwm_proto pwm_host)
  target_include_directories(pwm_proto_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  enable_strict_warnings(pwm_proto_demo)
endif()
