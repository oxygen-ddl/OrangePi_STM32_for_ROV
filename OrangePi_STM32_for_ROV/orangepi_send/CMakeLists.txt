cmake_minimum_required(VERSION 3.10)
project(orangepi_pwm_driver LANGUAGES C)

# ================== 语言与标准 ==================
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ================== 编译选项 ==================
if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
  add_compile_options(-Wshadow -Wformat=2 -Wundef)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
  endif()
endif()

add_compile_definitions(_POSIX_C_SOURCE=200809L)

# ================== 静态库：底层 PWM 驱动 ==================
add_library(pwm_host STATIC
  src/libpwm_host.c
  src/pwm_control.c
)

target_include_directories(pwm_host PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)
target_link_libraries(pwm_host PUBLIC Threads::Threads)

include(CheckLibraryExists)
check_library_exists(rt clock_gettime "" HAVE_LIBRT)
if (HAVE_LIBRT)
  target_link_libraries(pwm_host PUBLIC rt)
endif()

set_property(TARGET pwm_host PROPERTY POSITION_INDEPENDENT_CODE ON)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C: ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}")
