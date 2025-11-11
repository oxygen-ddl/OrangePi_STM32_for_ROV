cmake_minimum_required(VERSION 3.10)
project(pwm_udp_sender LANGUAGES C CXX)

# ================== 语言与标准 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ================== 编译选项（更稳更严） ==================
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
  add_compile_options(-Wshadow -Wformat=2 -Wundef)
  # Debug/Release 默认优化
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
  endif()
endif()

# 为 clock_gettime、pthread 等 POSIX API 启用宏（有些平台需要）
add_compile_definitions(_POSIX_C_SOURCE=200809L)

# ================== 源码清单 ==================
# 建议切换到 protocol_pack（主机侧轻量打包），移除旧的 PwmFrameBuilder/重复的 CRC
set(SRCS
  src/main.cpp
  src/UdpSender.cpp
  src/protocol_pack.c       # 👈 新增：上位机打包（C）
)

# 如果你暂时还在用旧的 PwmFrameBuilder，请取消上面这一行并改用下面两行：
# list(APPEND SRCS src/PwmFrameBuilder.cpp src/crc16_ccitt.cpp)

add_executable(pwm_udp_sender ${SRCS})

# ================== 头文件路径 ==================
target_include_directories(pwm_udp_sender PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ================== 线程 & 实时库（Linux） ==================
find_package(Threads REQUIRED)
target_link_libraries(pwm_udp_sender PRIVATE Threads::Threads)

# 老旧系统上 clock_gettime 可能还在 librt，做一次检查后有则链接
include(CheckLibraryExists)
check_library_exists(rt clock_gettime "" HAVE_LIBRT)
if (HAVE_LIBRT)
  target_link_libraries(pwm_udp_sender PRIVATE rt)
endif()

# ================== 平台特性（Linux 优化） ==================
if (UNIX AND NOT APPLE)
  # 禁止生成可执行栈，提高安全
  target_link_options(pwm_udp_sender PRIVATE "-Wl,-z,noexecstack")
endif()

# ================== 可选：更严格/更稳的构建开关 ==================
option(ENABLE_LTO "Enable link-time optimization if supported" ON)
if (ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_err)
  if (ipo_ok)
    set_property(TARGET pwm_udp_sender PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

# ================== 打印构建信息 ==================
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "CXX: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
