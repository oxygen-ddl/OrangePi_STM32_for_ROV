cmake_minimum_required(VERSION 3.14)
project(gateway LANGUAGES C CXX)

# ================== 全局编译选项 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 单配置生成器的默认构建类型（多配置例如 VS / Xcode 会忽略）
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(GATEWAY_BUILD_TESTS "是否编译 gateway 单元测试" ON)

# 编译告警（适度开启，避免对交叉编译工具链过于严苛）
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra)
endif()

# ================== 根目录定位 ==================
# 当前 CMakeLists.txt 路径：
#   UnderwaterRobotSystem/OrangePi_STM32_for_ROV/gateway/CMakeLists.txt
#
# 推导出：
#   ROV_ROOT   = OrangePi_STM32_for_ROV
#   UWSYS_ROOT = UnderwaterRobotSystem
get_filename_component(ROV_ROOT   "${CMAKE_CURRENT_LIST_DIR}/.."   ABSOLUTE)
get_filename_component(UWSYS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(SHARED_ROOT   "${UWSYS_ROOT}/shared")
set(PROTO_GCS_DIR "${ROV_ROOT}/proto_gcs")

# ---- 基本存在性检查（防止路径写错） ----
if (NOT EXISTS "${SHARED_ROOT}/msg" OR NOT EXISTS "${SHARED_ROOT}/shm")
    message(FATAL_ERROR
        "未找到 shared 目录: ${SHARED_ROOT} "
        "(期望存在 ${SHARED_ROOT}/msg 和 ${SHARED_ROOT}/shm)"
    )
endif()

if (NOT EXISTS "${PROTO_GCS_DIR}/include/proto_gcs/gcs_protocol.hpp")
    message(FATAL_ERROR
        "未找到 proto_gcs 头文件: ${PROTO_GCS_DIR}/include/proto_gcs/gcs_protocol.hpp"
    )
endif()

# ================== 依赖目标（如上层已定义则复用） ==================
# 说明：
#   - 在顶层工程中，一般会已有：
#       shared_msgs  (来自 ${UWSYS_ROOT}/shared)
#       proto_gcs    (来自 ${ROV_ROOT}/proto_gcs，通常为 STATIC 库)
#   - 为了让 gateway 可以“独立编译”，这里提供兜底 INTERFACE 目标。

if (NOT TARGET shared_msgs)
    add_library(shared_msgs INTERFACE)
    # 允许直接使用 #include "shared/msg/xxx.hpp"
    target_include_directories(shared_msgs INTERFACE "${UWSYS_ROOT}")
endif()

if (NOT TARGET proto_gcs)
    # 若上层未 add_subdirectory(proto_gcs)，则退化成 header-only INTERFACE
    add_library(proto_gcs INTERFACE)
    target_include_directories(proto_gcs INTERFACE "${PROTO_GCS_DIR}/include")
else()
    # 若 proto_gcs 已存在但没导出 include 目录，这里强制补上
    get_target_property(_proto_gcs_inc proto_gcs INTERFACE_INCLUDE_DIRECTORIES)
    if (NOT _proto_gcs_inc)
        target_include_directories(proto_gcs INTERFACE "${PROTO_GCS_DIR}/include")
    endif()
endif()

# ================== 接口库：gateway_proto ==================
# 仅提供头文件接口（协议 / 公共类型）
add_library(gateway_proto INTERFACE)
target_include_directories(gateway_proto INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/include"
)
target_link_libraries(gateway_proto INTERFACE
    shared_msgs
    proto_gcs
)

# ================== 核心库：gateway_core ==================
# 封装：
#   - SHM Intent / Nav 的收发
#   - UDP + 会话管理
#   - Intent 仲裁与多路复用
add_library(gateway_core
    # intent / shm
    src/intent_publisher_shm.cpp
    src/IPC/intent/intent_arbiter.cpp
    src/IPC/intent/intent_mux.cpp
    src/IPC/intent_subscriber_shm.cpp

    # 键盘事件 shm
    src/IPC/keys/key_event_publisher_shm.cpp
    src/IPC/keys/key_event_subscriber_shm.cpp

    # 导航共享内存桥接
    src/IPC/nav/nav_state_subscriber.cpp
    src/IPC/nav/nav_state_subscriber_shm.cpp
    src/IPC/nav/nav_view_builder.cpp
    src/IPC/nav/nav_view_publisher_shm.cpp

    # UDP + 会话管理
    src/udp/udp_client.cpp
    src/udp/udp_endpoint.cpp
    src/udp/udp_server.cpp
    src/session/gcs_session.cpp
)

target_link_libraries(gateway_core
    PUBLIC gateway_proto
)

# POSIX 线程
find_package(Threads REQUIRED)
target_link_libraries(gateway_core PUBLIC Threads::Threads)

# 可选：数学库（一般 C 代码里用到 libm）
if (UNIX AND NOT APPLE)
    target_link_libraries(gateway_core PUBLIC m)
endif()

# 可选：librt（某些 glibc 平台上 shm_open/shm_unlink 在 rt 中）
if (UNIX AND NOT APPLE)
    find_library(RT_LIB rt)
    if (RT_LIB)
        target_link_libraries(gateway_core PUBLIC ${RT_LIB})
    endif()
endif()

# ================== Apps 可执行程序 ==================
# 统一的 app 构建函数：
#   gateway_add_app(<名称> <源文件...>)
# 会自动链接 gateway_core
function(gateway_add_app app_name)
    add_executable(${app_name} ${ARGN})
    target_link_libraries(${app_name} PRIVATE gateway_core)
endfunction()

# ---- 1) GCS 服务器（对外 UDP 服务端）----
# 说明：
#   - apps/gcs_server.cpp 只包含 main() 和参数解析；
#   - apps/gcs_client.cpp 实现具体业务逻辑（会话回调、Intent 写 SHM 等）；
#   - 这里将两者一起编译成一个可执行文件 gcs_server。
gateway_add_app(gcs_server
    apps/gcs_server.cpp
    apps/gcs_client.cpp
)

# 旧版独立 gcs_client 可执行程序已废弃：
# gateway_add_app(gcs_client    apps/gcs_client.cpp)

# ---- 2) 本地 teleop → Intent ----
gateway_add_app(teleop_local  apps/teleop_local.cpp)

# ---- 3) 导航视图守护进程 / 调试工具 ----
gateway_add_app(nav_viewd     apps/nav_viewd.cpp)
gateway_add_app(nav_view_dump apps/nav_view_dump.cpp)

# ---- 4) Intent 守护进程 / 调试工具 ----
gateway_add_app(intentd       apps/intentd.cpp)
gateway_add_app(intent_dump   apps/intent_dump.cpp)

# ================== 测试可执行程序 ==================
if (GATEWAY_BUILD_TESTS)
    enable_testing()

    add_executable(test_codec tests/test_codec.cpp)
    target_link_libraries(test_codec PRIVATE gateway_core)
    add_test(NAME gateway_test_codec COMMAND test_codec)

    add_executable(test_session tests/test_session.cpp)
    target_link_libraries(test_session PRIVATE gateway_core)
    add_test(NAME gateway_test_session COMMAND test_session)
endif()
