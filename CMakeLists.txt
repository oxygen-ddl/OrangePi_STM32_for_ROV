cmake_minimum_required(VERSION 3.14)

project(ROV_PWM_STACK LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) 子项目：orangepi_send
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/OrangePi_STM32_for_ROV/orangepi_send
    ${CMAKE_BINARY_DIR}/orangepi_send_build
)

# 2) 子项目：pwm_control_program
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/pwm_control_program
    ${CMAKE_BINARY_DIR}/pwm_control_program
)
