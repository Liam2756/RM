cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/Src/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/Src/imu_attitude.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Src/bmi088.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Src/bsp_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/uart_ports.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/algorithm/freshness.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/algorithm/key_scan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/algorithm/lpf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/algorithm/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/application/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/application/DT7.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/BSP/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/BSP/bsp_debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/BSP/bsp_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/BSP/bsp_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/BSP/bsp_pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/BSP/filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/control/gimbal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/driver/dt7_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/driver/vision_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/driver/gyro_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/driver/hero_motor_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/driver/motor_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/driver/motor_tx.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/application
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/control
    ${CMAKE_CURRENT_SOURCE_DIR}/App/User/driver
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
