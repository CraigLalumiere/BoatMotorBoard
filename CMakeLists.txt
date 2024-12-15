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
set(CMAKE_PROJECT_NAME CubeMX)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

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
    
    ./Core/bsp/drivers/bsp.c
    ./Core/bsp/drivers/bsp_manual.c
    ./Core/bsp/drivers/shared_i2c.c
    ./Core/Src/blinky.c
    ./Core/Src/pressure_sensor.c
    ./Core/Src/SSD1306.c
    ./Core/Src/ssd1306_fonts.c
    ./Core/Src/LMT01.c
    ./Core/Src/data_manager.c
    ./Core/bsp/drivers/usb_board_api.c
    ./Core/bsp/drivers/usb_descriptors.c
    ./Core/bsp/drivers/usb.c
    ./Core/services/app_cli/app_cli.c
    ./Core/services/app_cli/app_cli_commands.c
    ./Core/services/app_cli/app_cli_manual_commands.c
    ./Core/services/fault_manager.c
    ./Core/services/safe_strncpy.c
    ./Core/bsp/drivers/i2c_bus.c
    ./Core/bsp/drivers/i2c_bus_stm32.c

    # Explicitly list required TinyUSB sources
    #   It might be preferable to do this with tinyusb/src/CMakeLists.txt, but this works 
    ./library/tinyusb/src/tusb.c
    ./library/tinyusb/src/common/tusb_fifo.c
    ./library/tinyusb/src/device/usbd.c
    ./library/tinyusb/src/device/usbd_control.c
    ./library/tinyusb/src/class/cdc/cdc_device.c
    ./library/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ./library/tinyusb/src
    ./library/embedded-cli
    ./Core/services
    ./Core/services/app_cli
    ./Core/bsp
    ./Core/bsp/interfaces
    ./Core/bsp/drivers
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
