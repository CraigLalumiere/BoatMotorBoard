cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(BoatMotorBoard)
enable_language(C ASM)

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)

set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
set(ROOT_PATH                       ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(SHARED_PATH                     ${ROOT_PATH}/shared)
set(LIBRARY_PATH                    ${ROOT_PATH}/library)
set(MIDDLEWARES_PATH                ${ROOT_PATH}/Middlewares)
set(MESSAGES_PATH                   ${ROOT_PATH}/messages/generated/c)

include(${ROOT_PATH}/messages/fetch_nanopb.cmake)

set(CPU_PARAMETERS
    -mthumb
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -fno-math-errno
    -fsingle-precision-constant
    -DARM_ARCH=4
    -DSTM32G474xx
)

add_compile_options(${CPU_PARAMETERS})

set(linker_script_SRC               ${PROJ_PATH}/src/bsp/device/stm32g474cetx_flash.ld)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

set(QPC_CFG_KERNEL qk)
set(QPC_CFG_GUI OFF)
set(QPC_CFG_UNIT_TEST OFF)
set(QPC_CFG_VERBOSE OFF)
set(QPC_CFG_PORT arm-cm)
add_subdirectory(${LIBRARY_PATH}/qpc/ qpc)

set(hal_SRCS
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
)

set(tinyusb_SRCS
    ${LIBRARY_PATH}/tinyusb-0.17.0/src/tusb.c
    ${LIBRARY_PATH}/tinyusb-0.17.0/src/common/tusb_fifo.c
    ${LIBRARY_PATH}/tinyusb-0.17.0/src/device/usbd.c
    ${LIBRARY_PATH}/tinyusb-0.17.0/src/device/usbd_control.c
    ${LIBRARY_PATH}/tinyusb-0.17.0/src/class/cdc/cdc_device.c
    ${LIBRARY_PATH}/tinyusb-0.17.0/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
)

set(messages_SRCS
    ${MESSAGES_PATH}/CLIData.pb.c
    ${MESSAGES_PATH}/ConfigDB.pb.c
    ${MESSAGES_PATH}/LogPrint.pb.c
    ${MESSAGES_PATH}/MessageType.pb.c
)

set(sources_SRCS
    ${PROJ_PATH}/src/bsp/device/startup_stm32g474xx.s

    ${PROJ_PATH}/src/stm32g4xx_it.c
    ${PROJ_PATH}/src/syscalls.c
    ${PROJ_PATH}/src/sysmem.c
    ${PROJ_PATH}/src/system_stm32g4xx.c
    ${PROJ_PATH}/src/stm32g4xx_hal_msp.c
    ${PROJ_PATH}/src/main.c
    ${PROJ_PATH}/src/interrupts.c
    ${PROJ_PATH}/src/safe_strncpy.c

    ${PROJ_PATH}/src/services/cli/cli_commands.c
    ${PROJ_PATH}/src/services/cli/cli_manual_commands.c
    ${PROJ_PATH}/src/services/blinky.c
    ${PROJ_PATH}/src/services/config.c
    ${PROJ_PATH}/src/services/director.c
    ${SHARED_PATH}/services/fault_manager.c
    ${PROJ_PATH}/src/services/flowsensor.c
    ${SHARED_PATH}/services/fram.c
    ${PROJ_PATH}/src/services/LMT01.c
    ${PROJ_PATH}/src/services/log_com.c
    ${PROJ_PATH}/src/services/pressure_sensor.c

    ${SHARED_PATH}/services/box_to_box.c
    ${SHARED_PATH}/services/reset.c
    ${SHARED_PATH}/services/reset_reason_print.c
    ${SHARED_PATH}/services/usb.c
    ${SHARED_PATH}/services/pc_com/crc16.c
    ${SHARED_PATH}/services/pc_com/hdlc.c
    ${PROJ_PATH}/src/services/pc_com.c

    ${PROJ_PATH}/src/bsp/bsp.c
    ${PROJ_PATH}/src/bsp/bsp_manual.c
    ${PROJ_PATH}/src/bsp/usb_descriptors.c

    ${SHARED_PATH}/bsp/bsp_backup_ram.c
    ${SHARED_PATH}/bsp/gpio_stm32.c
    ${SHARED_PATH}/bsp/i2c_bus.c
    ${SHARED_PATH}/bsp/i2c_bus_stm32.c
    ${SHARED_PATH}/bsp/shared_i2c.c
    ${SHARED_PATH}/bsp/usb_board_api.c

    ${hal_SRCS}
    ${tinyusb_SRCS}
    ${nanopb_SRCS}
    ${messages_SRCS}

    ${LIBRARY_PATH}/SEGGER_RTT_V810k/RTT/SEGGER_RTT.c
    ${LIBRARY_PATH}/SEGGER_RTT_V810k/RTT/SEGGER_RTT_printf.c
)

set(include_path_DIRS
    ${PROJ_PATH}/src
    ${PROJ_PATH}/src/bsp
    ${PROJ_PATH}/src/services
    ${PROJ_PATH}/src/services/cli

    ${SHARED_PATH}/bsp
    ${SHARED_PATH}/bsp/interfaces
    ${SHARED_PATH}/services
    ${SHARED_PATH}/services/pc_com
    ${ROOT_PATH}/messages/generated
    ${ROOT_PATH}/messages/generated/c
    ${nanopb_SOURCE_DIR}

    ${LIBRARY_PATH}/embedded-cli
    ${LIBRARY_PATH}/qpc/include
    ${LIBRARY_PATH}/qpc/ports/arm-cm/qk/config/
    ${LIBRARY_PATH}/qpc/ports/arm-cm/qk/gnu/
    ${LIBRARY_PATH}/tinyusb-0.17.0/src
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Inc
    ${LIBRARY_PATH}/STM32G4xx_HAL_Driver/Inc/Legacy
    ${LIBRARY_PATH}/CMSIS/Device/ST/STM32G4xx/Include
    ${LIBRARY_PATH}/CMSIS/Include
    ${MIDDLEWARES_PATH}/ST/ARM/DSP/Inc
    ${LIBRARY_PATH}/SEGGER_RTT_V810k/Config
    ${LIBRARY_PATH}/SEGGER_RTT_V810k/RTT
)

set(symbols_SYMB
    USE_HAL_DRIVER
    CFG_TUSB_MCU=OPT_MCU_STM32G4
    BOARD_MOTOR
)

add_executable(${EXECUTABLE} ${sources_SRCS})

target_include_directories(${EXECUTABLE} PRIVATE ${include_path_DIRS})
target_compile_definitions(${EXECUTABLE} PRIVATE ${symbols_SYMB})

add_compile_definitions(
    $<$<CONFIG:DEBUG>:DEBUG>
)

target_compile_options(${EXECUTABLE} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:DEBUG,RELEASE>: -O0 -g3>
)

target_link_libraries(${EXECUTABLE} PRIVATE m qpc)

target_link_options(${EXECUTABLE} PRIVATE
    -T${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    -u _printf_float
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,--print-memory-usage
)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)
