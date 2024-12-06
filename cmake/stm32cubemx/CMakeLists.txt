cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32G474xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32G4xx_HAL_Driver/Inc
    ../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ../../Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ../../Drivers/CMSIS/Include
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qv/gnu/
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qv/config
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32g4xx_it.c
    ../../Core/Src/stm32g4xx_hal_msp.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    ../../Core/Src/system_stm32g4xx.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qv/qv.c
    ../../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qv/gnu/qv_port.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32g474xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


