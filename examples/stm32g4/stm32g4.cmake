cmake_minimum_required (VERSION 3.23)

target_include_directories (${PROJECT_NAME} PUBLIC ${HAL_9000_INCLUDE}/vendor/stm32/stm32g4)

set (
    FREERTOS_ARCHITECTURE
    "ARM_CM4F"
    CACHE STRING "FreeRTOS Architecture" FORCE
)

set (HAL_9000_STM32_ADC_SUPPORTED TRUE)
set (HAL_9000_STM32_DAC_SUPPORTED TRUE)
set (HAL_9000_STM32_COMP_SUPPORTED TRUE)
set (HAL_9000_STM32_CAN_FD_SUPPORTED TRUE)
set (HAL_9000_STM32_CPU_SUPPORTED TRUE)
set (HAL_9000_STM32_CRC_SUPPORTED TRUE)
set (HAL_9000_STM32_DMA_SUPPORTED TRUE)
set (HAL_9000_STM32_FLASH_SUPPORTED TRUE)
set (HAL_9000_STM32_GPIO_SUPPORTED TRUE)
set (HAL_9000_STM32_I2C_MASTER_SUPPORTED TRUE)
set (HAL_9000_STM32_INPUT_CAPTURE_SUPPORTED TRUE)
set (HAL_9000_STM32_IWATCHDOG_SUPPORTED TRUE)
set (HAL_9000_STM32_LPTIMER_SUPPORTED TRUE)
set (HAL_9000_STM32_LPUART_SUPPORTED TRUE)
set (HAL_9000_STM32_OUTPUT_COMPARE_SUPPORTED TRUE)
set (HAL_9000_STM32_PWM_SUPPORTED TRUE)
set (HAL_9000_STM32_SPI_MASTER_SUPPORTED TRUE)
set (HAL_9000_STM32_SPI_SLAVE_SUPPORTED TRUE)
set (HAL_9000_STM32_TIMER_SUPPORTED TRUE)
set (HAL_9000_STM32_UART_SUPPORTED TRUE)
set (HAL_9000_STM32_WWATCHDOG_SUPPORTED TRUE)

# STM32CubeG4, https://github.com/STMicroelectronics/STM32CubeG4
cpmaddpackage ("gh:STMicroelectronics/STM32CubeG4#v1.5.1")

set (STM32G4_DRIVER_PATH ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver)
target_compile_definitions (${PROJECT_NAME} PUBLIC CMSIS_device_header="stm32g4xx.h" HAL_INTERRUPT_HANDLERS_COUNT=110)

# ${PROJECT_NAME} Source Files
list(APPEND STM32_SOURCE_FILES
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_adc_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_adc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_comp.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_cordic.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_cortex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_crc_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_crc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_cryp_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_cryp.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_dac_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_dac.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_dma_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_dma.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_exti.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_fdcan.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_flash_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_flash_ramfunc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_flash.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_fmac.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_gpio.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_hrtim.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_i2c_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_i2c.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_i2s.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_irda.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_iwdg.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_lptim.c
    # ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_msp_template.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_nand.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_nor.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_opamp_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_opamp.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_pcd_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_pcd.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_pwr_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_pwr.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_qspi.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_rcc_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_rcc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_rng.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_rtc_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_rtc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_sai_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_sai.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_smartcard_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_smartcard.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_smbus_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_smbus.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_spi_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_spi.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_sram.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_tim_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_tim.c
    # ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_timebase_tim_template.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_uart_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_uart.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_usart_ex.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_usart.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal_wwdg.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_hal.c
)

# Low Level Source files
list(APPEND STM32_SOURCE_FILES
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_adc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_comp.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_cordic.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_crc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_crs.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_dac.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_dma.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_exti.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_fmac.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_fmc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_gpio.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_hrtim.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_i2c.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_lptim.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_lpuart.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_opamp.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_pwr.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_rcc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_rng.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_rtc.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_spi.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_tim.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_ucpd.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_usart.c
    # ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_usb.c
    ${STM32G4_DRIVER_PATH}/Src/stm32g4xx_ll_utils.c
)

if (HAL_9000_COMPILE_SYSTEM_FILE)
    list(APPEND STM32_SOURCE_FILES
        ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c
    )
endif ()

target_include_directories (${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/Core/Include
    ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${STM32G4_DRIVER_PATH}/Inc
    ${STM32G4_DRIVER_PATH}/Inc/Legacy
    # ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/RTOS2/Include
    # ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
)

if (NOT HAL_9000_DONT_COMPILE_STARTUP_FILE)
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set (HAL_9000_CMSIS_BASE_FOLDER
             ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc
        )
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "IAR")
        set (HAL_9000_CMSIS_BASE_FOLDER
             ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar
        )
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "ARMCC")
        set (HAL_9000_CMSIS_BASE_FOLDER
             ${CPM_PACKAGE_STM32CubeG4_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/arm
        )
    endif ()

        list(APPEND STM32_SOURCE_FILES ${HAL_9000_CMSIS_BASE_FOLDER}/startup_stm32g474xx.s)
else ()
    message ("HAL_9000_DONT_COMPILE_STARTUP_FILE defined, user is expected to compile startup files")
endif ()

set(STM32_SOURCE_FILES ${STM32_SOURCE_FILES} CACHE INTERNAL "STM32_SOURCE_FILES")

target_sources (${PROJECT_NAME} PRIVATE ${STM32_SOURCE_FILES})
