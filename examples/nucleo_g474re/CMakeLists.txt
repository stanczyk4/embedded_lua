cmake_minimum_required (VERSION 3.15)

target_sources (
    ${PROJECT_NAME}
    PRIVATE stm32cubemx/Core/Src/main.c stm32cubemx/Core/Src/system_stm32g4xx.c
            stm32cubemx/Core/Src/stm32g4xx_hal_msp.c
)

target_include_directories (
    ${PROJECT_NAME} PUBLIC src stm32cubemx/Core/Inc
)

target_link_options (
    ${PROJECT_NAME} PUBLIC -T ${CMAKE_CURRENT_LIST_DIR}/stm32cubemx/STM32G474RETx_FLASH.ld
)

target_link_libraries (${PROJECT_NAME} segger_rtt_interface)

# Override hal_conf.h that the hal defaults to using
target_compile_definitions (${PROJECT_NAME} PUBLIC HSE_VALUE=24000000)
