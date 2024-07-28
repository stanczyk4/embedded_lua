cmake_minimum_required (VERSION 3.15)

target_sources (
    embedded_lua
    PRIVATE stm32cubemx/Core/Src/main.c stm32cubemx/Core/Src/system_stm32g4xx.c
            stm32cubemx/Core/Src/stm32g4xx_hal_msp.c
)

target_include_directories (
    embedded_lua PUBLIC src stm32cubemx/Core/Inc
)

target_link_options (
    embedded_lua PUBLIC -T ${CMAKE_CURRENT_LIST_DIR}/stm32cubemx/STM32G474RETx_FLASH.ld
)

set_target_properties(embedded_lua PROPERTIES OUTPUT_NAME embedded_lua.elf)

target_link_libraries (embedded_lua segger_rtt_interface)

# Override hal_conf.h that the hal defaults to using
target_compile_definitions (embedded_lua PUBLIC HSE_VALUE=24000000)
