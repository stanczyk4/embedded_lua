cmake_minimum_required(VERSION 3.26)

project(example C CXX ASM)

add_executable(${PROJECT_NAME}
    main.cpp
    common/sys.cpp
)

target_link_libraries(${PROJECT_NAME} embedded_lua)

add_subdirectory(nucleo_g474re)

include(${CMAKE_CURRENT_LIST_DIR}/stm32g4/stm32g4.cmake)

target_include_directories(${PROJECT_NAME} PUBLIC stm32g4)

target_compile_definitions(
    ${PROJECT_NAME} PUBLIC "-DSTM32G473xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER")

target_link_options(
    ${PROJECT_NAME}
  PUBLIC
  "LINKER:-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map,--cref"
  "-u_printf_float"
  "LINKER:--wrap,malloc"
  "LINKER:--wrap,realloc"
  "LINKER:--wrap,free"
)

include(common/SEGGER_RTT/segger_rtt.cmake)
