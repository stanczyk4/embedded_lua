cmake_minimum_required(VERSION 3.19.0)

#TODO this file should be converted into a function

add_library(segger_rtt_interface INTERFACE)
add_library(segger_rtt::interface ALIAS segger_rtt_interface)

target_sources(segger_rtt_interface INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT.c
  ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT_printf.c
)

target_include_directories(segger_rtt_interface INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/RTT
  ${CMAKE_CURRENT_LIST_DIR}/Config
)

if("${CMAKE_C_COMPILER_ID}" MATCHES "GNU")
  target_sources(segger_rtt_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/Syscalls/SEGGER_RTT_Syscalls_GCC.c
    ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT_ASM_ARMv7M.s  
  )
elseif("${CMAKE_C_COMPILER_ID}" MATCHES "IAR")
  target_sources(segger_rtt_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/Syscalls/SEGGER_RTT_Syscalls_IAR.c
    ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT_ASM_ARMv7M.s 
  )
elseif("${CMAKE_C_COMPILER_ID}" MATCHES "ARMCC")
  target_sources(segger_rtt_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/Syscalls/SEGGER_RTT_Syscalls_KEIL.c
    ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT_ASM_ARMv7M.s 
  )
elseif("${CMAKE_C_COMPILER_ID}" MATCHES "ARMClang")
  target_sources(segger_rtt_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/Syscalls/SEGGER_RTT_Syscalls_KEIL.c
    # ${CMAKE_CURRENT_LIST_DIR}/RTT/SEGGER_RTT_ASM_ARMv7M.s  #TODO according to segger this should work, but need to figure out why its not compiling
  )
endif()