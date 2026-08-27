cmake_minimum_required(VERSION 3.22)

#
# SEGGER RTT module
#

set(SEGGER_RTT_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(SEGGER_RTT_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/SEGGER_RTT.c
    ${CMAKE_CURRENT_SOURCE_DIR}/SEGGER_RTT_printf.c
)

# Create SEGGER_RTT object library
add_library(segger_rtt OBJECT)
target_sources(segger_rtt PRIVATE ${SEGGER_RTT_Src})
target_include_directories(segger_rtt PUBLIC ${SEGGER_RTT_Inc_Dirs})
