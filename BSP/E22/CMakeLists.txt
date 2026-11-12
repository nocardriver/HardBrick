cmake_minimum_required(VERSION 3.22)

#
# E22 module: Ebyte E22-400T22S LoRa transceiver driver (BSP layer)
#
#   e22.c       driver core: mode control (M1/M0), reset, AUX wait, transparent
#               TX/RX ring, HEX/AT parameter access. Pure C, no HAL/RTOS include,
#               so it can move to another MCU unchanged.
#   e22_port.c  platform glue: LPUART1 receive (ReceiveToIdle IT), blocking
#               transmit, M0/M1/NRST/AUX pins, EXTI re-arm, HAL tick/delay.
#               The only translation unit allowed to include HAL headers.
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(E22_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(E22_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/e22.c
    ${CMAKE_CURRENT_SOURCE_DIR}/e22_port.c
)

# Create E22 object library
add_library(e22 OBJECT)
target_sources(e22 PRIVATE ${E22_Src})
target_include_directories(e22 PUBLIC ${E22_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (USE_HAL_DRIVER, STM32U575xx);
# needed by e22_port.c for stm32u5xx_hal.h / usart.h / main.h.
target_link_libraries(e22 PUBLIC stm32cubemx)
