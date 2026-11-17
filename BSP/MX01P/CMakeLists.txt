cmake_minimum_required(VERSION 3.22)

#
# MX01P module: 妙享科技 MX-01P BLE 5.3 transparent bridge (BSP layer)
#
#   mx01p.c       driver core: RES/BRTS control, +READY handshake, RX demux
#                 (control line vs transparent payload), synchronous AT engine
#                 with a single-slot reply mailbox. Pure C, no HAL/RTOS include,
#                 so it can move to another MCU unchanged.
#   mx01p_port.c  platform glue: USART1 receive (ReceiveToIdle IT), blocking
#                 transmit, RES/BRTS pins, HAL tick/delay, IRQ critical section.
#                 The only translation unit allowed to include HAL headers.
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(MX01P_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(MX01P_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/mx01p.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mx01p_port.c
)

# Create MX01P object library
add_library(mx01p OBJECT)
target_sources(mx01p PRIVATE ${MX01P_Src})
target_include_directories(mx01p PUBLIC ${MX01P_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (USE_HAL_DRIVER, STM32U575xx);
# needed by mx01p_port.c for stm32u5xx_hal.h / usart.h / main.h.
target_link_libraries(mx01p PUBLIC stm32cubemx)
