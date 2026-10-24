cmake_minimum_required(VERSION 3.22)

#
# ILI9341 module: KD035QVRMA223 3.5" 240x320 TFT driver (BSP layer)
#
#   ili9341.c       driver core: init sequence, address window, fill, blit.
#                   No HAL/RTOS dependency, reusable as-is under LVGL's flush cb.
#   ili9341_port.c  platform glue: SPI1 blocking transmit, CS/DC/RST, HAL_Delay.
#                   The only translation unit allowed to include HAL headers.
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(ILI9341_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(ILI9341_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/ili9341.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ili9341_port.c
)

# Create ILI9341 object library
add_library(ili9341 OBJECT)
target_sources(ili9341 PRIVATE ${ILI9341_Src})
target_include_directories(ili9341 PUBLIC ${ILI9341_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (USE_HAL_DRIVER, STM32U575xx);
# needed by ili9341_port.c for the HAL/spi/main headers.
target_link_libraries(ili9341 PUBLIC stm32cubemx)
