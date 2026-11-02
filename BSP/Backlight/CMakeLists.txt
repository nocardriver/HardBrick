cmake_minimum_required(VERSION 3.22)

#
# Backlight module: screen backlight PWM enable (TIM17_CH1 on PB9, BSP layer)
#
#   backlight.c   start the PWM at 0% and map 0..100% brightness to CCR1.
#                 The only translation unit that touches TIM17.
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(BACKLIGHT_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(BACKLIGHT_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/backlight.c
)

# Create backlight object library
add_library(backlight OBJECT)
target_sources(backlight PRIVATE ${BACKLIGHT_Src})
target_include_directories(backlight PUBLIC ${BACKLIGHT_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (USE_HAL_DRIVER, STM32U575xx);
# needed for the HAL/tim/main headers.
target_link_libraries(backlight PUBLIC stm32cubemx)
