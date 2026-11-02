cmake_minimum_required(VERSION 3.22)

#
# Backlight application: KEY_LIGHT (EXTI0/PA0) rising edge lights the screen
# backlight for 3 seconds (App layer).
#
#   backlight_app.c   event flag from the EXTI ISR -> thread -> BSP PWM on/off.
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(BACKLIGHT_APP_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(BACKLIGHT_APP_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/backlight_app.c
)

# Create backlight application object library
add_library(backlight_app OBJECT)
target_sources(backlight_app PRIVATE ${BACKLIGHT_APP_Src})
target_include_directories(backlight_app PUBLIC ${BACKLIGHT_APP_Inc_Dirs})

# App layer may depend on BSP and Services (AGENTS.md): backlight for the PWM,
# threadx for the event flag/thread, segger_rtt for the init failure log.
target_link_libraries(backlight_app PUBLIC
    backlight
    threadx
    segger_rtt
)
