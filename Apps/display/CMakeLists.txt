cmake_minimum_required(VERSION 3.22)

#
# Display module: portrait GPS dashboard on the ILI9341 panel (App layer)
#
#   display.c       ThreadX thread, layout, formatting, dirty-field refresh
#   display_text.c  5x5 bitmap font -> RGB565 big-endian text buffer
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(DISPLAY_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(DISPLAY_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/display.c
    ${CMAKE_CURRENT_SOURCE_DIR}/display_text.c
)

# Create display object library
add_library(display OBJECT)
target_sources(display PRIVATE ${DISPLAY_Src})
target_include_directories(display PUBLIC ${DISPLAY_Inc_Dirs})

# App layer may depend on BSP and Services (AGENTS.md): ili9341 for the panel,
# gps for the snapshot, threadx for the thread, segger_rtt for the debug log.
target_link_libraries(display PUBLIC
    ili9341
    gps
    threadx
    segger_rtt
)
