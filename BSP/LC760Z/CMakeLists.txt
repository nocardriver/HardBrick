cmake_minimum_required(VERSION 3.22)

#
# LC760Z module: Quectel LC760Z(00) GNSS driver (BSP layer)
#
#   LC760Z.c        driver core: instance management, receive ring, snapshot API
#   LC760Z_port.c   platform glue: USART2 DMA receive, GPS_NRST - the only
#                   translation unit allowed to include HAL headers
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
# When the planned split is done (lc760z_nmea.c, later lc760z_bin.c), append
# the new file here.
#

set(LC760Z_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(LC760Z_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/LC760Z.c
    ${CMAKE_CURRENT_SOURCE_DIR}/LC760Z_port.c
)

# Create LC760Z object library
add_library(lc760z OBJECT)
target_sources(lc760z PRIVATE ${LC760Z_Src})
target_include_directories(lc760z PUBLIC ${LC760Z_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (USE_HAL_DRIVER, STM32U575xx);
# needed by LC760Z_port.c for the HAL/usart/main headers.
target_link_libraries(lc760z PUBLIC stm32cubemx)
