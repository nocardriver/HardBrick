cmake_minimum_required(VERSION 3.22)

#
# GPS module: GPS application layer (App layer)
#
#   gps.c   NMEA byte stream -> line splitter -> minmea -> gps_nav_t snapshot
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(GPS_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(GPS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/gps.c
)

# Create GPS object library
add_library(gps OBJECT)
target_sources(gps PRIVATE ${GPS_Src})
target_include_directories(gps PUBLIC ${GPS_Inc_Dirs})

# App layer may depend on BSP and Services (AGENTS.md): threadx for the RTOS
# primitives, minmea for NMEA parsing, segger_rtt for the debug output.
target_link_libraries(gps PUBLIC
    minmea
    segger_rtt
    threadx
)
