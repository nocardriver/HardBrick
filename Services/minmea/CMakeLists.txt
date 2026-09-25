cmake_minimum_required(VERSION 3.22)

#
# minmea module: NMEA 0183 sentence parser (upstream v1.0.0).
#
# Third-party, platform independent library:
#   - minmea.c / minmea.h : parser core
#   - README.minmea.md    : upstream documentation
# See LICENSE.grants / COPYING for licensing (WTFPL, MIT or LGPL-3.0).
#

set(MINMEA_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(MINMEA_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/minmea.c
)

# Create minmea object library
add_library(minmea OBJECT)
target_sources(minmea PRIVATE ${MINMEA_Src})
target_include_directories(minmea PUBLIC ${MINMEA_Inc_Dirs})
