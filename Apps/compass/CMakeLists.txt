cmake_minimum_required(VERSION 3.22)

#
# Compass module: QMC5883P 地磁罗盘应用层（App layer）
#
#   compass.c  25 Hz 采样线程 + 硬磁自动标定 + 航向角，快照用互斥量保护。
#              传感器配置与寄存器访问全部走 BSP/QMC5883P，本层不碰 HAL。
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(COMPASS_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(COMPASS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/compass.c
)

# Create compass object library
add_library(compass OBJECT)
target_sources(compass PRIVATE ${COMPASS_Src})
target_include_directories(compass PUBLIC ${COMPASS_Inc_Dirs})

# App layer may depend on BSP and Services (AGENTS.md): qmc5883p for the sensor,
# threadx for the RTOS primitives, segger_rtt for the 1 Hz summary.
target_link_libraries(compass PUBLIC
    qmc5883p
    param_store
    segger_rtt
    threadx
)
