cmake_minimum_required(VERSION 3.22)

#
# SDCard module: TF 卡底层探测（BSP 层）
#
#   sdcard.c  只读验证 SDMMC1 通路：卡信息 / MBR / FAT32 引导扇区。
#             只依赖 CubeMX 生成的 hsd1，不涉及文件系统。
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(SDCARD_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(SDCARD_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/sdcard.c
)

# Create sdcard object library
add_library(sdcard OBJECT)
target_sources(sdcard PRIVATE ${SDCARD_Src})
target_include_directories(sdcard PUBLIC ${SDCARD_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols; needed for HAL/sdmmc/main headers.
target_link_libraries(sdcard PUBLIC stm32cubemx)
