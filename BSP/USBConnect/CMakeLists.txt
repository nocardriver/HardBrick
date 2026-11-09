cmake_minimum_required(VERSION 3.22)

#
# USBConnect module: 用 GPIO 拉低 D+ 模拟 USB 拔出（BSP 层）
#
#   usb_connect.c  D+ (PA12) 在「GPIO 输出低」与「AF10/USB」之间切换，
#                  时序（保持多久、何时让 OTG 上拉）由 Apps/usb_msc 决定。
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(USBCONNECT_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(USBCONNECT_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_connect.c
)

# Create usb_connect object library
add_library(usb_connect OBJECT)
target_sources(usb_connect PRIVATE ${USBCONNECT_Src})
target_include_directories(usb_connect PUBLIC ${USBCONNECT_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (HAL / GPIO).
target_link_libraries(usb_connect PUBLIC stm32cubemx)
