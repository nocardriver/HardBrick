cmake_minimum_required(VERSION 3.22)

#
# usb_msc application: 把 TF 卡作为 U 盘暴露给 PC（App 层，USBX MSC/BOT）
#
#   usb_msc.c              USBX 设备栈启动、接入/拔出、重新枚举线程
#   usb_msc_descriptors.c  设备/配置/字符串描述符（全速、单配置）
#   usb_msc_media.c        MSC 介质回调 → BSP/SDCard 块接口 + 媒体互斥量
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(USB_MSC_App_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(USB_MSC_App_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_msc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_msc_descriptors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_msc_media.c
)

# Create usb_msc application object library
add_library(usb_msc OBJECT)
target_sources(usb_msc PRIVATE ${USB_MSC_App_Src})
target_include_directories(usb_msc PUBLIC ${USB_MSC_App_Inc_Dirs})

# App layer dependencies (AGENTS.md): USBX + ThreadX from Services, SDCard and
# USBConnect from BSP, SEGGER_RTT for the start-up log. stm32cubemx comes in
# transitively but is listed for clarity (HAL_PCD / HAL_SD handles).
target_link_libraries(usb_msc PUBLIC
    usbx
    threadx
    sdcard
    usb_connect
    segger_rtt
    stm32cubemx
)
