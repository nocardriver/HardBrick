cmake_minimum_required(VERSION 3.22)

#
# USBX device stack module (Eclipse ThreadX USBX 6.5.2.202603_rel, device side)
#
# Layout mirrors the upstream source tree so future upgrades stay simple:
#   common/core/                              device stack + utilities
#   common/usbx_device_classes/               MSC (storage) class only
#   common/usbx_stm32_device_controllers/     STM32 PCD HAL glue (taken from
#                                             STM32Cube FW U5 V1.9.0, V1.3.4)
#   ports/cortex_m33/gnu/inc/                 Cortex-M33 port header
#   config/                                   ux_user.h / ux_stm32_config.h
#
# Only the device side is built: the host stack and host controller drivers
# (ux_host_*.c / ux_hcd_*.c) are excluded, which keeps the build ~100 files
# smaller. The remaining sources are dead-stripped by --gc-sections anyway.
#

set(USBX_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Device stack + utilities + trace stubs
file(GLOB USBX_Core_Src CONFIGURE_DEPENDS ${USBX_DIR}/common/core/src/*.c)
list(FILTER USBX_Core_Src EXCLUDE REGEX "/ux_host_")
list(FILTER USBX_Core_Src EXCLUDE REGEX "/ux_hcd_")

# MSC class (only class used by this project)
file(GLOB USBX_Class_Src CONFIGURE_DEPENDS
     ${USBX_DIR}/common/usbx_device_classes/src/ux_device_class_storage_*.c)

# STM32 device controller driver (HAL_PCD based)
file(GLOB USBX_Dcd_Src CONFIGURE_DEPENDS
     ${USBX_DIR}/common/usbx_stm32_device_controllers/*.c)

# IMPORTANT: this must be an OBJECT library, not STATIC.
#
# ux_dcd_stm32_callback.c only *defines* HAL_PCD_*Callback(), which the HAL
# declares as __weak stubs. Nothing in the archive references those symbols, so
# a static library member would never be pulled in and the linker would keep the
# empty weak stubs -- the device would enumerate but never answer a control
# transfer. Object libraries always link every object; --gc-sections still drops
# whatever is really unused.
add_library(usbx OBJECT)
target_sources(usbx PRIVATE
    ${USBX_Core_Src}
    ${USBX_Class_Src}
    ${USBX_Dcd_Src}
)

target_include_directories(usbx
    PUBLIC
    ${USBX_DIR}/common/core/inc
    ${USBX_DIR}/common/usbx_device_classes/inc
    ${USBX_DIR}/common/usbx_stm32_device_controllers
    ${USBX_DIR}/ports/cortex_m33/gnu/inc
    ${USBX_DIR}/config
)

# ux_port.h includes ux_user.h when UX_INCLUDE_USER_DEFINE_FILE is defined.
target_compile_definitions(usbx
    PUBLIC
    UX_INCLUDE_USER_DEFINE_FILE
)

# USBX needs the ThreadX API (threads, mutexes, ticks) and the STM32 HAL (PCD
# types in ux_stm32_config.h). Both are exported as PUBLIC so every consumer
# sees the same configuration.
target_link_libraries(usbx PUBLIC
    threadx
    stm32cubemx
)

# Vendor code is MISRA-oriented and produces a lot of -Wall noise (unused
# parameters, old-style prototypes); keep the module build quiet like threadx.
target_compile_options(usbx PRIVATE -w)
