/**
 * @file    ux_stm32_config.h
 * @brief   STM32 device controller driver configuration for USBX
 *
 * The ST driver (common/usbx_stm32_device_controllers/) includes this header
 * from ux_dcd_stm32.h. Only two knobs are needed for this project:
 *
 *  - UX_DCD_STM32_MAX_ED: number of endpoint *numbers* the DCD tracks. The
 *    OTG_FS core of the STM32U575 exposes 6 bidirectional endpoints, which is
 *    what CubeMX configured (dev_endpoints = 6 in usb_otg.c).
 *  - UX_HCD_STM32_MAX_NB_CHANNELS: host-mode channels; unused here because the
 *    device only builds the DCD, kept so the shared header stays complete.
 */

#ifndef UX_STM32_CONFIG_H
#define UX_STM32_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"

/** 端点号数量（EP0..EP5），与 CubeMX 中 dev_endpoints = 6 对应 */
#define UX_DCD_STM32_MAX_ED                   6

/** 主机模式通道数；本工程只用设备模式，保留以兼容上游头文件 */
#define UX_HCD_STM32_MAX_NB_CHANNELS          12

#ifdef __cplusplus
}
#endif

#endif /* UX_STM32_CONFIG_H */
