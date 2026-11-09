/**
 * @file    usb_msc_media.h
 * @brief   MSC 介质回调：把 USBX 的 SCSI 读写接到 BSP/SDCard 的块接口
 *
 * 内部头文件，只给 Apps/usb_msc 内部使用；对外的接口（互斥量、主机占用标志）
 * 在 usb_msc.h 里。
 */

#ifndef USB_MSC_MEDIA_H
#define USB_MSC_MEDIA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ux_api.h"
#include "ux_device_class_storage.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  USB 链路事件（由 usb_msc.c 从 ST DCD 的回调里转译过来）
 * @note   这些事件来自 OTG 中断上下文：handler 里只置标志，真正的判断在
 *         usb_msc_media_poll() 里做。
 */
typedef enum {
  USB_MSC_LINK_SUSPENDED = 0,  /**< 总线挂起（主机 3 ms 没发 SOF）：可能拔线、可能是主机 selective suspend */
  USB_MSC_LINK_RESUMED,        /**< 主机恢复发帧 */
  USB_MSC_LINK_ATTACHED,       /**< 总线复位后重新接入（设备栈准备好接收 SETUP） */
  USB_MSC_LINK_DETACHED,       /**< 会话结束（需要 VBUS 检测，本板未开） */
} usb_msc_link_event_t;

/**
 * @brief  上报一个链路事件（可在中断上下文调用）
 * @param  event 事件类型
 */
void usb_msc_media_on_link_event(usb_msc_link_event_t event);

/**
 * @brief  当前总线是否还在活动（主机在发 SOF）
 * @retval true  主机在总线上
 * @note   给诊断用：拔线后应该变成 false
 */
bool usb_msc_media_bus_active(void);

/**
 * @brief  创建块设备互斥量与内部状态（在 usb_msc_init 里调用）
 * @retval true  互斥量就绪
 * @retval false ThreadX 对象创建失败
 */
bool usb_msc_media_create(void);

/**
 * @brief  填写 MSC 类的注册参数（LUN、回调、介质属性）
 * @param  parameter 调用者提供的参数结构体
 * @note   会先读一次卡容量；读不到时 last_lba = 0，主机侧表现为"无介质"。
 */
void usb_msc_media_fill_parameter(UX_SLAVE_CLASS_STORAGE_PARAMETER *parameter);

/**
 * @brief  周期检查卡片状态（在 usb_msc 线程的空闲超时里调用）
 * @note   卡被拔掉又插回时重新识别（CMD0/ACMD41）并同步容量。
 */
void usb_msc_media_poll(void);

/**
 * @brief  把当前卡容量同步给已注册的 MSC 类实例
 * @note   类在注册时把 LUN 参数复制进自己的实例，之后改参数结构体不生效，
 *         所以这里直接写实例内部字段（依赖 USBX 内部结构，升级 USBX 时要复查）。
 */
void usb_msc_media_sync_capacity(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_MSC_MEDIA_H */
