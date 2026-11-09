/**
 * @file    usb_msc_descriptors.h
 * @brief   USBX 设备/配置/字符串描述符（设备侧 MSC）
 *
 * 描述符用静态数组手写，而不是像 ST 的 Ux_Device_MSC 例程那样用一堆 builder
 * 函数拼装：本工程只有「一个配置 + 一个 MSC 接口 + 两个批量端点」，手写更直观，
 * 改端点或 VID/PID 时也是改一处、看得见。
 */

#ifndef USB_MSC_DESCRIPTORS_H
#define USB_MSC_DESCRIPTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ux_api.h"

/**
 * @brief  取设备框架（设备描述符 + 配置描述符 + 接口 + 端点，连续存放）
 * @param  framework 输出：缓冲区首地址（不拷贝，USBX 直接引用）
 * @param  length    输出：总长度
 * @note   全速设备只需要这一份框架；STM32U575 的 OTG_FS 不支持高速，
 *         高速框架传 NULL 即可（USBX 会按 DCD 报告的速率选择）。
 */
void usb_msc_descriptors_device(UCHAR **framework, ULONG *length);

/**
 * @brief  取字符串框架与语言 ID
 * @param  string_framework    输出：字符串表（格式见实现说明）
 * @param  string_length       输出：字符串表总字节数
 * @param  language_id         输出：语言 ID 表
 * @param  language_id_length  输出：语言 ID 表字节数
 * @note   字符串表第一次调用时生成，之后返回同一份静态缓冲。
 */
void usb_msc_descriptors_strings(UCHAR **string_framework, ULONG *string_length,
                                 UCHAR **language_id, ULONG *language_id_length);

#ifdef __cplusplus
}
#endif

#endif /* USB_MSC_DESCRIPTORS_H */
