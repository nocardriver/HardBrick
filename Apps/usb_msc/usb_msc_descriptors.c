/**
 * @file    usb_msc_descriptors.c
 * @brief   USBX 设备侧描述符实现（全速、单配置、MSC + BOT + 两个批量端点）
 *
 * 设备框架 = 设备描述符(18) || 配置描述符(9) || 接口描述符(9) ||
 *            端点描述符 OUT(7) || 端点描述符 IN(7)  = 50 字节
 *
 * 字符串框架是 USBX 自己的格式：每个条目 4 字节头 + ASCII 内容
 *   [lang_lo, lang_hi, index, length, chars...]
 * USBX 在响应 GET_DESCRIPTOR(STRING) 时按 index 查表，并现场转成 UTF-16。
 */

#include "usb_msc_descriptors.h"

#include "usb_msc.h"

#include <stdbool.h>
#include <string.h>

/* 端点描述符属性：0x02 = Bulk */
#define USB_MSC_EP_TYPE_BULK        0x02u

/* MSC 接口的三元组：类 0x08 / 子类 0x06(SCSI transparent) / 协议 0x50(BOT) */
#define USB_MSC_INTERFACE_CLASS     0x08u
#define USB_MSC_INTERFACE_SUBCLASS  0x06u
#define USB_MSC_INTERFACE_PROTOCOL  0x50u

/* 配置描述符的 bmAttributes：0x80 = 总线供电，不支持远程唤醒 */
#define USB_MSC_CONFIG_ATTRIBUTES   0x80u

/* 最大电流 100 mA（单位 2 mA） */
#define USB_MSC_CONFIG_MAX_POWER    0x32u

/* 字符串表缓冲：三条字符串（含 4 字节头）不到 64 B，128 B 留足余量 */
#define USB_MSC_STRING_FRAMEWORK_SIZE  128u

/* 单条字符串的字符数上限，避免写越界 */
#define USB_MSC_STRING_MAX_CHARS       32u

/* 设备描述符里的字符串索引 */
#define USB_MSC_IDX_MANUFACTURER    1u
#define USB_MSC_IDX_PRODUCT         2u
#define USB_MSC_IDX_SERIAL          3u

/* 配置描述符 + 接口 + 两个端点 = 32 字节 */
#define USB_MSC_CONFIG_TOTAL_LENGTH 32u

/* 框架总长 = 设备描述符 18 + 配置部分 32 */
#define USB_MSC_FRAMEWORK_LENGTH    (18u + USB_MSC_CONFIG_TOTAL_LENGTH)

#define USB_MSC_LOW_BYTE(v)  ((UCHAR)((v) & 0xFFu))
#define USB_MSC_HIGH_BYTE(v) ((UCHAR)(((v) >> 8u) & 0xFFu))

/**
 * 设备 + 配置框架
 * @note 数组不能是 const：USBX 的初始化接口形参是 UCHAR *（内部只读）
 */
static UCHAR g_framework[USB_MSC_FRAMEWORK_LENGTH] = {
    /* ---- 设备描述符（18 B）---- */
    0x12u,                              /* bLength */
    UX_DEVICE_DESCRIPTOR_ITEM,          /* bDescriptorType */
    USB_MSC_LOW_BYTE(0x0200u),          /* bcdUSB = 2.00 */
    USB_MSC_HIGH_BYTE(0x0200u),
    0x00u,                              /* bDeviceClass：类定义在接口上 */
    0x00u,                              /* bDeviceSubClass */
    0x00u,                              /* bDeviceProtocol */
    0x40u,                              /* bMaxPacketSize0 = 64 */
    USB_MSC_LOW_BYTE(USB_MSC_VID),      /* idVendor */
    USB_MSC_HIGH_BYTE(USB_MSC_VID),
    USB_MSC_LOW_BYTE(USB_MSC_PID),      /* idProduct */
    USB_MSC_HIGH_BYTE(USB_MSC_PID),
    0x00u, 0x01u,                       /* bcdDevice = 1.00 */
    USB_MSC_IDX_MANUFACTURER,           /* iManufacturer */
    USB_MSC_IDX_PRODUCT,                /* iProduct */
    USB_MSC_IDX_SERIAL,                 /* iSerialNumber */
    0x01u,                              /* bNumConfigurations */

    /* ---- 配置描述符（9 B）---- */
    0x09u,                              /* bLength */
    UX_CONFIGURATION_DESCRIPTOR_ITEM,   /* bDescriptorType */
    USB_MSC_LOW_BYTE(USB_MSC_CONFIG_TOTAL_LENGTH),   /* wTotalLength */
    USB_MSC_HIGH_BYTE(USB_MSC_CONFIG_TOTAL_LENGTH),
    0x01u,                              /* bNumInterfaces */
    0x01u,                              /* bConfigurationValue */
    0x00u,                              /* iConfiguration */
    USB_MSC_CONFIG_ATTRIBUTES,          /* bmAttributes */
    USB_MSC_CONFIG_MAX_POWER,           /* bMaxPower */

    /* ---- 接口描述符（9 B）---- */
    0x09u,                              /* bLength */
    UX_INTERFACE_DESCRIPTOR_ITEM,       /* bDescriptorType */
    0x00u,                              /* bInterfaceNumber */
    0x00u,                              /* bAlternateSetting */
    0x02u,                              /* bNumEndpoints：Bulk OUT + Bulk IN */
    USB_MSC_INTERFACE_CLASS,            /* bInterfaceClass = 0x08 */
    USB_MSC_INTERFACE_SUBCLASS,         /* bInterfaceSubClass = 0x06 */
    USB_MSC_INTERFACE_PROTOCOL,         /* bInterfaceProtocol = 0x50 (BOT) */
    0x00u,                              /* iInterface */

    /* ---- 端点描述符：OUT 0x01，Bulk，64 B（7 B）---- */
    0x07u,
    UX_ENDPOINT_DESCRIPTOR_ITEM,
    USB_MSC_EPOUT_ADDR,
    USB_MSC_EP_TYPE_BULK,
    USB_MSC_LOW_BYTE(USB_MSC_EP_MPS),
    USB_MSC_HIGH_BYTE(USB_MSC_EP_MPS),
    0x00u,                              /* bInterval：Bulk 忽略 */

    /* ---- 端点描述符：IN 0x81，Bulk，64 B（7 B）---- */
    0x07u,
    UX_ENDPOINT_DESCRIPTOR_ITEM,
    USB_MSC_EPIN_ADDR,
    USB_MSC_EP_TYPE_BULK,
    USB_MSC_LOW_BYTE(USB_MSC_EP_MPS),
    USB_MSC_HIGH_BYTE(USB_MSC_EP_MPS),
    0x00u,
};

/** 语言 ID 表：只支持 0x0409（英语/美国），主机基本都能接受 */
static UCHAR g_language_id[2] = {
    USB_MSC_LOW_BYTE(USB_MSC_LANGUAGE_ID),
    USB_MSC_HIGH_BYTE(USB_MSC_LANGUAGE_ID),
};

/** 字符串表运行时生成（见文件头说明的格式） */
static UCHAR g_string_framework[USB_MSC_STRING_FRAMEWORK_SIZE];
static ULONG g_string_framework_length;
static bool  g_strings_ready;

/**
 * @brief  往字符串表里追加一条 ASCII 字符串
 * @param  offset 当前写入位置（会被更新）
 * @param  index  字符串索引（对应设备描述符里的 iXxx）
 * @param  text   ASCII 字符串
 */
static void append_string(ULONG *offset, UCHAR index, const char *text)
{
  ULONG length = 0u;

  while ((text[length] != '\0') && (length < USB_MSC_STRING_MAX_CHARS)) {
    length++;
  }

  g_string_framework[*offset + 0u] = USB_MSC_LOW_BYTE(USB_MSC_LANGUAGE_ID);
  g_string_framework[*offset + 1u] = USB_MSC_HIGH_BYTE(USB_MSC_LANGUAGE_ID);
  g_string_framework[*offset + 2u] = index;
  g_string_framework[*offset + 3u] = (UCHAR)length;
  memcpy(&g_string_framework[*offset + 4u], text, length);

  *offset += 4u + length;
}

void usb_msc_descriptors_device(UCHAR **framework, ULONG *length)
{
  if ((framework == NULL) || (length == NULL)) {
    return;
  }

  *framework = g_framework;
  *length = USB_MSC_FRAMEWORK_LENGTH;
}

void usb_msc_descriptors_strings(UCHAR **string_framework, ULONG *string_length,
                                 UCHAR **language_id, ULONG *language_id_length)
{
  if ((string_framework == NULL) || (string_length == NULL) ||
      (language_id == NULL) || (language_id_length == NULL)) {
    return;
  }

  if (!g_strings_ready) {
    ULONG offset = 0u;

    memset(g_string_framework, 0, sizeof(g_string_framework));
    append_string(&offset, USB_MSC_IDX_MANUFACTURER, USB_MSC_MANUFACTURER_STRING);
    append_string(&offset, USB_MSC_IDX_PRODUCT, USB_MSC_PRODUCT_STRING);
    append_string(&offset, USB_MSC_IDX_SERIAL, USB_MSC_SERIAL_STRING);

    g_string_framework_length = offset;
    g_strings_ready = true;
  }

  *string_framework = g_string_framework;
  *string_length = g_string_framework_length;
  *language_id = g_language_id;
  *language_id_length = sizeof(g_language_id);
}
