/**
 * @file    usb_msc.c
 * @brief   USB 设备应用层实现：USBX 设备栈的启动、接入与重新枚举
 *
 * 启动顺序（都在 usb_msc 线程里，因为 USBX 注册类时会创建线程，必须内核已运行）：
 *
 *   main()                      usb_msc 线程
 *   ─────────────────────────   ────────────────────────────────────────────
 *   MX_USB_OTG_FS_PCD_Init()
 *   usb_msc_boot_detach()       ux_system_initialize()          内存池
 *     ├ HAL_PCD_Stop()          ux_device_stack_initialize()    设备框架/字符串
 *     └ D+ 拉低（模拟拔出）      ux_device_stack_class_register() MSC 类
 *                               ux_dcd_stm32_initialize()      STM32 DCD
 *                               tx_thread_sleep(300 ms)        保证主机看到断开
 *                               D+ 交还 + HAL_PCD_Start()      主机重新枚举
 *
 * 之后线程只在两件事上醒来：收到 usb_msc_reconnect() 的重新枚举请求，或者
 * 5 s 超时做一次卡片存在性检查。
 */

#include "usb_msc.h"

#include "usb_msc_descriptors.h"
#include "usb_msc_media.h"
#include "usb_connect.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "usb_otg.h"

#include "tx_api.h"
#include "ux_api.h"
#include "ux_dcd_stm32.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"

#include <string.h>

/** 类名：只用于 USBX 内部注册/注销匹配，内容随意但必须唯一 */
#define USB_MSC_CLASS_NAME          "ux_device_class_storage"

/** 重新枚举事件位 */
#define USB_MSC_EVT_RECONNECT       0x01u

/** 线程空闲等待上限（tick）：200 ms 一轮，用于总线挂起/卡片状态检测。
    @note 挂起释放靠这一轮询把"主机已不在总线"翻译成门控：OTG 中断只置标志，
    真正的判定（宽限期）在 usb_msc_media_poll() 里做。 */
#define USB_MSC_POLL_TICKS          20u

/** 本线程栈（静态分配，.bss） */
static uint8_t g_thread_stack[USB_MSC_THREAD_STACK_SIZE] __attribute__((aligned(8)));

/** USBX 内存池：设备栈、类实例、类线程栈、端点缓冲都从这里分配 */
static uint8_t g_usbx_pool[USB_MSC_POOL_SIZE] __attribute__((aligned(8)));

static TX_THREAD           g_thread;
static TX_EVENT_FLAGS_GROUP g_events;
static bool                g_events_ready;

/** MSC 类注册参数：类在初始化时会把字段复制走，但指针指向的字符串要保持有效 */
static UX_SLAVE_CLASS_STORAGE_PARAMETER g_storage_parameter;

volatile usb_msc_status_t g_usb_msc;

/*---------------------------------------------------------------------------*/
/* 内部                                                                      */
/*---------------------------------------------------------------------------*/

/**
 * @brief  把 D+ 交还给 OTG 外设并打开内部上拉（模拟插入）
 * @note   顺序：先恢复引脚复用，再启动 PCD（SDIS=0 → 内部 1.5k 上拉接入），
 *         最后才打开 NVIC。反过来做的话，中断可能在上拉之前就进来。
 */
static void link_attach(void)
{
  usb_connect_release();
  (void)HAL_PCD_Start(&hpcd_USB_OTG_FS);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/**
 * @brief  模拟拔出：设备栈掉线 + PHY 软断开 + D+ 拉低
 */
static void link_detach(void)
{
  /* 先让设备栈掉线：类去激活回调会清掉"主机占用"标志，释放介质 */
  if (g_usb_msc.stage == (uint32_t)USB_MSC_STAGE_READY) {
    (void)ux_device_stack_disconnect();
  }

  /* SDIS=1：OTG 内核不再驱动总线，此时才能安全地把 PA12 当 GPIO 用 */
  (void)HAL_PCD_Stop(&hpcd_USB_OTG_FS);
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

  usb_connect_hold_off();
}

/**
 * @brief  USBX 设备状态变化回调（由 ST DCD 在 OTG 中断里调用）
 * @param  event ST DCD 事件码（UX_DCD_STM32_DEVICE_xxx），另外初始化完成时会收到
 *               UX_DEVICE_ATTACHED
 * @retval UX_SUCCESS
 * @note   中断上下文：只把事件翻译成链路标志，具体判定在 usb_msc 线程里做。
 */
static UINT usb_msc_change_function(ULONG event)
{
  switch (event) {
    case UX_DCD_STM32_DEVICE_SUSPENDED:
      /* 主机停止发 SOF：拔线，或者主机把端口挂起（selective suspend） */
      usb_msc_media_on_link_event(USB_MSC_LINK_SUSPENDED);
      break;

    case UX_DCD_STM32_DEVICE_RESUMED:
      /* 只有主机真正驱动总线恢复（K 状态）才会来；CONNECT 事件不可靠，
         不用它重新置位门控，否则拔线后可能被一个假事件复原 */
      usb_msc_media_on_link_event(USB_MSC_LINK_RESUMED);
      break;

    case UX_DCD_STM32_DEVICE_DISCONNECTED:
      usb_msc_media_on_link_event(USB_MSC_LINK_DETACHED);
      break;

    default:
      /* UX_DEVICE_ATTACHED / UX_DCD_STM32_DEVICE_CONNECTED 等：枚举流程里
         SET_CONFIGURATION 会走类激活把门控置位，这里不处理 */
      break;
  }

  return UX_SUCCESS;
}

/**
 * @brief  初始化 USBX 设备栈并接入总线
 * @retval true  设备已接入
 * @retval false  初始化失败，g_usb_msc.stage 指出失败在哪一步
 */
static bool usb_msc_stack_start(void)
{
  UCHAR *framework = NULL;
  UCHAR *strings = NULL;
  UCHAR *language_id = NULL;
  ULONG  framework_length = 0u;
  ULONG  strings_length = 0u;
  ULONG  language_id_length = 0u;

  if (ux_system_initialize(g_usbx_pool, sizeof(g_usbx_pool), UX_NULL, 0u) != UX_SUCCESS) {
    g_usb_msc.stage = (uint32_t)USB_MSC_STAGE_POOL_FAIL;
    return false;
  }

  usb_msc_descriptors_device(&framework, &framework_length);
  usb_msc_descriptors_strings(&strings, &strings_length, &language_id,
                              &language_id_length);

  /* 全速设备：高速框架传 NULL/0（U575 的 OTG_FS 最高 12 Mbps）。
     最后一个参数是设备状态变化回调：拔线/挂起/恢复靠它上报（见函数实现）。 */
  if (ux_device_stack_initialize(UX_NULL, 0u,
                                 framework, framework_length,
                                 strings, strings_length,
                                 language_id, language_id_length,
                                 usb_msc_change_function) != UX_SUCCESS) {
    g_usb_msc.stage = (uint32_t)USB_MSC_STAGE_STACK_FAIL;
    return false;
  }

  /* 读卡容量 + 填 LUN 回调（必须早于类注册：类在注册时复制参数） */
  usb_msc_media_fill_parameter(&g_storage_parameter);

  if (ux_device_stack_class_register((UCHAR *)USB_MSC_CLASS_NAME,
                                     ux_device_class_storage_entry,
                                     1u,                 /* configuration number */
                                     0u,                 /* interface number */
                                     &g_storage_parameter) != UX_SUCCESS) {
    g_usb_msc.stage = (uint32_t)USB_MSC_STAGE_CLASS_FAIL;
    return false;
  }

  /* 把 USBX 的 DCD 接到 STM32 HAL 的 PCD 句柄上 */
  if (ux_dcd_stm32_initialize((ULONG)USB_OTG_FS, (ULONG)&hpcd_USB_OTG_FS) != UX_SUCCESS) {
    g_usb_msc.stage = (uint32_t)USB_MSC_STAGE_DCD_FAIL;
    return false;
  }

  g_usb_msc.stage = (uint32_t)USB_MSC_STAGE_READY;

  /* 从 main() 开始 D+ 一直是低的，这里再保持一会儿：Windows 判定断开需要时间，
     太短的断开会被当成"同一个设备"，从而不重新读描述符/容量 */
  tx_thread_sleep(UX_MS_TO_TICK(USB_MSC_ATTACH_DELAY_MS));

  link_attach();

  SEGGER_RTT_printf(0, "[USB] MSC device attached, last LBA %u, block %u B\r\n",
                    (unsigned)g_usb_msc.last_lba, (unsigned)g_usb_msc.block_length);
  return true;
}

/**
 * @brief  执行一次完整的"拔出 → 保持 → 插入"
 */
static void usb_msc_reconnect_now(void)
{
  g_usb_msc.reconnect_count++;
  SEGGER_RTT_WriteString(0, "[USB] re-enumerating (simulated unplug/replug)\r\n");

  link_detach();

  tx_thread_sleep(UX_MS_TO_TICK(USB_MSC_ATTACH_DELAY_MS));

  link_attach();
}

static void usb_msc_thread_entry(ULONG thread_input)
{
  UX_PARAMETER_NOT_USED(thread_input);

  SEGGER_RTT_WriteString(0, "[USB] usb_msc thread started\r\n");

  if (!usb_msc_stack_start()) {
    SEGGER_RTT_printf(0, "[USB] USBX init failed at stage %u\r\n",
                      (unsigned)g_usb_msc.stage);
  }

  for (;;) {
    ULONG flags = 0u;
    UINT  status = tx_event_flags_get(&g_events, USB_MSC_EVT_RECONNECT,
                                      TX_OR_CLEAR, &flags, USB_MSC_POLL_TICKS);

    /* 收到重新枚举请求 */
    if ((status == TX_SUCCESS) && ((flags & USB_MSC_EVT_RECONNECT) != 0u)) {
      if (g_usb_msc.stage != (uint32_t)USB_MSC_STAGE_READY) {
        /* 上次初始化失败：把这次请求当成重试 */
        (void)usb_msc_stack_start();
      } else {
        usb_msc_reconnect_now();
      }
    }

    /* 周期检查：卡片状态、主机是否已放开介质 */
    usb_msc_media_poll();
  }
}

/*---------------------------------------------------------------------------*/
/* 对外接口                                                                  */
/*---------------------------------------------------------------------------*/

void usb_msc_init(void)
{
  memset((void *)&g_usb_msc, 0, sizeof(g_usb_msc));
  g_usb_msc.magic = USB_MSC_STATUS_MAGIC;
  g_usb_msc.stage = (uint32_t)USB_MSC_STAGE_IDLE;

  /* 媒体互斥量必须先建好：storage 线程会用它串行化对 TF 卡的访问。
     本函数在 tx_application_define() 里被调用，早于任何线程运行。 */
  if (!usb_msc_media_create()) {
    SEGGER_RTT_WriteString(0, "[USB] media mutex create failed\r\n");
    return;
  }

  if (tx_event_flags_create(&g_events, "usb_msc") != TX_SUCCESS) {
    SEGGER_RTT_WriteString(0, "[USB] event flags create failed\r\n");
    return;
  }
  g_events_ready = true;

  if (tx_thread_create(&g_thread, "usb_msc", usb_msc_thread_entry, 0u,
                       g_thread_stack, sizeof(g_thread_stack),
                       USB_MSC_THREAD_PRIORITY, USB_MSC_THREAD_PRIORITY,
                       TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
    SEGGER_RTT_WriteString(0, "[USB] thread create failed\r\n");
    return;
  }
}

void usb_msc_boot_detach(void)
{
  /* 此时 CubeMX 的 MX_USB_OTG_FS_PCD_Init() 已经跑过：PCD 处于 READY、
     DCTL.SDIS=1，但 NVIC 里的 OTG 中断是打开的。USBX 的 DCD 回调没有空指针
     保护，所以先把中断关掉，等设备栈就绪后再由 link_attach() 打开。 */
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  (void)HAL_PCD_Stop(&hpcd_USB_OTG_FS);

  /* 把 D+ 拉低：即使板子上有对 D+ 硬上拉的电阻，主机也会判定设备已拔出 */
  usb_connect_hold_off();
}

void usb_msc_reconnect(void)
{
  if (g_events_ready) {
    (void)tx_event_flags_set(&g_events, USB_MSC_EVT_RECONNECT, TX_OR);
  }
}
