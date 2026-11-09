/**
 * @file    usb_msc.h
 * @brief   USB 设备应用层：把 TF 卡当作 U 盘暴露给 PC（MSC / BOT 类）
 *
 * 功能
 * ----
 * 上电后创建 usb_msc 线程，在线程里初始化 USBX 设备栈、注册 MSC 类、启动 OTG，
 * 之后 PC 通过 USB 枚举看到一个可读写的 U 盘，读写落在 BSP/SDCard 的块接口上。
 *
 * 「软件复位时模拟拔插」
 * ----------------------
 * 复位后 D+ (PA12) 先被 GPIO 拉低，主机判定设备已拔出；等 USBX 就绪后把 D+ 交还
 * 给 OTG 外设并打开内部上拉，主机看到一次干净的插入并重新枚举。这样调试时反复
 * 复位不需要动 USB 线，见 usb_msc_boot_detach() / usb_msc_reconnect()。
 *
 * 与固件自身 FatFs 的共存
 * ------------------------
 * 卡同一时刻只能有一个主人。主机枚举成功后本模块把「主机占用」标志置位，
 * Apps/storage 在动文件系统之前先用 usb_msc_host_active() 判断并放弃访问；
 * 主机断开后标志清除，storage 重新挂载即可。块设备本身用一把互斥量
 * （usb_msc_media_lock/unlock）串行化，避免 USB 侧和 FatFs 侧同时发 SD 命令。
 */

#ifndef USB_MSC_H
#define USB_MSC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* 配置                                                                      */
/*---------------------------------------------------------------------------*/

/** usb_msc 线程栈（静态分配，字节）。线程里要做 USBX 栈初始化，4 KB 起步 */
#define USB_MSC_THREAD_STACK_SIZE     4096u

/**
 * usb_msc 线程优先级
 * @note 现有线程：gps=4、display=5、backlight=6、storage=7。USB 枚举与块传输
 *       需要及时响应（主机等超时会重试），但又不能压住界面，取 6~7 之间；
 *       这里与 backlight 同档，靠 TX_NO_TIME_SLICE 轮转。
 */
#define USB_MSC_THREAD_PRIORITY       6u

/**
 * USBX 内存池大小（字节，静态分配，8 字节对齐）
 * @note 设备栈、MSC 类实例、类线程栈（UX_THREAD_STACK_SIZE=2 KB）、端点缓冲都
 *       从这里分配，16 KB 对单类设备足够。
 */
#define USB_MSC_POOL_SIZE             (16u * 1024u)

/** 描述符里的 VID/PID：沿用 ST 的 MSC 示例值，避免 Windows 反复装驱动 */
#define USB_MSC_VID                   0x0483u
#define USB_MSC_PID                   0x5720u

/** 字符串描述符（均为 ASCII） */
#define USB_MSC_MANUFACTURER_STRING   "HardBrick"
#define USB_MSC_PRODUCT_STRING        "HardBrick TF Card"
#define USB_MSC_SERIAL_STRING         "HB-MSC-0001"

/** USB 语言 ID：0x0409 = 英语（美国） */
#define USB_MSC_LANGUAGE_ID           0x0409u

/** 端点地址与包长（全速，单包 64 B） */
#define USB_MSC_EPIN_ADDR             0x81u
#define USB_MSC_EPOUT_ADDR            0x01u
#define USB_MSC_EP_MPS                64u

/**
 * 「模拟拔出」的保持时间（ms）
 * @note 主机判定断开只需要几微秒，但 Windows 重新枚举前有一段去抖；保持几百
 *       毫秒最稳，代价只是上电后晚一点出现盘符。
 */
#define USB_MSC_ATTACH_DELAY_MS       300u

/** 块设备互斥量的默认持锁超时（tick，100 Hz 下 200 tick = 2 s） */
#define USB_MSC_MEDIA_LOCK_TIMEOUT    200u

/**
 * 类激活回调里等媒体的超时（tick）
 * @note 激活回调发生在主机执行 SET_CONFIGURATION 的控制传输中间，等太久会让主机
 *       超时（Windows 控制传输约 5 s）。所以这里只等 500 ms：等不到也没关系，
 *       类线程随后每个块操作都会再抢这把锁，天然串行化。
 */
#define USB_MSC_MEDIA_LOCK_SHORT      50u

/*---------------------------------------------------------------------------*/
/* 诊断                                                                      */
/*---------------------------------------------------------------------------*/

/** 状态魔数：J-Link 里看到这个值才说明本模块真的跑过 */
#define USB_MSC_STATUS_MAGIC          0x4D534355u   /* 'USCM' */

/** 启动阶段：失败时 stage 停在出错的那一步 */
typedef enum {
  USB_MSC_STAGE_IDLE = 0,          /**< 还没开始 */
  USB_MSC_STAGE_POOL_FAIL,         /**< ux_system_initialize 失败 */
  USB_MSC_STAGE_STACK_FAIL,        /**< ux_device_stack_initialize 失败 */
  USB_MSC_STAGE_CLASS_FAIL,        /**< 注册 MSC 类失败 */
  USB_MSC_STAGE_DCD_FAIL,          /**< ux_dcd_stm32_initialize 失败 */
  USB_MSC_STAGE_READY,             /**< 设备栈就绪，已（或即将）接入总线 */
} usb_msc_stage_t;

typedef struct {
  uint32_t magic;                  /**< USB_MSC_STATUS_MAGIC */
  uint32_t stage;                  /**< usb_msc_stage_t */
  uint32_t last_lba;               /**< 报告给主机的最后一个 LBA（0 表示无卡） */
  uint32_t block_length;           /**< 报告给主机的块大小 */
  uint32_t configure_count;        /**< 主机 SET_CONFIGURATION 次数 */
  uint32_t deconfigure_count;      /**< 主机断开 / 复位导致类去激活的次数 */
  uint32_t reconnect_count;        /**< 主动重新枚举次数 */
  uint32_t read_count;             /**< 主机读块请求数 */
  uint32_t write_count;            /**< 主机写块请求数 */
  uint32_t read_errors;            /**< 读失败次数 */
  uint32_t write_errors;           /**< 写失败次数 */
  uint32_t media_busy;             /**< 取媒体锁超时次数 */
} usb_msc_status_t;

/** 诊断快照，可以用 J-Link 直接读内存，不依赖 RTT */
extern volatile usb_msc_status_t g_usb_msc;

/*---------------------------------------------------------------------------*/
/* 接口                                                                      */
/*---------------------------------------------------------------------------*/

/**
 * @brief  创建 USB 设备线程与媒体互斥量（在 tx_application_define 里调用）
 * @note   必须在 storage_init() 之前调用：storage 会用到这里的媒体锁。
 *         真正的 USB 初始化在这个线程里做，因为 USBX 建类线程需要内核已运行。
 */
void usb_msc_init(void);

/**
 * @brief  启动阶段模拟「USB 已拔出」（在 main() 的外设初始化之后调用）
 * @note   顺序是：关 OTG 中断 → HAL_PCD_Stop()（DCTL.SDIS=1，PHY 软断开）→
 *         D+ 拉低。调用后主机看到设备消失，直到 usb_msc 线程把 D+ 还回去。
 *         这样做的好处是每次软件复位（含调试器复位、看门狗复位）都会让 PC
 *         重新枚举，不用拔线。
 */
void usb_msc_boot_detach(void);

/**
 * @brief  主机是否正在占用这张卡（已枚举并配置）
 * @retval true  PC 已拿到卡，固件侧不应再访问文件系统/Raw 块
 * @note   主机拔出、USB 总线复位或重新枚举时自动清除。
 */
bool usb_msc_host_active(void);

/**
 * @brief  申请 TF 卡块设备访问权（USB 侧与 FatFs 侧互斥）
 * @param  timeout_ticks 等待上限（tick），USB_MSC_MEDIA_LOCK_TIMEOUT 为默认值
 * @retval true  拿到锁，用完必须调用 usb_msc_media_unlock()
 * @retval false 超时或互斥量还没创建
 */
bool usb_msc_media_lock(uint32_t timeout_ticks);

/** @brief 释放 TF 卡块设备访问权 */
void usb_msc_media_unlock(void);

/**
 * @brief  请求一次「拔出 → 插入」的重新枚举（任意线程可调）
 * @note   只置事件标志，真正的动作在 usb_msc 线程里做：先让设备栈掉线，
 *         停 PHY、拉低 D+ 保持 USB_MSC_ATTACH_DELAY_MS，再重新接入。
 *         调试时改了描述符/容量，不用重新烧卡就能让 PC 重新识别。
 */
void usb_msc_reconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_MSC_H */
