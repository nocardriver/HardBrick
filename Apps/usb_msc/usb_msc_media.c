/**
 * @file    usb_msc_media.c
 * @brief   MSC 介质回调实现：SCSI 块读写落到 BSP/SDCard
 *
 * 一次 SCSI READ(10)/WRITE(10) 的路径：
 *   PC → USB 批量端点 → USBX MSC 类线程 → media_read/media_write
 *      → 媒体互斥量 → sdcard_read_blocks/sdcard_write_blocks（SDMMC+IDMA）
 *
 * 回调用到的 media_status 是 SCSI 的 sense 数据（key/code/qualifier），
 * 主机随后用 REQUEST SENSE 取走，用来判断"没介质"还是"读写错误"。
 *
 * @note 互斥量的作用：同一个 SDMMC 外设不能被两个线程同时发命令。
 *       USB 侧每次块传输拿一次，FatFs 侧（Apps/storage）做文件操作时也拿一次。
 */

#include "usb_msc_media.h"

#include "usb_msc.h"
#include "sdcard.h"
#include "SEGGER_RTT.h"

#include "tx_api.h"
#include "ux_api.h"
#include "ux_system.h"

#include <string.h>

/* SCSI sense：无可用的介质 / 未就绪 */
#define SENSE_NO_MEDIA \
  UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_NOT_READY, \
                                       UX_SLAVE_CLASS_STORAGE_SENSE_CODE_NOT_PRESENT, 0x00u)

/* SCSI sense：读失败（UNRECOVERED READ ERROR） */
#define SENSE_READ_ERROR \
  UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_MEDIUM_ERROR, \
                                       0x11u, 0x00u)

/* SCSI sense：写失败（WRITE ERROR） */
#define SENSE_WRITE_ERROR \
  UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_MEDIUM_ERROR, \
                                       0x0Cu, 0x00u)

/* SCSI sense：LBA 越界（LOGICAL BLOCK ADDRESS OUT OF RANGE） */
#define SENSE_LBA_RANGE \
  UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_ILLEGAL_REQUEST, \
                                       0x21u, 0x00u)

/* SCSI sense：参数/内部状态异常，主机应当重试 */
#define SENSE_BUSY \
  UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_NOT_READY, \
                                       UX_SLAVE_CLASS_STORAGE_SENSE_CODE_NOT_READY, 0x00u)

/* 报告给主机的介质属性
 *
 * 0x80 不是随便取的：USBX 把这个字段原样当成 SCSI INQUIRY 响应第 1 字节
 * （RMB 位，bit7 = 可移动介质）。ST 的 Ux_Device_MSC 例程用的也是 0x80。
 * 上报"可移动"的好处：Windows 会当成 U 盘，关闭对这条设备的写缓存，
 * 拔线丢数据的风险比当作"固定磁盘"小；代价是写入吞吐略低。
 * 若想当成固定磁盘上报（Windows 允许对 FAT32 做更多缓存），把它改成 0。
 */
#define MEDIA_REMOVABLE_FLAG    0x80u

#define MEDIA_SERIAL_STRING     "HB-MSC-000000000001"

static TX_MUTEX g_media_mutex;
static bool     g_mutex_ready;

/** 主机是否已枚举并配置（由类激活/去激活回调维护） */
static volatile bool g_host_active;

/** 卡容量缓存：0 表示当时没读到卡 */
static uint32_t g_last_lba;
static uint32_t g_block_length;

/** 上一次读/写失败过，下次状态查询时尝试重新识别卡 */
static bool g_reinit_pending;

/** 上一次日志里的主机占用状态，用于只在变化时打印 */
static bool g_logged_host_active;

/*---------------------------------------------------------------------------*/
/* 内部工具                                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief  读一次卡容量，刷新 g_last_lba / g_block_length
 * @retval true 卡片就绪，容量有效
 */
static bool capacity_refresh(void)
{
  sdcard_info_t info;

  if (!sdcard_get_info(&info)) {
    g_last_lba = 0u;
    g_block_length = SDCARD_BLOCK_SIZE;
    return false;
  }

  g_last_lba = info.block_count - 1u;
  g_block_length = info.block_size;
  g_usb_msc.last_lba = g_last_lba;
  g_usb_msc.block_length = g_block_length;
  return true;
}

/**
 * @brief  卡片当前是否可读写
 * @note   只是查询状态（CMD13），几微秒；失败后的一次重试由 g_reinit_pending 控制，
 *         避免卡真的坏掉时每次 TEST UNIT READY 都跑一遍完整识别。
 */
static bool card_ready(void)
{
  if (sdcard_is_ready()) {
    return true;
  }

  if (g_reinit_pending) {
    g_reinit_pending = false;
    if (sdcard_init()) {
      (void)capacity_refresh();
      usb_msc_media_sync_capacity();
      SEGGER_RTT_WriteString(0, "[USB] TF card re-initialized\r\n");
      return true;
    }
    SEGGER_RTT_WriteString(0, "[USB] TF card re-init failed\r\n");
  }

  return false;
}

/*---------------------------------------------------------------------------*/
/* 互斥量与主机占用标志                                                      */
/*---------------------------------------------------------------------------*/

bool usb_msc_media_create(void)
{
  if (tx_mutex_create(&g_media_mutex, "usb_msc_media", TX_INHERIT) != TX_SUCCESS) {
    return false;
  }

  g_mutex_ready = true;
  return true;
}

bool usb_msc_media_lock(uint32_t timeout_ticks)
{
  if (!g_mutex_ready) {
    return false;
  }

  if (tx_mutex_get(&g_media_mutex, (ULONG)timeout_ticks) != TX_SUCCESS) {
    g_usb_msc.media_busy++;
    return false;
  }
  return true;
}

void usb_msc_media_unlock(void)
{
  if (g_mutex_ready) {
    (void)tx_mutex_put(&g_media_mutex);
  }
}

bool usb_msc_host_active(void)
{
  return g_host_active;
}

/*---------------------------------------------------------------------------*/
/* MSC 类回调                                                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief  主机配置完成、开始使用介质（SET_CONFIGURATION 之后，类线程上下文）
 */
static VOID media_activate(VOID *storage_instance)
{
  UX_PARAMETER_NOT_USED(storage_instance);

  g_host_active = true;
  g_usb_msc.configure_count++;

  /* 尽量等在途的块操作结束（例如 storage 正在跑 f_getfree，32 GB 卡上要几秒）。
     只等一小会儿：这里是 SET_CONFIGURATION 的响应路径，等太久主机会超时。
     这次尝试不计入 media_busy —— 它本来就允许失败，失败也不影响功能。 */
  if (g_mutex_ready &&
      (tx_mutex_get(&g_media_mutex, USB_MSC_MEDIA_LOCK_SHORT) == TX_SUCCESS)) {
    (void)capacity_refresh();
    (void)tx_mutex_put(&g_media_mutex);
  }

  SEGGER_RTT_WriteString(0, "[USB] host configured, TF card is now owned by the PC\r\n");
}

/**
 * @brief  主机断开 / 总线复位导致类去激活
 * @note   这个回调可能运行在 OTG 中断上下文（总线复位路径），只做标志位操作，
 *         不加锁、不打印。
 */
static VOID media_deactivate(VOID *storage_instance)
{
  UX_PARAMETER_NOT_USED(storage_instance);

  g_host_active = false;
  g_usb_msc.deconfigure_count++;
}

/**
 * @brief  SCSI READ(10)：从卡里读若干块到 USBX 的传输缓冲
 */
static UINT media_read(VOID *storage, ULONG lun, UCHAR *data_pointer,
                       ULONG number_blocks, ULONG lba, ULONG *media_status)
{
  bool ok;
  UINT ret = UX_ERROR;

  UX_PARAMETER_NOT_USED(storage);

  if ((lun != 0u) || (media_status == UX_NULL)) {
    return UX_ERROR;
  }

  if ((number_blocks == 0u) || (data_pointer == UX_NULL)) {
    *media_status = SENSE_LBA_RANGE;
    return UX_ERROR;
  }

  /* 先把整段「查卡状态 + 块传输」都放进媒体锁里：卡状态查询本身也是一条 SD
     命令（CMD13），不能和 FatFs 侧的传输交错。ThreadX 的互斥量对持有者是
     可重入的，所以 card_ready() 里再取锁也不会死锁。 */
  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    *media_status = SENSE_BUSY;
    return UX_ERROR;
  }

  if (!card_ready()) {
    *media_status = SENSE_NO_MEDIA;
    goto out;
  }

  /* 越界检查用减法形式，避免 lba + number_blocks 溢出 */
  if ((lba > g_last_lba) || (number_blocks > ((g_last_lba - lba) + 1u))) {
    *media_status = SENSE_LBA_RANGE;
    goto out;
  }

  ok = sdcard_read_blocks(lba, data_pointer, number_blocks);
  if (!ok) {
    g_reinit_pending = true;
    g_usb_msc.read_errors++;
    *media_status = SENSE_READ_ERROR;
    goto out;
  }

  g_usb_msc.read_count++;
  *media_status = 0u;
  ret = UX_SUCCESS;

out:
  usb_msc_media_unlock();
  return ret;
}

/**
 * @brief  SCSI WRITE(10)：把 USBX 的传输缓冲写进卡
 */
static UINT media_write(VOID *storage, ULONG lun, UCHAR *data_pointer,
                        ULONG number_blocks, ULONG lba, ULONG *media_status)
{
  bool ok;
  UINT ret = UX_ERROR;

  UX_PARAMETER_NOT_USED(storage);

  if ((lun != 0u) || (media_status == UX_NULL)) {
    return UX_ERROR;
  }

  if ((number_blocks == 0u) || (data_pointer == UX_NULL)) {
    *media_status = SENSE_LBA_RANGE;
    return UX_ERROR;
  }

  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    *media_status = SENSE_BUSY;
    return UX_ERROR;
  }

  if (!card_ready()) {
    *media_status = SENSE_NO_MEDIA;
    goto out;
  }

  if ((lba > g_last_lba) || (number_blocks > ((g_last_lba - lba) + 1u))) {
    *media_status = SENSE_LBA_RANGE;
    goto out;
  }

  ok = sdcard_write_blocks(lba, data_pointer, number_blocks);
  if (!ok) {
    g_reinit_pending = true;
    g_usb_msc.write_errors++;
    *media_status = SENSE_WRITE_ERROR;
    goto out;
  }

  g_usb_msc.write_count++;
  *media_status = 0u;
  ret = UX_SUCCESS;

out:
  usb_msc_media_unlock();
  return ret;
}

/**
 * @brief  SCSI SYNCHRONIZE CACHE：没有写缓存，卡写完即落盘
 */
static UINT media_flush(VOID *storage, ULONG lun, ULONG number_blocks, ULONG lba,
                        ULONG *media_status)
{
  UX_PARAMETER_NOT_USED(storage);
  UX_PARAMETER_NOT_USED(number_blocks);
  UX_PARAMETER_NOT_USED(lba);

  if ((lun != 0u) || (media_status == UX_NULL)) {
    return UX_ERROR;
  }

  /* sdcard_write_blocks() 返回时卡已经回到 TRANSFER 状态，这里无需再做动作 */
  *media_status = 0u;
  return UX_SUCCESS;
}

/**
 * @brief  TEST UNIT READY 等命令的介质状态查询
 */
static UINT media_status(VOID *storage, ULONG lun, ULONG media_id, ULONG *media_st)
{
  bool ready;

  UX_PARAMETER_NOT_USED(storage);
  UX_PARAMETER_NOT_USED(media_id);

  if ((lun != 0u) || (media_st == UX_NULL)) {
    return UX_ERROR;
  }

  /* 查卡状态（CMD13）也要在媒体锁里做，避免和 FatFs 侧的传输交错 */
  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    *media_st = SENSE_BUSY;
    return UX_ERROR;
  }

  ready = card_ready();
  usb_msc_media_unlock();

  if (!ready) {
    *media_st = SENSE_NO_MEDIA;
    return UX_ERROR;
  }

  *media_st = 0u;
  return UX_SUCCESS;
}

/**
 * @brief  GET STATUS NOTIFICATION：不支持，返回错误让类走普通路径
 */
static UINT media_notification(VOID *storage, ULONG lun, ULONG media_id,
                               ULONG notification_class, UCHAR **media_notification,
                               ULONG *media_notification_length)
{
  UX_PARAMETER_NOT_USED(storage);
  UX_PARAMETER_NOT_USED(lun);
  UX_PARAMETER_NOT_USED(media_id);
  UX_PARAMETER_NOT_USED(notification_class);
  UX_PARAMETER_NOT_USED(media_notification);
  UX_PARAMETER_NOT_USED(media_notification_length);

  return UX_ERROR;
}

/*---------------------------------------------------------------------------*/
/* 类参数与周期检查                                                          */
/*---------------------------------------------------------------------------*/

void usb_msc_media_fill_parameter(UX_SLAVE_CLASS_STORAGE_PARAMETER *parameter)
{
  UX_SLAVE_CLASS_STORAGE_LUN *lun;

  if (parameter == NULL) {
    return;
  }

  memset(parameter, 0, sizeof(*parameter));

  parameter->ux_slave_class_storage_instance_activate = media_activate;
  parameter->ux_slave_class_storage_instance_deactivate = media_deactivate;

  /* 只有一个逻辑单元：整张 TF 卡 */
  parameter->ux_slave_class_storage_parameter_number_lun = 1u;

  lun = &parameter->ux_slave_class_storage_parameter_lun[0];
  lun->ux_slave_class_storage_media_type = UX_SLAVE_CLASS_STORAGE_MEDIA_FAT_DISK;
  lun->ux_slave_class_storage_media_removable_flag = MEDIA_REMOVABLE_FLAG;
  /* 读写都开放；要临时改成只读 U 盘，把它改成 UX_TRUE 即可 */
  lun->ux_slave_class_storage_media_read_only_flag = UX_FALSE;
  lun->ux_slave_class_storage_media_id = 0u;

  lun->ux_slave_class_storage_media_read = media_read;
  lun->ux_slave_class_storage_media_write = media_write;
  lun->ux_slave_class_storage_media_flush = media_flush;
  lun->ux_slave_class_storage_media_status = media_status;
  lun->ux_slave_class_storage_media_notification = media_notification;

  /* INQUIRY 里显示的厂商/型号/版本，长度由 SCSI 规定：8 / 16 / 4 字节 */
  parameter->ux_slave_class_storage_parameter_vendor_id = (UCHAR *)"HardBric";
  parameter->ux_slave_class_storage_parameter_product_id = (UCHAR *)"TF Card Storage ";
  parameter->ux_slave_class_storage_parameter_product_rev = (UCHAR *)"0100";
  parameter->ux_slave_class_storage_parameter_product_serial = (UCHAR *)MEDIA_SERIAL_STRING;

  /* 读一次容量；失败说明卡没插好，last_lba = 0，主机会看到"无介质" */
  if (!capacity_refresh()) {
    SEGGER_RTT_WriteString(0, "[USB] no TF card at startup, LUN reports no media\r\n");
  }

  lun->ux_slave_class_storage_media_last_lba = (ULONG)g_last_lba;
  lun->ux_slave_class_storage_media_block_length = (ULONG)g_block_length;
}

void usb_msc_media_sync_capacity(void)
{
  UX_SLAVE_CLASS *class_array;
  uint32_t        i;

  class_array = _ux_system_slave->ux_system_slave_class_array;
  if (class_array == UX_NULL) {
    return;
  }

  /* 只有一个类（UX_MAX_SLAVE_CLASS_DRIVER=1），仍然循环以免以后加类时漏掉 */
  for (i = 0u; i < UX_MAX_SLAVE_CLASS_DRIVER; i++) {
    UX_SLAVE_CLASS_STORAGE *storage =
        (UX_SLAVE_CLASS_STORAGE *)class_array[i].ux_slave_class_instance;

    if (storage == UX_NULL) {
      continue;
    }

    storage->ux_slave_class_storage_lun[0].ux_slave_class_storage_media_last_lba =
        (ULONG)g_last_lba;
    storage->ux_slave_class_storage_lun[0].ux_slave_class_storage_media_block_length =
        (ULONG)g_block_length;
  }
}

void usb_msc_media_poll(void)
{
  bool ready;

  /* 主机占用状态变化时才打印，避免刷屏 */
  if (g_logged_host_active != g_host_active) {
    g_logged_host_active = g_host_active;
    SEGGER_RTT_printf(0, "[USB] %s\r\n",
                      g_host_active ? "PC owns the card"
                                    : "PC released the card, MCU can use it again");
  }

  /* 主机在用卡时不去动 SD 命令；空闲时才检查卡是否还在 */
  if (g_host_active) {
    return;
  }

  /* 卡状态查询 / 重新识别都是 SD 命令，必须和 FatFs 侧互斥 */
  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    return;
  }

  ready = sdcard_is_ready();

  if (!ready) {
    /* 卡被拔掉又插回：重走 CMD0/ACMD41，并把新容量同步给已注册的 MSC 类 */
    if (sdcard_init()) {
      (void)capacity_refresh();
      usb_msc_media_sync_capacity();
      SEGGER_RTT_WriteString(0, "[USB] TF card detected, capacity updated\r\n");
    }
    usb_msc_media_unlock();
    return;
  }

  /* 卡在但容量缓存是空的（上电时没读到卡）：补一次 */
  if ((g_last_lba == 0u) && capacity_refresh()) {
    usb_msc_media_sync_capacity();
    SEGGER_RTT_WriteString(0, "[USB] TF card capacity refreshed\r\n");
  }

  usb_msc_media_unlock();
}
