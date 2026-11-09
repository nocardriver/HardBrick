/**
 * @file    diskio.c
 * @brief   FatFs 底层磁盘接口：把 BSP/SDCard 的裸块读写接进来（移植层）
 *
 * FatFs 只认 sector + 缓冲区，块设备细节全部收敛在这里：
 *   disk_initialize() → sdcard_init()          卡识别（幂等）
 *   disk_read()       → sdcard_read_blocks()   裸块读
 *   disk_write()      → sdcard_write_blocks()  裸块写
 *   disk_ioctl()      → sdcard_get_info()      容量 / 扇区大小 / 擦除块
 *
 * 两个容易踩的坑，都在这里处理掉了：
 *
 * 1. 状态必须缓存。ff.c 的 validate() 和 mount_volume() 几乎每次文件操作都会问
 *    一次 disk_status()；如果每次都真发 CMD13 去问卡，每个 f_read 都要多几十
 *    微秒的同步命令开销。所以 g_sd_stat 只是缓存，真动作放在 disk_initialize()。
 *
 * 2. 分区偏移不用自己加。FF_MULTI_PARTITION = 0 时 ff.c 自己扫 MBR，把分区起始
 *    LBA 加到扇区号上再调 disk_read()，所以这里的 sector 已经是整张卡的绝对
 *    扇区号，直接送给 SDMMC 就行（sdcard.c 的探测也是这么读引导扇区的）。
 *
 * @note 传输失败时把盘打回 STA_NOINIT，这样再次 f_mount() 会重走 disk_initialize()
 *       重新识别卡（典型场景：运行中把卡拔了又插回去）。应用层拿到 FR_DISK_ERR
 *       后重新 f_mount 即可，不需要重新上电。
 */

#include "ff.h"
#include "diskio.h"

#include "sdcard.h"

#include <stdbool.h>
#include <stddef.h>

/* 物理盘编号。FF_MULTI_PARTITION = 0 时逻辑盘号直接当物理盘号用，只有 0 号盘 */
#define DEV_SD   ((BYTE)0)

/* 缓存的盘状态，初值是"没初始化过"。多线程会读，ISR 不碰 */
static volatile DSTATUS g_sd_stat = STA_NOINIT;

/**
 * @brief  把一次块传输的结果翻译成 FatFs 的返回码
 * @note   失败即把盘标成未初始化：要么卡真没了，要么链路坏了，两种情况都应该
 *         让下一次 f_mount() 重新识别，而不是继续拿旧状态硬跑。
 */
static DRESULT sd_result(bool ok)
{
  if (ok) {
    return RES_OK;
  }

  g_sd_stat = STA_NOINIT;
  return RES_ERROR;
}

DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != DEV_SD) {
    return STA_NOINIT | STA_NODISK;
  }
  return g_sd_stat;
}

DSTATUS disk_initialize(BYTE pdrv)
{
  sdcard_info_t info;

  if (pdrv != DEV_SD) {
    return STA_NOINIT | STA_NODISK;
  }

  /* 先当失败处理：下面任何一步不成功，FatFs 拿到的都是"不可用" */
  g_sd_stat = STA_NOINIT | STA_NODISK;

  if (!sdcard_init() || !sdcard_get_info(&info)) {
    return g_sd_stat;
  }

  g_sd_stat = info.write_protected ? STA_PROTECT : 0u;
  return g_sd_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  if (pdrv != DEV_SD) {
    return RES_PARERR;
  }
  if ((g_sd_stat & STA_NOINIT) != 0u) {
    return RES_NOTRDY;
  }
  if ((buff == NULL) || (count == 0u)) {
    return RES_PARERR;
  }

  return sd_result(sdcard_read_blocks((uint32_t)sector, buff, (uint32_t)count));
}

#if FF_FS_READONLY == 0

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  if (pdrv != DEV_SD) {
    return RES_PARERR;
  }
  if ((g_sd_stat & STA_NOINIT) != 0u) {
    return RES_NOTRDY;
  }
  if ((g_sd_stat & STA_PROTECT) != 0u) {
    return RES_WRPRT;
  }
  if ((buff == NULL) || (count == 0u)) {
    return RES_PARERR;
  }

  return sd_result(sdcard_write_blocks((uint32_t)sector, buff, (uint32_t)count));
}

#endif /* FF_FS_READONLY == 0 */

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  sdcard_info_t info;

  if (pdrv != DEV_SD) {
    return RES_PARERR;
  }

  switch (cmd) {
  case CTRL_SYNC:
    /* 每次写都要等卡回到 TRANSFER 才返回（见 sdcard.c），没有待刷的写缓存 */
    return RES_OK;

  case GET_SECTOR_COUNT:
    if (buff == NULL) {
      return RES_PARERR;
    }
    if (!sdcard_get_info(&info)) {
      return RES_NOTRDY;
    }
    *(DWORD *)buff = (DWORD)info.block_count;
    return RES_OK;

  case GET_BLOCK_SIZE:
    /* 擦除块大小只是 f_mkfs 给数据区做对齐用的提示，1 个扇区是保守值。
       SD 的 AU 大小在 CSD 里，为它去解析 CSD 不值得，格式化时对齐差一点无妨。 */
    if (buff == NULL) {
      return RES_PARERR;
    }
    *(DWORD *)buff = 1u;
    return RES_OK;

  case GET_SECTOR_SIZE:
    /* 只有 FF_MAX_SS != FF_MIN_SS 时 FatFs 才会问；这里两者都是 512，
       实现它只是为了将来改扇区大小配置时不用回头补 */
    if (buff == NULL) {
      return RES_PARERR;
    }
    *(DWORD *)buff = SDCARD_BLOCK_SIZE;
    return RES_OK;

  default:
    /* CTRL_TRIM / CTRL_POWER 等要开 FF_USE_TRIM 才用得到，当前配置不开 */
    return RES_PARERR;
  }
}
