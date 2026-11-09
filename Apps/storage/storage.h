/**
 * @file    storage.h
 * @author  YangKY
 * @brief   存储应用层：挂载 TF 卡上的 FAT 卷，并做一次文件读写自检
 *
 * 这一层负责"这个卷是谁的、挂没挂上"，FatFs 的磁盘细节在 Services/FatFs。
 * 上电后在 tx_application_define() 里创建 storage 线程，线程启动后：
 *
 *   1. f_mount() 强制挂载 0 号卷（走 disk_initialize → sdcard_init）
 *   2. 报告卷类型和剩余空间
 *   3. 追加一行启动记录到 0:/HARDBRICK.TXT
 *   4. 读回最后一行做比对，证明写进去的确实是本轮数据
 *   5. 列一遍根目录前几项（顺带验证 f_opendir / f_readdir）
 *
 * 自检结果同时写进 g_storage，可以用 J-Link 直接读内存看，不依赖 RTT——
 * 和 BSP 层 g_sd_probe 的做法一致，方便把"卡/链路"和"文件系统"分开定位。
 *
 * @note 线程自检完就挂起。后续要落盘的功能（轨迹记录、配置保存）在这个线程上
 *       加队列即可，不要在各处直接调 f_write——FatFs 虽然有卷锁保证不崩，但
 *       多个线程各写各的文件会让卡上的目录结构难以预期。
 *
 * @note TF 卡会通过 USB（Apps/usb_msc）暴露给 PC。PC 枚举成功后 usb_msc 会置位
 *       "主机占用"标志，本线程发现后不再动文件系统；PC 断开后标志清除，
 *       重新挂载（storage_resync）即可继续使用。块设备级别由 usb_msc 提供的
 *       媒体互斥量串行化，见 usb_msc_media_lock()。
 */

#ifndef STORAGE_H
#define STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ff.h"

/** 自检结果魔数：J-Link 里看到这个值才说明自检真的跑过 */
#define STORAGE_MAGIC   0x46415458u   /* 'FATX' */

/** 逻辑卷号字符串。0 号卷 = 唯一的物理盘 SDMMC1 */
#define STORAGE_DRIVE   "0:"

/** 自检文件路径：根目录下的固定名（8.3 规范，不额外占用 LFN 目录项） */
#define STORAGE_TEST_PATH   STORAGE_DRIVE "/HARDBRICK.TXT"

/**
 * 存储线程栈大小（字节，静态分配）
 * @note FF_USE_LFN = 2 时每次解析路径会在栈上开 (FF_MAX_LFN + 1) * 2 = 512 B，
 *       再叠加 f_readdir 里的 FILINFO（约 300 B），2048 太紧，这里给 4096。
 */
#define STORAGE_THREAD_STACK_SIZE   4096u

/** storage 线程优先级：最低的一档，磁盘 I/O 不该抢在 GPS / 显示前面 */
#define STORAGE_THREAD_PRIORITY     7u

/**
 * PC 占用 TF 卡时 storage 线程的等待粒度（tick）
 * @note 100 tick = 1 s。PC 通过 USB 用着卡的时候本线程只是定期醒来看看，
 *       等主机放开（拔掉 USB）再补做自检，详见 usb_msc.h。
 */
#define STORAGE_HOST_WAIT_TICKS     100u

/** 自检阶段：失败时 stage 停在出错的那一步 */
typedef enum {
  STORAGE_STAGE_IDLE = 0,        /**< 还没跑 */
  STORAGE_STAGE_MOUNT_FAIL,      /**< f_mount 失败 */
  STORAGE_STAGE_MOUNT_OK,        /**< 卷挂上了 */
  STORAGE_STAGE_INFO_OK,         /**< 容量查询完成 */
  STORAGE_STAGE_APPEND_FAIL,     /**< 追加记录失败 */
  STORAGE_STAGE_APPEND_OK,       /**< 记录写完并关闭 */
  STORAGE_STAGE_VERIFY_FAIL,     /**< 读回比对失败 */
  STORAGE_STAGE_VERIFY_OK,       /**< 读回内容与写入一致，自检通过 */
} storage_stage_t;

typedef struct {
  uint32_t magic;          /**< STORAGE_MAGIC */
  uint32_t stage;          /**< storage_stage_t */
  uint32_t rc;             /**< 失败时的 FRESULT */
  uint32_t fs_type;        /**< FS_FAT12 / FS_FAT16 / FS_FAT32 */
  uint32_t free_clusters;  /**< 空闲簇数（f_getfree 的结果） */
  uint32_t free_mb;        /**< 剩余空间，MB */
  uint32_t total_mb;       /**< 卷总容量，MB（按 FAT 簇数算） */
  uint32_t file_bytes;     /**< 自检文件读回时的大小 */
} storage_status_t;

extern volatile storage_status_t g_storage;

/**
 * @brief  创建存储线程（在 tx_application_define 里调用）
 * @note   必须在 tx_kernel_enter() 之后：FatFs 的卷互斥量是 ThreadX 对象，
 *         内核没起来之前 f_mount 会失败。
 */
void storage_init(void);

/**
 * @brief  卷是否已挂载且自检通过
 * @retval true 可以对 STORAGE_DRIVE 做文件操作
 */
bool storage_ready(void);

/**
 * @brief  取已挂载的卷对象，供其它模块直接调用 f_open / f_opendir 等
 * @retval NULL 卷还没挂上
 * @note   调用方自己在多线程下要配合 FatFs 的卷锁使用（FF_FS_REENTRANT 已打开，
 *         每个 f_* 调用内部会自己取锁，不需要再加外层锁）。
 */
FATFS *storage_volume(void);

/**
 * @brief  主机（PC）动过卡之后，强制重新挂载本地的 FAT 卷
 * @retval true  重新挂载成功
 * @retval false 卷没挂过 / 媒体锁超时 / f_mount 失败
 * @note   通过 USB 暴露这张卡时，PC 对 FAT 的改动只有重新挂载才可见（FatFs 会缓存
 *         BPB 和 FAT 窗口）。目前 storage 线程自检完就挂起了，这个接口留给后续
 *         "PC 断开后继续写日志" 之类的功能用，调用方要顺便拿媒体锁——
 *         本函数内部已经拿了 USB_MSC_MEDIA_LOCK_TIMEOUT 的锁。
 */
bool storage_resync(void);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_H */
