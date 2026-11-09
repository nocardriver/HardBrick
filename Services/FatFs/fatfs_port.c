/**
 * @file    fatfs_port.c
 * @brief   FatFs 线程安全移植层：用 ThreadX 互斥量实现 FF_FS_REENTRANT 钩子
 *
 * 本工程有 gps / display / storage 多个线程，它们最终可能同时摸同一个 SD 卷。
 * FatFs 自己不保证可重入（ff.c 里对同一卷的内部状态是全局的），所以打开
 * FF_FS_REENTRANT，由这里提供 ff_mutex_* 四个钩子，替换上游 ffsystem.c 的
 * FreeRTOS/uC-OS/CMSIS-RTOS 示例实现。
 *
 * 调用时机（都在 ff.c 内部，应用层不用管）：
 *   f_mount(fs, path, opt)  → ff_mutex_create(vol) / ff_mutex_delete(vol)
 *   每个文件/目录接口进出     → ff_mutex_take(vol) / ff_mutex_give(vol)
 *
 * @note 互斥量用 TX_INHERIT（优先级继承）：低优先级线程做磁盘 I/O 时把锁拿走，
 *       高优先级的显示线程来等锁，继承后低优先级线程不会被同级的其它线程压住，
 *       避免优先级反转把界面卡住。
 * @note f_mount() 本身不可重入（FatFs 文档明确要求），调用方要保证同一时刻只有
 *       一个线程在挂载/卸载。
 */

#include "ff.h"

#include "tx_api.h"

#include <stdbool.h>

#if FF_FS_REENTRANT

/* 每个逻辑卷一把锁；FF_VOLUMES 号槽位是打开 FF_FS_LOCK 时用的系统锁。
   当前 FF_FS_LOCK = 0 用不到它，槽位先留着，将来打开不用改这里。 */
#define FATFS_MUTEX_SLOTS   (FF_VOLUMES + 1)

static TX_MUTEX g_fatfs_mutex[FATFS_MUTEX_SLOTS];
static bool     g_fatfs_mutex_ready[FATFS_MUTEX_SLOTS];

/**
 * @brief  创建卷互斥量
 * @param  vol 卷号 0..FF_VOLUMES-1；FF_VOLUMES 为系统锁
 * @retval 1 成功；0 失败（f_mount 会因此返回 FR_INT_ERR）
 */
int ff_mutex_create(int vol)
{
  if ((vol < 0) || (vol >= FATFS_MUTEX_SLOTS)) {
    return 0;
  }

  /* 同号重复创建时 ThreadX 会回 TX_MUTEX_ERROR，这里直接认已有对象 */
  if (g_fatfs_mutex_ready[vol]) {
    return 1;
  }

  if (tx_mutex_create(&g_fatfs_mutex[vol], "fatfs", TX_INHERIT) != TX_SUCCESS) {
    return 0;
  }

  g_fatfs_mutex_ready[vol] = true;
  return 1;
}

/**
 * @brief  删除卷互斥量（f_mount(0, ...) 时调用）
 * @note   若仍有线程持有，ThreadX 返回 TX_MUTEX_ERROR，此处忽略：f_mount 卸载时
 *         不应有人还在用这个卷，真出现了也属于调用方的用法问题。
 */
void ff_mutex_delete(int vol)
{
  if ((vol < 0) || (vol >= FATFS_MUTEX_SLOTS) || !g_fatfs_mutex_ready[vol]) {
    return;
  }

  (void)tx_mutex_delete(&g_fatfs_mutex[vol]);
  g_fatfs_mutex_ready[vol] = false;
}

/**
 * @brief  申请卷访问权
 * @param  vol 卷号
 * @retval 1 拿到锁；0 超时（FatFs 会返回 FR_TIMEOUT）
 * @note   FF_FS_TIMEOUT 的单位是 ThreadX tick（10 ms），配置见 ffconf.h
 */
int ff_mutex_take(int vol)
{
  if ((vol < 0) || (vol >= FATFS_MUTEX_SLOTS) || !g_fatfs_mutex_ready[vol]) {
    return 0;
  }

  return (tx_mutex_get(&g_fatfs_mutex[vol], FF_FS_TIMEOUT) == TX_SUCCESS) ? 1 : 0;
}

/**
 * @brief  释放卷访问权
 * @param  vol 卷号
 */
void ff_mutex_give(int vol)
{
  if ((vol < 0) || (vol >= FATFS_MUTEX_SLOTS) || !g_fatfs_mutex_ready[vol]) {
    return;
  }

  (void)tx_mutex_put(&g_fatfs_mutex[vol]);
}

#endif /* FF_FS_REENTRANT */
