/**
 * @file    storage.c
 * @author  YangKY
 * @brief   存储应用层实现：挂载 FAT 卷 + 上电自检
 *
 * 自检按"先挂载、再读容量、再写、最后读回比对"的顺序推进，任何一步失败
 * stage 就停在那一格，rc 是 FatFs 的 FRESULT。这样现场只凭 RTT 一行输出或者
 * J-Link 读 g_storage 就能判断是卡的问题、挂载的问题还是写入的问题。
 */

#include "storage.h"

#include "tx_api.h"
#include "SEGGER_RTT.h"

#include "usb_msc.h"

#include <string.h>

/* 追加记录的内容：前后缀写死，中间的数字用 ThreadX tick，
   每次上电都不一样，读回比对才能证明写进去的是本轮数据 */
static const char STORAGE_LINE_PREFIX[] = "boot ";
static const char STORAGE_LINE_SUFFIX[] = " tick FATFS ok\r\n";

/* 行缓冲：前缀 + 最多 10 位十进制 + 后缀，48 B 足够 */
#define STORAGE_LINE_MAX   (sizeof(STORAGE_LINE_PREFIX) + 11u + sizeof(STORAGE_LINE_SUFFIX))

/* STORAGE_LINE_SUFFIX 结尾占的字节数（"\r\n"），打印时把这部分从行里摘掉 */
#define STORAGE_LINE_EOL_LEN       2u

/* 根目录最多打印几项，太多了会刷屏，这里只用来验证 readdir 通路 */
#define STORAGE_LIST_MAX   8u

/* 线程挂起后若被唤醒，睡 1 s 再看一次队列（当前没有活，只做占位） */
#define STORAGE_IDLE_TICKS 100u

static FATFS     g_fs;
static TX_THREAD g_thread;
static uint8_t   g_stack[STORAGE_THREAD_STACK_SIZE] __attribute__((aligned(8)));
static char      g_line[STORAGE_LINE_MAX];
/* 上一次 build_line() 写出的字节数，report() 打印时要去掉结尾的 CRLF */
static uint32_t  g_line_len;

volatile storage_status_t g_storage;

/**
 * @brief  把非负整数写成十进制，返回写出的字符数
 * @note   为了打一个序号不值得把 picolibc 的 printf 格式化整段拉进 flash
 */
static uint32_t u32_to_dec(uint32_t value, char *out)
{
  char     tmp[10];
  uint32_t n = 0u;

  do {
    tmp[n] = (char)('0' + (value % 10u));
    n++;
    value /= 10u;
  } while ((value != 0u) && (n < sizeof(tmp)));

  /* tmp 里是低位在前，倒过来写出去 */
  for (uint32_t i = 0u; i < n; i++) {
    out[i] = tmp[n - 1u - i];
  }
  return n;
}

/**
 * @brief  组一行启动记录，返回长度（不含结束符，本来也没有）
 */
static uint32_t build_line(char *buf, uint32_t cap)
{
  uint32_t n = 0u;

  if (cap < STORAGE_LINE_MAX) {
    return 0u;
  }

  memcpy(&buf[n], STORAGE_LINE_PREFIX, sizeof(STORAGE_LINE_PREFIX) - 1u);
  n += sizeof(STORAGE_LINE_PREFIX) - 1u;
  n += u32_to_dec((uint32_t)tx_time_get(), &buf[n]);
  memcpy(&buf[n], STORAGE_LINE_SUFFIX, sizeof(STORAGE_LINE_SUFFIX) - 1u);
  n += sizeof(STORAGE_LINE_SUFFIX) - 1u;

  return n;
}

/**
 * @brief  读容量：空闲簇用 f_getfree，总容量用 FAT 簇数算
 * @note   失败不致命，只影响报告，卷照样可用
 */
static void read_volume_info(void)
{
  FATFS   *fs = NULL;
  DWORD    nclst = 0u;
  uint64_t cluster_bytes;

  /* 一个簇多少字节：csize 是"每簇几个扇区"，扇区大小固定 FF_MAX_SS */
  cluster_bytes = (uint64_t)g_fs.csize * (uint64_t)FF_MAX_SS;

  if (f_getfree(STORAGE_DRIVE, &nclst, &fs) == FR_OK) {
    g_storage.free_clusters = (uint32_t)nclst;
    g_storage.free_mb =
        (uint32_t)(((uint64_t)nclst * cluster_bytes) / (1024u * 1024u));
  }

  /* n_fatent = 簇数 + 2（FAT 表头两个保留项） */
  if (g_fs.n_fatent > 2u) {
    g_storage.total_mb = (uint32_t)(((uint64_t)(g_fs.n_fatent - 2u) * cluster_bytes) /
                                    (1024u * 1024u));
  }
}

/**
 * @brief  列根目录前几项，验证 f_opendir / f_readdir
 */
static void list_root(void)
{
  DIR      dir;
  FILINFO  fno;
  uint32_t shown = 0u;

  if (f_opendir(&dir, STORAGE_DRIVE "/") != FR_OK) {
    SEGGER_RTT_WriteString(0, "[SD] root open failed\r\n");
    return;
  }

  while ((f_readdir(&dir, &fno) == FR_OK) && (fno.fname[0] != '\0') &&
         (shown < STORAGE_LIST_MAX)) {
    SEGGER_RTT_printf(0, "[SD]   %s%s  %u B\r\n",
                      ((fno.fattrib & AM_DIR) != 0u) ? "<DIR> " : "",
                      fno.fname,
                      (unsigned)fno.fsize);
    shown++;
  }

  (void)f_closedir(&dir);
}

/**
 * @brief  追加一行启动记录，然后读回最后一行做比对
 * @retval true 自检通过
 */
static bool append_and_verify(void)
{
  FIL     fil;
  FRESULT rc;
  UINT    bw = 0u;
  UINT    br = 0u;
  uint32_t len;
  char    readback[STORAGE_LINE_MAX];

  len = build_line(g_line, sizeof(g_line));
  g_line_len = len;
  if (len == 0u) {
    g_storage.stage = (uint32_t)STORAGE_STAGE_APPEND_FAIL;
    return false;
  }

  /* FA_OPEN_APPEND：文件不存在就建，存在就把指针定到末尾 */
  rc = f_open(&fil, STORAGE_TEST_PATH, FA_WRITE | FA_OPEN_APPEND);
  if (rc != FR_OK) {
    g_storage.stage = (uint32_t)STORAGE_STAGE_APPEND_FAIL;
    g_storage.rc = (uint32_t)rc;
    return false;
  }

  rc = f_write(&fil, g_line, len, &bw);
  if (rc == FR_OK) {
    /* f_close 内部会 f_sync，把 FIL 里可能还脏着的扇区刷到卡上 */
    rc = f_close(&fil);
  } else {
    (void)f_close(&fil);
  }

  if ((rc != FR_OK) || (bw != len)) {
    g_storage.stage = (uint32_t)STORAGE_STAGE_APPEND_FAIL;
    g_storage.rc = (uint32_t)rc;
    return false;
  }
  g_storage.stage = (uint32_t)STORAGE_STAGE_APPEND_OK;

  /* 读回文件末尾 len 字节，必须和刚写的一模一样 */
  rc = f_open(&fil, STORAGE_TEST_PATH, FA_READ);
  if (rc != FR_OK) {
    g_storage.stage = (uint32_t)STORAGE_STAGE_VERIFY_FAIL;
    g_storage.rc = (uint32_t)rc;
    return false;
  }

  g_storage.file_bytes = (uint32_t)f_size(&fil);

  if (f_size(&fil) < (FSIZE_t)len) {
    (void)f_close(&fil);
    g_storage.stage = (uint32_t)STORAGE_STAGE_VERIFY_FAIL;
    g_storage.rc = (uint32_t)FR_INT_ERR;
    return false;
  }

  rc = f_lseek(&fil, f_size(&fil) - (FSIZE_t)len);
  if (rc == FR_OK) {
    rc = f_read(&fil, readback, len, &br);
  }
  (void)f_close(&fil);

  if ((rc != FR_OK) || (br != len) || (memcmp(readback, g_line, len) != 0)) {
    g_storage.stage = (uint32_t)STORAGE_STAGE_VERIFY_FAIL;
    g_storage.rc = (uint32_t)rc;
    return false;
  }

  g_storage.stage = (uint32_t)STORAGE_STAGE_VERIFY_OK;
  return true;
}

/**
 * @brief  自检主流程
 */
static void run_selftest(void)
{
  FRESULT rc;

  memset((void *)&g_storage, 0, sizeof(g_storage));
  g_storage.magic = STORAGE_MAGIC;

  /* 1. 挂载。opt = 1 强制重新走 disk_initialize，卡换过也能立刻反映出来 */
  rc = f_mount(&g_fs, STORAGE_DRIVE, 1u);
  if (rc != FR_OK) {
    g_storage.stage = (uint32_t)STORAGE_STAGE_MOUNT_FAIL;
    g_storage.rc = (uint32_t)rc;
    return;
  }
  g_storage.stage = (uint32_t)STORAGE_STAGE_MOUNT_OK;
  g_storage.fs_type = (uint32_t)g_fs.fs_type;

  /* 2. 容量 */
  read_volume_info();
  g_storage.stage = (uint32_t)STORAGE_STAGE_INFO_OK;

  /* 3. 写 + 读回比对 */
  if (!append_and_verify()) {
    return;
  }

  /* 4. 目录列举：只做通路验证，失败不影响自检结论 */
  list_root();
}

/**
 * @brief  把自检结果打到 RTT
 */
static void report(void)
{
  const char *fs_name = "?";

  if (g_storage.fs_type == FS_FAT12) {
    fs_name = "FAT12";
  } else if (g_storage.fs_type == FS_FAT16) {
    fs_name = "FAT16";
  } else if (g_storage.fs_type == FS_FAT32) {
    fs_name = "FAT32";
  }

  SEGGER_RTT_printf(0, "[SD] stage=%u rc=%u fs=%s\r\n",
                    (unsigned)g_storage.stage, (unsigned)g_storage.rc, fs_name);

  if (g_storage.stage >= (uint32_t)STORAGE_STAGE_INFO_OK) {
    SEGGER_RTT_printf(0, "[SD] total=%u MB free=%u MB, file=%u B\r\n",
                      (unsigned)g_storage.total_mb, (unsigned)g_storage.free_mb,
                      (unsigned)g_storage.file_bytes);
  }

  if (g_storage.stage == (uint32_t)STORAGE_STAGE_VERIFY_OK) {
    /* g_line 自带结尾的 CRLF，这里只打内容，避免日志里多出空行 */
    SEGGER_RTT_WriteString(0, "[SD] fatfs self-test PASS, wrote \"");
    (void)SEGGER_RTT_Write(0, g_line, g_line_len - STORAGE_LINE_EOL_LEN);
    SEGGER_RTT_WriteString(0, "\"\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "[SD] fatfs self-test FAIL\r\n");
  }
}

static void storage_thread_entry(ULONG thread_input)
{
  (void)thread_input;

  /* PC 通过 USB 用着这张卡时，固件不能再去挂载/写文件：两边同时改 FAT 会互相
     破坏。这里等主机把卡放开（拔掉 USB）再做自检；PC 一直在用时本线程就一直
     睡，不占用 CPU，也不影响其它线程。日志只打一次，避免每秒刷屏。 */
  if (usb_msc_host_active()) {
    SEGGER_RTT_WriteString(0, "[SD] PC owns the TF card, FAT self-test deferred\r\n");
  }
  while (usb_msc_host_active()) {
    tx_thread_sleep(STORAGE_HOST_WAIT_TICKS);
  }

  /* 卡是共享外设：拿到媒体锁再操作，避免和 USB 侧的块传输交叉 */
  if (usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    run_selftest();
    usb_msc_media_unlock();
  } else {
    SEGGER_RTT_WriteString(0, "[SD] media lock timeout, FAT self-test skipped\r\n");
  }

  report();

  /* 自检做完没有活干，直接挂起；将来落盘功能在这个线程上加队列后由队列唤醒 */
  (void)tx_thread_suspend(tx_thread_identify());

  for (;;) {
    tx_thread_sleep(STORAGE_IDLE_TICKS);
  }
}

void storage_init(void)
{
  if (tx_thread_create(&g_thread, "storage", storage_thread_entry, 0u,
                       g_stack, sizeof(g_stack),
                       STORAGE_THREAD_PRIORITY, STORAGE_THREAD_PRIORITY,
                       TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
    SEGGER_RTT_WriteString(0, "[SD] storage thread create failed\r\n");
  }
}

bool storage_ready(void)
{
  return g_storage.stage == (uint32_t)STORAGE_STAGE_VERIFY_OK;
}

FATFS *storage_volume(void)
{
  if (g_storage.stage < (uint32_t)STORAGE_STAGE_MOUNT_OK) {
    return NULL;
  }
  return &g_fs;
}

bool storage_resync(void)
{
  FRESULT rc;

  /* 还没挂过卷就没什么可刷新的，交给存储线程下次自检 */
  if (g_storage.stage < (uint32_t)STORAGE_STAGE_MOUNT_OK) {
    return false;
  }

  /* PC 可能改过卡上的目录结构，本地 FATFS 缓存（BPB / FAT 窗口）已经过期，
     opt=1 强制重新走 disk_initialize + 重新读引导扇区 */
  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    return false;
  }

  rc = f_mount(&g_fs, STORAGE_DRIVE, 1u);
  usb_msc_media_unlock();

  return rc == FR_OK;
}
