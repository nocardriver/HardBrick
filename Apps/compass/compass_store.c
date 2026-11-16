/**
 * @file    compass_store.c
 * @author  YangKY
 * @brief   罗盘标定的文件持久化实现（TF 卡 + FATFS）
 *
 * 每次读写都套一层 usb_msc_media_lock()：SD 卡块设备是 USB MSC 和 FATFS 共用的，
 * 只有 media 锁能把两边的命令串起来；同时用 usb_msc_host_active() 躲开
 * "PC 正在用这张卡"的时间段（那时卡上的 FAT 归 PC 管，插进去写只会互相破坏）。
 */

#include "compass_store.h"

#include "ff.h"
#include "storage.h"
#include "usb_msc.h"

#include <string.h>

#define COMPASS_STORE_MAGIC 0x50415241u /**< 'PARA' */
#define COMPASS_STORE_SIZE 16u

static uint32_t crc32(const uint8_t *p, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
  uint32_t i;
  uint32_t b;

  for (i = 0u; i < len; i++) {
    crc ^= p[i];
    for (b = 0u; b < 8u; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }

  return ~crc;
}

static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void put_s16(uint8_t *p, int32_t v)
{
  p[0] = (uint8_t)((uint32_t)v & 0xFFu);
  p[1] = (uint8_t)(((uint32_t)v >> 8) & 0xFFu);
}

static int32_t get_s16(const uint8_t *p)
{
  return (int32_t)(int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool compass_store_available(void)
{
  return storage_ready() && (!usb_msc_host_active());
}

bool compass_store_load(int32_t *off_x, int32_t *off_y, int32_t *off_z,
                        int32_t *radius)
{
  FIL fil;
  FRESULT rc;
  UINT got = 0u;
  uint8_t buf[COMPASS_STORE_SIZE];
  bool ok = false;

  if ((off_x == NULL) || (off_y == NULL) || (off_z == NULL) || (radius == NULL) ||
      (!compass_store_available())) {
    return false;
  }

  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    return false;
  }

  rc = f_open(&fil, COMPASS_STORE_PATH, FA_READ);
  if ((rc == FR_OK) && (f_size(&fil) == COMPASS_STORE_SIZE)) {
    if ((f_read(&fil, buf, COMPASS_STORE_SIZE, &got) == FR_OK) &&
        (got == COMPASS_STORE_SIZE) &&
        (get_u32(&buf[0]) == COMPASS_STORE_MAGIC) &&
        (get_u32(&buf[12]) == crc32(buf, 12u))) {
      *off_x = get_s16(&buf[4]);
      *off_y = get_s16(&buf[6]);
      *off_z = get_s16(&buf[8]);
      *radius = get_s16(&buf[10]);
      ok = true;
    }
  }
  if (rc == FR_OK) {
    (void)f_close(&fil);
  }

  usb_msc_media_unlock();

  return ok;
}

bool compass_store_save(int32_t off_x, int32_t off_y, int32_t off_z,
                        int32_t radius)
{
  FIL fil;
  FRESULT rc;
  UINT wrote = 0u;
  UINT got = 0u;
  uint8_t buf[COMPASS_STORE_SIZE];
  uint8_t check[COMPASS_STORE_SIZE];
  bool ok = false;

  if (!compass_store_available()) {
    return false;
  }

  put_u32(&buf[0], COMPASS_STORE_MAGIC);
  put_s16(&buf[4], off_x);
  put_s16(&buf[6], off_y);
  put_s16(&buf[8], off_z);
  put_s16(&buf[10], radius);
  put_u32(&buf[12], crc32(buf, 12u));

  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    return false;
  }

  rc = f_open(&fil, COMPASS_STORE_PATH, FA_WRITE | FA_CREATE_ALWAYS);
  if (rc == FR_OK) {
    if ((f_write(&fil, buf, COMPASS_STORE_SIZE, &wrote) == FR_OK) &&
        (wrote == COMPASS_STORE_SIZE) && (f_sync(&fil) == FR_OK)) {
      ok = true;
    }
    (void)f_close(&fil);
  }

  /* 回读比对：卡写坏/掉电导致内容不一致时不要假装成功 */
  if (ok) {
    ok = false;
    rc = f_open(&fil, COMPASS_STORE_PATH, FA_READ);
    if (rc == FR_OK) {
      if ((f_read(&fil, check, COMPASS_STORE_SIZE, &got) == FR_OK) &&
          (got == COMPASS_STORE_SIZE) && (memcmp(buf, check, COMPASS_STORE_SIZE) == 0)) {
        ok = true;
      }
      (void)f_close(&fil);
    }
  }

  usb_msc_media_unlock();

  return ok;
}

bool compass_store_clear(void)
{
  FRESULT rc;
  bool ok = false;

  if (!compass_store_available()) {
    return false;
  }

  if (!usb_msc_media_lock(USB_MSC_MEDIA_LOCK_TIMEOUT)) {
    return false;
  }

  rc = f_unlink(COMPASS_STORE_PATH);
  ok = (rc == FR_OK) || (rc == FR_NO_FILE);

  usb_msc_media_unlock();

  return ok;
}
