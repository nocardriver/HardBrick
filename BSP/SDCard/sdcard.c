/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdcard.c
  * @brief   TF 卡底层探测实现（BSP 层）
  *
  * 只做只读验证：卡信息 → MBR → FAT32 引导扇区。
  * 目的是把"引脚/时钟/4-bit 总线"这条通路和文件系统问题分开定位。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sdcard.h"

#include "main.h"
#include "sdmmc.h"

#include <stdbool.h>
#include <string.h>

/* 写测试默认关闭：会往卡的最后一个扇区写东西（写完恢复），
   确认只读通路正常后再打开 */
#ifndef SDCARD_PROBE_WRITE_TEST
#define SDCARD_PROBE_WRITE_TEST   0
#endif

volatile sdcard_probe_t g_sd_probe;

/* HAL_SD 内部把缓冲当 uint32_t* 用，按 32 B 对齐最稳妥 */
static uint8_t g_sector[512] __attribute__((aligned(32)));

#if SDCARD_PROBE_WRITE_TEST
static uint8_t g_sector_orig[512] __attribute__((aligned(32)));
#endif

static uint16_t rd16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* IDMA 传输用：回调里只置标志，真正的等待在查询方 */
static volatile bool g_sd_xfer_done;
static volatile bool g_sd_xfer_error;

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (hsd->Instance == SDMMC1) {
    g_sd_xfer_done = true;
  }
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (hsd->Instance == SDMMC1) {
    g_sd_xfer_done = true;
  }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  if (hsd->Instance == SDMMC1) {
    g_sd_xfer_error = true;
    g_sd_xfer_done = true;
  }
}

/* HAL_SD_ReadBlocks 轮询返回只代表数据搬完，卡内部时序还没结束；
   等状态回到 TRANSFER 再发下一条命令，否则会偶发 CRC/超时错误 */
static bool sd_wait_transfer(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
    if ((HAL_GetTick() - start) > timeout_ms) {
      return false;
    }
  }
  return true;
}

/* 等 IDMA 传输结束；-O0 下轮询 FIFO 喂不赢 20 MHz，读写统一走内部 DMA */
static bool sd_wait_xfer(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (!g_sd_xfer_done) {
    if ((HAL_GetTick() - start) > timeout_ms) {
      return false;
    }
  }
  return !g_sd_xfer_error;
}

static bool sd_read_sector(uint32_t lba)
{
  g_sd_xfer_done = false;
  g_sd_xfer_error = false;

  if (HAL_SD_ReadBlocks_DMA(&hsd1, g_sector, lba, 1u) != HAL_OK) {
    return false;
  }
  if (!sd_wait_xfer(1000u)) {
    return false;
  }
  return sd_wait_transfer(1000u);
}

#if SDCARD_PROBE_WRITE_TEST
static bool sd_write_sector(uint32_t lba)
{
  g_sd_xfer_done = false;
  g_sd_xfer_error = false;

  if (HAL_SD_WriteBlocks_DMA(&hsd1, g_sector, lba, 1u) != HAL_OK) {
    return false;
  }
  if (!sd_wait_xfer(1000u)) {
    return false;
  }
  return sd_wait_transfer(1000u);
}
#endif

/**
 * @brief  只读探测 TF 卡
 * @note   失败时 g_sd_probe.stage 停在出错的那一步，hal_error 是 hsd1.ErrorCode。
 */
void sdcard_probe(void)
{
  HAL_SD_CardInfoTypeDef info;

  memset((void *)&g_sd_probe, 0, sizeof(g_sd_probe));
  g_sd_probe.magic = SDCARD_PROBE_MAGIC;
  g_sd_probe.cd_pin = (uint32_t)HAL_GPIO_ReadPin(SDMMC_CD_GPIO_Port, SDMMC_CD_Pin);

  /* 1. 卡信息：能拿到说明 CMD0/CMD8/ACMD41/CMD2/CMD3 全过了 */
  if (HAL_SD_GetCardInfo(&hsd1, &info) != HAL_OK) {
    g_sd_probe.stage = (uint32_t)SDCARD_STAGE_INFO_FAIL;
    g_sd_probe.hal_error = hsd1.ErrorCode;
    return;
  }

  g_sd_probe.stage = (uint32_t)SDCARD_STAGE_INFO_OK;
  g_sd_probe.card_type = info.CardType;
  g_sd_probe.card_version = info.CardVersion;
  g_sd_probe.card_class = info.Class;
  g_sd_probe.rca = info.RelCardAdd;
  g_sd_probe.block_nbr = info.BlockNbr;
  g_sd_probe.block_size = info.BlockSize;
  g_sd_probe.log_block_nbr = info.LogBlockNbr;
  g_sd_probe.log_block_size = info.LogBlockSize;

  /* 2. MBR（0 号扇区）：验证 4-bit 读通路 */
  if (!sd_read_sector(0u)) {
    g_sd_probe.stage = (uint32_t)SDCARD_STAGE_MBR_READ_FAIL;
    g_sd_probe.hal_error = hsd1.ErrorCode;
    return;
  }

  g_sd_probe.stage = (uint32_t)SDCARD_STAGE_MBR_OK;
  g_sd_probe.mbr_sig_ok =
      ((g_sector[510] == 0x55u) && (g_sector[511] == 0xAAu)) ? 1u : 0u;
  g_sd_probe.part_type = g_sector[446 + 4];
  g_sd_probe.part_lba = rd32(&g_sector[446 + 8]);

  /* 3. 分区引导扇区：确认是 FAT32，顺便把 BPB 抄出来 */
  if (!sd_read_sector(g_sd_probe.part_lba)) {
    g_sd_probe.stage = (uint32_t)SDCARD_STAGE_BS_READ_FAIL;
    g_sd_probe.hal_error = hsd1.ErrorCode;
    return;
  }

  g_sd_probe.stage = (uint32_t)SDCARD_STAGE_BS_OK;
  g_sd_probe.bs_sig_ok =
      ((g_sector[510] == 0x55u) && (g_sector[511] == 0xAAu)) ? 1u : 0u;
  g_sd_probe.fat32_ok =
      (memcmp(&g_sector[0x52], "FAT32   ", 8u) == 0) ? 1u : 0u;
  g_sd_probe.bytes_per_sector = rd16(&g_sector[0x0B]);
  g_sd_probe.sectors_per_cluster = g_sector[0x0D];
  g_sd_probe.reserved_sectors = rd16(&g_sector[0x0E]);
  g_sd_probe.num_fats = g_sector[0x10];
  g_sd_probe.total_sectors = rd32(&g_sector[0x20]);
  g_sd_probe.fat_size = rd32(&g_sector[0x24]);
  g_sd_probe.root_cluster = rd32(&g_sector[0x2C]);
  memcpy((void *)g_sd_probe.label, &g_sector[0x47], 11u);

#if SDCARD_PROBE_WRITE_TEST
  /* 4. 写通路：备份最后一个扇区 → 写测试图案 → 读回比对 → 恢复原内容 */
  {
    uint32_t last_lba = (g_sd_probe.block_nbr > 0u) ? (g_sd_probe.block_nbr - 1u) : 0u;

    g_sd_probe.wr_lba = last_lba;

    if (!sd_read_sector(last_lba)) {
      g_sd_probe.wr_stage = 1u;                       /* 备份失败 */
      g_sd_probe.wr_err = hsd1.ErrorCode;
      g_sd_probe.wr_card_state = (uint32_t)HAL_SD_GetCardState(&hsd1);
    } else {
      memcpy(g_sector_orig, g_sector, sizeof(g_sector_orig));

      for (uint32_t i = 0u; i < sizeof(g_sector); i++) {
        g_sector[i] = (uint8_t)(i ^ 0xA5u);
      }

      if (!sd_write_sector(last_lba)) {
        g_sd_probe.wr_stage = 2u;                     /* 写失败 */
        g_sd_probe.wr_err = hsd1.ErrorCode;
        g_sd_probe.wr_card_state = (uint32_t)HAL_SD_GetCardState(&hsd1);
      } else if (!sd_read_sector(last_lba)) {
        g_sd_probe.wr_stage = 3u;                     /* 读回失败 */
        g_sd_probe.wr_err = hsd1.ErrorCode;
        g_sd_probe.wr_card_state = (uint32_t)HAL_SD_GetCardState(&hsd1);
      } else {
        bool match = true;

        for (uint32_t i = 0u; i < sizeof(g_sector); i++) {
          if (g_sector[i] != (uint8_t)(i ^ 0xA5u)) {
            match = false;
            break;
          }
        }
        if (!match) {
          g_sd_probe.wr_stage = 4u;                   /* 内容比对失败 */
        } else {
          g_sd_probe.wr_ok = 1u;
          g_sd_probe.wr_stage = 6u;                   /* 写通路通过 */
        }
      }

      /* 无论成败都恢复原内容 */
      memcpy(g_sector, g_sector_orig, sizeof(g_sector));
      g_sd_probe.wr_restore_ok = sd_write_sector(last_lba) ? 1u : 0u;
      if ((g_sd_probe.wr_restore_ok == 0u) && (g_sd_probe.wr_stage != 2u)) {
        g_sd_probe.wr_stage = 5u;                     /* 恢复失败 */
      }
    }
  }
#endif
}
