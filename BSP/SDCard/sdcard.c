/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdcard.c
  * @brief   TF 卡底层驱动实现（BSP 层）
  *
  * 裸块读写 + 只读自检。文件系统相关的逻辑一律在 Services/FatFs，这一层只保证
  * "给定 lba 和缓冲，数据一定搬对"。
  *
  * 为什么走 IDMA 而不是轮询：-O0 构建下 CPU 轮询 SDMMC FIFO 喂不赢 20 MHz 的
  * 4-bit 总线，会出现偶发 CRC/超时；DMA 只由中断置标志，等待方轮询标志。
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

/* IDMA 只认 32 位字的传输，缓冲按 32 B 对齐最稳妥（同时满足 cache line 对齐） */
#define SDCARD_DMA_ALIGN   32u

volatile sdcard_probe_t g_sd_probe;
volatile uint32_t g_sdcard_last_error;

static uint8_t g_sector[SDCARD_BLOCK_SIZE] __attribute__((aligned(SDCARD_DMA_ALIGN)));

/* 调用者缓冲不是 4 字节对齐时的中转块：IDMA 的地址必须按字对齐，
   否则轻则数据错位，重则总线错误，所以宁可多一次拷贝 */
static uint8_t g_bounce[SDCARD_BLOCK_SIZE] __attribute__((aligned(SDCARD_DMA_ALIGN)));

#if SDCARD_PROBE_WRITE_TEST
static uint8_t g_sector_orig[SDCARD_BLOCK_SIZE] __attribute__((aligned(SDCARD_DMA_ALIGN)));
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

/**
 * @brief  一次 DMA 块传输（内部使用）
 * @param  write true=写 false=读
 * @param  lba   起始块地址
 * @param  buf   数据缓冲；本函数不检查对齐，调用方必须保证 4 字节对齐
 * @param  count 块数
 * @retval true  传输完成且卡已回到 TRANSFER 状态
 */
static bool sd_transfer(bool write, uint32_t lba, const void *buf, uint32_t count)
{
  HAL_StatusTypeDef hal;

  g_sd_xfer_done = false;
  g_sd_xfer_error = false;

  if (write) {
    hal = HAL_SD_WriteBlocks_DMA(&hsd1, (const uint8_t *)buf, lba, count);
  } else {
    /* 读方向的 HAL 形参是非 const：数据是往 buf 里写的，这里只是类型对不上 */
    hal = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)buf, lba, count);
  }
  if (hal != HAL_OK) {
    return false;
  }
  if (!sd_wait_xfer(SDCARD_XFER_TIMEOUT_MS)) {
    return false;
  }
  return sd_wait_transfer(SDCARD_XFER_TIMEOUT_MS);
}

/**
 * @brief  确保卡已识别并就绪（幂等）
 */
bool sdcard_init(void)
{
  /* MX_SDMMC1_SD_Init() 上电时已经识别过卡，正常路径在这里直接返回 */
  if (sdcard_is_ready()) {
    return true;
  }

  /* 卡可能插得比上电晚，或运行中被换过：重走 HAL 的识别流程。
     State != RESET 时 HAL_SD_Init 不会重复 MspInit，只重发 CMD0/ACMD41/... */
  if (HAL_SD_Init(&hsd1) != HAL_OK) {
    /* 打回 RESET，让下一次调用连 GPIO/时钟一起重新初始化 */
    hsd1.State = HAL_SD_STATE_RESET;
    return false;
  }
  return sdcard_is_ready();
}

bool sdcard_is_ready(void)
{
  return HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER;
}

bool sdcard_get_info(sdcard_info_t *info)
{
  HAL_SD_CardInfoTypeDef card;

  if (info == NULL) {
    return false;
  }

  if (HAL_SD_GetCardInfo(&hsd1, &card) != HAL_OK) {
    return false;
  }

  info->block_count = card.LogBlockNbr;
  info->block_size = card.LogBlockSize;
  /* 本板 SDMMC1 没接写保护引脚，SD 协议也没有写保护状态可查 */
  info->write_protected = false;

  /* 文件系统层按 512 B/块算偏移，块大小对不上就不能给它用 */
  return (info->block_size == SDCARD_BLOCK_SIZE) && (info->block_count != 0u);
}

bool sdcard_read_blocks(uint32_t lba, void *buf, uint32_t count)
{
  bool ok;

  g_sdcard_last_error = 0u;
  if ((buf == NULL) || (count == 0u)) {
    return false;
  }

  /* IDMA 按 32 位字搬运，缓冲必须 4 字节对齐。FatFs 自己的 win/buf 天然对齐，
     这条兜底分支只应付应用层直接传进来的缓冲区。 */
  if (((uintptr_t)buf & 3u) == 0u) {
    ok = sd_transfer(false, lba, buf, count);
  } else {
    uint8_t *dst = (uint8_t *)buf;

    ok = true;
    for (uint32_t i = 0u; i < count; i++) {
      if (!sd_transfer(false, lba + i, g_bounce, 1u)) {
        ok = false;
        break;
      }
      memcpy(dst + (i * SDCARD_BLOCK_SIZE), g_bounce, SDCARD_BLOCK_SIZE);
    }
  }

  if (!ok) {
    g_sdcard_last_error = hsd1.ErrorCode;
  }
  return ok;
}

bool sdcard_write_blocks(uint32_t lba, const void *buf, uint32_t count)
{
  bool ok;

  g_sdcard_last_error = 0u;
  if ((buf == NULL) || (count == 0u)) {
    return false;
  }

  if (((uintptr_t)buf & 3u) == 0u) {
    ok = sd_transfer(true, lba, buf, count);
  } else {
    const uint8_t *src = (const uint8_t *)buf;

    ok = true;
    for (uint32_t i = 0u; i < count; i++) {
      /* sd_transfer 返回时卡已回到 TRANSFER，g_bounce 可以安全复用 */
      memcpy(g_bounce, src + (i * SDCARD_BLOCK_SIZE), SDCARD_BLOCK_SIZE);
      if (!sd_transfer(true, lba + i, g_bounce, 1u)) {
        ok = false;
        break;
      }
    }
  }

  if (!ok) {
    g_sdcard_last_error = hsd1.ErrorCode;
  }
  return ok;
}

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
  if (!sdcard_read_blocks(0u, g_sector, 1u)) {
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
  if (!sdcard_read_blocks(g_sd_probe.part_lba, g_sector, 1u)) {
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

    if (!sdcard_read_blocks(last_lba, g_sector, 1u)) {
      g_sd_probe.wr_stage = 1u;                       /* 备份失败 */
      g_sd_probe.wr_err = hsd1.ErrorCode;
      g_sd_probe.wr_card_state = (uint32_t)HAL_SD_GetCardState(&hsd1);
    } else {
      memcpy(g_sector_orig, g_sector, sizeof(g_sector_orig));

      for (uint32_t i = 0u; i < sizeof(g_sector); i++) {
        g_sector[i] = (uint8_t)(i ^ 0xA5u);
      }

      if (!sdcard_write_blocks(last_lba, g_sector, 1u)) {
        g_sd_probe.wr_stage = 2u;                     /* 写失败 */
        g_sd_probe.wr_err = hsd1.ErrorCode;
        g_sd_probe.wr_card_state = (uint32_t)HAL_SD_GetCardState(&hsd1);
      } else if (!sdcard_read_blocks(last_lba, g_sector, 1u)) {
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
      g_sd_probe.wr_restore_ok = sdcard_write_blocks(last_lba, g_sector, 1u) ? 1u : 0u;
      if ((g_sd_probe.wr_restore_ok == 0u) && (g_sd_probe.wr_stage != 2u)) {
        g_sd_probe.wr_stage = 5u;                     /* 恢复失败 */
      }
    }
  }
#endif
}
