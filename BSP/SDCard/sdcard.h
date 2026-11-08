/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdcard.h
  * @brief   TF 卡底层探测：只读验证 SDMMC1 硬件通路（BSP 层）
  *
  * 这个模块只依赖 HAL / CubeMX 生成的 hsd1，不涉及文件系统。
  * 探测结果写进 g_sd_probe，方便用 J-Link 直接读内存，不依赖 RTT。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SDCARD_H
#define SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 结果结构体的魔数：J-Link 里看到这个值才说明探测函数真的跑过 */
#define SDCARD_PROBE_MAGIC   0x5DC4A001u

/* 探测阶段：失败时 stage 停在出问题的那一步 */
typedef enum {
  SDCARD_STAGE_IDLE = 0,        /**< 还没跑 */
  SDCARD_STAGE_INFO_FAIL,       /**< HAL_SD_GetCardInfo 失败 */
  SDCARD_STAGE_INFO_OK,         /**< 拿到卡信息 */
  SDCARD_STAGE_MBR_READ_FAIL,   /**< 读 0 号扇区失败 */
  SDCARD_STAGE_MBR_OK,          /**< 0 号扇区读成功 */
  SDCARD_STAGE_BS_READ_FAIL,    /**< 读分区引导扇区失败 */
  SDCARD_STAGE_BS_OK,           /**< 引导扇区解析完成 */
} sdcard_stage_t;

typedef struct {
  uint32_t magic;               /**< SDCARD_PROBE_MAGIC */
  uint32_t stage;               /**< sdcard_stage_t */
  uint32_t hal_error;           /**< 失败时 hsd1.ErrorCode */
  uint32_t cd_pin;              /**< PC7(SDMMC_CD) 电平，0 表示低 */

  /* HAL_SD_CardInfoTypeDef */
  uint32_t card_type;           /**< 1=SDSC 2=SDSC 3=SDHC/SDXC */
  uint32_t card_version;
  uint32_t card_class;
  uint32_t rca;
  uint32_t block_nbr;           /**< 物理块数（512 B/块） */
  uint32_t block_size;          /**< 物理块大小，应为 512 */
  uint32_t log_block_nbr;       /**< 逻辑块数 */
  uint32_t log_block_size;      /**< 逻辑块大小，应为 512 */

  /* MBR（0 号扇区） */
  uint32_t mbr_sig_ok;          /**< 0x55AA 结尾 */
  uint32_t part_type;           /**< 分区类型，0x0C = FAT32 LBA */
  uint32_t part_lba;            /**< 分区起始 LBA */

  /* FAT32 引导扇区 */
  uint32_t bs_sig_ok;
  uint32_t fat32_ok;            /**< 0x52 处是否为 "FAT32   " */
  uint32_t bytes_per_sector;
  uint32_t sectors_per_cluster;
  uint32_t reserved_sectors;
  uint32_t num_fats;
  uint32_t fat_size;            /**< 每个 FAT 占的扇区数 */
  uint32_t total_sectors;
  uint32_t root_cluster;
  uint8_t  label[12];           /**< 卷标（11 字节 + '\0'） */

  /* 写测试（需要显式打开 SDCARD_PROBE_WRITE_TEST） */
  uint32_t wr_stage;            /**< 0=没跑 1=读原扇区失败 2=写失败 3=读回失败 4=比对失败 5=恢复失败 6=通过 */
  uint32_t wr_lba;              /**< 测试用的 LBA */
  uint32_t wr_err;              /**< 失败时 hsd1.ErrorCode */
  uint32_t wr_card_state;       /**< 失败时 HAL_SD_GetCardState() */
  uint32_t wr_ok;               /**< 写入并读回比对成功 */
  uint32_t wr_restore_ok;       /**< 原内容已恢复 */
} sdcard_probe_t;

extern volatile sdcard_probe_t g_sd_probe;

/**
 * @brief  只读探测 TF 卡：卡信息 → MBR → 分区引导扇区
 * @note   调用前必须已经跑过 MX_SDMMC1_SD_Init()。不写卡，可反复调用。
 */
void sdcard_probe(void);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_H */
