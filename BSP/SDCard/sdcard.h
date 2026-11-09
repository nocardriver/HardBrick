/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdcard.h
  * @brief   TF 卡底层驱动：SDMMC1 上的裸块读写 + 上电自检（BSP 层）
  *
  * 这一层只认「块」，不认文件：给上层文件系统（Services/FatFs 的 diskio）提供
  * lba + 缓冲区 形式的读写。FAT/MBR/簇这些概念都在文件系统层，这里一概不管。
  *
  * 两块内容：
  *   1. 块设备接口 sdcard_read_blocks() / sdcard_write_blocks() 等：主接口，
  *      上电时 MX_SDMMC1_SD_Init() 已把卡识别过一遍，这里只负责传输。
  *   2. sdcard_probe()：只读自检，把 MBR / FAT32 引导扇区解析出来，用于把
  *      "引脚/时钟/总线宽度" 的问题和"文件系统"的问题分开定位。探测结果写进
  *      g_sd_probe，可以直接用 J-Link 读内存看，不依赖 RTT。
  *
  * @note 所有块接口内部走 SDMMC 的 IDMA（-O0 下轮询 FIFO 喂不赢 20 MHz），
  *       因此是阻塞式调用：等到整笔传输完成才返回，不要在中断里调用。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SDCARD_H
#define SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* 块设备接口                                                                */
/*---------------------------------------------------------------------------*/

/** 逻辑块大小（字节）。SD 卡固定 512，也是 FatFs 的 FF_MIN_SS / FF_MAX_SS */
#define SDCARD_BLOCK_SIZE       512u

/** 单次传输超时（ms）。20 MHz × 4-bit 下 >5 MB/s，1 s 足够 5 MB 的一笔请求 */
#define SDCARD_XFER_TIMEOUT_MS  1000u

/** 卡信息快照（只取文件系统层关心的字段） */
typedef struct {
  uint32_t block_count;     /**< 逻辑块总数（每块 SDCARD_BLOCK_SIZE 字节） */
  uint32_t block_size;      /**< 逻辑块大小，正常为 512 */
  bool     write_protected; /**< 卡是否写保护（本板未接 WP 引脚，恒为 false） */
} sdcard_info_t;

/**
 * @brief  最近一次块传输的 HAL 错误码（0 表示没有失败）
 * @note   只在失败时写，方便 RTT / J-Link 定位（CRC、超时、IDMA 等）。
 */
extern volatile uint32_t g_sdcard_last_error;

/**
 * @brief  确保卡已识别并就绪（幂等）
 * @retval true  卡处于 TRANSFER 状态，可以读写
 * @retval false 无卡 / 识别失败，g_sdcard_last_error 为 0（失败发生在 HAL_SD_Init 内部，
 *               原因看 hsd1.ErrorCode）
 * @note   已就绪时立刻返回，不会重复走 CMD0/ACMD41。上电时卡没插好、或运行中
 *         换过卡，再调用一次就能重新识别（HAL_SD_Init 会重配 4-bit 总线）。
 *         不要在中断里调用：内部要发命令、可能要等几十毫秒。
 */
bool sdcard_init(void);

/**
 * @brief  查询卡当前是否在 TRANSFER 状态
 * @note   会真的发一条 CMD13，耗时约几十微秒。文件系统每次操作都会问一次状态，
 *         那条路径用的是 diskio.c 里的缓存标志，不要拿这个函数去轮询。
 */
bool sdcard_is_ready(void);

/**
 * @brief  取卡容量信息
 * @param  info 调用者提供的结构体，不能为 NULL
 * @retval true  读取成功
 * @retval false info 为 NULL
 */
bool sdcard_get_info(sdcard_info_t *info);

/**
 * @brief  读若干个连续块
 * @param  lba   起始逻辑块地址（相对整张卡的物理地址，含 MBR 的绝对扇区号）
 * @param  buf   目标缓冲；4 字节对齐时直接给 IDMA，否则按块经内部对齐缓冲中转
 * @param  count 块数（>= 1）
 * @retval true  全部读到
 * @retval false 参数非法 / 传输失败 / 超时，g_sdcard_last_error 给出 HAL 错误码
 * @note   传输完成后会等卡回到 TRANSFER 状态再返回，所以返回 true 就是真的写完了。
 */
bool sdcard_read_blocks(uint32_t lba, void *buf, uint32_t count);

/**
 * @brief  写若干个连续块
 * @param  lba   起始逻辑块地址
 * @param  buf   源缓冲；对齐要求同 sdcard_read_blocks()
 * @param  count 块数（>= 1）
 * @retval true  全部写入并等到卡回到 TRANSFER
 * @retval false 参数非法 / 传输失败 / 超时，g_sdcard_last_error 给出 HAL 错误码
 */
bool sdcard_write_blocks(uint32_t lba, const void *buf, uint32_t count);

/*---------------------------------------------------------------------------*/
/* 只读自检                                                                  */
/*---------------------------------------------------------------------------*/

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
  uint32_t card_type;           /**< CARD_SDSC(0)=标准容量 CARD_SDHC_SDXC(1)=高/扩展容量 */
  uint32_t card_version;        /**< CARD_V1_X(0) / CARD_V2_X(1) */
  uint32_t card_class;          /**< CCC(卡命令集) */
  uint32_t rca;                 /**< 相对卡地址 */
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
