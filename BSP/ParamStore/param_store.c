/**
 * @file    param_store.c
 * @author  YangKY
 * @brief   小参数持久化实现：内部 Flash 日志式存储
 *
 * 存储布局：一个 8 KB 页里顺序追加 16 字节一条的记录，页写满再整页擦除重来。
 * 一次擦除能写 512 条，避免"每存一次擦一次页"（擦写寿命按页算）。
 *
 * 记录（16 B，正好一个 quadword）：
 *   [0..3]   魔数 0x50415241（'PARA'）
 *   [4..11]  负载，调用者定义
 *   [12..15] CRC32（覆盖前 12 字节）
 *
 * 页面选择：本板实测 FLASH_SIZE 寄存器（0x0BFA07A0）= 1024 KB，所以最后一个
 * 8 KB 页是 0x080FE000——双 bank 下是 bank2 第 63 页，单 bank 下是 bank1 第 127 页，
 * 两种布局地址相同；先按双 bank 擦，失败再按单 bank 擦。固件只有 350 KB 左右，
 * 离这页很远；万一以后长过来，擦除前的"异物检查"会拒绝执行，不会把代码擦掉。
 */

#include "param_store.h"

#include "main.h"

#include <string.h>

/** 'PARA'：用来把本模块的记录和"别的数据"区分开 */
#define PARAM_STORE_MAGIC 0x50415241u

/** 1 MB 器件的最后一个 8 KB 页（见文件头注释） */
#define PARAM_STORE_BASE 0x080FE000u

#define PARAM_STORE_PAGE_SIZE 0x2000u
#define PARAM_STORE_RECORD_SIZE 16u
#define PARAM_STORE_SLOTS (PARAM_STORE_PAGE_SIZE / PARAM_STORE_RECORD_SIZE)

/** 魔数之后是负载，最后 4 字节是 CRC */
#define PARAM_STORE_CRC_LEN (PARAM_STORE_RECORD_SIZE - 4u)

static uint32_t g_write_slot; /**< 下一条记录的槽位（= 第一个空槽） */
static bool g_ready;

static const uint8_t *slot_addr(uint32_t slot)
{
  return (const uint8_t *)(uintptr_t)(PARAM_STORE_BASE +
                                      slot * PARAM_STORE_RECORD_SIZE);
}

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

/** @brief 从 Flash 拷一条记录到 RAM（Flash 读取走字节指针，避免对齐假设） */
static void slot_read(uint32_t slot, uint8_t out[PARAM_STORE_RECORD_SIZE])
{
  const volatile uint8_t *src = (const volatile uint8_t *)slot_addr(slot);
  uint32_t i;

  for (i = 0u; i < PARAM_STORE_RECORD_SIZE; i++) {
    out[i] = src[i];
  }
}

static bool slot_blank(uint32_t slot)
{
  uint8_t rec[PARAM_STORE_RECORD_SIZE];
  uint32_t i;

  slot_read(slot, rec);

  for (i = 0u; i < PARAM_STORE_RECORD_SIZE; i++) {
    if (rec[i] != 0xFFu) {
      return false;
    }
  }

  return true;
}

static bool slot_valid(uint32_t slot)
{
  uint8_t rec[PARAM_STORE_RECORD_SIZE];
  uint32_t magic;

  slot_read(slot, rec);

  magic = (uint32_t)rec[0] | ((uint32_t)rec[1] << 8) |
          ((uint32_t)rec[2] << 16) | ((uint32_t)rec[3] << 24);
  if (magic != PARAM_STORE_MAGIC) {
    return false;
  }

  return crc32(rec, PARAM_STORE_CRC_LEN) ==
         ((uint32_t)rec[12] | ((uint32_t)rec[13] << 8) |
          ((uint32_t)rec[14] << 16) | ((uint32_t)rec[15] << 24));
}

/**
 * @brief 擦除存储页
 * @note 双 bank（bank2 第 63 页）优先，失败再按单 bank（bank1 第 127 页）；
 *       两者都指向 0x080FE000。
 */
static bool page_erase(void)
{
  FLASH_EraseInitTypeDef init;
  uint32_t page_error = 0u;
  HAL_StatusTypeDef st;
  uint32_t primask;

  memset(&init, 0, sizeof(init));
  init.TypeErase = FLASH_TYPEERASE_PAGES;
  init.NbPages = 1u;

  primask = __get_PRIMASK();
  __disable_irq();
  (void)HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  init.Banks = FLASH_BANK_2;
  init.Page = 63u;
  st = HAL_FLASHEx_Erase(&init, &page_error);

  if (st != HAL_OK) {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    init.Banks = FLASH_BANK_1;
    init.Page = 127u;
    st = HAL_FLASHEx_Erase(&init, &page_error);
  }

  (void)HAL_FLASH_Lock();
  __set_PRIMASK(primask);

  (void)HAL_ICACHE_Invalidate();

  return (st == HAL_OK);
}

bool param_store_init(void)
{
  uint32_t i;

  g_write_slot = PARAM_STORE_SLOTS;
  g_ready = false;

  /* 顺序扫描：找到第一个空槽；中途遇到既非空也非本模块记录的数据就放弃使用 */
  for (i = 0u; i < PARAM_STORE_SLOTS; i++) {
    if (slot_blank(i)) {
      g_write_slot = i;
      break;
    }
    if (!slot_valid(i)) {
      return false; /* 异物：可能是固件长到了这一页，绝不碰它 */
    }
  }

  g_ready = true;

  return true;
}

bool param_store_load(uint8_t payload[PARAM_STORE_PAYLOAD_SIZE])
{
  uint8_t rec[PARAM_STORE_RECORD_SIZE];
  uint32_t slot;

  if ((!g_ready) || (payload == NULL)) {
    return false;
  }

  slot = (g_write_slot == 0u) ? 0u : (g_write_slot - 1u);

  for (;;) {
    if (slot_valid(slot)) {
      slot_read(slot, rec);
      memcpy(payload, &rec[4], PARAM_STORE_PAYLOAD_SIZE);
      return true;
    }
    if (slot == 0u) {
      return false;
    }
    slot--;
  }
}

/** @brief 页里是否只有本模块的数据（空白或合法记录）；不是就别擦 */
static bool page_is_ours(void)
{
  uint32_t i;

  for (i = 0u; i < PARAM_STORE_SLOTS; i++) {
    if ((!slot_blank(i)) && (!slot_valid(i))) {
      return false;
    }
  }

  return true;
}

bool param_store_save(const uint8_t payload[PARAM_STORE_PAYLOAD_SIZE])
{
  uint8_t rec[PARAM_STORE_RECORD_SIZE];
  uint32_t words[4];
  uint32_t addr;
  uint32_t primask;
  HAL_StatusTypeDef st;
  uint32_t i;
  bool ok = true;

  if ((!g_ready) || (payload == NULL)) {
    return false;
  }

  /* 槽位用完或该槽被占用：擦页重来 */
  if ((g_write_slot >= PARAM_STORE_SLOTS) || (!slot_blank(g_write_slot))) {
    if (!page_is_ours()) {
      return false;
    }
    if (!page_erase()) {
      return false;
    }
    g_write_slot = 0u;
  }

  rec[0] = (uint8_t)(PARAM_STORE_MAGIC & 0xFFu);
  rec[1] = (uint8_t)((PARAM_STORE_MAGIC >> 8) & 0xFFu);
  rec[2] = (uint8_t)((PARAM_STORE_MAGIC >> 16) & 0xFFu);
  rec[3] = (uint8_t)((PARAM_STORE_MAGIC >> 24) & 0xFFu);
  memcpy(&rec[4], payload, PARAM_STORE_PAYLOAD_SIZE);

  {
    uint32_t crc = crc32(rec, PARAM_STORE_CRC_LEN);

    rec[12] = (uint8_t)(crc & 0xFFu);
    rec[13] = (uint8_t)((crc >> 8) & 0xFFu);
    rec[14] = (uint8_t)((crc >> 16) & 0xFFu);
    rec[15] = (uint8_t)((crc >> 24) & 0xFFu);
  }

  for (i = 0u; i < 4u; i++) {
    words[i] = (uint32_t)rec[i * 4u] | ((uint32_t)rec[i * 4u + 1u] << 8) |
               ((uint32_t)rec[i * 4u + 2u] << 16) |
               ((uint32_t)rec[i * 4u + 3u] << 24);
  }

  addr = PARAM_STORE_BASE + g_write_slot * PARAM_STORE_RECORD_SIZE;

  primask = __get_PRIMASK();
  __disable_irq();
  (void)HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, addr,
                         (uint32_t)(uintptr_t)words);
  (void)HAL_FLASH_Lock();
  __set_PRIMASK(primask);

  (void)HAL_ICACHE_Invalidate();

  if (st != HAL_OK) {
    return false;
  }

  /* 回读校验：编程被静默丢弃（写保护/电压不足）时不要假装成功 */
  for (i = 0u; i < PARAM_STORE_RECORD_SIZE; i++) {
    if (((const volatile uint8_t *)(uintptr_t)addr)[i] != rec[i]) {
      ok = false;
      break;
    }
  }

  if (ok) {
    g_write_slot++;
  }

  return ok;
}

bool param_store_clear(void)
{
  if (!g_ready) {
    return false;
  }

  if (!page_is_ours()) {
    return false;
  }
  if (!page_erase()) {
    return false;
  }

  g_write_slot = 0u;

  return true;
}
