/**
 * @file    ili9341_port.c
 * @author  YangKY
 * @brief   ILI9341 平台移植层实现（唯一允许接触 HAL / ThreadX 的文件）
 *
 * 发送分两条路：
 *   - 轮询：命令、参数、初始化序列，长度小，DMA 的两次中断开销不划算；
 *     内核启动前也只能走这条（ili9341_init() 在 tx_kernel_enter() 之前）。
 *   - DMA：内核启动并 attach 之后的大块像素，线程在信号量上睡眠，等 SPI 的
 *     EOT 中断收尾后再返回。
 *
 * DMA 完成链路（HAL 的实现决定，不能省）：
 *   HAL_SPI_Transmit_DMA → GPDMA 传输完成中断 → 只打开 SPI 的 EOT 中断
 *   → SPI EOT 中断里 SPI_CloseTransfer()（SPE=0、清 TXDMAEN）
 *   → HAL_SPI_TxCpltCallback() → 这里投递信号量。
 * 因此 SPI1 全局中断必须使能，否则 DMA 永远"不完成"。
 *
 * 另一个硬性前提：SPI 在 SPE=0 期间必须继续驱动 SCK/MOSI（SPI_CFG2.AFCNTR=1）。
 * HAL 每笔传输结束都会关 SPI；AFCNTR=0 时引脚悬空，浮空 SCK 上的毛刺会被面板
 * 当成时钟沿，像素位相位逐笔漂移（纯色填充显示成彩色横条）。见 Core/Src/spi.c。
 */

#include "ili9341_port.h"

#include "stm32u5xx_hal.h"
#include "main.h"
#include "spi.h"
#include "tx_api.h"

/** 单次传输基础超时（ms）。40 MHz 下 4 KB 约 0.8 ms；基数给大是为了容忍调度抖动 */
#define ILI9341_PORT_TIMEOUT_BASE_MS  100u

/** 小于该字节数的写走轮询：命令/参数只有 1~16 字节 */
#define ILI9341_PORT_DMA_MIN_BYTES    64u

/** DMA 启动失败时最多自动重试几次（每次间隔 1 tick = 10 ms） */
#define ILI9341_PORT_DMA_RETRY_MAX    2u

/** 连续多少次"靠 EOT 复查救回"就认为收尾中断不可用，永久回退轮询 */
#define ILI9341_PORT_DMA_RECOVER_MAX  3u

/** 单次 DMA 传输的结果 */
typedef enum {
  ILI9341_DMA_OK = 0,   /**< 数据已全部发出 */
  ILI9341_DMA_RETRY,    /**< 还没开始发数据，重发安全 */
  ILI9341_DMA_FAIL,     /**< 可能已发出一部分，重发会让 GRAM 错位，不能再试 */
} ili9341_dma_result_t;

static TX_SEMAPHORE  g_dma_sem;      /**< EOT 完成信号：SPI 中断里 put，发送线程 get */
static volatile bool g_dma_error;    /**< 本次 DMA 传输是否被 HAL 报错（UDR/OVR 等） */
static bool          g_dma_ready;    /**< 信号量已创建、可走 DMA */
static bool          g_dma_fallback; /**< 收尾中断不可用，后续永久回退轮询 */
static uint32_t      g_dma_recover;  /**< 连续"靠 EOT 复查救回"的次数 */
static uint32_t      g_dma_errors;   /**< 累计异常次数（含已自动恢复的） */
static uint32_t      g_dma_retries;  /**< 累计自动重试次数 */

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

/** @brief 毫秒 → ThreadX tick（100 Hz，向上取整） */
static ULONG dma_timeout_ticks(uint16_t len)
{
  uint32_t ms = ILI9341_PORT_TIMEOUT_BASE_MS + ((uint32_t)len / 1000u);

  return (ULONG)(((ms * TX_TIMER_TICKS_PER_SECOND) + 999u) / 1000u);
}

/**
 * @brief 清掉可能残留的完成令牌
 * @note  超时与 EOT 中断存在极窄的竞争窗口，中止后 ISR 仍可能补发一个令牌；
 *        不清掉的话下一笔传输会立刻"完成"，导致 CS 提前拉高、数据被截断。
 */
static void dma_drain(void)
{
  while (tx_semaphore_get(&g_dma_sem, TX_NO_WAIT) == TX_SUCCESS) {
  }
}

/** @brief 轮询发送（命令 / 小包 / 内核启动前） */
static bool write_polling(const uint8_t *data, uint16_t len)
{
  uint32_t timeout = ILI9341_PORT_TIMEOUT_BASE_MS + ((uint32_t)len / 1000u);

  return HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, timeout) == HAL_OK;
}

/** @brief 尝试一次 DMA 发送，区分"可重试"和"不能重试" */
static ili9341_dma_result_t write_dma_once(const uint8_t *data, uint16_t len)
{
  dma_drain();
  g_dma_error = false;

  if (HAL_SPI_Transmit_DMA(&hspi1, data, len) != HAL_OK) {
    /* HAL_BUSY / HAL_ERROR：这笔数据一个字节都没发出去，重试是安全的 */
    return ILI9341_DMA_RETRY;
  }

  if (tx_semaphore_get(&g_dma_sem, dma_timeout_ticks(len)) != TX_SUCCESS) {
    /* EOT 没来。先看硬件是不是其实已经发完（只是收尾中断没到，例如漏开 SPI1 中断）：
       这种情况数据是完整的，按成功处理；否则中止总线，避免后续传输永远卡住 */
    bool eot = __HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_EOT);

    (void)HAL_SPI_Abort(&hspi1);
    dma_drain();

    if (eot) {
      g_dma_errors++;   /* 已恢复，但属于异常：正常情况下不该走到这里 */
      /* 连续几次都这样，说明收尾中断没按预期工作（例如 SPI1 中断没使能）。
         继续走 DMA 只会每笔都等满超时，直接回退轮询：宁可慢，也不要卡 */
      if (++g_dma_recover >= ILI9341_PORT_DMA_RECOVER_MAX) {
        g_dma_fallback = true;
      }
      return ILI9341_DMA_OK;
    }
    return ILI9341_DMA_FAIL;
  }

  if (g_dma_error) {
    return ILI9341_DMA_FAIL;
  }

  g_dma_recover = 0u;
  return ILI9341_DMA_OK;
}

/**
 * @brief DMA 发送，启动失败时自动重试
 * @note  已经发出部分数据的失败（超时/下溢）不重发：像素流是连续的，
 *        重发会把后续像素整体后移，画面比"停画"更糟。这类错误只统计并上报。
 */
static bool write_dma(const uint8_t *data, uint16_t len)
{
  uint32_t attempt = 0u;

  for (;;) {
    ili9341_dma_result_t result = write_dma_once(data, len);

    if (result == ILI9341_DMA_OK) {
      return true;
    }

    /* 已发出部分数据的失败不能重发，直接上报；启动失败才值得再试 */
    if ((result == ILI9341_DMA_FAIL) || (attempt >= ILI9341_PORT_DMA_RETRY_MAX)) {
      g_dma_errors++;
      return false;
    }

    attempt++;
    g_dma_retries++;
    (void)tx_thread_sleep(1u);   /* 1 tick = 10 ms，等 HAL 状态机恢复 */
  }
}

/* ------------------------------------------------------------------------- */
/* 引脚与延时                                                                  */
/* ------------------------------------------------------------------------- */

void ili9341_port_init(void)
{
  /* 总线空闲：CS 高、DC 低、RST 释放 */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIS_DC_GPIO_Port, DIS_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIS_RST_GPIO_Port, DIS_RST_Pin, GPIO_PIN_SET);
}

void ili9341_port_cs(bool low)
{
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,
                    low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void ili9341_port_dc(bool data_mode)
{
  HAL_GPIO_WritePin(DIS_DC_GPIO_Port, DIS_DC_Pin,
                    data_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ili9341_port_rst(bool high)
{
  HAL_GPIO_WritePin(DIS_RST_GPIO_Port, DIS_RST_Pin,
                    high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ili9341_port_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

/* ------------------------------------------------------------------------- */
/* 发送                                                                        */
/* ------------------------------------------------------------------------- */

bool ili9341_port_write(const uint8_t *data, uint16_t len)
{
  if (!data || (len == 0u)) {
    return false;
  }

  if (g_dma_ready && !g_dma_fallback && (len >= ILI9341_PORT_DMA_MIN_BYTES)) {
    return write_dma(data, len);
  }

  return write_polling(data, len);
}

bool ili9341_port_dma_attach(void)
{
  if (g_dma_ready) {
    return true;
  }

  /* CubeMX 没配 SPI1_TX DMA 时 hdmatx 为空，HAL 会解引用空指针，必须提前拦下 */
  if (hspi1.hdmatx == NULL) {
    return false;
  }

  if (tx_semaphore_create(&g_dma_sem, "lcd dma", 0u) != TX_SUCCESS) {
    return false;
  }

  g_dma_ready = true;
  return true;
}

uint32_t ili9341_port_dma_error_count(void)
{
  return g_dma_errors;
}

uint32_t ili9341_port_dma_retry_count(void)
{
  return g_dma_retries;
}

/* ------------------------------------------------------------------------- */
/* HAL 回调（EOT 收尾后由 SPI1 中断调用）                                      */
/* ------------------------------------------------------------------------- */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi->Instance != SPI1) || !g_dma_ready) {
    return;
  }
  (void)tx_semaphore_put(&g_dma_sem);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi->Instance != SPI1) || !g_dma_ready) {
    return;
  }
  g_dma_error = true;
  (void)tx_semaphore_put(&g_dma_sem);
}
