/**
  ******************************************************************************
  * @file    uart_dma.c
  * @brief   Generic UART reception infrastructure: DMA + IDLE + ring buffer.
  ******************************************************************************
  */
#include "uart_dma.h"

/* 当前绑定的上下文（回调使用）。单 UART 实例足够 */
static uart_dma_t *g_ctx = NULL;

/* -------------------------------------------------------------------------- */
/* 环形缓冲内部操作（单生产者 ISR / 单消费者线程）                             */
/* -------------------------------------------------------------------------- */

static uint16_t ring_count(const uart_dma_t *ctx)
{
  if (ctx->ring_head <= ctx->ring_tail)
  {
    return (uint16_t)(ctx->ring_tail - ctx->ring_head);
  }
  return (uint16_t)(UART_DMA_RING_SIZE - ctx->ring_head + ctx->ring_tail);
}

static uint16_t ring_space(const uart_dma_t *ctx)
{
  return (uint16_t)(UART_DMA_RING_SIZE - 1u - ring_count(ctx));
}

static void ring_write(uart_dma_t *ctx, const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    ctx->ring[ctx->ring_tail] = data[i];
    ctx->ring_tail++;
    if (ctx->ring_tail >= UART_DMA_RING_SIZE)
    {
      ctx->ring_tail = 0;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* HAL 回调（覆盖 __weak 版本）                                               */
/* -------------------------------------------------------------------------- */

/* IDLE 事件：收到一段完整数据 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if ((g_ctx != NULL) && (g_ctx->huart == huart))
  {
    uint16_t space = ring_space(g_ctx);
    uint16_t n = (Size <= space) ? Size : space;   /* 满则丢弃超出部分 */
    ring_write(g_ctx, huart->pRxBuffPtr, n);

    /* 重新启动接收，等待下一条数据 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, g_ctx->chunk, UART_DMA_CHUNK_SIZE);
  }
}

/* DMA 缓冲写满（未出现 IDLE），防止数据丢失 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((g_ctx != NULL) && (g_ctx->huart == huart))
  {
    uint16_t space = ring_space(g_ctx);
    uint16_t n = (UART_DMA_CHUNK_SIZE <= space) ? UART_DMA_CHUNK_SIZE : space;
    ring_write(g_ctx, huart->pRxBuffPtr, n);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, g_ctx->chunk, UART_DMA_CHUNK_SIZE);
  }
}

/* UART 错误（如溢出 ORE），复位接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((g_ctx != NULL) && (g_ctx->huart == huart))
  {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, g_ctx->chunk, UART_DMA_CHUNK_SIZE);
  }
}

/* -------------------------------------------------------------------------- */
/* 对外 API                                                                   */
/* -------------------------------------------------------------------------- */

void uart_dma_init(uart_dma_t *ctx, UART_HandleTypeDef *huart)
{
  ctx->huart      = huart;
  ctx->ring_head  = 0;
  ctx->ring_tail  = 0;
  ctx->started    = false;
}

void uart_dma_start(uart_dma_t *ctx)
{
  if (ctx->started)
  {
    return;
  }
  g_ctx = ctx;
  ctx->started = true;
  HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart, ctx->chunk, UART_DMA_CHUNK_SIZE);
}

uint16_t uart_dma_available(const uart_dma_t *ctx)
{
  return ring_count(ctx);
}

bool uart_dma_read_line(uart_dma_t *ctx, char *line, uint16_t line_max)
{
  uint16_t n = ring_count(ctx);
  if (n == 0u)
  {
    return false;
  }

  /* 1. 在已接收数据中查找 '\n'（行结束符） */
  uint16_t idx  = ctx->ring_head;
  uint16_t len  = 0u;                  /* 到 '\n' 为止的总长度（含 '\n'） */
  bool     found = false;

  while (len < n)
  {
    uint8_t c = ctx->ring[idx];
    idx = (uint16_t)((idx + 1u >= UART_DMA_RING_SIZE) ? 0u : idx + 1u);
    len++;
    if (c == '\n')
    {
      found = true;
      break;
    }
  }
  if (!found)
  {
    return false;                      /* 行尚未完整 */
  }

  /* 2. 行文本长度 = 去掉末尾 '\n'；若 '\n' 前是 '\r' 也去掉 */
  uint16_t text_len = (uint16_t)(len - 1u);
  if (text_len > 0u)
  {
    uint16_t last = (uint16_t)((ctx->ring_head + text_len - 1u) % UART_DMA_RING_SIZE);
    if (ctx->ring[last] == '\r')
    {
      text_len--;
    }
  }

  /* 3. 拷贝行内容（超长则截断） */
  uint16_t copy = (text_len < line_max) ? text_len : (uint16_t)(line_max - 1u);
  for (uint16_t i = 0; i < copy; i++)
  {
    line[i] = (char)ctx->ring[ctx->ring_head];
    ctx->ring_head = (uint16_t)((ctx->ring_head + 1u >= UART_DMA_RING_SIZE) ? 0u : ctx->ring_head + 1u);
  }
  /* 4. 跳过剩余部分（超长内容 + 结束符） */
  for (uint16_t i = copy; i < len; i++)
  {
    ctx->ring_head = (uint16_t)((ctx->ring_head + 1u >= UART_DMA_RING_SIZE) ? 0u : ctx->ring_head + 1u);
  }

  line[copy] = '\0';
  return true;
}
