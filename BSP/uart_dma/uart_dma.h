/**
  ******************************************************************************
  * @file    uart_dma.h
  * @brief   Generic UART reception infrastructure: DMA + IDLE + ring buffer.
  *
  * 数据流：
  *   UART 外设 --DMA--> chunk 缓冲 --IDLE/满中断--> 环形缓冲 --read_line--> 消费端
  *
  * 用法：
  *   1. 在 CubeMX 中为该 UART 配置 GPDMA RX（Normal 模式，目的地址递增），
  *      MSP 里通过 __HAL_LINKDMA(huart, hdmarx, ...) 绑定。
  *   2. 调用者提供静态缓冲（ring / chunk）并指定大小，调用 uart_dma_init()。
  *   3. UART 初始化完成后调用 uart_dma_start() 开始接收。
  *   4. 消费线程循环调用 uart_dma_read_line() 提取完整行（以 '\n' 结尾）。
  *
  * 说明：
  *   - 单生产者（ISR）/ 单消费者（线程）模型，无需加锁。
  *   - 支持多实例并行：每个实例独立提供 ring/chunk 缓冲与大小，
  *     ISR 回调按 huart 查表定位实例。实例上限 UART_DMA_MAX_INSTANCES（默认 4）。
  ******************************************************************************
  */
#ifndef UART_DMA_H
#define UART_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32u5xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 同时支持的 UART 实例上限（ISR 回调按 huart 查表定位实例） */
#ifndef UART_DMA_MAX_INSTANCES
#define UART_DMA_MAX_INSTANCES   4u
#endif

typedef struct uart_dma uart_dma_t;

struct uart_dma
{
  UART_HandleTypeDef *huart;      /* 绑定的 UART 句柄 */
  uint8_t *ring;                  /* 环形缓冲（调用者提供） */
  uint16_t ring_size;             /* 环形缓冲大小（字节） */
  uint8_t *chunk;                 /* DMA 直接接收缓冲（调用者提供） */
  uint16_t chunk_size;            /* DMA 接收缓冲大小（需大于最长的一条消息） */
  uint16_t ring_head;             /* 读位置（消费端） */
  uint16_t ring_tail;             /* 写位置（ISR 端） */
  bool     started;
};

/**
  * @brief  初始化并注册一个 UART 接收实例。
  * @param ctx        实例
  * @param huart      UART 句柄（需已通过 MSP 绑定 hdmarx）
  * @param ring       环形缓冲（调用者提供，可静态数组）
  * @param ring_size  环形缓冲大小（字节）
  * @param chunk      DMA 直接接收缓冲（调用者提供，可静态数组）
  * @param chunk_size DMA 接收缓冲大小（需大于最长的一条消息）
  */
void uart_dma_init(uart_dma_t *ctx, UART_HandleTypeDef *huart,
                   uint8_t *ring, uint16_t ring_size,
                   uint8_t *chunk, uint16_t chunk_size);

/* 启动 DMA + IDLE 接收（注册进回调查表并开始接收） */
void uart_dma_start(uart_dma_t *ctx);

/* 环形缓冲内已接收但尚未消费的字节数 */
uint16_t uart_dma_available(const uart_dma_t *ctx);

/**
  * @brief  非阻塞读取一行。
  * @param  line     输出缓冲（必须以 '\0' 结尾）
  * @param  line_max 输出缓冲大小
  * @retval true : 取到完整的一行（不含 '\r' 和 '\n'）；false : 数据不足 / 行未完整
  * @note   若一行超过 line_max-1 字节，则截断并丢弃超长部分。
  */
bool uart_dma_read_line(uart_dma_t *ctx, char *line, uint16_t line_max);

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_H */
