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
  *   2. 在 UART 初始化完成之后调用 uart_dma_start(&ctx, &huart)。
  *   3. 消费线程循环调用 uart_dma_read_line() 提取完整行（以 '\n' 结尾）。
  *
  * 说明：
  *   - 单生产者（ISR）/ 单消费者（线程）模型，无需加锁。
  *   - 同一时刻仅支持一个 UART 实例（回调使用内部全局上下文）。
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

/* 应用层环形缓冲大小（字节）。1Hz GNSS 全量输出突发约 500~800 字节，取 1024 留足余量 */
#define UART_DMA_RING_SIZE     1024u
/* 单次 DMA 接收缓冲大小，需大于最长的一条 NMEA 语句（一般 < 100 字节） */
#define UART_DMA_CHUNK_SIZE    256u

typedef struct
{
  UART_HandleTypeDef *huart;                      /* 绑定的 UART 句柄 */
  uint8_t  chunk[UART_DMA_CHUNK_SIZE];            /* DMA 直接接收缓冲 */
  uint8_t  ring[UART_DMA_RING_SIZE];              /* 应用层环形缓冲 */
  uint16_t ring_head;                             /* 读位置（消费端） */
  uint16_t ring_tail;                             /* 写位置（ISR 端） */
  bool     started;
} uart_dma_t;

/* 初始化上下文（不清缓冲内容，仅复位索引） */
void uart_dma_init(uart_dma_t *ctx, UART_HandleTypeDef *huart);

/* 启动 DMA + IDLE 接收，并挂接 HAL 回调 */
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
