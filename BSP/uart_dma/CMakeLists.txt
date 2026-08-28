cmake_minimum_required(VERSION 3.22)

#
# uart_dma: 通用 UART 接收基础设施。
# 采用 DMA + IDLE 行检测 + 应用层环形缓冲：
#   - ISR 收到一段完整数据（IDLE）后拷入环形缓冲并重新启动 DMA
#   - 消费端（线程）调用 uart_dma_read_line() 提取完整的行
# 依赖 HAL（经 stm32cubemx 接口库提供 include 路径和宏定义）。
#

set(UART_DMA_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(UART_DMA_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/uart_dma.c
)

# Create uart_dma static library
add_library(uart_dma STATIC)
target_sources(uart_dma PRIVATE ${UART_DMA_Src})
target_include_directories(uart_dma PUBLIC ${UART_DMA_Inc_Dirs})
target_link_libraries(uart_dma PUBLIC stm32cubemx)
