/**
 * @file    ili9341_port.h
 * @author  YangKY
 * @brief   ILI9341 平台移植层：SPI1 数据通道（轮询 / DMA）+ CS/DC/RST 引脚 + 毫秒延时
 *
 * 约定：接口只用可移植类型，HAL / ThreadX 头文件只允许出现在 ili9341_port.c。
 *       调用 ili9341_port_* 之前，main() 必须已经执行过 MX_GPIO_Init() 与
 *       MX_SPI1_Init()（引脚与 SPI1 由 CubeMX 生成）。
 *
 * 发送策略：
 *   - 命令、参数、初始化序列（< 64 字节）走 HAL_SPI_Transmit 轮询；
 *   - 像素数据（>= 64 字节）在 ili9341_port_dma_attach() 成功后走 SPI1_TX DMA，
 *     线程在信号量上睡眠等待 EOT，CPU 不再忙等。
 *
 *   ili9341_init() 在 tx_kernel_enter() 之前执行，此时内核对象还不存在，
 *   所以 attach 之前一律是轮询 —— 这也是 DMA 只能在 attach 之后启用的原因。
 */

#ifndef ILI9341_PORT_H
#define ILI9341_PORT_H

#include <stdint.h>
#include <stdbool.h>

/** @brief 把 CS/DC/RST 置为总线空闲状态（CS 高、DC 低、RST 高） */
void ili9341_port_init(void);

/**
 * @brief 控制片选
 * @param low true = 拉低（选中），false = 拉高（释放）
 */
void ili9341_port_cs(bool low);

/**
 * @brief 控制数据/命令选择脚
 * @param data_mode true = 数据（DC 高），false = 命令（DC 低）
 */
void ili9341_port_dc(bool data_mode);

/**
 * @brief 控制复位脚
 * @param high true = 释放复位（RST 高），false = 保持复位（RST 低）
 */
void ili9341_port_rst(bool high);

/** @brief 毫秒级延时（HAL_Delay；仅用于上电/复位时序） */
void ili9341_port_delay_ms(uint32_t ms);

/**
 * @brief 通过 SPI1 发送一段字节（命令/小包轮询，大块像素在 attach 后走 DMA）
 * @param data 数据指针
 * @param len  字节数（HAL 的单次上限为 65535）
 * @retval true 发送完成；false 超时或总线错误
 * @note  超时按"至少 1 MB/s"估算并额外留 100 ms 调度余量，40 MHz 下余量充足；
 *        DMA 路径会等待 EOT 完成，返回时 CS 才允许拉高。
 */
bool ili9341_port_write(const uint8_t *data, uint16_t len);

/**
 * @brief 启用 SPI1 TX DMA 发送（创建 EOT 完成信号量）
 * @retval true 已启用（或此前已启用）；false 未配置 SPI1_TX DMA / 信号量创建失败，
 *         此时发送永久保持轮询，行为与改造前一致
 * @note  必须在 tx_application_define() 内、任何显示线程使用驱动之前调用：
 *        内核未启动时无法创建信号量，也无法等待。
 *        运行时若连续靠 EOT 复查救回（收尾中断不可用），会自动永久回退轮询。
 */
bool ili9341_port_dma_attach(void);

/** @brief 累计 DMA 异常次数（启动失败 / 超时 / HAL 报错，含已自动恢复的），供 RTT 统计 */
uint32_t ili9341_port_dma_error_count(void);

/** @brief 累计自动重试次数（仅对"尚未发出数据"的失败重试） */
uint32_t ili9341_port_dma_retry_count(void);

#endif /* ILI9341_PORT_H */
