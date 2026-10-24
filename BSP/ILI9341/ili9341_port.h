/**
 * @file    ili9341_port.h
 * @author  YangKY
 * @brief   ILI9341 平台移植层：SPI1 数据通道 + CS/DC/RST 引脚 + 毫秒延时
 *
 * 约定：接口只用可移植类型，HAL 头文件只允许出现在 ili9341_port.c。
 *       调用 ili9341_port_* 之前，main() 必须已经执行过 MX_GPIO_Init() 与
 *       MX_SPI1_Init()（引脚与 SPI1 由 CubeMX 生成）。
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
 * @brief 通过 SPI1 阻塞发送一段字节
 * @param data 数据指针
 * @param len  字节数（HAL 的单次上限为 65535）
 * @retval true 发送完成；false 超时或总线错误
 * @note  超时按"至少 1 MB/s"估算，40 MHz 下余量约 40 倍
 */
bool ili9341_port_write(const uint8_t *data, uint16_t len);

#endif /* ILI9341_PORT_H */
