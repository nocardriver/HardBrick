/**
 * @file    ili9341_port.c
 * @author  YangKY
 * @brief   ILI9341 平台移植层实现（唯一允许接触 HAL 的文件）
 */

#include "ili9341_port.h"

#include "stm32u5xx_hal.h"
#include "main.h"
#include "spi.h"

/** 单次传输基础超时（ms）；40 MHz 下 4 KB 只需约 0.8 ms */
#define ILI9341_PORT_TIMEOUT_BASE_MS  10u

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

bool ili9341_port_write(const uint8_t *data, uint16_t len)
{
  uint32_t timeout;

  if (!data || len == 0u) {
    return false;
  }

  /* 约 1 MB/s 的保守下限：40 MHz 实际速率下余量充足，异常时也不会永久挂死 */
  timeout = ILI9341_PORT_TIMEOUT_BASE_MS + ((uint32_t)len / 1000u);

  return HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, timeout) == HAL_OK;
}
