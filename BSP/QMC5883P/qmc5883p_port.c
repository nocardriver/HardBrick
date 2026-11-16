/**
 * @file    qmc5883p_port.c
 * @author  YangKY
 * @brief   QMC5883P 平台移植层实现：I2C2 阻塞收发
 *
 * 这里是 BSP/QMC5883P 里唯一允许 include HAL 头文件的文件。所有板级细节
 * （用哪路 I2C、地址左移、超时时间）都写死在这里，驱动核心保持可移植。
 */

#include "qmc5883p_port.h"

#include "qmc5883p.h"

#include "i2c.h"
#include "main.h"

#include <stddef.h>

/** HAL 的地址参数是 8 位（7 位地址左移一位）：0x2C << 1 = 0x58 写 / 0x59 读 */
#define QMC5883P_PORT_ADDR8 ((uint16_t)(QMC5883P_I2C_ADDR_7BIT << 1))

/**
 * 单次事务超时。400 kHz 下读写 6 字节不到 200 µs，20 ms 是给总线被拉死、
 * 从机 0 V 供电等异常留的余量：宁可返回错误，也不让采集线程卡住。
 */
#define QMC5883P_PORT_TIMEOUT_MS 20u

/** 地址探测的重试次数与超时 */
#define QMC5883P_PORT_PROBE_TRIALS 3u

/** 最近一次 HAL 返回码，J-Link 可直接看，用于区分 NACK / 忙 / 超时 */
static volatile uint32_t g_last_error;

bool qmc5883p_port_i2c_write(uint8_t reg, const uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef st;

  if ((data == NULL) || (len == 0u)) {
    return false;
  }

  /* HAL 的发送接口没有 const，这里只做类型转换，不改缓冲区 */
  st = HAL_I2C_Mem_Write(&hi2c2, QMC5883P_PORT_ADDR8, reg,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)(uintptr_t)data, len,
                         QMC5883P_PORT_TIMEOUT_MS);
  g_last_error = (uint32_t)st;

  return (st == HAL_OK);
}

bool qmc5883p_port_i2c_read(uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef st;

  if ((data == NULL) || (len == 0u)) {
    return false;
  }

  /* 地址自增：一次 Mem_Read 就是 datasheet §8.2.4 的"重复起始 + 连读"时序 */
  st = HAL_I2C_Mem_Read(&hi2c2, QMC5883P_PORT_ADDR8, reg,
                        I2C_MEMADD_SIZE_8BIT, data, len,
                        QMC5883P_PORT_TIMEOUT_MS);
  g_last_error = (uint32_t)st;

  return (st == HAL_OK);
}

bool qmc5883p_port_is_ready(void)
{
  HAL_StatusTypeDef st;

  st = HAL_I2C_IsDeviceReady(&hi2c2, QMC5883P_PORT_ADDR8,
                             QMC5883P_PORT_PROBE_TRIALS,
                             QMC5883P_PORT_TIMEOUT_MS);
  g_last_error = (uint32_t)st;

  return (st == HAL_OK);
}

uint32_t qmc5883p_port_last_error(void)
{
  return g_last_error;
}

uint32_t qmc5883p_port_tick_ms(void)
{
  return HAL_GetTick();
}

void qmc5883p_port_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}
