/**
 * @file    qmc5883p_port.h
 * @author  YangKY
 * @brief   QMC5883P 平台移植层：I2C2（PB13/PB14）+ 毫秒计时
 *
 * 约定：接口只用可移植类型；HAL 头文件只允许出现在 qmc5883p_port.c。
 *       调用 qmc5883p_init() 之前，main() 必须已经执行过 MX_I2C2_Init()。
 *
 * 硬件绑定（本板）：
 *   I2C2_SCL = PB13 -> 传感器 SCK
 *   I2C2_SDA = PB14 -> 传感器 SDA
 *   7 位地址 0x2C，Fast mode 400 kHz，外部 2.7 kΩ 上拉到 3.3 V
 */

#ifndef QMC5883P_PORT_H
#define QMC5883P_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 向寄存器写 1~N 个字节（HAL_I2C_Mem_Write，阻塞）
 * @param reg  寄存器地址
 * @param data 数据源
 * @param len  字节数（>=1）
 * @retval true 器件应答且写完成
 * @retval false 参数非法 / NACK / 超时
 */
bool qmc5883p_port_i2c_write(uint8_t reg, const uint8_t *data, uint16_t len);

/**
 * @brief 从寄存器连读 1~N 个字节（HAL_I2C_Mem_Read，阻塞）
 * @note 器件内部地址自增，因此读 0x01 起 6 字节就是一次完整的 X/Y/Z 采样。
 */
bool qmc5883p_port_i2c_read(uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief 地址探测：地址 0x2C 是否有应答
 * @note 用来把"没接线/没上拉"和"器件存在但 ID 不对"区分开，上电自检里用。
 */
bool qmc5883p_port_is_ready(void);

/** @brief 最近一次 I2C 事务的 HAL 返回码（HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT） */
uint32_t qmc5883p_port_last_error(void);

/** @brief 毫秒计时（HAL_GetTick） */
uint32_t qmc5883p_port_tick_ms(void);

/** @brief 毫秒延时（HAL_Delay；只用于器件时序，不用于周期调度） */
void qmc5883p_port_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883P_PORT_H */
