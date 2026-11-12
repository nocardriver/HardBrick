/**
 * @file    e22_port.h
 * @author  YangKY
 * @brief   E22-400T22S 平台移植层：LPUART1 + M0/M1/NRST/AUX 板级绑定
 *
 * 约定：接口只用可移植类型；HAL 头文件只允许出现在 e22_port.c。
 *       调用 e22_init() 之前，main() 必须已经执行过 MX_GPIO_Init() 与
 *       MX_LPUART1_UART_Init()（引脚与串口由 CubeMX 生成）。
 *
 * 硬件绑定（本板）：
 *   LPUART1_TX = PC1  -> 模块 RXD
 *   LPUART1_RX = PC0  <- 模块 TXD
 *   LORA_M0    = PC6  输出
 *   LORA_M1    = PB15 输出
 *   LORA_NRST  = PB2  输出，低有效
 *   LORA_AUX   = PB10 输入，EXTI 双边沿
 */

#ifndef E22_PORT_H
#define E22_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "e22.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 绑定实例并启用板级资源
 * @param inst 驱动实例
 * @note 把 AUX 重配为双边沿（CubeMX 里 PB10 原为下降沿，HardBrick.ioc 已同步
 *       改为 RISING_FALLING；重新生成代码后本函数的重配是幂等的），清掉挂起
 *       标志，然后启动 LPUART1 的 ReceiveToIdle 中断接收。
 */
void e22_port_attach(e22_t *inst);

/** @brief 读 AUX 电平：true = 高（空闲） */
bool e22_port_aux_level(void);

/** @brief 驱动 M1/M0 电平（内部按模式真值表调用） */
void e22_port_set_mode_pins(bool m1, bool m0);

/** @brief 控制 NRST：true = 拉低复位，false = 释放 */
void e22_port_reset(bool low);

/**
 * @brief 通过 LPUART1 发送一段字节（阻塞，HAL_UART_Transmit）
 * @param timeout_ms HAL 超时
 * @retval true 发送完成
 * @retval false 参数非法 / HAL 错误 / 超时
 */
bool e22_port_uart_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/** @brief 毫秒计时（HAL_GetTick） */
uint32_t e22_port_tick_ms(void);

/** @brief 毫秒延时（HAL_Delay；只用于模块时序，不用于周期调度） */
void e22_port_delay_ms(uint32_t ms);

/**
 * @brief 补挂接收
 * @note 若某次 RX 事件恰逢阻塞发送（HAL 句柄锁被占用），rearm 会失败并置位
 *       内部标志；本函数在句柄空闲时重试。驱动在延时、发送返回和 e22_read()
 *       里会调用它，App 无需关心。
 */
void e22_port_poll(void);

/*---------------------------------------------------------------------------*/
/* 中断入口：由 main.c 的 HAL 回调分发，ISR 安全                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief UART 接收事件（IDLE / 缓冲满）入口
 * @param size 本次 burst 的有效字节数
 * @note 从 LPUART1 暂存缓冲拷入驱动环形缓冲并立即 rearm。
 */
void e22_port_rx_event(uint16_t size);

/** @brief UART 错误（ORE 等）入口：复位接收并 rearm，不 panic */
void e22_port_uart_error(void);

/** @brief AUX EXTI 入口：读引脚实际电平后上报给驱动 */
void e22_port_aux_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* E22_PORT_H */
