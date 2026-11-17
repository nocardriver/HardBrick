/**
 * @file    mx01p_port.h
 * @author  YangKY
 * @brief   MX-01P 平台移植层：USART1 + RES/BRTS 板级绑定
 *
 * 约定：接口只用可移植类型；HAL 头文件只允许出现在 mx01p_port.c。
 *       调用 mx01p_init() 之前，main() 必须已经执行过 MX_GPIO_Init() 与
 *       MX_USART1_UART_Init()（引脚与串口由 CubeMX 生成）。
 *
 * 硬件绑定（本板）：
 *   USART1_TX = PA9  -> 模块 RXD (Pin13)
 *   USART1_RX = PA10 <- 模块 TXD (Pin14)
 *   BT_RES    = PC3  输出，低有效，复位模块
 *   BT_BRTS   = PA1  输出，高 = 睡眠，低 = 工作
 *
 * 未连接的模块引脚（本板没引到 MCU，驱动只能按手册的休眠值假定）：
 *   CDS  (AT 命令使能) 悬空 = 高 → AT 命令始终有效、非 AT 数据透传；
 *   LINK (连接指示)    悬空 → 连接状态只能靠 +CONNECTED/+DISCONN 事件跟踪；
 *   FC   (流控)        悬空 → 模块无法通知 MCU "现在别发"，发送靠上层自己限速。
 */

#ifndef MX01P_PORT_H
#define MX01P_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "mx01p.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USART1 实际波特率（CubeMX 里固定 115200）
 * @note 驱动用它与 `AT+UART?` 的查询结果比对，避免把模块配成本板收不到的值。
 */
#define MX01P_PORT_UART_BAUD 115200u

/** 单次 UART 发送的字节上限，超过 HAL/驱动都不接受 */
#define MX01P_PORT_TX_MAX 512u

/**
 * @brief 绑定实例并启动 USART1 的 ReceiveToIdle 中断接收
 * @param inst 驱动实例
 * @note 必须在 MX_USART1_UART_Init() 之后调用；重复调用是幂等的（会先中止再重挂）。
 */
void mx01p_port_attach(mx01p_t *inst);

/**
 * @brief 控制 RES 引脚
 * @param low true = 拉低复位，false = 释放（拉高，模块运行）
 */
void mx01p_port_reset(bool low);

/**
 * @brief 控制 BRTS 引脚
 * @param sleep true = 拉高（睡眠），false = 拉低（工作）
 */
void mx01p_port_set_sleep(bool sleep);

/**
 * @brief 通过 USART1 发送一段字节（阻塞，HAL_UART_Transmit）
 * @param timeout_ms HAL 超时
 * @retval true 发送完成
 * @retval false 参数非法 / HAL 错误 / 超时
 */
bool mx01p_port_uart_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/** @brief 毫秒计时（HAL_GetTick） */
uint32_t mx01p_port_tick_ms(void);

/** @brief 毫秒延时（HAL_Delay；只用于模块时序，不用于周期调度） */
void mx01p_port_delay_ms(uint32_t ms);

/**
 * @brief 补挂中断接收
 * @note 若某次 RX 事件恰逢阻塞发送（HAL 句柄锁被占用），rearm 会失败并置位内部
 *       标志；本函数在句柄空闲时重试。驱动会在延时、发送返回、读缓冲时调用它。
 */
void mx01p_port_poll(void);

/**
 * @brief 进/出临界区，保护「控制行收集器 ↔ 读缓冲」之间的短暂竞态
 * @return 进入前的 PRIMASK，原样传给 mx01p_port_irq_unlock()
 * @note 只包住几十条指令，不要在临界区里做发送、等待或打印。
 */
uint32_t mx01p_port_irq_lock(void);
void mx01p_port_irq_unlock(uint32_t key);

/*---------------------------------------------------------------------------*/
/* 中断入口：由 main.c 的 HAL 回调分发，ISR 安全                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief UART 接收事件（IDLE / 缓冲满）入口
 * @param size 本次 burst 的有效字节数
 */
void mx01p_port_rx_event(uint16_t size);

/** @brief UART 错误（ORE 等）入口：复位接收并 rearm，不 panic */
void mx01p_port_uart_error(void);

#ifdef __cplusplus
}
#endif

#endif /* MX01P_PORT_H */
