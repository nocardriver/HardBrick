/**
 * @file    mx01p_port.c
 * @author  YangKY
 * @brief   MX-01P 平台移植层实现：USART1 中断收发 + RES/BRTS 控制脚
 *
 * 这里是 BSP/MX01P 里唯一允许 include HAL 头文件的文件。所有板级细节
 * （串口句柄、引脚宏、暂存缓冲大小）都写死在本文件，驱动核心保持可移植。
 */

#include "mx01p_port.h"

#include "main.h"
#include "usart.h"

#include <stddef.h>

/** UART 暂存缓冲：一包透传数据远小于此，留足余量。
 *  即使一段被拆成两次事件，ISR 里搬运 + rearm 也远快于 115200 下的字节间隔。 */
#define MX01P_PORT_RX_BUF_SIZE 256u

static mx01p_t *g_inst;
static uint8_t g_rx_buf[MX01P_PORT_RX_BUF_SIZE];
/* 上次 rearm 因 HAL 句柄被占用（阻塞发送期间）而失败时置位，稍后补挂 */
static volatile bool g_rx_rearm_pending;

/**
 * @brief 挂起一次 ReceiveToIdle 中断接收
 * @retval true 已挂起
 * @note 错误回调里 HAL 可能尚未把 RxState 复位（帧错误/噪声等非阻塞错误），
 *       此时先中止再重挂。中止只在异常路径出现，正常 IDLE/满缓冲路径直接成功。
 */
static bool mx01p_port_rx_start(void)
{
  HAL_StatusTypeDef st;

  st = HAL_UARTEx_ReceiveToIdle_IT(&huart1, g_rx_buf,
                                   (uint16_t)MX01P_PORT_RX_BUF_SIZE);
  if (st == HAL_OK) {
    g_rx_rearm_pending = false;
    return true;
  }

  (void)HAL_UART_AbortReceive(&huart1);
  st = HAL_UARTEx_ReceiveToIdle_IT(&huart1, g_rx_buf,
                                   (uint16_t)MX01P_PORT_RX_BUF_SIZE);
  if (st == HAL_OK) {
    g_rx_rearm_pending = false;
    return true;
  }

  g_rx_rearm_pending = true;
  return false;
}

void mx01p_port_attach(mx01p_t *inst)
{
  g_inst = inst;

  (void)mx01p_port_rx_start();
}

void mx01p_port_poll(void)
{
  if (g_rx_rearm_pending) {
    (void)mx01p_port_rx_start();
  }
}

void mx01p_port_reset(bool low)
{
  /* low=true：RES 拉低（复位）；low=false：释放复位，模块运行 */
  HAL_GPIO_WritePin(BT_RES_GPIO_Port, BT_RES_Pin,
                    low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void mx01p_port_set_sleep(bool sleep)
{
  /* 手册 Pin15：BRTS 高电平或悬空 = 睡眠，低电平 = 退出睡眠 */
  HAL_GPIO_WritePin(BT_BRTS_GPIO_Port, BT_BRTS_Pin,
                    sleep ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool mx01p_port_uart_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef st;

  if ((data == NULL) || (len == 0u)) {
    return false;
  }

  /* HAL 的发送接口没有 const，这里只做类型转换，不改缓冲区 */
  st = HAL_UART_Transmit(&huart1, (uint8_t *)(uintptr_t)data, len, timeout_ms);

  /* 阻塞发送期间 HAL 持有句柄锁，若恰好来了 RX 事件，rearm 会失败；
     发送返回后句柄锁已释放，这里补挂一次 */
  mx01p_port_poll();

  return (st == HAL_OK);
}

uint32_t mx01p_port_tick_ms(void)
{
  return HAL_GetTick();
}

void mx01p_port_delay_ms(uint32_t ms)
{
  mx01p_port_poll();
  HAL_Delay(ms);
}

uint32_t mx01p_port_irq_lock(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  return primask;
}

void mx01p_port_irq_unlock(uint32_t key)
{
  /* 只在进入前本来就没关中断时才开回来，避免破坏调用者的临界区 */
  if (key == 0u) {
    __enable_irq();
  }
}

void mx01p_port_rx_event(uint16_t size)
{
  if (size > (uint16_t)MX01P_PORT_RX_BUF_SIZE) {
    size = (uint16_t)MX01P_PORT_RX_BUF_SIZE;
  }

  if ((g_inst != NULL) && (size != 0u)) {
    mx01p_rx_feed(g_inst, g_rx_buf, size);
  }

  (void)mx01p_port_rx_start();
}

void mx01p_port_uart_error(void)
{
  /* ORE/帧错误最多毁掉当前一包，复位接收后继续；不 panic、不打印。
     模块复位/唤醒的瞬间本来就容易出一次，属正常现象，所以只计数不中断流程 */
  if (g_inst != NULL) {
    g_inst->rx_errors++;
  }
  (void)mx01p_port_rx_start();
}
