/**
 * @file    e22_port.c
 * @author  YangKY
 * @brief   E22-400T22S 平台移植层实现：LPUART1 中断收发 + 控制引脚 + AUX EXTI
 *
 * 这里是 BSP/E22 里唯一允许 include HAL 头文件的文件。所有板级细节
 * （串口句柄、引脚宏、暂存缓冲大小）都写死在本文件，驱动核心保持可移植。
 */

#include "e22_port.h"

#include "main.h"
#include "usart.h"

#include <stddef.h>

/** UART 暂存缓冲：覆盖模块单包最大 240 B，留 16 B 余量。
 *  即使一包被拆成两次事件，ISR 里搬运 + rearm 也远快于 9600 下的字节间隔。 */
#define E22_PORT_RX_BUF_SIZE 256u

static e22_t *g_inst;
static uint8_t g_rx_buf[E22_PORT_RX_BUF_SIZE];
/* 上次 rearm 因 HAL 句柄被占用（阻塞发送期间）而失败时置位，稍后补挂 */
static volatile bool g_rx_rearm_pending;

/**
 * @brief 挂起一次 ReceiveToIdle 中断接收
 * @retval true 已挂起
 * @note 错误回调里 HAL 可能尚未把 RxState 复位（帧错误/噪声等非阻塞错误），
 *       此时先中止再重挂。中止只在异常路径出现，正常 IDLE/满缓冲路径直接成功。
 */
static bool e22_port_rx_start(void)
{
  HAL_StatusTypeDef st;

  st = HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, g_rx_buf,
                                   (uint16_t)E22_PORT_RX_BUF_SIZE);
  if (st == HAL_OK) {
    g_rx_rearm_pending = false;
    return true;
  }

  (void)HAL_UART_AbortReceive(&hlpuart1);
  st = HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, g_rx_buf,
                                   (uint16_t)E22_PORT_RX_BUF_SIZE);
  if (st == HAL_OK) {
    g_rx_rearm_pending = false;
    return true;
  }

  g_rx_rearm_pending = true;
  return false;
}

void e22_port_attach(e22_t *inst)
{
  GPIO_InitTypeDef gpio = {0};

  g_inst = inst;

  /* AUX 用双边沿：下降沿=模块开始忙（收/发/切模式），上升沿=空闲。
     HardBrick.ioc 里 PB10 已改为 GPIO_MODE_IT_RISING_FALLING；重新生成后
     这段重配与生成代码一致，是幂等的。保留它是为了在旧生成代码上也能立即工作。 */
  gpio.Pin = LORA_AUX_Pin;
  gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &gpio);

  __HAL_GPIO_EXTI_CLEAR_RISING_IT(LORA_AUX_Pin);
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(LORA_AUX_Pin);

  (void)e22_port_rx_start();
}

/**
 * @brief 补挂接收：上次 rearm 时 HAL 句柄被阻塞发送占用，会在这里重试
 * @note 由驱动在延时/发送返回/读缓冲时调用，避免 RX 永久停摆。
 */
void e22_port_poll(void)
{
  if (g_rx_rearm_pending) {
    (void)e22_port_rx_start();
  }
}

bool e22_port_aux_level(void)
{
  return (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET);
}

void e22_port_set_mode_pins(bool m1, bool m0)
{
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin,
                    m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin,
                    m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void e22_port_reset(bool low)
{
  /* low=true：NRST 拉低（复位）；low=false：释放复位，NRST 拉高 */
  HAL_GPIO_WritePin(LORA_NRST_GPIO_Port, LORA_NRST_Pin,
                    low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool e22_port_uart_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef st;

  if ((data == NULL) || (len == 0u)) {
    return false;
  }

  /* HAL 的发送接口没有 const，这里只做类型转换，不改缓冲区 */
  st = HAL_UART_Transmit(&hlpuart1, (uint8_t *)(uintptr_t)data, len,
                         timeout_ms);

  /* 阻塞发送期间 HAL 持有句柄锁，若恰好来了 RX 事件，rearm 会失败；
     发送返回后句柄锁已释放，这里补挂一次 */
  e22_port_poll();

  return (st == HAL_OK);
}

uint32_t e22_port_tick_ms(void)
{
  return HAL_GetTick();
}

void e22_port_delay_ms(uint32_t ms)
{
  e22_port_poll();
  HAL_Delay(ms);
}

void e22_port_rx_event(uint16_t size)
{
  if (size > (uint16_t)E22_PORT_RX_BUF_SIZE) {
    size = (uint16_t)E22_PORT_RX_BUF_SIZE;
  }

  if ((g_inst != NULL) && (size != 0u)) {
    e22_rx_feed(g_inst, g_rx_buf, size);
  }

  (void)e22_port_rx_start();
}

void e22_port_uart_error(void)
{
  /* ORE/帧错误最多毁掉当前一包，复位接收后继续；不 panic、不打印 */
  (void)e22_port_rx_start();
}

void e22_port_aux_isr(void)
{
  bool busy;

  if (g_inst == NULL) {
    return;
  }

  /* 边沿可能抖动，以读到的实际电平为准 */
  busy = !e22_port_aux_level();
  e22_aux_event(g_inst, busy);
}
