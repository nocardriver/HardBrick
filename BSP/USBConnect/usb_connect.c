/**
 * @file    usb_connect.c
 * @brief   USB 连接线控制实现（BSP 层）
 *
 * 只做两件事：把 D+ 引脚在「GPIO 输出低」和「AF10/USB」之间来回切换。
 * 时序、延时、OTG 的启停都由 Apps/usb_msc 负责。
 */

#include "usb_connect.h"

#include "main.h"

/** D+ 引脚：OTG_FS 的 DP 固定是 PA12（AF10），这里按 GPIO 驱动它 */
#define USB_CONNECT_DP_GPIO_PORT   GPIOA
#define USB_CONNECT_DP_GPIO_PIN    GPIO_PIN_12
#define USB_CONNECT_DP_GPIO_AF     GPIO_AF10_USB

/* D+ 引脚状态，仅用于状态查询与日志 */
static bool g_held_off;

void usb_connect_hold_off(void)
{
  GPIO_InitTypeDef init = {0};

  /* main() 里走到这里时 GPIOA 时钟通常已经打开，但本模块不假设调用时机 */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* 先写输出锁存再切模式：切换的瞬间引脚就是低电平，不会先冒一个高电平脉冲 */
  HAL_GPIO_WritePin(USB_CONNECT_DP_GPIO_PORT, USB_CONNECT_DP_GPIO_PIN, GPIO_PIN_RESET);

  init.Pin = USB_CONNECT_DP_GPIO_PIN;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;
  /* 低速足够：这里只是把线拉低，不传输数据，低频边沿反而更干净 */
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_CONNECT_DP_GPIO_PORT, &init);

  g_held_off = true;
}

void usb_connect_release(void)
{
  GPIO_InitTypeDef init = {0};

  init.Pin = USB_CONNECT_DP_GPIO_PIN;
  init.Mode = GPIO_MODE_AF_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  init.Alternate = USB_CONNECT_DP_GPIO_AF;
  HAL_GPIO_Init(USB_CONNECT_DP_GPIO_PORT, &init);

  g_held_off = false;
}

bool usb_connect_is_held_off(void)
{
  return g_held_off;
}
