/**
 * @file    backlight_app.c
 * @author  YangKY
 * @brief   屏幕背光按键行为：KEY_LIGHT 上升沿点亮背光，保持 3 s
 *
 * 为什么用「事件标志 + 线程」而不是在中断里直接开灯、用 HAL_Delay 计时：
 *   ISR 里不能阻塞，且 PWM 硬件只应有一个上下文访问。中断只置事件标志，
 *   线程在事件上等待 3 s 超时；等待期间再次按键会立刻返回，相当于重新计时。
 */

#include "backlight_app.h"

#include "backlight.h"
#include "tx_api.h"
#include "SEGGER_RTT.h"

/** 背光线程栈（字节，静态分配） */
#define BACKLIGHT_APP_STACK_SIZE 512u

/** 保持点亮时间：10 s（ThreadX tick = 100 Hz，即 10 ms/tick） */
#define BACKLIGHT_APP_HOLD_TICKS 1000u

/** 事件标志：按键出现上升沿 */
#define BACKLIGHT_APP_EVENT_PRESS 0x1u

static TX_THREAD            g_thread;
static TX_EVENT_FLAGS_GROUP g_events;
static uint8_t              g_stack[BACKLIGHT_APP_STACK_SIZE] __attribute__((aligned(8)));

void backlight_app_button_isr(void)
{
  /* 未初始化完成时 set 会返回 TX_EVENT_FLAGS_ERROR，直接忽略即可 */
  (void)tx_event_flags_set(&g_events, BACKLIGHT_APP_EVENT_PRESS, TX_OR);
}

static void backlight_app_thread_entry(ULONG thread_input)
{
  ULONG actual;

  (void)thread_input;

  for (;;) {
    /* 平时休眠，不占 CPU；等到按键上升沿才亮屏 */
    (void)tx_event_flags_get(&g_events, BACKLIGHT_APP_EVENT_PRESS,
                             TX_OR_CLEAR, &actual, TX_WAIT_FOREVER);

    backlight_set_brightness(BACKLIGHT_BRIGHTNESS_MAX);

    /* 3 s 内再次按下 -> 本次返回 TX_SUCCESS，继续循环并重新计时；
       超时 -> 返回 TX_NO_EVENT，退出循环关背光 */
    while (tx_event_flags_get(&g_events, BACKLIGHT_APP_EVENT_PRESS,
                              TX_OR_CLEAR, &actual,
                              BACKLIGHT_APP_HOLD_TICKS) == TX_SUCCESS) {
      /* 重新计时 */
    }

    backlight_set_brightness(0u);
  }
}

void backlight_app_init(void)
{
  if (tx_event_flags_create(&g_events, "backlight") != TX_SUCCESS) {
    SEGGER_RTT_WriteString(0, "[BL] event group create failed\r\n");
    return;
  }

  /* 优先级 6，低于 GPS(4) / display(5)：背光不是实时任务，事件到达后
     等显示线程本轮 SPI 刷完再执行即可 */
  if (tx_thread_create(&g_thread, "backlight",
                       backlight_app_thread_entry, 0u,
                       g_stack, sizeof(g_stack),
                       6u, 6u, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
    SEGGER_RTT_WriteString(0, "[BL] thread create failed\r\n");
  }
}
