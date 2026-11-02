/**
 * @file    backlight.c
 * @author  YangKY
 * @brief   屏幕背光 PWM 输出（TIM17_CH1 / PB9）
 *
 * 只负责 TIM17 通道的输出：初始化时以 0% 占空比启动，之后由业务层
 * 调用 backlight_set_brightness() 改 CCR1。停止计数没有必要——0% 时
 * 引脚恒低，背光电路不工作，保留 PWM 运行可以让点亮响应无延迟。
 */

#include "backlight.h"

#include "main.h"
#include "tim.h"

bool backlight_init(void)
{
  /* 先清 CCR1 再启动：PWM 使能瞬间不会按旧比较值闪一下 */
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0u);

  if (HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1) != HAL_OK) {
    return false;
  }

  return true;
}

void backlight_set_brightness(uint8_t percent)
{
  /* 一个 PWM 周期有 ARR+1 个计数；CCR 等于该值时 PWM1 模式输出恒高 */
  uint32_t period_ticks = (uint32_t)htim17.Init.Period + 1u;

  if (percent > BACKLIGHT_BRIGHTNESS_MAX) {
    percent = BACKLIGHT_BRIGHTNESS_MAX;
  }

  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,
                        (period_ticks * (uint32_t)percent) / 100u);
}
