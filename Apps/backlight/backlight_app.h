/**
 * @file    backlight_app.h
 * @author  YangKY
 * @brief   屏幕背光按键应用：KEY_LIGHT 上升沿点亮背光，保持 3 s
 *
 * 通路：EXTI0(PA0) --> HAL_GPIO_EXTI_Rising_Callback() --> backlight_app_button_isr()
 *       --事件标志--> backlight 线程 --> 点亮 --> 等待 3 s 超时 --> 熄灭
 *
 * 在保持窗口内再次按下（上升沿）会重新计时，即从最后一次按键算 3 s。
 */

#ifndef BACKLIGHT_APP_H
#define BACKLIGHT_APP_H

/**
 * @brief 初始化背光按键应用（在 tx_application_define 里调用）
 * @note  创建事件组与背光线程；调用前必须先执行 backlight_init() 启动
 *        TIM17_CH1 PWM（main() 里、tx_kernel_enter() 之前完成）。
 */
void backlight_app_init(void);

/**
 * @brief 背光按键上升沿通知（在 EXTI0 中断里调用）
 * @note  只置事件标志，不操作 PWM；点亮与计时都在背光线程里完成。
 */
void backlight_app_button_isr(void);

#endif /* BACKLIGHT_APP_H */
