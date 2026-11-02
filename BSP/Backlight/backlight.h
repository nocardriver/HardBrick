/**
 * @file    backlight.h
 * @author  YangKY
 * @brief   屏幕背光 PWM 输出（TIM17_CH1 / PB9）
 *
 * 硬件：PB9 复用为 TIM17_CH1，PWM 频率 = TIM17 计数时钟 / 1000
 *       （当前配置 APB2 160 MHz、PSC=15、ARR=999 → 10 kHz），
 *       占空比即背光亮度，0 = 熄灭，100 = 最亮。
 *
 * 分层：本模块只做硬件输出，不依赖 RTOS；按键触发与 3 s 保持策略在
 *       Apps/backlight/backlight_app.c。
 */

#ifndef BACKLIGHT_H
#define BACKLIGHT_H

#include <stdbool.h>
#include <stdint.h>

/** 100% 占空比（最亮） */
#define BACKLIGHT_BRIGHTNESS_MAX 100u

/**
 * @brief 初始化背光 PWM，并把占空比置 0（熄灭）
 * @retval true  PWM 启动成功
 * @retval false HAL_TIM_PWM_Start() 失败（TIM17 未初始化或通道状态非法）
 * @note  必须在 MX_TIM17_Init() 之后调用；本函数不依赖 RTOS。
 */
bool backlight_init(void);

/**
 * @brief 设置背光亮度
 * @param percent 0 = 熄灭，100 = 最亮；大于 100 按 100 处理
 * @note  可在任意上下文调用（只写 CCR1 寄存器，不阻塞）。
 */
void backlight_set_brightness(uint8_t percent);

#endif /* BACKLIGHT_H */
