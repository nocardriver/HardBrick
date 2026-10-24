/**
 * @file    display.h
 * @author  YangKY
 * @brief   显示应用层：竖屏 GPS 数据面板（ThreadX 线程）
 *
 * 职责：周期性读取 gps_get_nav() 快照，格式化后按"脏字段"局部刷新 ILI9341。
 * 画面布局与刷新策略见 display.c；屏幕驱动见 BSP/ILI9341。
 */

#ifndef DISPLAY_H
#define DISPLAY_H

/**
 * @brief 创建显示线程（在 tx_application_define 里、gps_init() 之后调用）
 * @note  线程栈静态分配；线程优先级 5，低于 GPS 解析线程（4）。
 *        硬件初始化（ili9341_init）已在 main() 里完成。
 */
void display_init(void);

#endif /* DISPLAY_H */
