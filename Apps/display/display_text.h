/**
 * @file    display_text.h
 * @author  YangKY
 * @brief   5x5 点阵字体渲染：把 ASCII 文本画进 RGB565 大端缓冲区
 *
 * 字模来自 Matej Artnak 的 ILI9341_GFX（MIT License）：
 *   每个字符在 6 列 x 8 行的单元内，实际墨迹占第 2~6 行、第 0~4 列，
 *   即视觉上是 5x5。放大采用整数倍（1x/2x/3x），不做抗锯齿。
 *
 * 渲染结果是"大端 RGB565"字节流，可直接交给 ili9341_blit()。
 */

#ifndef DISPLAY_TEXT_H
#define DISPLAY_TEXT_H

#include <stdint.h>

/** 字符单元宽度（列），含 1 列字符间距 */
#define DISPLAY_TEXT_CELL_W  6u
/** 墨迹宽度（列） */
#define DISPLAY_TEXT_INK_W   5u
/** 墨迹高度（行） */
#define DISPLAY_TEXT_INK_H   5u

/** 文本水平对齐方式 */
typedef enum {
  DISPLAY_TEXT_LEFT = 0,
  DISPLAY_TEXT_CENTER,
  DISPLAY_TEXT_RIGHT,
} display_text_align_t;

/**
 * @brief 计算文本的视觉宽度（像素，已扣掉最后一列空白）
 * @param text  以 '\0' 结尾的 ASCII 文本
 * @param scale 整数放大倍数（不能为 0）
 * @retval 像素宽度；text 为空或 scale 为 0 时返回 0
 */
uint16_t display_text_width(const char *text, uint8_t scale);

/**
 * @brief 把一行文本渲染到 RGB565 大端缓冲区
 * @param dst   目标缓冲，容量必须 >= w*h*2 字节
 * @param w     缓冲宽度（像素），也是渲染区域的宽度
 * @param h     缓冲高度（像素），文本在垂直方向居中
 * @param text  ASCII 文本；不支持的控制字符会按 '?' 显示
 * @param scale 整数放大倍数
 * @param fg    文本颜色（RGB565）
 * @param bg    背景颜色（RGB565），整个区域先被它填满
 * @param align 水平对齐方式
 * @note  文本比 w 宽时从右侧截断；不做换行。
 */
void display_text_render(uint8_t *dst, uint16_t w, uint16_t h,
                         const char *text, uint8_t scale,
                         uint16_t fg, uint16_t bg,
                         display_text_align_t align);

#endif /* DISPLAY_TEXT_H */
