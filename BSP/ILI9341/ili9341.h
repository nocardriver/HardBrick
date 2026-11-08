/**
 * @file    ili9341.h
 * @author  YangKY
 * @brief   ILI9341 3.5" 240x320 TFT 驱动（4-line SPI，RGB565）
 *
 * 硬件：STARTEK KD035QVRMA223（3.5" 透反屏，240x320，控制器 ILI9341）。
 *       4-line SPI 的 CS/DC/RST 由 GPIO 控制，数据走 SPI1。
 *
 * 分层：本文件与 ili9341.c 只做控制器寄存器操作（初始化序列、窗口、填充、
 *       像素块写入），不依赖 HAL / RTOS；SPI 与引脚操作全部收敛在
 *       ili9341_port.c。后续接 LVGL 时，flush 回调直接复用 ili9341_blit()。
 *
 * 像素格式：RGB565，**大端字节序**（高字节先发）。调用 ili9341_blit() 时
 *           缓冲区必须已经按大端排好：buf[0]=color>>8, buf[1]=color&0xFF。
 *           文本/图形渲染在 App 层完成，驱动不持有帧缓冲。
 */

#ifndef ILI9341_H
#define ILI9341_H

#include <stdint.h>
#include <stdbool.h>

/** 面板原生分辨率（竖屏） */
#define ILI9341_WIDTH   240u
#define ILI9341_HEIGHT  320u

/** 8-bit RGB → RGB565 */
#define ILI9341_RGB565(r, g, b) \
  ((uint16_t)((((uint16_t)(r) & 0xF8u) << 8) | \
              (((uint16_t)(g) & 0xFCu) << 3) | \
              (((uint16_t)(b) & 0xF8u) >> 3)))

/** 屏幕方向（MADCTL 值沿用已验证的 ILI9341 演示工程，均保持 BGR=1） */
typedef enum {
  ILI9341_ROT_PORTRAIT_1 = 0,   /**< 240x320，MX=1（0x48），面板正向装配 */
  ILI9341_ROT_LANDSCAPE_1,      /**< 320x240，MV=1（0x28） */
  ILI9341_ROT_PORTRAIT_2,       /**< 240x320，MX=0 MY=1（0x88），相对 PORTRAIT_1 旋转 180° */
  ILI9341_ROT_LANDSCAPE_2,      /**< 320x240，MV=1 MX=1 MY=1（0xE8） */
} ili9341_rotation_t;

/**
 * 上电后的默认方向。
 *
 * 本机面板是倒装的：逻辑坐标相对正向（PORTRAIT_1）需要旋转 180°，所以默认用
 * PORTRAIT_2。宽高仍是 240x320，App 的布局坐标不需要为装配方向做任何补偿。
 * 面板正装时只改这一个宏即可，不必再动初始化序列。
 */
#define ILI9341_DEFAULT_ROTATION  ILI9341_ROT_PORTRAIT_2

/**
 * @brief 初始化面板：硬件复位 → 厂商初始化序列 → 清屏为黑
 * @retval true 初始化成功；false 任一 SPI 传输失败（随后所有绘制接口变为空操作）
 * @note  内部使用毫秒级延时约 1.3 s（复位 + SWRESET + SLPOUT），
 *        因此应在 tx_kernel_enter() 之前调用，或在启动早期允许阻塞的上下文调用。
 */
bool ili9341_init(void);

/** @brief 面板是否已成功初始化（失败后绘制接口全部空操作） */
bool ili9341_is_ready(void);

/**
 * @brief 设置屏幕方向
 * @param rot 方向枚举
 * @note  改变方向会同时改变 ili9341_width()/ili9341_height() 的返回值；
 *        切换后窗口坐标系随之变化，调用者负责重绘。
 */
void ili9341_set_rotation(ili9341_rotation_t rot);

/** @brief 当前方向下的逻辑宽度（像素） */
uint16_t ili9341_width(void);

/** @brief 当前方向下的逻辑高度（像素） */
uint16_t ili9341_height(void);

/**
 * @brief 用单色填充矩形（自动裁剪到屏幕内）
 * @param x,y 左上角
 * @param w,h 宽高，允许为 0
 * @param color RGB565 颜色
 */
void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief 写入一块 RGB565 像素（大端）
 * @param x,y    左上角
 * @param w,h    宽高，必须完全落在屏幕内（不做裁剪，避免破坏源缓冲的行跨度）
 * @param pixels 像素数据，长度 = w*h*2 字节
 */
void ili9341_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels);

/** @brief 画一个像素（越界自动忽略） */
void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

#endif /* ILI9341_H */
