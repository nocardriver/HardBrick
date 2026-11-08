/**
 * @file    ili9341.c
 * @author  YangKY
 * @brief   ILI9341 控制器驱动核心：初始化序列、地址窗口、填充与像素块写入
 *
 * 不含 HAL / RTOS 依赖；SPI 与引脚操作全部经由 ili9341_port.c。
 * 初始化序列沿用已验证的 STM32-ILI9341 演示工程（Brick），只把引脚换成
 * HardBrick 的 SPI1_CS / DIS_DC / DIS_RST。
 */

#include "ili9341.h"
#include "ili9341_port.h"

/* ILI9341 命令码（只列本驱动用到的） */
#define ILI9341_CMD_SWRESET       0x01u
#define ILI9341_CMD_SLPOUT        0x11u
#define ILI9341_CMD_DISPON        0x29u
#define ILI9341_CMD_CASET         0x2Au
#define ILI9341_CMD_PASET         0x2Bu
#define ILI9341_CMD_RAMWR         0x2Cu
#define ILI9341_CMD_MADCTL        0x36u

/** MADCTL 的 BGR 位：面板按 BGR 排布；方向位由 ili9341_set_rotation() 统一设置 */
#define ILI9341_MADCTL_BGR        0x08u

/** 单色填充时复用的行缓冲像素数（240 px = 480 B） */
#define ILI9341_FILL_CHUNK_PIXELS  240u

/** 像素块单次 HAL 传输上限（字节）；HAL 的 Size 是 uint16_t */
#define ILI9341_BLIT_CHUNK_BYTES   4096u

/** 初始化表中的一条命令 */
typedef struct {
  uint8_t  cmd;
  uint8_t  len;
  uint8_t  data[16];
  uint16_t delay_ms;   /**< 命令发送后的延时（ms），0 表示不延时 */
} ili9341_init_cmd_t;

/**
 * 初始化序列：STARTEK KD035QVRMA223 面板 + ILI9341，已验证于 Brick 演示工程。
 * 0x3A=0x55 选 16 bit/pixel；0x36(MADCTL) 在这里只定 BGR 位，方向位由
 * ili9341_init() 末尾的 ili9341_set_rotation(ILI9341_DEFAULT_ROTATION) 写入，
 * 保证"方向"只有一处定义。
 */
static const ili9341_init_cmd_t g_init_seq[] = {
  {ILI9341_CMD_SWRESET, 0u, {0}, 1000u},

  {0xCBu, 5u, {0x39u, 0x2Cu, 0x00u, 0x34u, 0x02u}, 0u},              /* POWER CONTROL A */
  {0xCFu, 3u, {0x00u, 0xC1u, 0x30u}, 0u},                            /* POWER CONTROL B */
  {0xE8u, 3u, {0x85u, 0x00u, 0x78u}, 0u},                            /* DRIVER TIMING A */
  {0xEAu, 2u, {0x00u, 0x00u}, 0u},                                   /* DRIVER TIMING B */
  {0xEDu, 4u, {0x64u, 0x03u, 0x12u, 0x81u}, 0u},                     /* POWER ON SEQUENCE */
  {0xF7u, 1u, {0x20u}, 0u},                                          /* PUMP RATIO */
  {0xC0u, 1u, {0x23u}, 0u},                                          /* POWER CONTROL 1 */
  {0xC1u, 1u, {0x10u}, 0u},                                          /* POWER CONTROL 2 */
  {0xC5u, 2u, {0x3Eu, 0x28u}, 0u},                                   /* VCOM CONTROL 1 */
  {0xC7u, 1u, {0x86u}, 0u},                                          /* VCOM CONTROL 2 */
  {ILI9341_CMD_MADCTL, 1u, {ILI9341_MADCTL_BGR}, 0u},                /* 只定 BGR，方向在 init 末尾设置 */
  {0x3Au, 1u, {0x55u}, 0u},                                          /* PIXEL FORMAT = RGB565 */
  {0xB1u, 2u, {0x00u, 0x18u}, 0u},                                   /* FRAME RATE */
  {0xB6u, 3u, {0x08u, 0x82u, 0x27u}, 0u},                            /* DISPLAY FUNCTION */
  {0xF2u, 1u, {0x00u}, 0u},                                          /* 3-GAMMA DISABLE */
  {0x26u, 1u, {0x01u}, 0u},                                          /* GAMMA CURVE */
  {0xE0u, 15u, {0x0Fu, 0x31u, 0x2Bu, 0x0Cu, 0x0Eu, 0x08u, 0x4Eu,
                0xF1u, 0x37u, 0x07u, 0x10u, 0x03u, 0x0Eu, 0x09u, 0x00u}, 0u},  /* +GAMMA */
  {0xE1u, 15u, {0x00u, 0x0Eu, 0x14u, 0x03u, 0x11u, 0x07u, 0x31u,
                0xC1u, 0x48u, 0x08u, 0x0Fu, 0x0Cu, 0x31u, 0x36u, 0x0Fu}, 0u},  /* -GAMMA */
  {ILI9341_CMD_SLPOUT, 0u, {0}, 120u},
  {ILI9341_CMD_DISPON, 0u, {0}, 0u},
};

static bool     g_ready;
static uint16_t g_width  = ILI9341_WIDTH;
static uint16_t g_height = ILI9341_HEIGHT;

/** 单色填充行缓冲：模块内唯一一块静态缓冲，480 B */
static uint8_t g_fill_chunk[ILI9341_FILL_CHUNK_PIXELS * 2u];

/**
 * @brief 发送一条命令（可选带参数）
 * @note  CS 在命令+参数期间保持低电平；任一传输失败就把 g_ready 清零，
 *        后续绘制接口退化为空操作，避免在半初始化状态下继续刷屏。
 */
static bool ili9341_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{
  bool ok;

  ili9341_port_cs(true);
  ili9341_port_dc(false);
  ok = ili9341_port_write(&cmd, 1u);

  if (ok && (len > 0u)) {
    ili9341_port_dc(true);
    ok = ili9341_port_write(data, len);
  }

  ili9341_port_cs(false);

  if (!ok) {
    g_ready = false;
  }

  return ok;
}

/**
 * @brief 设置矩形地址窗口，并把写指针复位到窗口起点（0x2C）
 * @note  调用前必须已做边界裁剪，x1/y1 为闭区间右下角
 */
static void ili9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t xbuf[4] = {
    (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFFu),
    (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFFu),
  };
  uint8_t ybuf[4] = {
    (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFFu),
    (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFFu),
  };

  (void)ili9341_send_cmd(ILI9341_CMD_CASET, xbuf, 4u);
  (void)ili9341_send_cmd(ILI9341_CMD_PASET, ybuf, 4u);
  (void)ili9341_send_cmd(ILI9341_CMD_RAMWR, (const uint8_t *)0, 0u);
}

bool ili9341_init(void)
{
  uint32_t i;

  g_ready  = false;
  g_width  = ILI9341_WIDTH;
  g_height = ILI9341_HEIGHT;

  ili9341_port_init();

  /* 硬件复位：高 1 ms → 低 10 ms → 高 120 ms（面板规格书复位时序） */
  ili9341_port_rst(true);
  ili9341_port_delay_ms(1u);
  ili9341_port_rst(false);
  ili9341_port_delay_ms(10u);
  ili9341_port_rst(true);
  ili9341_port_delay_ms(120u);

  for (i = 0u; i < (sizeof(g_init_seq) / sizeof(g_init_seq[0])); i++) {
    const ili9341_init_cmd_t *cmd = &g_init_seq[i];

    if (!ili9341_send_cmd(cmd->cmd,
                          (cmd->len > 0u) ? cmd->data : (const uint8_t *)0,
                          cmd->len)) {
      return false;
    }
    if (cmd->delay_ms > 0u) {
      ili9341_port_delay_ms(cmd->delay_ms);
    }
  }

  g_ready = true;
  /* 面板倒装：默认方向相对 PORTRAIT_1 旋转 180°（见 ILI9341_DEFAULT_ROTATION） */
  ili9341_set_rotation(ILI9341_DEFAULT_ROTATION);

  if (!g_ready) {
    return false;
  }

  ili9341_fill_rect(0u, 0u, g_width, g_height, 0x0000u);
  return g_ready;
}

bool ili9341_is_ready(void)
{
  return g_ready;
}

bool ili9341_rtos_attach(void)
{
  return ili9341_port_dma_attach();
}

void ili9341_set_rotation(ili9341_rotation_t rot)
{
  uint8_t madctl;

  switch (rot) {
  case ILI9341_ROT_PORTRAIT_1:
    madctl   = 0x48u;
    g_width  = ILI9341_WIDTH;
    g_height = ILI9341_HEIGHT;
    break;
  case ILI9341_ROT_LANDSCAPE_1:
    madctl   = 0x28u;
    g_width  = ILI9341_HEIGHT;
    g_height = ILI9341_WIDTH;
    break;
  case ILI9341_ROT_PORTRAIT_2:
    madctl   = 0x88u;
    g_width  = ILI9341_WIDTH;
    g_height = ILI9341_HEIGHT;
    break;
  case ILI9341_ROT_LANDSCAPE_2:
    madctl   = 0xE8u;
    g_width  = ILI9341_HEIGHT;
    g_height = ILI9341_WIDTH;
    break;
  default:
    return;
  }

  (void)ili9341_send_cmd(ILI9341_CMD_MADCTL, &madctl, 1u);
}

uint16_t ili9341_width(void)
{
  return g_width;
}

uint16_t ili9341_height(void)
{
  return g_height;
}

void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  uint32_t remaining;
  uint16_t chunk;
  uint16_t i;

  if (!g_ready || (w == 0u) || (h == 0u)) {
    return;
  }
  if ((x >= g_width) || (y >= g_height)) {
    return;
  }

  /* 裁剪到屏幕内（纯色填充没有行跨度问题） */
  if (((uint32_t)x + w) > g_width) {
    w = (uint16_t)(g_width - x);
  }
  if (((uint32_t)y + h) > g_height) {
    h = (uint16_t)(g_height - y);
  }

  ili9341_set_window(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));
  if (!g_ready) {
    return;
  }

  for (i = 0u; i < ILI9341_FILL_CHUNK_PIXELS; i++) {
    g_fill_chunk[2u * i]      = (uint8_t)(color >> 8);
    g_fill_chunk[2u * i + 1u] = (uint8_t)(color & 0xFFu);
  }

  remaining = (uint32_t)w * (uint32_t)h;

  ili9341_port_cs(true);
  ili9341_port_dc(true);
  while (remaining > 0u) {
    chunk = (remaining > ILI9341_FILL_CHUNK_PIXELS)
            ? (uint16_t)ILI9341_FILL_CHUNK_PIXELS : (uint16_t)remaining;

    if (!ili9341_port_write(g_fill_chunk, (uint16_t)(chunk * 2u))) {
      g_ready = false;
      break;
    }
    remaining -= chunk;
  }
  ili9341_port_cs(false);
}

void ili9341_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels)
{
  uint32_t remaining;
  uint16_t chunk;

  if (!g_ready || !pixels || (w == 0u) || (h == 0u)) {
    return;
  }

  /* 不做裁剪：裁剪会改变源缓冲的行跨度，调用者必须传屏幕内的矩形 */
  if (((uint32_t)x + w) > g_width) {
    return;
  }
  if (((uint32_t)y + h) > g_height) {
    return;
  }

  ili9341_set_window(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));
  if (!g_ready) {
    return;
  }

  remaining = (uint32_t)w * (uint32_t)h * 2u;

  ili9341_port_cs(true);
  ili9341_port_dc(true);
  while (remaining > 0u) {
    chunk = (remaining > ILI9341_BLIT_CHUNK_BYTES)
            ? (uint16_t)ILI9341_BLIT_CHUNK_BYTES : (uint16_t)remaining;

    if (!ili9341_port_write(pixels, chunk)) {
      g_ready = false;
      break;
    }
    pixels    += chunk;
    remaining -= chunk;
  }
  ili9341_port_cs(false);
}

void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
  uint8_t pixel[2];

  if (!g_ready || (x >= g_width) || (y >= g_height)) {
    return;
  }

  pixel[0] = (uint8_t)(color >> 8);
  pixel[1] = (uint8_t)(color & 0xFFu);

  ili9341_set_window(x, y, x, y);
  if (!g_ready) {
    return;
  }

  ili9341_port_cs(true);
  ili9341_port_dc(true);
  (void)ili9341_port_write(pixel, 2u);
  ili9341_port_cs(false);
}
