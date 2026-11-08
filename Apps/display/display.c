/**
 * @file    display.c
 * @author  YangKY
 * @brief   竖屏 GPS 数据面板：布局、格式化、脏字段局部刷新
 *
 * 画面（240x320，竖屏）：
 *
 *   +--------------------------------------+
 *   | HARDBRICK                   GNSS     |  标题栏
 *   +--------------------------------------+
 *   |              12:34:56                |  本地时间 UTC+8（3x）
 *   |             2026-10-24               |  日期（2x）
 *   +--------------------------------------+
 *   | LAT                                  |  标签 2x
 *   |  31.230416 N                         |  坐标 3x
 *   | LON                                  |
 *   | 121.473701 E                         |
 *   | ALT   12.3 m        SAT   08         |  标签 2x + 数值 3x
 *   +--------------------------------------+
 *   |            FIX  SINGLE               |  状态条（颜色随定位状态变化）
 *   +--------------------------------------+
 *
 * 刷新策略：每 200 ms 取一次快照，格式化后与上一次的字符串比较，
 * 只有变化的字段才重绘（重绘 = 铺背景 + 写文字，一次 blit），因此
 * 静止时几乎不占总线，时间跳秒时也不会整屏闪烁。
 *
 * 线程优先级 5，低于 GPS 解析线程（4）；像素数据由 SPI DMA 发送，线程在完成
 * 信号量上睡眠等待（不忙等 CPU），单次字段刷新最多约 4 ms。
 */

#include "display.h"

#include <stdio.h>
#include <string.h>

#include "gps.h"
#include "ili9341.h"
#include "display_text.h"
#include "tx_api.h"
#include "SEGGER_RTT.h"

/* ------------------------------------------------------------------------- */
/* 配置与布局                                                                  */
/* ------------------------------------------------------------------------- */

/** 显示线程栈（字节，静态分配） */
#define DISPLAY_THREAD_STACK_SIZE  3072u

/** 刷新周期：20 tick = 200 ms（ThreadX tick = 10 ms） */
#define DISPLAY_REFRESH_TICKS      20u

/** 超过 10 s 没有新语句就认为 GPS 数据过期 */
#define DISPLAY_STALE_TICKS        1000u

/** 文本渲染缓冲上限（像素）；最大字段是整屏宽 240、3x 文字高 15 */
#define DISPLAY_TEXT_BUF_W         240u
#define DISPLAY_TEXT_BUF_H         24u

/* 颜色（RGB565） */
#define COLOR_BG       ILI9341_RGB565(8, 12, 18)
#define COLOR_HEADER   ILI9341_RGB565(16, 24, 32)
#define COLOR_ACCENT   ILI9341_RGB565(0, 200, 255)
#define COLOR_TEXT     ILI9341_RGB565(235, 241, 247)
#define COLOR_LABEL    ILI9341_RGB565(122, 138, 153)
#define COLOR_DATE     ILI9341_RGB565(176, 196, 212)
#define COLOR_DIVIDER  ILI9341_RGB565(42, 58, 74)
#define COLOR_FIX      ILI9341_RGB565(14, 90, 42)
#define COLOR_NOFIX    ILI9341_RGB565(122, 30, 30)
#define COLOR_LOST     ILI9341_RGB565(122, 58, 10)
#define COLOR_WAIT     ILI9341_RGB565(42, 58, 74)

#define SCREEN_W       240u
#define SCREEN_H       320u

/* 标题栏 */
#define HEADER_H       32u
#define TITLE_X        10u
#define TITLE_Y        11u
#define GNSS_X         182u
#define GNSS_Y         11u

/* 时间 / 日期 */
#define TIME_X         0u
#define TIME_Y         46u
#define TIME_W         240u
#define TIME_H         15u
#define DATE_X         0u
#define DATE_Y         72u
#define DATE_W         240u
#define DATE_H         10u

/* 分隔线 */
#define DIV_X          12u
#define DIV_W          216u
#define DIV_TOP_Y      96u
#define DIV_BOTTOM_Y   240u

/* 经纬度：标签 2x，数值 3x */
#define LABEL_X        12u
#define LABEL_H        10u
#define LAT_LABEL_Y    108u
#define LAT_VALUE_Y    122u
#define LON_LABEL_Y    148u
#define LON_VALUE_Y    162u
#define COORD_X        12u
#define COORD_W        216u
#define COORD_H        15u

/* 海拔 / 卫星：标签与数值同一行 */
#define ALT_LABEL_Y    195u
#define ALT_VALUE_Y    190u
#define SAT_LABEL_Y    223u
#define SAT_VALUE_Y    218u
#define SIDE_VALUE_X   56u
#define SIDE_VALUE_W   172u
#define SIDE_VALUE_H   15u

/* 状态条 */
#define STATUS_X       12u
#define STATUS_Y       250u
#define STATUS_W       216u
#define STATUS_H       40u
#define STATUS_TEXT_Y  (STATUS_Y + ((STATUS_H - LABEL_H) / 2u))

/* 布局自检：改坐标宏时如果越界，编译期就报错，而不是在屏幕上画花 */
_Static_assert((COORD_X + COORD_W) <= SCREEN_W, "coordinate field exceeds screen width");
_Static_assert((SIDE_VALUE_X + SIDE_VALUE_W) <= SCREEN_W, "side value field exceeds screen width");
_Static_assert((STATUS_X + STATUS_W) <= SCREEN_W, "status bar exceeds screen width");
_Static_assert((STATUS_Y + STATUS_H) <= SCREEN_H, "status bar exceeds screen height");
_Static_assert(DISPLAY_TEXT_BUF_W >= SCREEN_W, "text buffer narrower than screen");
_Static_assert(DISPLAY_TEXT_BUF_H >= TIME_H, "text buffer lower than time field");

/* ------------------------------------------------------------------------- */
/* 内部状态                                                                    */
/* ------------------------------------------------------------------------- */

/** 字段刷新缓存：只有字符串或背景色变化时才重绘 */
typedef struct {
  char     text[24];
  uint16_t bg;
  bool     valid;
} display_field_t;

/** 状态条的文案与底色 */
typedef struct {
  const char *text;
  uint16_t    bg;
} display_status_t;

static TX_THREAD g_thread;
static uint8_t   g_stack[DISPLAY_THREAD_STACK_SIZE] __attribute__((aligned(8)));

/** 文本渲染缓冲：所有字段复用，线程独占 */
static uint8_t g_text_buf[DISPLAY_TEXT_BUF_W * DISPLAY_TEXT_BUF_H * 2u];

/* ------------------------------------------------------------------------- */
/* 绘制辅助                                                                    */
/* ------------------------------------------------------------------------- */

/** @brief 在 (x,y) 处画一行文字，背景只覆盖文字自身尺寸 */
static void display_text_at(const char *text, uint16_t x, uint16_t y,
                            uint8_t scale, uint16_t fg, uint16_t bg)
{
  uint16_t w = display_text_width(text, scale);
  uint16_t h = (uint16_t)(DISPLAY_TEXT_INK_H * scale);

  if ((w == 0u) || (h == 0u) || (w > DISPLAY_TEXT_BUF_W) || (h > DISPLAY_TEXT_BUF_H)) {
    return;
  }

  display_text_render(g_text_buf, w, h, text, scale, fg, bg, DISPLAY_TEXT_LEFT);
  ili9341_blit(x, y, w, h, g_text_buf);
}

/**
 * @brief 刷新一个动态字段（带脏检查）
 * @param field 上一次的字符串与背景色缓存
 */
static void display_field_update(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                 const char *text, uint8_t scale,
                                 uint16_t fg, uint16_t bg,
                                 display_text_align_t align,
                                 display_field_t *field)
{
  if (field->valid && (field->bg == bg) && (strcmp(field->text, text) == 0)) {
    return;   /* 内容没变，不占总线 */
  }

  if ((w == 0u) || (h == 0u) || (w > DISPLAY_TEXT_BUF_W) || (h > DISPLAY_TEXT_BUF_H)) {
    return;
  }

  display_text_render(g_text_buf, w, h, text, scale, fg, bg, align);
  ili9341_blit(x, y, w, h, g_text_buf);

  (void)strncpy(field->text, text, sizeof(field->text) - 1u);
  field->text[sizeof(field->text) - 1u] = '\0';
  field->bg    = bg;
  field->valid = true;
}

/** @brief 刷新状态条：底色变化时整条重铺，文字居中 */
static void display_status_update(const display_status_t *status, display_field_t *field)
{
  if (field->valid && (field->bg == status->bg) && (strcmp(field->text, status->text) == 0)) {
    return;
  }

  ili9341_fill_rect(STATUS_X, STATUS_Y, STATUS_W, STATUS_H, status->bg);
  display_text_render(g_text_buf, STATUS_W, LABEL_H, status->text, 2u,
                      COLOR_TEXT, status->bg, DISPLAY_TEXT_CENTER);
  ili9341_blit(STATUS_X, STATUS_TEXT_Y, STATUS_W, LABEL_H, g_text_buf);

  (void)strncpy(field->text, status->text, sizeof(field->text) - 1u);
  field->text[sizeof(field->text) - 1u] = '\0';
  field->bg    = status->bg;
  field->valid = true;
}

/** @brief 画一次静态框架：背景、标题栏、分隔线、字段标签 */
static void display_draw_static(void)
{
  ili9341_fill_rect(0u, 0u, SCREEN_W, SCREEN_H, COLOR_BG);

  /* 标题栏 */
  ili9341_fill_rect(0u, 0u, SCREEN_W, HEADER_H, COLOR_HEADER);
  display_text_at("HARDBRICK", TITLE_X, TITLE_Y, 2u, COLOR_TEXT, COLOR_HEADER);
  display_text_at("GNSS", GNSS_X, GNSS_Y, 2u, COLOR_ACCENT, COLOR_HEADER);
  ili9341_fill_rect(0u, HEADER_H, SCREEN_W, 2u, COLOR_ACCENT);

  /* 分隔线 */
  ili9341_fill_rect(DIV_X, DIV_TOP_Y, DIV_W, 1u, COLOR_DIVIDER);
  ili9341_fill_rect(DIV_X, DIV_BOTTOM_Y, DIV_W, 1u, COLOR_DIVIDER);

  /* 字段标签（静态，只画一次） */
  display_text_at("LAT", LABEL_X, LAT_LABEL_Y, 2u, COLOR_LABEL, COLOR_BG);
  display_text_at("LON", LABEL_X, LON_LABEL_Y, 2u, COLOR_LABEL, COLOR_BG);
  display_text_at("ALT", LABEL_X, ALT_LABEL_Y, 2u, COLOR_LABEL, COLOR_BG);
  display_text_at("SAT", LABEL_X, SAT_LABEL_Y, 2u, COLOR_LABEL, COLOR_BG);
}

/* ------------------------------------------------------------------------- */
/* 数据格式化                                                                  */
/* ------------------------------------------------------------------------- */

/** @brief 某年某月的天数（含闰年判断） */
static uint8_t display_days_in_month(int year, int month)
{
  static const uint8_t days[12] = {
    31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u
  };

  if ((month < 1) || (month > 12)) {
    return 31u;
  }
  if ((month == 2) &&
      ((((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0))) {
    return 29u;
  }
  return days[month - 1];
}

/**
 * @brief 把 GPS 的 UTC 日期/时间转成本地时间（UTC+8）并格式化
 * @param nav 快照
 * @param tbuf 时间输出缓冲（至少 9 字节）
 * @param tsize 时间缓冲大小
 * @param dbuf 日期输出缓冲（至少 11 字节）
 * @param dsize 日期缓冲大小
 * @note  GPS 时间是 UTC；+8 小时后可能跨天/跨月/跨年，这里一并处理。
 *        ！只处理整小时偏移，不涉及闰秒。
 */
static void display_format_datetime(const gps_nav_t *nav,
                                    char *tbuf, size_t tsize,
                                    char *dbuf, size_t dsize)
{
  int year;
  int month;
  int day;
  int hour;

  if ((nav->tick == 0u) || (nav->date.year < 0) ||
      (nav->date.month < 1) || (nav->date.month > 12) ||
      (nav->date.day < 1) || (nav->time.hours < 0)) {
    (void)snprintf(tbuf, tsize, "--:--:--");
    (void)snprintf(dbuf, dsize, "----/--/--");
    return;
  }

  year  = nav->date.year + 2000;   /* minmea 的 year 是两位 */
  month = nav->date.month;
  day   = nav->date.day;
  hour  = nav->time.hours + 8;

  if (hour >= 24) {
    hour -= 24;
    day++;
    if (day > (int)display_days_in_month(year, month)) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  }

  (void)snprintf(tbuf, tsize, "%02d:%02d:%02d",
                 hour, nav->time.minutes, nav->time.seconds);
  (void)snprintf(dbuf, dsize, "%04d-%02d-%02d", year, month, day);
}

/**
 * @brief 格式化经纬度：1e-7 度 -> "dd.dddddd N"
 * @param e7 1e-7 度，北/东为正
 * @param positive 正方向字符（N/E）
 * @param negative 负方向字符（S/W）
 */
static void display_format_coord(int32_t e7, char positive, char negative,
                                 char *buf, size_t size)
{
  char     hemi = (e7 >= 0) ? positive : negative;
  uint32_t abs_e7 = (uint32_t)((e7 >= 0) ? (int64_t)e7 : -(int64_t)e7);
  uint32_t e6 = (abs_e7 + 5u) / 10u;   /* 四舍五入到 1e-6 度 */

  (void)snprintf(buf, size, "%lu.%06lu %c",
                 (unsigned long)(e6 / 1000000u),
                 (unsigned long)(e6 % 1000000u),
                 hemi);
}

/**
 * @brief 格式化海拔：毫米 -> "12.3 m"，分辨率 0.1 m
 */
static void display_format_alt(int32_t alt_mm, bool valid, char *buf, size_t size)
{
  uint32_t abs_mm;
  uint32_t tenths;
  const char *sign = "";

  if (!valid) {
    (void)snprintf(buf, size, "--");
    return;
  }

  if (alt_mm < 0) {
    sign   = "-";
    abs_mm = (uint32_t)(-(int64_t)alt_mm);
  } else {
    abs_mm = (uint32_t)alt_mm;
  }

  tenths = (abs_mm + 50u) / 100u;   /* 四舍五入到 0.1 m */
  (void)snprintf(buf, size, "%s%lu.%01lu m", sign,
                 (unsigned long)(tenths / 10u),
                 (unsigned long)(tenths % 10u));
}

/**
 * @brief 根据快照选择状态条文案与颜色
 * @param now 当前 ThreadX tick
 */
static display_status_t display_status_of(const gps_nav_t *nav, uint32_t now)
{
  display_status_t status;

  if (nav->tick == 0u) {
    status.text = "WAITING GPS";
    status.bg   = COLOR_WAIT;
  } else if ((uint32_t)(now - nav->tick) > DISPLAY_STALE_TICKS) {
    status.text = "GPS LOST";
    status.bg   = COLOR_LOST;
  } else if (nav->fix == 0u) {
    status.text = "NO FIX";
    status.bg   = COLOR_NOFIX;
  } else if (nav->fix == 2u) {
    status.text = "FIX DGPS";
    status.bg   = COLOR_FIX;
  } else {
    status.text = "FIX SINGLE";
    status.bg   = COLOR_FIX;
  }

  return status;
}

/* ------------------------------------------------------------------------- */
/* 线程                                                                        */
/* ------------------------------------------------------------------------- */

static void display_thread_entry(ULONG thread_input)
{
  gps_nav_t       nav;
  display_field_t f_time   = {0};
  display_field_t f_date   = {0};
  display_field_t f_lat    = {0};
  display_field_t f_lon    = {0};
  display_field_t f_alt    = {0};
  display_field_t f_sat    = {0};
  display_field_t f_status = {0};
  char            tbuf[16];
  char            dbuf[16];
  char            latbuf[24];
  char            lonbuf[24];
  char            altbuf[16];
  char            satbuf[8];
  uint32_t        now;

  (void)thread_input;

  if (!ili9341_is_ready()) {
    SEGGER_RTT_WriteString(0, "[LCD] panel not ready, UI drawing disabled\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "[LCD] UI thread started\r\n");
  }

  display_draw_static();

  for (;;) {
    gps_get_nav(&nav);

    /* 时间 / 日期 */
    display_format_datetime(&nav, tbuf, sizeof(tbuf), dbuf, sizeof(dbuf));
    display_field_update(TIME_X, TIME_Y, TIME_W, TIME_H, tbuf, 3u,
                         COLOR_TEXT, COLOR_BG, DISPLAY_TEXT_CENTER, &f_time);
    display_field_update(DATE_X, DATE_Y, DATE_W, DATE_H, dbuf, 2u,
                         COLOR_DATE, COLOR_BG, DISPLAY_TEXT_CENTER, &f_date);

    /* 经纬度 */
    if (nav.pos_valid) {
      display_format_coord(nav.lat_e7, 'N', 'S', latbuf, sizeof(latbuf));
      display_format_coord(nav.lon_e7, 'E', 'W', lonbuf, sizeof(lonbuf));
    } else {
      (void)snprintf(latbuf, sizeof(latbuf), "--");
      (void)snprintf(lonbuf, sizeof(lonbuf), "--");
    }
    display_field_update(COORD_X, LAT_VALUE_Y, COORD_W, COORD_H, latbuf, 3u,
                         COLOR_TEXT, COLOR_BG, DISPLAY_TEXT_LEFT, &f_lat);
    display_field_update(COORD_X, LON_VALUE_Y, COORD_W, COORD_H, lonbuf, 3u,
                         COLOR_TEXT, COLOR_BG, DISPLAY_TEXT_LEFT, &f_lon);

    /* 海拔 / 卫星数 */
    display_format_alt(nav.alt_mm, nav.alt_valid, altbuf, sizeof(altbuf));
    if (nav.tick == 0u) {
      (void)snprintf(satbuf, sizeof(satbuf), "--");
    } else {
      (void)snprintf(satbuf, sizeof(satbuf), "%02u", (unsigned)nav.numsat);
    }
    display_field_update(SIDE_VALUE_X, ALT_VALUE_Y, SIDE_VALUE_W, SIDE_VALUE_H,
                         altbuf, 3u, COLOR_TEXT, COLOR_BG,
                         DISPLAY_TEXT_LEFT, &f_alt);
    display_field_update(SIDE_VALUE_X, SAT_VALUE_Y, SIDE_VALUE_W, SIDE_VALUE_H,
                         satbuf, 3u, COLOR_TEXT, COLOR_BG,
                         DISPLAY_TEXT_LEFT, &f_sat);

    /* 状态条 */
    now = (uint32_t)tx_time_get();
    {
      display_status_t status = display_status_of(&nav, now);
      display_status_update(&status, &f_status);
    }

    tx_thread_sleep(DISPLAY_REFRESH_TICKS);
  }
}

void display_init(void)
{
  (void)tx_thread_create(&g_thread, "display", display_thread_entry, 0u,
                         g_stack, sizeof(g_stack),
                         5u, 5u, TX_NO_TIME_SLICE, TX_AUTO_START);
}
