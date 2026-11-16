/**
 * @file    compass.c
 * @author  YangKY
 * @brief   地磁罗盘应用层实现：25 Hz 采集、硬磁自动标定、航向角计算
 *
 * 为什么用 atan2f/sqrtf 而不是自己写整数 CORDIC：本板 Cortex-M33 配了单精度
 * FPU（编译选项 -mfpu=fpv5-sp-d16 -mfloat-abi=hard，SystemInit 里已打开 CPACR），
 * 25 Hz 下每周期两次浮点运算的开销可以忽略；RTT 打印仍然只用整数（不支持 %f）。
 */

#include "compass.h"

#include "qmc5883p.h"
#include "SEGGER_RTT.h"
#include "tx_api.h"

#include <math.h>
#include <stddef.h>

/** 采集周期（内核 tick 数）：100 Hz tick ÷ 25 Hz = 4 个 tick = 40 ms */
#define COMPASS_PERIOD_TICKS (TX_TIMER_TICKS_PER_SECOND / COMPASS_SAMPLE_HZ)

/** 每采这么多个样本打印一次摘要（25 个 = 1 s） */
#define COMPASS_PRINT_EVERY 25u

/** 弧度 → 0.1° 的换算系数 */
#define COMPASS_RAD_TO_DEG10 (1800.0f / 3.14159265f)

static qmc5883p_t g_mag;
static TX_THREAD g_thread;
static uint8_t g_stack[COMPASS_THREAD_STACK_SIZE] __attribute__((aligned(8)));
static TX_MUTEX g_lock;
static compass_snapshot_t g_snap;

/** 安装角补偿，0.1° 单位；只由 compass_set_heading_offset() 写 */
static int16_t g_heading_offset_deg10;

/** 标定重置请求：写者只置位，采集线程在下一个周期处理 */
static volatile bool g_calib_reset_req;

/** compass_hw_init() 的结果；false 时线程空转（只累计 missed） */
static bool g_hw_ready;

/** 互斥量是否已创建：compass_get() 在 compass_init() 之前调用时直接返回全零快照 */
static bool g_lock_ready;

static void compass_publish(const compass_snapshot_t *snap)
{
  (void)tx_mutex_get(&g_lock, TX_WAIT_FOREVER);
  g_snap = *snap;
  (void)tx_mutex_put(&g_lock);
}

/**
 * @brief 把磁场矢量换成航向角（0.1°，0=北、顺时针增大）
 * @param x_mg 标定后 X 轴磁场
 * @param y_mg 标定后 Y 轴磁场
 * @note 水平放置时，航向角 θ 满足 Hx = |H|cosθ、Hy = -|H|sinθ，
 *       所以 θ = atan2(-Hy, Hx)；再加安装角补偿并归一化到 [0, 3600)。
 */
static int32_t compass_heading_deg10(int32_t x_mg, int32_t y_mg)
{
  float rad = atan2f(-(float)y_mg, (float)x_mg);
  int32_t deg10 = (int32_t)(rad * COMPASS_RAD_TO_DEG10);

  deg10 += (int32_t)g_heading_offset_deg10;
  deg10 %= 3600;
  if (deg10 < 0) {
    deg10 += 3600;
  }

  return deg10;
}

static void compass_thread_entry(ULONG thread_input)
{
  qmc5883p_raw_t raw;
  compass_snapshot_t snap = {0};
  int32_t min_x = 0;
  int32_t max_x = 0;
  int32_t min_y = 0;
  int32_t max_y = 0;
  int32_t min_z = 0;
  int32_t max_z = 0;
  int32_t off_x = 0;
  int32_t off_y = 0;
  int32_t off_z = 0;
  bool window_open = false;
  bool calibrated = false;
  uint32_t missed = 0u;
  uint32_t since_print = 0u;
  uint32_t sequence = 0u;

  (void)thread_input;

  for (;;) {
    if (g_calib_reset_req) {
      g_calib_reset_req = false;
      window_open = false;
      calibrated = false;
      off_x = 0;
      off_y = 0;
      off_z = 0;
    }

    /* 只读、不等：ODR 100 Hz 下 40 ms 周期里 DRDY 一定已经置位；
       万一没置位就跳过本周期，绝不在采集线程里忙等 */
    if ((!g_hw_ready) ||
        (qmc5883p_read_raw(&g_mag, &raw) != QMC5883P_OK) || (!raw.fresh)) {
      missed++;
      tx_thread_sleep(COMPASS_PERIOD_TICKS);
      continue;
    }

    {
      int32_t x = qmc5883p_raw_to_mg(&g_mag, raw.x);
      int32_t y = qmc5883p_raw_to_mg(&g_mag, raw.y);
      int32_t z = qmc5883p_raw_to_mg(&g_mag, raw.z);

      /* 硬磁标定：记录水平面内的极值，用中点当零偏。中转一圈后跨度够了才生效，
         之后继续旋转还会继续修正（温漂/安装变化都能跟上）。 */
      if (!window_open) {
        min_x = max_x = x;
        min_y = max_y = y;
        min_z = max_z = z;
        window_open = true;
      } else {
        if (x < min_x) { min_x = x; }
        if (x > max_x) { max_x = x; }
        if (y < min_y) { min_y = y; }
        if (y > max_y) { max_y = y; }
        if (z < min_z) { min_z = z; }
        if (z > max_z) { max_z = z; }
      }

      if (((max_x - min_x) >= COMPASS_CALIB_MIN_SPAN_MG) &&
          ((max_y - min_y) >= COMPASS_CALIB_MIN_SPAN_MG)) {
        off_x = (min_x + max_x) / 2;
        off_y = (min_y + max_y) / 2;
        /* Z 轴的跨度取决于倾斜程度，不够就不减，免得把 |H| 算歪 */
        off_z = ((max_z - min_z) >= COMPASS_CALIB_MIN_SPAN_MG)
                    ? ((min_z + max_z) / 2)
                    : 0;
        calibrated = true;
      }

      {
        int32_t cx = x - off_x;
        int32_t cy = y - off_y;
        int32_t cz = z - off_z;

        snap.heading_deg10 = compass_heading_deg10(cx, cy);
        snap.field_mg = (int32_t)sqrtf((float)cx * (float)cx +
                                       (float)cy * (float)cy +
                                       (float)cz * (float)cz);
      }

      snap.x_mg = (int16_t)x;
      snap.y_mg = (int16_t)y;
      snap.z_mg = (int16_t)z;
      snap.off_x_mg = (int16_t)off_x;
      snap.off_y_mg = (int16_t)off_y;
      snap.off_z_mg = (int16_t)off_z;
      snap.calibrated = calibrated;
      snap.sequence = ++sequence;
      snap.tick = tx_time_get();
      snap.missed = missed;
      compass_publish(&snap);
    }

    /* 1 Hz 摘要：25 Hz 全打会把 RTT 刷爆，也看不出趋势 */
    if (++since_print >= COMPASS_PRINT_EVERY) {
      since_print = 0u;
      SEGGER_RTT_printf(0, "[MAG] hdg=%d.%d |H|=%d mG x=%d y=%d z=%d cal=%u miss=%u\r\n",
                        (int)(snap.heading_deg10 / 10),
                        (int)(snap.heading_deg10 % 10), (int)snap.field_mg,
                        (int)snap.x_mg, (int)snap.y_mg, (int)snap.z_mg,
                        (unsigned)snap.calibrated, (unsigned)snap.missed);
    }

    tx_thread_sleep(COMPASS_PERIOD_TICKS);
  }
}

void compass_hw_init(void)
{
  static const qmc5883p_cfg_t cfg = {
      .mode = QMC5883P_MODE_NORMAL,
      .odr = QMC5883P_ODR_100HZ,
      .osr1 = QMC5883P_OSR1_8,
      .osr2 = QMC5883P_OSR2_1,
      .range = QMC5883P_RANGE_8G,
      .set_reset = QMC5883P_SET_RESET_ON,
  };

  qmc5883p_status_t st = qmc5883p_init(&g_mag, &cfg);

  g_hw_ready = (st == QMC5883P_OK);
  if (!g_hw_ready) {
    /* 具体卡在哪一步看 qmc5883p_probe() 的 g_qmc_probe.stage */
    SEGGER_RTT_printf(0, "[MAG] compass init failed, st=%d\r\n", (int)st);
    return;
  }

  SEGGER_RTT_printf(0, "[MAG] compass ready: %u Hz sampling, ODR=100Hz, range=8G\r\n",
                    (unsigned)COMPASS_SAMPLE_HZ);
}

void compass_init(void)
{
  if (tx_mutex_create(&g_lock, "compass", TX_INHERIT) != TX_SUCCESS) {
    SEGGER_RTT_WriteString(0, "[MAG] compass mutex create failed\r\n");
    return;
  }
  g_lock_ready = true;

  if (tx_thread_create(&g_thread, "compass", compass_thread_entry, 0u,
                       g_stack, sizeof(g_stack), COMPASS_THREAD_PRIORITY,
                       COMPASS_THREAD_PRIORITY, TX_NO_TIME_SLICE,
                       TX_AUTO_START) != TX_SUCCESS) {
    SEGGER_RTT_WriteString(0, "[MAG] compass thread create failed\r\n");
  }
}

void compass_get(compass_snapshot_t *out)
{
  if (out == NULL) {
    return;
  }

  /* 内核起来之前（或互斥量创建失败时）直接给零快照，避免对未创建的互斥量操作 */
  if (!g_lock_ready) {
    *out = (compass_snapshot_t){0};
    return;
  }

  (void)tx_mutex_get(&g_lock, TX_WAIT_FOREVER);
  *out = g_snap;
  (void)tx_mutex_put(&g_lock);
}

void compass_calib_reset(void)
{
  g_calib_reset_req = true;
}

void compass_set_heading_offset(int16_t deg10)
{
  g_heading_offset_deg10 = deg10;
}
