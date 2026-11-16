/**
 * @file    compass.c
 * @author  YangKY
 * @brief   地磁罗盘应用层实现：25 Hz 采集、硬磁圆拟合标定、航向角计算
 *
 * 为什么用 atan2f/sqrtf 而不是自己写整数 CORDIC：本板 Cortex-M33 配了单精度
 * FPU（编译选项 -mfpu=fpv5-sp-d16 -mfloat-abi=hard，SystemInit 里已打开 CPACR），
 * 25 Hz 下每周期两次浮点运算的开销可以忽略；RTT 打印仍然只用整数（不支持 %f）。
 *
 * 标定为什么用圆拟合而不是 min/max 中点：只转一小段角度时，min/max 的"中点"
 * 并不等于圆心。实测在 50° 的圆弧上取中点会得到 216 mG 的假零偏，|H| 从 465 mG
 * 掉到 223 mG，航向角整体歪掉且随角度变化。这里改成对最近 256 个样本做 Kåsa
 * 圆拟合：圆心 = 硬磁零偏、半径 = 水平场强、残差 RMS = 圆度；只有"覆盖 12 个
 * 30° 扇区里的 10 个以上 + 半径/残差/零偏都合理"时才采纳，转不够就保持未标定。
 */

#include "compass.h"

#include "qmc5883p.h"
#include "SEGGER_RTT.h"
#include "tx_api.h"

#include <math.h>
#include <stddef.h>

/** 采集周期（内核 tick 数）：100 Hz tick ÷ 25 Hz = 4 个 tick = 40 ms */
#define COMPASS_PERIOD_TICKS (TX_TIMER_TICKS_PER_SECOND / COMPASS_SAMPLE_HZ)

/** 每采这么多个样本做一次拟合并打印摘要（25 个 = 1 s） */
#define COMPASS_PRINT_EVERY 25u

/** 弧度 → 0.1° 的换算系数 */
#define COMPASS_RAD_TO_DEG10 (1800.0f / 3.14159265f)

/** 标定窗口：25 Hz 下 512 个样本约 20 s，够慢慢转一整圈，也便于事后回溯点云 */
#define COMPASS_CALIB_WINDOW 512u

/** 拟合所需的最少样本（2.4 s），太少时 3×3 法方程病态 */
#define COMPASS_CALIB_MIN_SAMPLES 60u

/** 12 个 30° 扇区里至少覆盖多少个才算"转够一圈"；10 个 = 300° */
#define COMPASS_CALIB_MIN_BINS 10u

/** 拟合半径（水平场强）的合理区间，mG；上海约 350~370，留宽一点防误判 */
#define COMPASS_CALIB_RADIUS_MIN 100
#define COMPASS_CALIB_RADIUS_MAX 1000

/** 拟合残差 RMS 上限，mG；超过说明软磁/倾斜让点云不是圆 */
#define COMPASS_CALIB_RMS_MAX 15

/** 零偏绝对值上限，mG；超过基本都是拟合发散 */
#define COMPASS_CALIB_OFFSET_MAX 1500

/**
 * 标定期间 Z 轴允许的跨度，mG。
 * 绕竖直轴转时（哪怕板子本身是斜的），地磁的竖直分量不变、xy 轨迹仍是正圆，
 * 标定依然有效；但手持旋转常常边转边上下翻，z 大范围变化会让 xy 轨迹被压扁，
 * 拟合出的半径和圆心都不对（实测手持转一圈 z 跨度 158 mG，半径被压到 193，
 * 而真实水平分量约 340）。超过这个门限就拒绝采纳。
 */
#define COMPASS_CALIB_Z_SPAN_MAX 60

/**
 * Y 轴符号：芯片 Z 轴朝上（顶面朝上）且 X-Y-Z 右手系时，航向角 = atan2(+y, x)；
 * 模块把芯片贴在板子背面（Z 朝下）时同一旋转方向下航向会整体反向，要取负。
 * 判定：从上往下看把板子顺时针转 90°，hdg 应增加约 90°；若减少约 90° 就改成 1。
 */
#define COMPASS_Y_INVERT 1

typedef struct {
  int16_t x; /**< X 轴磁场，mG（未减零偏） */
  int16_t y; /**< Y 轴磁场，mG */
  int16_t z; /**< Z 轴磁场，mG */
} compass_point_t;

static qmc5883p_t g_mag;
static TX_THREAD g_thread;
static uint8_t g_stack[COMPASS_THREAD_STACK_SIZE] __attribute__((aligned(8)));
static TX_MUTEX g_lock;
static compass_snapshot_t g_snap;

/** 标定窗口（环形）：既用来拟合，也可以在调试时用 J-Link 整块读出来看点云 */
static compass_point_t g_window[COMPASS_CALIB_WINDOW];
static uint32_t g_window_pos;   /**< 下一个写入位置 */
static uint32_t g_window_count; /**< 已填样本数，封顶 COMPASS_CALIB_WINDOW */

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

/*---------------------------------------------------------------------------*/
/* 三阶线性方程组（Cramer 法则），只给圆拟合用                                */
/*---------------------------------------------------------------------------*/

static double det3(const double m[3][3])
{
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

static bool solve3(const double m[3][3], const double b[3], double out[3])
{
  double det = det3(m);
  double tmp[3][3];
  uint8_t col;
  uint8_t row;

  if (fabs(det) < 1e-6) {
    return false;
  }

  for (col = 0u; col < 3u; col++) {
    for (row = 0u; row < 3u; row++) {
      tmp[row][0] = m[row][0];
      tmp[row][1] = m[row][1];
      tmp[row][2] = m[row][2];
      tmp[row][col] = b[row];
    }
    out[col] = det3(tmp) / det;
  }

  return true;
}

/*---------------------------------------------------------------------------*/
/* 硬磁标定：对窗口内样本做 Kåsa 圆拟合                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief 用窗口内样本拟合圆，返回圆心（硬磁零偏）、半径与质量指标
 * @param cx     输出：圆心 X（零偏），mG
 * @param cy     输出：圆心 Y，mG
 * @param radius 输出：拟合半径（水平场强），mG
 * @param rms    输出：径向残差 RMS，mG
 * @param bins   输出：覆盖到的 30° 扇区数（0~12）
 * @retval true 拟合结果通过全部质量门限，可以采纳
 * @note 模型：x²+y²+Dx+Ey+F=0，即圆心 (-D/2, -E/2)、半径² = (D²+E²)/4 - F；
 *       对 [Sxx Sxy Sx; Sxy Syy Sy; Sx Sy n]·[D E F]ᵀ = -[Σxw Σyw Σw] 求解
 *       （w = x²+y²）。
 */
static bool compass_fit(int32_t *cx, int32_t *cy, int32_t *radius, int32_t *rms,
                        uint16_t *bins, int8_t *rot, int16_t *z_span)
{
  const uint32_t n = g_window_count;
  double m[3][3];
  double b[3];
  double sol[3];
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;
  double sw = 0.0;
  double sxw = 0.0;
  double syw = 0.0;
  double cxd;
  double cyd;
  double rd;
  double acc = 0.0;
  double rmsd;
  int16_t zmin = 32767;
  int16_t zmax = -32768;
  uint16_t mask = 0u;
  uint32_t i;
  uint8_t covered = 0u;

  if (n < COMPASS_CALIB_MIN_SAMPLES) {
    return false;
  }

  for (i = 0u; i < n; i++) {
    double x = (double)g_window[i].x;
    double y = (double)g_window[i].y;
    double w = x * x + y * y;

    sx += x;
    sy += y;
    sxx += x * x;
    syy += y * y;
    sxy += x * y;
    sw += w;
    sxw += x * w;
    syw += y * w;

    if (g_window[i].z < zmin) {
      zmin = g_window[i].z;
    }
    if (g_window[i].z > zmax) {
      zmax = g_window[i].z;
    }
  }

  /* 转的圈里只要板子上下翻过，xy 轨迹就不是正圆，标定结果不可信（门限在末尾统一判） */
  *z_span = (int16_t)(zmax - zmin);

  m[0][0] = sxx; m[0][1] = sxy; m[0][2] = sx;
  m[1][0] = sxy; m[1][1] = syy; m[1][2] = sy;
  m[2][0] = sx;  m[2][1] = sy;  m[2][2] = (double)n;
  b[0] = -sxw;
  b[1] = -syw;
  b[2] = -sw;

  if (!solve3(m, b, sol)) {
    return false;
  }

  cxd = -sol[0] * 0.5;
  cyd = -sol[1] * 0.5;

  {
    double r2 = cxd * cxd + cyd * cyd - sol[2];

    if (!(r2 > 0.0)) {
      return false;
    }
    rd = sqrt(r2);
  }

  if ((rd < (double)COMPASS_CALIB_RADIUS_MIN) ||
      (rd > (double)COMPASS_CALIB_RADIUS_MAX)) {
    return false;
  }
  if ((fabs(cxd) > (double)COMPASS_CALIB_OFFSET_MAX) ||
      (fabs(cyd) > (double)COMPASS_CALIB_OFFSET_MAX)) {
    return false;
  }

  /* 第二遍：残差 + 角度覆盖率 + 旋转方向（按时间顺序算有向面积）。
     有向面积 >0 表示磁场矢量在传感器坐标系里逆时针转，也就是板子顺时针转
     （Z 轴朝上时）——用来判定 COMPASS_Y_INVERT 该取哪一个。 */
  {
    const uint32_t start = (n == COMPASS_CALIB_WINDOW) ? g_window_pos : 0u;
    uint32_t prev = 0u;
    double area = 0.0;

    for (i = 0u; i < n; i++) {
      uint32_t idx = (start + i) % COMPASS_CALIB_WINDOW;
      double dx = (double)g_window[idx].x - cxd;
      double dy = (double)g_window[idx].y - cyd;
      double err = sqrt(dx * dx + dy * dy) - rd;
      double ang = atan2(dy, dx);
      int bin = (int)((ang + 3.14159265) * (12.0 / 6.28318531));

      acc += err * err;

      if (bin < 0) {
        bin = 0;
      } else if (bin > 11) {
        bin = 11;
      }
      mask = (uint16_t)(mask | (uint16_t)(1u << (unsigned)bin));

      if (i > 0u) {
        double px = (double)g_window[prev].x - cxd;
        double py = (double)g_window[prev].y - cyd;

        area += px * dy - py * dx;
      }
      prev = idx;
    }

    /* 总转角不到约 11° 时面积没有意义，直接报 0 */
    if (fabs(area) < 0.2 * rd * rd) {
      *rot = 0;
    } else {
      *rot = (area > 0.0) ? 1 : -1;
    }
  }

  rmsd = sqrt(acc / (double)n);

  for (i = 0u; i < 12u; i++) {
    if ((mask & (uint16_t)(1u << i)) != 0u) {
      covered++;
    }
  }

  /* 先把诊断量填出去：即使门限不过，调用者也能从 RTT/J-Link 看到"差在哪一项" */
  *cx = (int32_t)cxd;
  *cy = (int32_t)cyd;
  *radius = (int32_t)rd;
  *rms = (int32_t)rmsd;
  *bins = (uint16_t)covered;

  if (((int32_t)*z_span > COMPASS_CALIB_Z_SPAN_MAX) ||
      (rmsd > (double)COMPASS_CALIB_RMS_MAX) ||
      (covered < COMPASS_CALIB_MIN_BINS)) {
    return false;
  }

  return true;
}

/**
 * @brief 把磁场矢量换成航向角（0.1°，0=北、顺时针增大）
 * @param x_mg 标定后 X 轴磁场
 * @param y_mg 标定后 Y 轴磁场
 * @note 水平放置时，航向角 θ 满足 Hx = |H|cosθ、Hy = -|H|sinθ（Y 轴朝左、Z 轴朝上），
 *       所以 θ = atan2(-Hy, Hx)；本板轴向与上述不一致时用 COMPASS_Y_INVERT 翻符号。
 *       再加安装角补偿并归一化到 [0, 3600)。
 */
static int32_t compass_heading_deg10(int32_t x_mg, int32_t y_mg)
{
  float rad;
  int32_t deg10;

#if COMPASS_Y_INVERT
  rad = atan2f(-(float)y_mg, (float)x_mg);
#else
  rad = atan2f((float)y_mg, (float)x_mg);
#endif

  deg10 = (int32_t)(rad * COMPASS_RAD_TO_DEG10);
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
  int32_t off_x = 0;
  int32_t off_y = 0;
  int32_t radius = 0;
  int32_t fit_rms = 0;
  uint16_t fit_bins = 0u;
  int8_t rot_sign = 0;
  int16_t z_span = 0;
  bool calibrated = false;
  uint32_t missed = 0u;
  uint32_t sequence = 0u;
  uint32_t tick_div = 0u;

  (void)thread_input;

  for (;;) {
    if (g_calib_reset_req) {
      g_calib_reset_req = false;
      g_window_pos = 0u;
      g_window_count = 0u;
      calibrated = false;
      off_x = 0;
      off_y = 0;
      radius = 0;
      fit_rms = 0;
      fit_bins = 0u;
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
      int32_t cx;
      int32_t cy;
      bool do_print = false;

      /* 进窗口（环形覆盖）：既是标定数据源，也是调试用的原始点云 */
      g_window[g_window_pos].x = (int16_t)x;
      g_window[g_window_pos].y = (int16_t)y;
      g_window[g_window_pos].z = (int16_t)z;
      g_window_pos++;
      if (g_window_pos >= COMPASS_CALIB_WINDOW) {
        g_window_pos = 0u;
      }
      if (g_window_count < COMPASS_CALIB_WINDOW) {
        g_window_count++;
      }

      /* 每秒拟合一次：转够一圈就会通过门限，之后继续修正（温漂/重新安装） */
      if (++tick_div >= COMPASS_PRINT_EVERY) {
        int32_t new_rms = 0;
        int32_t new_radius = 0;
        uint16_t new_bins = 0u;
        int8_t new_rot = 0;
        int16_t new_zspan = 0;
        bool fit_ok;

        tick_div = 0u;
        do_print = true;

        fit_ok = compass_fit(&cx, &cy, &new_radius, &new_rms, &new_bins,
                             &new_rot, &new_zspan);

        /* 诊断量无论成败都更新：失败了也能从 RTT 里看出是哪一项没达标 */
        radius = new_radius;
        fit_rms = new_rms;
        fit_bins = new_bins;
        z_span = new_zspan;
        rot_sign = new_rot;

        if (fit_ok) {
          off_x = cx;
          off_y = cy;
          calibrated = true;
        }
      }

      cx = x - off_x;
      cy = y - off_y;

      snap.heading_deg10 = compass_heading_deg10(cx, cy);
      snap.field_mg = (int32_t)sqrtf((float)cx * (float)cx +
                                     (float)cy * (float)cy +
                                     (float)z * (float)z);
      snap.x_mg = (int16_t)x;
      snap.y_mg = (int16_t)y;
      snap.z_mg = (int16_t)z;
      snap.off_x_mg = (int16_t)off_x;
      snap.off_y_mg = (int16_t)off_y;
      snap.radius_mg = radius;
      snap.fit_rms_mg = fit_rms;
      snap.coverage_bins = fit_bins;
      snap.rotation_sign = rot_sign;
      snap.z_span_mg = z_span;
      snap.calib_samples = g_window_count;
      snap.calibrated = calibrated;
      snap.sequence = ++sequence;
      snap.tick = tx_time_get();
      snap.missed = missed;
      compass_publish(&snap);

      /* 1 Hz 摘要：25 Hz 全打会把 RTT 刷爆，也看不出趋势 */
      if (do_print) {
        SEGGER_RTT_printf(0,
                          "[MAG] hdg=%d.%d |H|=%d x=%d y=%d z=%d cal=%u off=%d/%d r=%d rms=%d cov=%u zsp=%d rot=%d\r\n",
                          (int)(snap.heading_deg10 / 10),
                          (int)(snap.heading_deg10 % 10), (int)snap.field_mg,
                          (int)snap.x_mg, (int)snap.y_mg, (int)snap.z_mg,
                          (unsigned)snap.calibrated, (int)snap.off_x_mg,
                          (int)snap.off_y_mg, (int)snap.radius_mg,
                          (int)snap.fit_rms_mg, (unsigned)snap.coverage_bins,
                          (int)snap.z_span_mg, (int)snap.rotation_sign);
      }
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
