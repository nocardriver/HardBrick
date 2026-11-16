/**
 * @file    compass.c
 * @author  YangKY
 * @brief   地磁罗盘应用层：25 Hz 采样、3D 球拟合硬磁标定、航向角计算
 *
 * 为什么用 atan2f/sqrtf：本板 Cortex-M33 配了单精度 FPU（-mfpu=fpv5-sp-d16
 * -mfloat-abi=hard，SystemInit 已开 CPACR），25 Hz 下每周期两次浮点运算的开销
 * 可以忽略；RTT 打印仍然只用整数（不支持 %f）。
 *
 * 标定模型（teslabs / Li & Griffiths 2004）：h_m = A·h + b。把 A 当成各轴增益
 * 的近似缩放、忽略非正交项，测量点就落在一个球上，球的**球心就是硬磁零偏 b**、
 * 半径就是当地总场强 |B|。只要采集到的手部姿态在三维上分布足够（画 8 字、来回
 * 翻滚），球就唯一确定；这与"必须水平转一整圈"的 2D 圆拟合相比，正是手机那种
 * "随便晃几下就能标定"的做法。
 *
 * 历史教训（实测数据，别再走回头路）：
 *   - min/max 中点：只转 50° 就触发，得到 216 mG 假零偏，|H| 从 465 掉到 223；
 *   - 2D 圆拟合：转轴不竖直时 xy 轨迹是椭圆，半径被压到 125~193（真实水平分量
 *     362），圆心也偏，所以加了"Z 轴跨度 ≤60 mG"的门限；
 *   - 但手转很难保持水平，于是改成 3D 球拟合 + 三维分布（协方差特征值比）门限：
 *     板子怎么翻都行，只要姿态覆盖够。
 *
 * 软磁（各轴增益不等）本层不修正，误差通常在几度量级；需要更准就上 IMU 做倾角
 * 补偿 + 完整椭球拟合（A 的矩阵平方根）。
 */

#include "compass.h"

#include "param_store.h"
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

/** 标定窗口：25 Hz 下 512 个样本约 20 s，够画十几秒的 8 字 */
#define COMPASS_CALIB_WINDOW 512u

/** 拟合所需的最少样本，太少时 4×4 法方程病态 */
#define COMPASS_CALIB_MIN_SAMPLES 120u

/** 12 个 30° 扇区里 xy 投影至少覆盖多少个（航向只用 x/y，必须转遍平面） */
#define COMPASS_CALIB_MIN_BINS 10u

/** 拟合球半径（= 当地总场强）的合理区间，mG；地球 25000~65000 nT 即 250~650 mg */
#define COMPASS_FIELD_MIN_MG 250
#define COMPASS_FIELD_MAX_MG 750

/** 拟合残差 RMS 上限，占拟合半径的比例；球模型吸收不了软磁，给 8% 的余量 */
#define COMPASS_CALIB_RMS_RATIO 0.08

/**
 * 三维分布门限：样本协方差的最小/最大特征值之比（×1000 存进快照）。
 * 只在一个平面里转时最小特征值≈0（比值≈0），球心沿该平面法向不唯一，
 * 标定结果不可信；真正翻滚过的数据比值通常 >0.3。
 */
#define COMPASS_CALIB_MIN_SPREAD_PM 150u

/** 零偏绝对值上限，mG；超过基本都是拟合发散 */
#define COMPASS_CALIB_OFFSET_MAX 1500

/** 自动落盘门限：零偏变化超过这个值才写 Flash（避免每秒重拟合都写一次） */
#define COMPASS_STORE_MIN_DELTA_MG 20

/** 自动落盘最小间隔（ThreadX tick，100 Hz → 60000 = 10 min） */
#define COMPASS_STORE_MIN_INTERVAL 60000u

/**
 * Y 轴符号：实测平放（板子水平贴在桌面上）时 Z 读到 +350 mG 左右，
 * 而地磁竖直分量在北半球是向下的——说明传感器 +Z 指向桌面向下，
 * 于是 X→Y 从板子正面看是顺时针，航向角 = atan2(-y, x)。
 * 若换板子/Z 轴朝向不同，把这里改 0 即可。
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

/** 标定窗口（环形）：拟合数据源，调试时也可用 J-Link 整块读出来看点云 */
static compass_point_t g_window[COMPASS_CALIB_WINDOW];
static uint32_t g_window_pos;   /**< 下一个写入位置 */
static uint32_t g_window_count; /**< 已填样本数，封顶 COMPASS_CALIB_WINDOW */

/** 安装角补偿，0.1° 单位；只由 compass_set_heading_offset() 写 */
static int16_t g_heading_offset_deg10;

/** 标定重置请求：写者只置位，采集线程在下一个周期处理 */
static volatile bool g_calib_reset_req;

/** compass_hw_init() 的结果；false 时线程空转（只累计 missed） */
static bool g_hw_ready;

/** 互斥量是否已创建：compass_get() 在 compass_init() 之前调用时返回零快照 */
static bool g_lock_ready;

/** 从 Flash 恢复的标定（compass_hw_init 里填，采集线程启动时套用） */
static struct {
  bool valid;
  int16_t off_x;
  int16_t off_y;
  int16_t off_z;
  int16_t radius;
} g_saved;

/** 上一次写 Flash 的时刻，用于限流（避免每秒拟合都写一次） */
static uint32_t g_last_store_tick;

/*---------------------------------------------------------------------------*/
/* 标定结果的持久化（内部 Flash，见 BSP/ParamStore）                          */
/*---------------------------------------------------------------------------*/

/** @brief 32 位取绝对值（避免引入 stdlib 的 abs 与隐式转换告警） */
static int32_t abs32(int32_t v)
{
  return (v < 0) ? -v : v;
}

/** @brief 负载编码：4 个 int16（小端）= off_x, off_y, off_z, radius */
static void calib_encode(int32_t ox, int32_t oy, int32_t oz, int32_t radius,
                         uint8_t out[PARAM_STORE_PAYLOAD_SIZE])
{
  int32_t v[4];
  uint32_t i;

  v[0] = ox;
  v[1] = oy;
  v[2] = oz;
  v[3] = radius;

  for (i = 0u; i < 4u; i++) {
    out[i * 2u] = (uint8_t)(v[i] & 0xFF);
    out[i * 2u + 1u] = (uint8_t)((v[i] >> 8) & 0xFF);
  }
}

/** @brief 负载解码；半径明显不合理说明记录是旧的/坏的，按无效处理 */
static bool calib_decode(const uint8_t in[PARAM_STORE_PAYLOAD_SIZE], int32_t *ox,
                         int32_t *oy, int32_t *oz, int32_t *radius)
{
  int16_t v[4];
  uint32_t i;

  for (i = 0u; i < 4u; i++) {
    v[i] = (int16_t)((uint16_t)in[i * 2u] | ((uint16_t)in[i * 2u + 1u] << 8));
  }

  if ((v[3] < COMPASS_FIELD_MIN_MG) || (v[3] > COMPASS_FIELD_MAX_MG)) {
    return false;
  }

  *ox = v[0];
  *oy = v[1];
  *oz = v[2];
  *radius = v[3];

  return true;
}

/** @brief 写一条记录到 Flash 并更新内存里的副本 */
static bool calib_store(int32_t ox, int32_t oy, int32_t oz, int32_t radius)
{
  uint8_t payload[PARAM_STORE_PAYLOAD_SIZE];

  calib_encode(ox, oy, oz, radius, payload);
  if (!param_store_save(payload)) {
    return false;
  }

  g_saved.valid = true;
  g_saved.off_x = (int16_t)ox;
  g_saved.off_y = (int16_t)oy;
  g_saved.off_z = (int16_t)oz;
  g_saved.radius = (int16_t)radius;
  g_last_store_tick = tx_time_get();

  return true;
}

/**
 * @brief 标定成功后按门限决定要不要落盘
 * @note 门限：零偏变化 >20 mG，且距上次写 Flash 超过 10 min。
 *       不限流的话"每秒重拟合"会把 Flash 写爆（虽然日志式存储已经省了擦除次数）。
 */
static void calib_maybe_store(int32_t ox, int32_t oy, int32_t oz, int32_t radius)
{
  int32_t delta;

  if (g_saved.valid) {
    delta = abs32(ox - g_saved.off_x) + abs32(oy - g_saved.off_y) +
            abs32(oz - g_saved.off_z);
    if (delta < COMPASS_STORE_MIN_DELTA_MG) {
      return;
    }
    if ((tx_time_get() - g_last_store_tick) < COMPASS_STORE_MIN_INTERVAL) {
      return;
    }
  }

  (void)calib_store(ox, oy, oz, radius);
}

static void compass_publish(const compass_snapshot_t *snap)
{
  (void)tx_mutex_get(&g_lock, TX_WAIT_FOREVER);
  g_snap = *snap;
  (void)tx_mutex_put(&g_lock);
}

/*---------------------------------------------------------------------------*/
/* 线性代数小工具：3×3 行列式、4×4 高斯消元、对称 3×3 特征值                  */
/*---------------------------------------------------------------------------*/

static double det3(const double m[3][3])
{
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

/**
 * @brief 4×4 线性方程组（a·x = b），高斯消元 + 部分主元 + 回代
 * @note 直接改写传入的 a/b（调用者本来就要传副本）；奇异时返回 false
 */
static bool solve4(double a[4][4], double b[4], double x[4])
{
  int i;
  int j;
  int k;
  int piv;

  for (i = 0; i < 4; i++) {
    piv = i;
    for (j = i + 1; j < 4; j++) {
      if (fabs(a[j][i]) > fabs(a[piv][i])) {
        piv = j;
      }
    }
    if (fabs(a[piv][i]) < 1e-12) {
      return false;
    }
    if (piv != i) {
      for (k = 0; k < 4; k++) {
        double t = a[i][k];
        a[i][k] = a[piv][k];
        a[piv][k] = t;
      }
      {
        double t = b[i];
        b[i] = b[piv];
        b[piv] = t;
      }
    }
    for (j = i + 1; j < 4; j++) {
      double f = a[j][i] / a[i][i];
      for (k = i; k < 4; k++) {
        a[j][k] -= f * a[i][k];
      }
      b[j] -= f * b[i];
    }
  }

  for (i = 3; i >= 0; i--) {
    double s = b[i];
    for (j = i + 1; j < 4; j++) {
      s -= a[i][j] * x[j];
    }
    x[i] = s / a[i][i];
  }

  return true;
}

/** @brief 对称 3×3 矩阵的三个特征值（解析解，按 b 中的顺序乱序返回） */
static void eig3_sym(const double m[3][3], double e[3])
{
  double p1 = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
  double q = (m[0][0] + m[1][1] + m[2][2]) / 3.0;
  double p2 = (m[0][0] - q) * (m[0][0] - q) + (m[1][1] - q) * (m[1][1] - q) +
              (m[2][2] - q) * (m[2][2] - q) + 2.0 * p1;
  double p = sqrt(p2 / 6.0);
  double b[3][3];
  double r;
  double phi;

  if (p < 1e-12) {
    e[0] = e[1] = e[2] = q;
    return;
  }

  b[0][0] = (m[0][0] - q) / p;
  b[0][1] = m[0][1] / p;
  b[0][2] = m[0][2] / p;
  b[1][0] = b[0][1];
  b[1][1] = (m[1][1] - q) / p;
  b[1][2] = m[1][2] / p;
  b[2][0] = b[0][2];
  b[2][1] = b[1][2];
  b[2][2] = (m[2][2] - q) / p;

  r = det3(b) / 2.0;
  if (r < -1.0) {
    r = -1.0;
  } else if (r > 1.0) {
    r = 1.0;
  }

  phi = acos(r) / 3.0;
  e[0] = q + 2.0 * p * cos(phi);
  e[2] = q + 2.0 * p * cos(phi + 2.0943951023931953); /* +2π/3 */
  e[1] = 3.0 * q - e[0] - e[2];
}

/*---------------------------------------------------------------------------*/
/* 硬磁标定：三维球拟合（Kåsa 形式，带坐标归一化）                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief 用窗口内样本拟合球，返回球心（硬磁零偏）、半径与质量指标
 * @param off_x/off_y/off_z 输出：球心，mG
 * @param radius 输出：球半径（= 当地总场强），mG
 * @param rms    输出：径向残差 RMS，mG
 * @param spread_pm 输出：样本协方差最小/最大特征值之比 ×1000
 * @param bins   输出：xy 投影覆盖到的 30° 扇区数（0~12）
 * @retval true 通过全部门限，可以采纳
 * @note 模型 |m - c|² = R² 展开为 m² + D x + E y + F z + G = 0，对
 *       [D E F G] 线性最小二乘。先把数据减均值、除以标准差再拟合，
 *       否则 x² 与常数项差 5 个数量级，法方程条件数会烂掉。
 */
static bool compass_fit(int32_t *off_x, int32_t *off_y, int32_t *off_z,
                        int32_t *radius, int32_t *rms, uint16_t *spread_pm,
                        uint16_t *bins)
{
  const uint32_t n = g_window_count;
  double mx = 0.0;
  double my = 0.0;
  double mz = 0.0;
  double s = 0.0;
  double a[4][4];
  double b[4];
  double x[4];
  double cx;
  double cy;
  double cz;
  double rd;
  double acc = 0.0;
  double rmsd;
  double cov[3][3];
  double eig[3];
  double emax;
  double emin;
  uint16_t mask = 0u;
  uint32_t i;
  uint32_t j;
  uint8_t covered = 0u;

  if (n < COMPASS_CALIB_MIN_SAMPLES) {
    return false;
  }

  /* 1. 均值与整体尺度（球拟合只用单一尺度，保持球在归一化后仍是球） */
  for (i = 0u; i < n; i++) {
    mx += (double)g_window[i].x;
    my += (double)g_window[i].y;
    mz += (double)g_window[i].z;
  }
  mx /= (double)n;
  my /= (double)n;
  mz /= (double)n;

  for (i = 0u; i < n; i++) {
    double dx = (double)g_window[i].x - mx;
    double dy = (double)g_window[i].y - my;
    double dz = (double)g_window[i].z - mz;

    s += dx * dx + dy * dy + dz * dz;
  }
  s = sqrt(s / (3.0 * (double)n));
  if (s < 1.0) {
    return false; /* 数据几乎不动，没东西可拟合 */
  }

  /* 2. 归一化坐标下装配 4×4 法方程：[u²+v²+w²] + D u + E v + F w + G = 0 */
  for (i = 0u; i < 4u; i++) {
    b[i] = 0.0;
    for (j = 0u; j < 4u; j++) {
      a[i][j] = 0.0;
    }
  }
  for (i = 0u; i < 3u; i++) {
    cov[i][0] = 0.0;
    cov[i][1] = 0.0;
    cov[i][2] = 0.0;
  }

  for (i = 0u; i < n; i++) {
    double u = ((double)g_window[i].x - mx) / s;
    double v = ((double)g_window[i].y - my) / s;
    double w = ((double)g_window[i].z - mz) / s;
    double row[4];
    double rhs = -(u * u + v * v + w * w);
    uint32_t r;
    uint32_t c;

    row[0] = u;
    row[1] = v;
    row[2] = w;
    row[3] = 1.0;

    for (r = 0u; r < 4u; r++) {
      b[r] += row[r] * rhs;
      for (c = 0u; c < 4u; c++) {
        a[r][c] += row[r] * row[c];
      }
    }

    /* 同时累计协方差（判断三维分布是否够，而不是只在一个平面里转） */
    cov[0][0] += u * u;
    cov[1][1] += v * v;
    cov[2][2] += w * w;
    cov[0][1] += u * v;
    cov[1][2] += v * w;
    cov[0][2] += u * w;
  }

  if (!solve4(a, b, x)) {
    return false;
  }

  /* 3. 还原到物理量：球心与半径 */
  cx = -x[0] * 0.5;
  cy = -x[1] * 0.5;
  cz = -x[2] * 0.5;
  {
    double r2 = cx * cx + cy * cy + cz * cz - x[3];

    if (!(r2 > 0.0)) {
      return false;
    }
    rd = sqrt(r2) * s;
  }

  cx = cx * s + mx;
  cy = cy * s + my;
  cz = cz * s + mz;

  /* 4. 残差、xy 覆盖、三维分布 */
  for (i = 0u; i < n; i++) {
    double dx = (double)g_window[i].x - cx;
    double dy = (double)g_window[i].y - cy;
    double dz = (double)g_window[i].z - cz;
    double err = sqrt(dx * dx + dy * dy + dz * dz) - rd;
    double ang = atan2(dy, dx);
    int bin = (int)((ang + 3.14159265) * (12.0 / 6.28318531));

    acc += err * err;

    if (bin < 0) {
      bin = 0;
    } else if (bin > 11) {
      bin = 11;
    }
    mask = (uint16_t)(mask | (uint16_t)(1u << (unsigned)bin));
  }

  rmsd = sqrt(acc / (double)n);

  for (i = 0u; i < 12u; i++) {
    if ((mask & (uint16_t)(1u << i)) != 0u) {
      covered++;
    }
  }

  cov[0][0] /= (double)n;
  cov[1][1] /= (double)n;
  cov[2][2] /= (double)n;
  cov[0][1] /= (double)n;
  cov[1][2] /= (double)n;
  cov[0][2] /= (double)n;
  cov[1][0] = cov[0][1];
  cov[2][1] = cov[1][2];
  cov[2][0] = cov[0][2];

  eig3_sym(cov, eig);
  emax = eig[0];
  emin = eig[0];
  for (i = 1u; i < 3u; i++) {
    if (eig[i] > emax) {
      emax = eig[i];
    }
    if (eig[i] < emin) {
      emin = eig[i];
    }
  }
  *spread_pm = (emax > 1e-9) ? (uint16_t)(1000.0 * emin / emax) : 0u;

  /* 诊断量先填：即使门限不过，调用者也能看到"差在哪一项" */
  *off_x = (int32_t)cx;
  *off_y = (int32_t)cy;
  *off_z = (int32_t)cz;
  *radius = (int32_t)rd;
  *rms = (int32_t)rmsd;
  *bins = (uint16_t)covered;

  if ((rd < (double)COMPASS_FIELD_MIN_MG) || (rd > (double)COMPASS_FIELD_MAX_MG)) {
    return false;
  }
  if ((fabs(cx) > (double)COMPASS_CALIB_OFFSET_MAX) ||
      (fabs(cy) > (double)COMPASS_CALIB_OFFSET_MAX) ||
      (fabs(cz) > (double)COMPASS_CALIB_OFFSET_MAX)) {
    return false;
  }
  if ((rmsd > (COMPASS_CALIB_RMS_RATIO * rd)) ||
      (covered < COMPASS_CALIB_MIN_BINS) ||
      (*spread_pm < COMPASS_CALIB_MIN_SPREAD_PM)) {
    return false;
  }

  return true;
}

/**
 * @brief 把磁场矢量换成航向角（0.1°，0=北、顺时针增大）
 * @param x_mg 标定后 X 轴磁场
 * @param y_mg 标定后 Y 轴磁场
 * @note 航向角是"板子 +X 轴相对磁北的方位"；本板传感器 +Z 朝下（实测平放时
 *       z≈+350 mG），所以 X→Y 从板子正面看是顺时针，θ = atan2(-Hy, Hx)。
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
  int32_t off_z = 0;
  int32_t radius = 0;
  int32_t fit_rms = 0;
  uint16_t fit_bins = 0u;
  uint16_t fit_spread = 0u;
  bool calibrated = false;
  uint32_t missed = 0u;
  uint32_t sequence = 0u;
  uint32_t tick_div = 0u;

  (void)thread_input;

  /* 上次标定过就直接用，不必每次上电重新画 8 字 */
  if (g_saved.valid) {
    off_x = g_saved.off_x;
    off_y = g_saved.off_y;
    off_z = g_saved.off_z;
    radius = g_saved.radius;
    calibrated = true;
  }

  for (;;) {
    if (g_calib_reset_req) {
      g_calib_reset_req = false;
      g_window_pos = 0u;
      g_window_count = 0u;
      calibrated = false;
      off_x = 0;
      off_y = 0;
      off_z = 0;
      radius = 0;
      fit_rms = 0;
      fit_bins = 0u;
      fit_spread = 0u;
    }

    /* 只读、不等：ODR 200 Hz（有效 50 Hz）下 40 ms 周期里 DRDY 一定已经置位；
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
      int32_t cz;
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

      /* 每秒拟合一次：姿态覆盖够了就会通过门限，之后继续修正（温漂/重新安装） */
      if (++tick_div >= COMPASS_PRINT_EVERY) {
        int32_t new_rms = 0;
        int32_t new_radius = 0;
        int32_t new_ox = 0;
        int32_t new_oy = 0;
        int32_t new_oz = 0;
        uint16_t new_bins = 0u;
        uint16_t new_spread = 0u;
        bool fit_ok;

        tick_div = 0u;
        do_print = true;

        fit_ok = compass_fit(&new_ox, &new_oy, &new_oz, &new_radius, &new_rms,
                             &new_spread, &new_bins);

        /* 诊断量无论成败都更新：失败了也能从 RTT 里看出是哪一项没达标 */
        radius = new_radius;
        fit_rms = new_rms;
        fit_bins = new_bins;
        fit_spread = new_spread;

        if (fit_ok) {
          off_x = new_ox;
          off_y = new_oy;
          off_z = new_oz;
          calibrated = true;
          calib_maybe_store(new_ox, new_oy, new_oz, new_radius);
        }
      }

      cx = x - off_x;
      cy = y - off_y;
      cz = z - off_z;

      snap.heading_deg10 = compass_heading_deg10(cx, cy);
      snap.field_mg = (int32_t)sqrtf((float)cx * (float)cx +
                                     (float)cy * (float)cy +
                                     (float)cz * (float)cz);
      snap.x_mg = (int16_t)x;
      snap.y_mg = (int16_t)y;
      snap.z_mg = (int16_t)z;
      snap.off_x_mg = (int16_t)off_x;
      snap.off_y_mg = (int16_t)off_y;
      snap.off_z_mg = (int16_t)off_z;
      snap.radius_mg = radius;
      snap.fit_rms_mg = fit_rms;
      snap.spread_pm = fit_spread;
      snap.coverage_bins = fit_bins;
      snap.calib_samples = g_window_count;
      snap.calibrated = calibrated;
      snap.sequence = ++sequence;
      snap.tick = tx_time_get();
      snap.missed = missed;
      compass_publish(&snap);

      /* 1 Hz 摘要：25 Hz 全打会把 RTT 刷爆，也看不出趋势 */
      if (do_print) {
        SEGGER_RTT_printf(0,
                          "[MAG] hdg=%d.%d |B|=%d x=%d y=%d z=%d cal=%u off=%d/%d/%d r=%d rms=%d spr=%u cov=%u\r\n",
                          (int)(snap.heading_deg10 / 10),
                          (int)(snap.heading_deg10 % 10), (int)snap.field_mg,
                          (int)snap.x_mg, (int)snap.y_mg, (int)snap.z_mg,
                          (unsigned)snap.calibrated, (int)snap.off_x_mg,
                          (int)snap.off_y_mg, (int)snap.off_z_mg,
                          (int)snap.radius_mg, (int)snap.fit_rms_mg,
                          (unsigned)snap.spread_pm, (unsigned)snap.coverage_bins);
      }
    }

    tx_thread_sleep(COMPASS_PERIOD_TICKS);
  }
}

void compass_hw_init(void)
{
  /*
   * OSR2=4 倍降采样、ODR=200 Hz：PX4 在同一个器件上实测发现，默认的 OSR2=1
   * 会让 Z 轴静止噪声超过 50 mG（PX4 PR #26350 就是为这个改的）。有效输出率
   * 200/4 = 50 Hz，仍然高于我们 25 Hz 的采样率，不会读到重复数据。
   */
  static const qmc5883p_cfg_t cfg = {
      .mode = QMC5883P_MODE_NORMAL,
      .odr = QMC5883P_ODR_200HZ,
      .osr1 = QMC5883P_OSR1_8,
      .osr2 = QMC5883P_OSR2_4,
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

  SEGGER_RTT_printf(0, "[MAG] compass ready: %u Hz sampling, ODR=200Hz/OSR2=4, range=8G\r\n",
                    (unsigned)COMPASS_SAMPLE_HZ);

  /* 从 Flash 恢复上次标定，避免每次上电都重新画 8 字 */
  if (!param_store_init()) {
    SEGGER_RTT_WriteString(0, "[MAG] param store unavailable, calibration not persisted\r\n");
    return;
  }

  {
    uint8_t payload[PARAM_STORE_PAYLOAD_SIZE];
    int32_t ox = 0;
    int32_t oy = 0;
    int32_t oz = 0;
    int32_t r = 0;

    if (param_store_load(payload) && calib_decode(payload, &ox, &oy, &oz, &r)) {
      g_saved.valid = true;
      g_saved.off_x = (int16_t)ox;
      g_saved.off_y = (int16_t)oy;
      g_saved.off_z = (int16_t)oz;
      g_saved.radius = (int16_t)r;
      SEGGER_RTT_printf(0, "[MAG] calibration from flash: off=%d/%d/%d r=%d\r\n",
                        (int)g_saved.off_x, (int)g_saved.off_y,
                        (int)g_saved.off_z, (int)g_saved.radius);
    } else {
      SEGGER_RTT_WriteString(0, "[MAG] no stored calibration, draw a figure-8 once\r\n");
    }
  }
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

bool compass_calib_save(void)
{
  compass_snapshot_t snap;
  uint8_t payload[PARAM_STORE_PAYLOAD_SIZE];

  if (!g_lock_ready) {
    return false;
  }

  compass_get(&snap);
  if (!snap.calibrated) {
    return false;
  }

  calib_encode(snap.off_x_mg, snap.off_y_mg, snap.off_z_mg, snap.radius_mg,
               payload);

  if (!param_store_save(payload)) {
    return false;
  }

  g_saved.valid = true;
  g_saved.off_x = snap.off_x_mg;
  g_saved.off_y = snap.off_y_mg;
  g_saved.off_z = snap.off_z_mg;
  g_saved.radius = (int16_t)snap.radius_mg;

  return true;
}

bool compass_calib_clear(void)
{
  g_saved.valid = false;
  g_calib_reset_req = true;

  return param_store_clear();
}

void compass_set_heading_offset(int16_t deg10)
{
  g_heading_offset_deg10 = deg10;
}
