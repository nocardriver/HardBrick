/**
 * @file    qmc5883p.c
 * @author  YangKY
 * @brief   QMC5883P 三轴地磁传感器驱动核心实现（BSP 层）
 *
 * 只做"寄存器协议 + 单位换算"，不 include 任何 HAL 头文件：
 * I2C 事务、毫秒计时与延时统一走 qmc5883p_port.c，换 MCU 时只需重写那一层。
 *
 * 关键时序（数据手册 Rev.E）：
 *   - POR 后默认 Suspend，只有写 0x0A[MODE] 才开始测量；
 *   - Normal/Single/Continuous 之间切换要先经过 Suspend（§9.2.3）；
 *   - 状态寄存器 0x09 读一次就把 DRDY/OVFL 清零，所以"先读状态、再读数据"，
 *     且状态只读一次（读两次会把 DRDY 读丢）；
 *   - 数据 0x01~0x06 必须一次连读，分轴读会跨采样点。
 */

#include "qmc5883p.h"

#include "qmc5883p_port.h"

#include <stddef.h>

/*---------------------------------------------------------------------------*/
/* 寄存器地址与位定义（§9.1 寄存器表）                                        */
/*---------------------------------------------------------------------------*/

#define QMC_REG_CHIP_ID   0x00u /**< CHIPID，固定 0x80 */
#define QMC_REG_DATA_X_LSB 0x01u /**< 数据区首地址，连读 6 字节 = X/Y/Z 小端 */
#define QMC_REG_DATA_LEN  6u    /**< X/Y/Z 各 2 字节 */
#define QMC_REG_STATUS    0x09u /**< bit0 DRDY、bit1 OVFL，读后清零 */
#define QMC_REG_CTRL1     0x0Au /**< OSR2[7:6] OSR1[5:4] ODR[3:2] MODE[1:0] */
#define QMC_REG_CTRL2     0x0Bu /**< SOFT_RST[7] SELF_TEST[6] RNG[3:2] SET/RESET[1:0] */
#define QMC_REG_SIGN      0x29u /**< 三轴符号定义，手册示例固定写 0x06 */

#define QMC_STATUS_DRDY   (1u << 0)
#define QMC_STATUS_OVFL   (1u << 1)

#define QMC_CTRL2_SOFT_RST  (1u << 7)
#define QMC_CTRL2_SELF_TEST (1u << 6)

/** 手册 §7.1 示例：三轴符号寄存器固定写 0x06 */
#define QMC_SIGN_VALUE 0x06u

/** 切模式时插入 Suspend 后的等待（远小于 ODR 周期，只是让模拟前端停稳） */
#define QMC_MODE_SWITCH_DELAY_MS 1u

/** 轮询 DRDY 的间隔 */
#define QMC_POLL_INTERVAL_MS 1u

/** 自检打开内部激励后等待新一轮数据的时间（§7.3 给的是 5 ms） */
#define QMC_SELFTEST_DELAY_MS 5u

/*---------------------------------------------------------------------------*/
/* 内部工具                                                                  */
/*---------------------------------------------------------------------------*/

static qmc5883p_status_t io_fail(qmc5883p_t *inst)
{
  inst->io_errors++;
  return QMC5883P_ERR_IO;
}

static qmc5883p_status_t reg_write(qmc5883p_t *inst, uint8_t reg, uint8_t val)
{
  if (!qmc5883p_port_i2c_write(reg, &val, 1u)) {
    return io_fail(inst);
  }

  return QMC5883P_OK;
}

/** 控制寄存器 1：OSR2[7:6] OSR1[5:4] ODR[3:2] MODE[1:0] */
static uint8_t ctrl1_byte(const qmc5883p_cfg_t *cfg, qmc5883p_mode_t mode)
{
  return (uint8_t)(((uint8_t)cfg->osr2 << 6) | ((uint8_t)cfg->osr1 << 4) |
                   ((uint8_t)cfg->odr << 2) | (uint8_t)mode);
}

/** 控制寄存器 2：RNG[3:2] SET/RESET[1:0]（SOFT_RST / SELF_TEST 平时为 0） */
static uint8_t ctrl2_byte(const qmc5883p_cfg_t *cfg)
{
  return (uint8_t)(((uint8_t)cfg->range << 2) | (uint8_t)cfg->set_reset);
}

static bool cfg_valid(const qmc5883p_cfg_t *cfg)
{
  return ((unsigned)cfg->mode <= 3u) && ((unsigned)cfg->odr <= 3u) &&
         ((unsigned)cfg->osr1 <= 3u) && ((unsigned)cfg->osr2 <= 3u) &&
         ((unsigned)cfg->range <= 3u) && ((unsigned)cfg->set_reset <= 3u);
}

/**
 * @brief 连读 6 字节数据区并填 out
 * @param fresh  本次读数是否来自刚置位的 DRDY（由调用者从状态寄存器带进来）
 * @param ovfl   同一次状态读取里的 OVFL 位
 * @note 不自己读状态寄存器：DRDY 读一次就清零，状态必须由调用者只读一次。
 */
static qmc5883p_status_t read_burst(qmc5883p_t *inst, qmc5883p_raw_t *out,
                                    bool fresh, bool ovfl)
{
  uint8_t buf[QMC_REG_DATA_LEN];

  if (!qmc5883p_port_i2c_read(QMC_REG_DATA_X_LSB, buf, QMC_REG_DATA_LEN)) {
    return io_fail(inst);
  }

  /* 小端、16 位补码：0x01=X_LSB 0x02=X_MSB ... 0x06=Z_MSB */
  out->x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
  out->y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
  out->z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
  out->fresh = fresh;
  out->overflow = ovfl;

  if (ovfl) {
    inst->overflows++;
  }
  inst->samples++;

  return QMC5883P_OK;
}

/*---------------------------------------------------------------------------*/
/* 生命周期与配置                                                            */
/*---------------------------------------------------------------------------*/

qmc5883p_status_t qmc5883p_read_chip_id(qmc5883p_t *inst, uint8_t *id)
{
  if ((inst == NULL) || (id == NULL)) {
    return QMC5883P_ERR_PARAM;
  }

  if (!qmc5883p_port_i2c_read(QMC_REG_CHIP_ID, id, 1u)) {
    return io_fail(inst);
  }

  return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_soft_reset(qmc5883p_t *inst)
{
  qmc5883p_status_t st;

  if (inst == NULL) {
    return QMC5883P_ERR_PARAM;
  }

  st = reg_write(inst, QMC_REG_CTRL2, QMC_CTRL2_SOFT_RST);
  if (st != QMC5883P_OK) {
    return st;
  }

  /* 复位后所有寄存器回默认值、器件回到 Suspend：缓存里的配置已经作废 */
  inst->ready = false;
  qmc5883p_port_delay_ms(QMC5883P_RESET_DELAY_MS);

  return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_apply_config(qmc5883p_t *inst,
                                        const qmc5883p_cfg_t *cfg)
{
  qmc5883p_status_t st;

  if ((inst == NULL) || (cfg == NULL) || !cfg_valid(cfg)) {
    return QMC5883P_ERR_PARAM;
  }
  if (!inst->ready) {
    return QMC5883P_ERR_STATE;
  }

  /* 改量程/滤波时先落回 Suspend：避免器件带着旧量程继续往数据寄存器写 */
  st = reg_write(inst, QMC_REG_CTRL1,
                 ctrl1_byte(&inst->cfg, QMC5883P_MODE_SUSPEND));
  if (st != QMC5883P_OK) {
    return st;
  }
  qmc5883p_port_delay_ms(QMC_MODE_SWITCH_DELAY_MS);

  st = reg_write(inst, QMC_REG_CTRL2, ctrl2_byte(cfg));
  if (st != QMC5883P_OK) {
    return st;
  }

  st = reg_write(inst, QMC_REG_CTRL1, ctrl1_byte(cfg, cfg->mode));
  if (st != QMC5883P_OK) {
    return st;
  }

  inst->cfg = *cfg;
  inst->lsb_per_gauss = qmc5883p_range_sensitivity(cfg->range);

  return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_set_mode(qmc5883p_t *inst, qmc5883p_mode_t mode)
{
  qmc5883p_status_t st;

  if ((inst == NULL) || ((unsigned)mode > 3u)) {
    return QMC5883P_ERR_PARAM;
  }
  if (!inst->ready) {
    return QMC5883P_ERR_STATE;
  }
  if (inst->cfg.mode == mode) {
    return QMC5883P_OK;
  }

  /* 手册要求 Normal/Single/Continuous 之间切换必须经过 Suspend */
  if (mode != QMC5883P_MODE_SUSPEND) {
    st = reg_write(inst, QMC_REG_CTRL1,
                   ctrl1_byte(&inst->cfg, QMC5883P_MODE_SUSPEND));
    if (st != QMC5883P_OK) {
      return st;
    }
    qmc5883p_port_delay_ms(QMC_MODE_SWITCH_DELAY_MS);
  }

  st = reg_write(inst, QMC_REG_CTRL1, ctrl1_byte(&inst->cfg, mode));
  if (st != QMC5883P_OK) {
    return st;
  }

  inst->cfg.mode = mode;

  return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_suspend(qmc5883p_t *inst)
{
  return qmc5883p_set_mode(inst, QMC5883P_MODE_SUSPEND);
}

int32_t qmc5883p_range_sensitivity(qmc5883p_range_t range)
{
  switch (range) {
  case QMC5883P_RANGE_30G: return 1000;
  case QMC5883P_RANGE_12G: return 2500;
  case QMC5883P_RANGE_8G:  return 3750;
  case QMC5883P_RANGE_2G:  return 15000;
  default:                 return 0;
  }
}

qmc5883p_status_t qmc5883p_init(qmc5883p_t *inst, const qmc5883p_cfg_t *cfg)
{
  static const qmc5883p_cfg_t default_cfg = QMC5883P_CFG_DEFAULT;

  qmc5883p_status_t st;
  const qmc5883p_cfg_t *use;
  uint8_t id;

  if (inst == NULL) {
    return QMC5883P_ERR_PARAM;
  }

  use = (cfg != NULL) ? cfg : &default_cfg;
  if (!cfg_valid(use)) {
    return QMC5883P_ERR_PARAM;
  }

  *inst = (qmc5883p_t){0};

  /* 先把"器件在不在"和"是什么器件"分开报错：地址对不上时读 ID 本身就会失败 */
  st = qmc5883p_read_chip_id(inst, &id);
  if (st != QMC5883P_OK) {
    return st;
  }
  if (id != QMC5883P_CHIP_ID) {
    return QMC5883P_ERR_ID;
  }

  st = qmc5883p_soft_reset(inst);
  if (st != QMC5883P_OK) {
    return st;
  }

  /* 软复位把 0x29 的轴符号也清了，必须重写 */
  st = reg_write(inst, QMC_REG_SIGN, QMC_SIGN_VALUE);
  if (st != QMC5883P_OK) {
    return st;
  }

  /* 配置不在这里直接写寄存器，而是复用 apply_config 的"先显式 Suspend 再改控制
     寄存器"顺序：实测软复位后直接写 0x0B/0x0A，首轮读数仍是默认的 ±30 G 档
     （量程没生效，读数小了 3.75 倍）；走 Suspend 通路后第一轮就正确。 */
  inst->cfg = *use;
  inst->lsb_per_gauss = qmc5883p_range_sensitivity(use->range);
  inst->ready = true;

  st = qmc5883p_apply_config(inst, use);
  if (st != QMC5883P_OK) {
    inst->ready = false;
    return st;
  }

  /* 软复位后的第一次配置会被器件还原：实测 0x0B 读回 0x00（量程退回 ±30 G，
     读数只有应有值的 1/3.75），而同一时刻 0x0A 是正确的。不猜内部原因，
     回读比对 + 再走一遍是最省事的兜底（第二次一定生效）。 */
  {
    uint8_t ctrl2 = 0u;

    if (qmc5883p_port_i2c_read(QMC_REG_CTRL2, &ctrl2, 1u) &&
        (ctrl2 != ctrl2_byte(use))) {
      st = qmc5883p_apply_config(inst, use);
      if (st != QMC5883P_OK) {
        inst->ready = false;
      }
    }
  }

  return st;
}

/*---------------------------------------------------------------------------*/
/* 数据读取                                                                  */
/*---------------------------------------------------------------------------*/

qmc5883p_status_t qmc5883p_read_status(qmc5883p_t *inst, bool *drdy,
                                       bool *ovfl)
{
  uint8_t status;

  if ((inst == NULL) || (drdy == NULL)) {
    return QMC5883P_ERR_PARAM;
  }
  if (!inst->ready) {
    return QMC5883P_ERR_STATE;
  }

  if (!qmc5883p_port_i2c_read(QMC_REG_STATUS, &status, 1u)) {
    return io_fail(inst);
  }

  *drdy = ((status & QMC_STATUS_DRDY) != 0u);
  if (ovfl != NULL) {
    *ovfl = ((status & QMC_STATUS_OVFL) != 0u);
  }

  return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_read_raw(qmc5883p_t *inst, qmc5883p_raw_t *out)
{
  qmc5883p_status_t st;
  uint8_t status;
  bool drdy;
  bool ovfl;

  if ((inst == NULL) || (out == NULL)) {
    return QMC5883P_ERR_PARAM;
  }
  if (!inst->ready) {
    return QMC5883P_ERR_STATE;
  }

  if (!qmc5883p_port_i2c_read(QMC_REG_STATUS, &status, 1u)) {
    return io_fail(inst);
  }
  drdy = ((status & QMC_STATUS_DRDY) != 0u);
  ovfl = ((status & QMC_STATUS_OVFL) != 0u);

  st = read_burst(inst, out, drdy, ovfl);

  return st;
}

qmc5883p_status_t qmc5883p_read_sample(qmc5883p_t *inst, qmc5883p_raw_t *out,
                                       uint32_t timeout_ms)
{
  uint32_t start;
  uint8_t status;
  bool ovfl;

  if ((inst == NULL) || (out == NULL) || (timeout_ms == 0u)) {
    return QMC5883P_ERR_PARAM;
  }
  if (!inst->ready) {
    return QMC5883P_ERR_STATE;
  }

  start = qmc5883p_port_tick_ms();

  for (;;) {
    if (!qmc5883p_port_i2c_read(QMC_REG_STATUS, &status, 1u)) {
      return io_fail(inst);
    }

    if ((status & QMC_STATUS_DRDY) != 0u) {
      ovfl = ((status & QMC_STATUS_OVFL) != 0u);
      return read_burst(inst, out, true, ovfl);
    }

    if ((qmc5883p_port_tick_ms() - start) >= timeout_ms) {
      inst->timeouts++;
      return QMC5883P_ERR_TIMEOUT;
    }

    qmc5883p_port_delay_ms(QMC_POLL_INTERVAL_MS);
  }
}

int32_t qmc5883p_raw_to_mg(const qmc5883p_t *inst, int16_t raw)
{
  if ((inst == NULL) || (inst->lsb_per_gauss <= 0)) {
    return 0;
  }

  /* mg = raw / (LSB/G) * 1000；先乘后除，整数够用（最大 32767*1000 不溢出 int32） */
  return ((int32_t)raw * 1000) / inst->lsb_per_gauss;
}

qmc5883p_status_t qmc5883p_read_mg(qmc5883p_t *inst, qmc5883p_mg_t *out,
                                   uint32_t timeout_ms)
{
  qmc5883p_status_t st;
  qmc5883p_raw_t raw;

  if ((inst == NULL) || (out == NULL)) {
    return QMC5883P_ERR_PARAM;
  }

  st = qmc5883p_read_sample(inst, &raw, timeout_ms);
  if (st != QMC5883P_OK) {
    return st;
  }

  out->x_mg = qmc5883p_raw_to_mg(inst, raw.x);
  out->y_mg = qmc5883p_raw_to_mg(inst, raw.y);
  out->z_mg = qmc5883p_raw_to_mg(inst, raw.z);
  out->fresh = raw.fresh;
  out->overflow = raw.overflow;

  return QMC5883P_OK;
}

/*---------------------------------------------------------------------------*/
/* 内部自检：打开/关闭 Set-Reset 激励，比较三轴增量（§7.3）                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief 自检一次，返回激励前后的差值
 * @note 只有 Continuous 模式能开 SELF_TEST；结束后恢复 inst->cfg 里的配置。
 *       差值没有公开门限，调用者自行判断（正常应是明显非零、且重复性好）。
 */
static qmc5883p_status_t self_test(qmc5883p_t *inst, int16_t *dx, int16_t *dy,
                                   int16_t *dz)
{
  static const uint32_t timeout_ms = 50u;

  qmc5883p_status_t st;
  qmc5883p_raw_t before;
  qmc5883p_raw_t after;
  qmc5883p_cfg_t saved = inst->cfg;

  /* 手册示例用 Continuous 模式跑自检 */
  st = qmc5883p_set_mode(inst, QMC5883P_MODE_CONTINUOUS);
  if (st != QMC5883P_OK) {
    return st;
  }

  st = qmc5883p_read_sample(inst, &before, timeout_ms);
  if (st != QMC5883P_OK) {
    goto restore;
  }

  /* 在控制寄存器 2 上叠 SELF_TEST 位，保留量程与 Set/Reset 设置 */
  st = reg_write(inst, QMC_REG_CTRL2,
                 (uint8_t)(ctrl2_byte(&inst->cfg) | QMC_CTRL2_SELF_TEST));
  if (st != QMC5883P_OK) {
    goto restore;
  }

  qmc5883p_port_delay_ms(QMC_SELFTEST_DELAY_MS);
  st = qmc5883p_read_sample(inst, &after, timeout_ms);
  if (st != QMC5883P_OK) {
    goto restore;
  }

  *dx = (int16_t)(after.x - before.x);
  *dy = (int16_t)(after.y - before.y);
  *dz = (int16_t)(after.z - before.z);
  st = QMC5883P_OK;

restore:
  /* 自检结束器件会回 Suspend：把配置和模式整体写回，保证后续采集照常 */
  (void)qmc5883p_apply_config(inst, &saved);
  return st;
}

/*---------------------------------------------------------------------------*/
/* 上电自检                                                                  */
/*---------------------------------------------------------------------------*/

volatile qmc5883p_probe_t g_qmc_probe;

/** 自检采样窗口：16 个样本 @ 200 Hz ≈ 80 ms */
#define QMC_PROBE_SAMPLES    16u
#define QMC_PROBE_TIMEOUT_MS 20u

/** 32 位有符号取绝对值（避免引入 stdlib 的 abs 与隐式 int 转换告警） */
static int32_t abs32(int32_t v)
{
  return (v < 0) ? -v : v;
}

static void probe_track(volatile qmc5883p_probe_t *pr, const qmc5883p_raw_t *raw,
                        bool first)
{
  if (first) {
    pr->x_min = raw->x;
    pr->x_max = raw->x;
    pr->y_min = raw->y;
    pr->y_max = raw->y;
    pr->z_min = raw->z;
    pr->z_max = raw->z;
  } else {
    if (raw->x < pr->x_min) { pr->x_min = raw->x; }
    if (raw->x > pr->x_max) { pr->x_max = raw->x; }
    if (raw->y < pr->y_min) { pr->y_min = raw->y; }
    if (raw->y > pr->y_max) { pr->y_max = raw->y; }
    if (raw->z < pr->z_min) { pr->z_min = raw->z; }
    if (raw->z > pr->z_max) { pr->z_max = raw->z; }
  }
}

/**
 * @brief 采 n 个样本，返回 |x|+|y|+|z| 的平均值（LSB）
 * @param track true 时同时更新 min/max（只有第一轮需要）
 * @note 用 L1 范数比较两轮读数的大小，不需要 sqrt 和 libm。
 */
static int32_t probe_round(qmc5883p_t *inst, uint32_t n, bool track)
{
  qmc5883p_raw_t raw;
  int32_t acc = 0;
  uint32_t ok = 0u;
  uint32_t i;
  bool first = true;

  for (i = 0u; i < n; i++) {
    if (qmc5883p_read_sample(inst, &raw, QMC_PROBE_TIMEOUT_MS) != QMC5883P_OK) {
      g_qmc_probe.errors++;
      continue;
    }

    if (track) {
      probe_track(&g_qmc_probe, &raw, first);
      first = false;
    }

    acc += abs32(raw.x) + abs32(raw.y) + abs32(raw.z);
    ok++;
    g_qmc_probe.samples++;
    g_qmc_probe.x_last = raw.x;
    g_qmc_probe.y_last = raw.y;
    g_qmc_probe.z_last = raw.z;
  }

  return (ok == 0u) ? 0 : (acc / (int32_t)ok);
}

void qmc5883p_probe(void)
{
  static qmc5883p_t inst;
  qmc5883p_cfg_t cfg = QMC5883P_CFG_DEFAULT;
  qmc5883p_status_t st;
  uint8_t id = 0u;

  g_qmc_probe = (qmc5883p_probe_t){0};
  g_qmc_probe.magic = QMC5883P_PROBE_MAGIC;
  g_qmc_probe.stage = QMC5883P_STAGE_IDLE;

  /* 1. 地址探测：把"完全没应答"（供电/上拉/接线）和"应答了但 ID 不对"分开 */
  if (!qmc5883p_port_is_ready()) {
    g_qmc_probe.stage = QMC5883P_STAGE_NO_ACK;
    return;
  }

  /* 2. CHIPID：应答正常但 ID 不是 0x80，说明挂在 0x2C 上的不是 QMC5883P */
  st = qmc5883p_read_chip_id(&inst, &id);
  g_qmc_probe.chip_id = id;
  if (st != QMC5883P_OK) {
    g_qmc_probe.stage = QMC5883P_STAGE_ID_FAIL;
    return;
  }
  if (id != QMC5883P_CHIP_ID) {
    g_qmc_probe.stage = QMC5883P_STAGE_ID_FAIL;
    return;
  }
  g_qmc_probe.stage = QMC5883P_STAGE_ID_OK;

  /* 3. 配置：200 Hz Normal + ±8 G，采样窗口短一点，别拖慢开机 */
  cfg.odr = QMC5883P_ODR_200HZ;
  cfg.range = QMC5883P_RANGE_8G;
  st = qmc5883p_init(&inst, &cfg);
  if (st != QMC5883P_OK) {
    g_qmc_probe.stage = QMC5883P_STAGE_INIT_FAIL;
    return;
  }
  g_qmc_probe.stage = QMC5883P_STAGE_CONFIGURED;

  /* 4. 连采 16 个样本，记录各轴最小/最大值——数据在变才说明真的在测量 */
  (void)probe_round(&inst, QMC_PROBE_SAMPLES, true);
  if (g_qmc_probe.samples == 0u) {
    g_qmc_probe.stage = QMC5883P_STAGE_SAMPLE_FAIL;
    return;
  }
  g_qmc_probe.stage = QMC5883P_STAGE_SAMPLED;

  /* 5. 读回控制寄存器，并把最后一个样本换算成 mG：
        ±8 G 档地球磁场约 0.5 G，|x|+|y|+|z| 应在 700~900 mG 量级；
        若只有 200 mG 左右，就是量程没生效（读到的是 ±30 G 档的计数） */
  {
    uint8_t reg = 0u;

    if (qmc5883p_port_i2c_read(QMC_REG_CTRL1, &reg, 1u)) {
      g_qmc_probe.reg_ctrl1 = reg;
    }
    if (qmc5883p_port_i2c_read(QMC_REG_CTRL2, &reg, 1u)) {
      g_qmc_probe.reg_ctrl2 = reg;
    }

    g_qmc_probe.last_mg_l1 =
        abs32(qmc5883p_raw_to_mg(&inst, g_qmc_probe.x_last)) +
        abs32(qmc5883p_raw_to_mg(&inst, g_qmc_probe.y_last)) +
        abs32(qmc5883p_raw_to_mg(&inst, g_qmc_probe.z_last));
  }

  /* 6. 内部自检（Set/Reset 激励）：差值明显非零说明磁阻桥与模拟链路正常 */
  {
    int16_t dx = 0;
    int16_t dy = 0;
    int16_t dz = 0;

    st = self_test(&inst, &dx, &dy, &dz);
    g_qmc_probe.st_dx = dx;
    g_qmc_probe.st_dy = dy;
    g_qmc_probe.st_dz = dz;
  }
  if (st != QMC5883P_OK) {
    g_qmc_probe.stage = QMC5883P_STAGE_SELFTEST_FAIL;
    return;
  }

  /* 7. 记录最后读到的状态位，便于事后判断有没有出现过 OVFL */
  {
    bool drdy = false;
    bool ovfl = false;

    if (qmc5883p_read_status(&inst, &drdy, &ovfl) == QMC5883P_OK) {
      g_qmc_probe.status = (uint8_t)((drdy ? QMC_STATUS_DRDY : 0u) |
                                     (ovfl ? QMC_STATUS_OVFL : 0u));
    }
  }

  g_qmc_probe.stage = QMC5883P_STAGE_DONE;
}
