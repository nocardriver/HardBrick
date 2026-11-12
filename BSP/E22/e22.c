/**
 * @file    e22.c
 * @author  YangKY
 * @brief   E22-400T22S LoRa 模块驱动核心实现
 *
 * 结构：
 *   - 生命周期：init / reset / set_mode（M1/M0 + AUX 忙闲）
 *   - 数据面  ：e22_send（按 240 B 分块）/ 环形缓冲接收 / e22_read
 *   - 控制面  ：配置模式下 HEX 指令（C0/C1）读写寄存器、AT 查询
 *
 * 关键时序（手册第五、六章）：
 *   - 上电到可操作 T1 = 16 ms；复位脉冲 > 100 us；
 *   - 模式切换 9~11 ms，期间 AUX 拉低；
 *   - 写入参数后必须切出配置模式才生效；
 *   - 配置模式串口强制 9600-8-N-1。
 */

#include "e22.h"
#include "e22_port.h"

#include <stddef.h>
#include <string.h>

/** 模式切换典型 9~11 ms，留 15 ms 余量 */
#define E22_MODE_SWITCH_MS 15u
/** NRST 低电平保持（手册要求 > 100 us） */
#define E22_RESET_LOW_MS 2u
/** 上电到可操作 T1 = 16 ms，留 20 ms 余量 */
#define E22_BOOT_MS 20u
/** 复位后等待 AUX 空闲的上限 */
#define E22_INIT_TIMEOUT_MS 1000u
/** 切模式前等待模块空闲的上限（开着 UAUX 时一包 240 B 的空中时间约 0.8 s） */
#define E22_MODE_WAIT_MS 1000u
/** 配置模式单条指令的应答超时 */
#define E22_CFG_TIMEOUT_MS 500u
/** 配置模式切回原模式的 AUX 超时 */
#define E22_CFG_RESTORE_TIMEOUT_MS 1000u
/** AT 响应行尾：连续 10 ms 没有新字节就认为一行结束（9600 下约 10 个字节时间） */
#define E22_AT_IDLE_GAP_MS 10u

/** 配置模式自动切换的上下文：调用前保存，调用后恢复 */
typedef struct {
  e22_mode_t saved; /**< 进入配置模式前的模式 */
  bool nested;      /**< 调用前本来就在配置模式，结束后保持不动 */
} e22_cfg_ctx_t;

/*---------------------------------------------------------------------------*/
/* 环形缓冲（SPSC：RX 中断写 head，消费线程写 tail）                           */
/*---------------------------------------------------------------------------*/

static uint16_t e22_ring_mask(const e22_t *inst)
{
  return (uint16_t)(inst->rx_ring_size - 1u);
}

static bool e22_ring_pop(e22_t *inst, uint8_t *byte)
{
  uint16_t tail = inst->rx_tail;

  if (tail == inst->rx_head) {
    return false;
  }

  *byte = inst->rx_ring[tail];
  inst->rx_tail = (uint16_t)((tail + 1u) & e22_ring_mask(inst));
  return true;
}

void e22_rx_feed(e22_t *inst, const uint8_t *data, uint16_t len)
{
  uint16_t mask;
  uint16_t head;
  uint16_t used;
  uint16_t space;
  uint16_t n;
  uint16_t i;

  if ((inst == NULL) || (data == NULL) || (len == 0u) ||
      (inst->rx_ring == NULL)) {
    return;
  }

  mask = e22_ring_mask(inst);
  head = inst->rx_head;
  used = (uint16_t)((head - inst->rx_tail) & mask);
  space = (uint16_t)(mask - used); /* 容量 N 的环形缓冲最多存 N-1 字节 */
  n = (len <= space) ? len : space;

  for (i = 0u; i < n; i++) {
    inst->rx_ring[head] = data[i];
    head = (uint16_t)((head + 1u) & mask); /* 先写数据，后移游标 */
  }
  inst->rx_head = head;

  /* 缓冲满：丢弃新数据，绝不覆盖未消费数据（AGENTS.md 约定） */
  if (n < len) {
    inst->rx_dropped += (uint32_t)(len - n);
  }
}

uint16_t e22_available(const e22_t *inst)
{
  if ((inst == NULL) || (inst->rx_ring == NULL)) {
    return 0u;
  }

  return (uint16_t)((inst->rx_head - inst->rx_tail) & e22_ring_mask(inst));
}

uint16_t e22_read(e22_t *inst, uint8_t *buf, uint16_t max_len)
{
  uint16_t n = 0u;

  if ((inst == NULL) || (buf == NULL) || (inst->rx_ring == NULL)) {
    return 0u;
  }

  /* 若上次 rearm 被阻塞发送挡掉，这里补挂，避免 RX 停摆 */
  e22_port_poll();

  while (n < max_len) {
    uint16_t tail = inst->rx_tail;

    if (tail == inst->rx_head) {
      break;
    }

    buf[n++] = inst->rx_ring[tail];
    inst->rx_tail = (uint16_t)((tail + 1u) & e22_ring_mask(inst));
  }

  return n;
}

void e22_flush_rx(e22_t *inst)
{
  if (inst == NULL) {
    return;
  }

  inst->rx_tail = inst->rx_head;
}

/*---------------------------------------------------------------------------*/
/* 模式、复位、AUX                                                           */
/*---------------------------------------------------------------------------*/

void e22_aux_event(e22_t *inst, bool busy)
{
  if (inst == NULL) {
    return;
  }

  inst->aux_busy = busy;

  if (inst->aux_cb != NULL) {
    inst->aux_cb(inst, busy);
  }
}

bool e22_is_busy(const e22_t *inst)
{
  (void)inst;
  return !e22_port_aux_level(); /* 低电平 = 忙 */
}

e22_status_t e22_wait_idle(e22_t *inst, uint32_t timeout_ms)
{
  uint32_t t0;

  if (inst == NULL) {
    return E22_ERR_PARAM;
  }

  t0 = e22_port_tick_ms();
  while (!e22_port_aux_level()) {
    if ((uint32_t)(e22_port_tick_ms() - t0) >= timeout_ms) {
      inst->aux_busy = true;
      return E22_ERR_TIMEOUT;
    }
    e22_port_delay_ms(1u);
  }

  inst->aux_busy = false;
  return E22_OK;
}

e22_mode_t e22_get_mode(const e22_t *inst)
{
  /* 空实例没有有效模式；返回透传只是给调试器一个确定值 */
  return (inst != NULL) ? inst->mode : E22_MODE_TRANSMISSION;
}

/**
 * @brief 真正执行模式切换（不检查 ready，供 init 内部使用）
 * @note  M1/M0 的枚举值就是引脚真值：bit1 = M1、bit0 = M0。
 */
static e22_status_t e22_mode_apply(e22_t *inst, e22_mode_t mode,
                                   uint32_t timeout_ms)
{
  e22_status_t st;
  bool m1;
  bool m0;

  if (inst->mode == mode) {
    return E22_OK;
  }

  /* AUX 低时模块不检测模式引脚，先等它闲下来。
     从休眠唤醒时模块可能已不驱动 AUX，跳过这次等待。 */
  if (inst->mode != E22_MODE_SLEEP) {
    st = e22_wait_idle(inst, timeout_ms);
    if (st != E22_OK) {
      return st;
    }
  }

  m1 = (((unsigned)mode & 0x02u) != 0u);
  m0 = (((unsigned)mode & 0x01u) != 0u);
  e22_port_set_mode_pins(m1, m0);
  inst->mode = mode; /* 引脚已变，状态先跟上；AUX 等不到再报超时 */
  e22_port_delay_ms(E22_MODE_SWITCH_MS);

  /* 目标不是休眠时，等模块完成切换并重新驱动 AUX */
  if (mode != E22_MODE_SLEEP) {
    st = e22_wait_idle(inst, timeout_ms);
    if (st != E22_OK) {
      return st;
    }
  }

  return E22_OK;
}

e22_status_t e22_set_mode(e22_t *inst, e22_mode_t mode, uint32_t timeout_ms)
{
  if ((inst == NULL) || ((unsigned)mode > (unsigned)E22_MODE_SLEEP)) {
    return E22_ERR_PARAM;
  }
  if (!inst->ready) {
    return E22_ERR_STATE;
  }

  return e22_mode_apply(inst, mode, timeout_ms);
}

e22_status_t e22_reset(e22_t *inst)
{
  e22_status_t st;

  if (inst == NULL) {
    return E22_ERR_PARAM;
  }

  /* 复位期间先把模式脚摆成透传，模块重新启动后直接进入工作模式 */
  e22_port_set_mode_pins(false, false);

  /* NRST 拉低：手册要求低电平 >100 us，这里保持 2 ms */
  e22_port_reset(true);
  e22_port_delay_ms(E22_RESET_LOW_MS);
  /* 释放复位：NRST 拉高，模块开始启动（T1 = 16 ms 后才可操作） */
  e22_port_reset(false);

  /* 等启动完成，再确认 AUX 回到空闲 */
  e22_port_delay_ms(E22_BOOT_MS);

  inst->mode = E22_MODE_TRANSMISSION;
  e22_flush_rx(inst);

  st = e22_wait_idle(inst, E22_INIT_TIMEOUT_MS);
  inst->ready = (st == E22_OK);
  if (st == E22_OK) {
    e22_flush_rx(inst); /* 丢掉启动期间的空闲字节 */
  }
  return st;
}

e22_status_t e22_init(e22_t *inst, const e22_cfg_t *cfg)
{
  if ((inst == NULL) || (cfg == NULL) || (cfg->rx_ring == NULL) ||
      (cfg->rx_ring_size < 2u) || (cfg->rx_ring_size > 32768u) ||
      ((cfg->rx_ring_size & (uint16_t)(cfg->rx_ring_size - 1u)) != 0u)) {
    return E22_ERR_PARAM;
  }

  memset(inst, 0, sizeof(*inst));
  inst->rx_ring = cfg->rx_ring;
  inst->rx_ring_size = cfg->rx_ring_size;
  inst->aux_cb = cfg->aux_cb;
  inst->user = cfg->user;
  inst->mode = E22_MODE_TRANSMISSION;

  /* 绑定板级资源并先开接收，避免漏掉模块启动后可能吐出的字节 */
  e22_port_attach(inst);

  return e22_reset(inst);
}

/*---------------------------------------------------------------------------*/
/* 透传发送                                                                  */
/*---------------------------------------------------------------------------*/

e22_status_t e22_send(e22_t *inst, const uint8_t *data, uint16_t len,
                      uint32_t timeout_ms)
{
  e22_status_t st;
  uint16_t sent = 0u;

  if ((inst == NULL) || ((data == NULL) && (len != 0u))) {
    return E22_ERR_PARAM;
  }
  if (!inst->ready) {
    return E22_ERR_STATE;
  }
  if ((inst->mode != E22_MODE_TRANSMISSION) && (inst->mode != E22_MODE_WOR)) {
    return E22_ERR_STATE;
  }
  if (len == 0u) {
    return E22_OK;
  }
  /* 定点传输的 3 字节目标头只在每包开头，分块后后续块会被当成新目标头 */
  if (inst->fixed_tx && (len > E22_PACKET_PAYLOAD_MAX)) {
    return E22_ERR_PARAM;
  }

  while (sent < len) {
    uint16_t chunk = (uint16_t)(len - sent);
    uint32_t tx_timeout;

    if (chunk > E22_PACKET_PAYLOAD_MAX) {
      chunk = E22_PACKET_PAYLOAD_MAX;
    }

    /* 模块串口缓存 1000 B，AUX 高表示上一包已被接纳；这里形成背压 */
    st = e22_wait_idle(inst, timeout_ms);
    if (st != E22_OK) {
      return st;
    }

    /* 9600 下约 1.04 ms/字节，按 2 倍留余量；HAL 的超时是整笔传输的上限 */
    tx_timeout = ((uint32_t)chunk * 2u) + 100u;
    if (!e22_port_uart_send(&data[sent], chunk, tx_timeout)) {
      return E22_ERR_IO;
    }

    sent = (uint16_t)(sent + chunk);
  }

  return E22_OK;
}

/*---------------------------------------------------------------------------*/
/* 配置模式：HEX 指令                                                        */
/*---------------------------------------------------------------------------*/

static e22_status_t e22_cfg_enter(e22_t *inst, e22_cfg_ctx_t *ctx)
{
  e22_status_t st;

  ctx->saved = inst->mode;
  ctx->nested = (inst->mode == E22_MODE_CONFIG);
  if (ctx->nested) {
    return E22_OK;
  }

  st = e22_set_mode(inst, E22_MODE_CONFIG, E22_MODE_WAIT_MS);
  if (st != E22_OK) {
    return st;
  }

  /* 丢掉切模式期间串口上的残留字节 */
  e22_flush_rx(inst);
  e22_port_delay_ms(2u);
  return E22_OK;
}

static e22_status_t e22_cfg_leave(e22_t *inst, const e22_cfg_ctx_t *ctx)
{
  if (ctx->nested || (ctx->saved == E22_MODE_CONFIG)) {
    return E22_OK;
  }

  /* 切出配置模式才会重新加载参数；失败时由调用者把错误上报 */
  return e22_set_mode(inst, ctx->saved, E22_CFG_RESTORE_TIMEOUT_MS);
}

static e22_status_t e22_cfg_send(const uint8_t *buf, uint16_t len)
{
  return e22_port_uart_send(buf, len, E22_CFG_TIMEOUT_MS) ? E22_OK : E22_ERR_IO;
}

/**
 * @brief 等一条 HEX 应答：C1 + 起始地址 + 长度 + 数据
 * @note  用 3 字节滑动窗口找应答头，容忍进入配置模式前残留的零星字节；
 *        收到 FF FF FF 视为格式错误。
 */
static e22_status_t e22_wait_hex(e22_t *inst, uint8_t start, uint8_t len,
                                 uint8_t *data, uint32_t timeout_ms)
{
  uint8_t win[3] = {0u, 0u, 0u};
  uint16_t got = 0u;
  uint16_t n = 0u;
  bool header = false;
  uint32_t t0 = e22_port_tick_ms();

  for (;;) {
    uint8_t b;

    if ((uint32_t)(e22_port_tick_ms() - t0) >= timeout_ms) {
      return E22_ERR_TIMEOUT;
    }

    if (!e22_ring_pop(inst, &b)) {
      e22_port_delay_ms(1u);
      continue;
    }

    if (!header) {
      win[0] = win[1];
      win[1] = win[2];
      win[2] = b;
      got++;

      if (got >= 3u) {
        if ((win[0] == 0xFFu) && (win[1] == 0xFFu) && (win[2] == 0xFFu)) {
          return E22_ERR_PROTOCOL;
        }
        if ((win[0] == 0xC1u) && (win[1] == start) && (win[2] == len)) {
          header = true;
        }
      }
    } else {
      data[n++] = b;
      if (n >= len) {
        return E22_OK;
      }
    }
  }
}

e22_status_t e22_read_regs(e22_t *inst, uint8_t start, uint8_t len,
                           uint8_t *buf)
{
  e22_cfg_ctx_t ctx;
  e22_status_t st;
  uint8_t cmd[3];

  if ((inst == NULL) || (buf == NULL) || (len == 0u) || (len > 16u)) {
    return E22_ERR_PARAM;
  }
  if (((uint16_t)start + (uint16_t)len) > 0x87u) {
    return E22_ERR_PARAM;
  }

  st = e22_cfg_enter(inst, &ctx);
  if (st != E22_OK) {
    return st;
  }

  cmd[0] = 0xC1u;
  cmd[1] = start;
  cmd[2] = len;

  e22_flush_rx(inst);
  st = e22_cfg_send(cmd, (uint16_t)sizeof(cmd));
  if (st == E22_OK) {
    st = e22_wait_hex(inst, start, len, buf, E22_CFG_TIMEOUT_MS);
  }

  {
    e22_status_t leave = e22_cfg_leave(inst, &ctx);
    if (st == E22_OK) {
      st = leave;
    }
  }
  return st;
}

e22_status_t e22_write_regs(e22_t *inst, uint8_t start, uint8_t len,
                            const uint8_t *buf)
{
  e22_cfg_ctx_t ctx;
  e22_status_t st;
  uint8_t cmd[3u + 16u];
  uint8_t echo[16];

  if ((inst == NULL) || (buf == NULL) || (len == 0u) || (len > 16u)) {
    return E22_ERR_PARAM;
  }
  /* 可写寄存器只到 0x0A（密钥、WOR 窗口） */
  if ((start > 0x0Au) || (((uint16_t)start + (uint16_t)len) > 0x0Bu)) {
    return E22_ERR_PARAM;
  }

  st = e22_cfg_enter(inst, &ctx);
  if (st != E22_OK) {
    return st;
  }

  cmd[0] = 0xC0u;
  cmd[1] = start;
  cmd[2] = len;
  (void)memcpy(&cmd[3], buf, len);

  e22_flush_rx(inst);
  st = e22_cfg_send(cmd, (uint16_t)(3u + len));
  if (st == E22_OK) {
    st = e22_wait_hex(inst, start, len, echo, E22_CFG_TIMEOUT_MS);
  }
  if ((st == E22_OK) && (memcmp(echo, buf, len) != 0)) {
    st = E22_ERR_PROTOCOL; /* 模块回显与写入不一致，参数很可能没落盘 */
  }

  {
    e22_status_t leave = e22_cfg_leave(inst, &ctx);
    if (st == E22_OK) {
      st = leave;
    }
  }
  return st;
}

/*---------------------------------------------------------------------------*/
/* 参数快照                                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief REG0[2:0] → 空中速率枚举
 * @note  400/900 频段码值 0/1/2 都是 2.4 kbps，统一归一化成 E22_AIR_RATE_2K4。
 */
static e22_air_rate_t e22_air_rate_decode(uint8_t code)
{
  return (code <= 2u) ? E22_AIR_RATE_2K4 : (e22_air_rate_t)code;
}

/** @brief 空中速率枚举 → 寄存器码值（2.4 kbps 写模块默认的 2） */
static uint8_t e22_air_rate_encode(e22_air_rate_t rate)
{
  return (rate == E22_AIR_RATE_2K4) ? 2u : (uint8_t)rate;
}

e22_status_t e22_read_params(e22_t *inst, e22_params_t *out)
{
  uint8_t regs[7];
  e22_status_t st;
  uint8_t reg0;
  uint8_t reg1;
  uint8_t reg3;

  if ((inst == NULL) || (out == NULL)) {
    return E22_ERR_PARAM;
  }

  st = e22_read_regs(inst, 0x00u, (uint8_t)sizeof(regs), regs);
  if (st != E22_OK) {
    return st;
  }

  reg0 = regs[3];
  reg1 = regs[4];
  reg3 = regs[6];

  out->address = (uint16_t)(((uint16_t)regs[0] << 8) | regs[1]);
  out->netid = regs[2];
  out->channel = regs[5];

  out->uart_baud = (e22_uart_baud_t)(reg0 >> 5);
  out->uart_parity = (e22_uart_parity_t)((reg0 >> 3) & 0x03u);
  out->air_rate = e22_air_rate_decode((uint8_t)(reg0 & 0x07u));

  out->packet_len = (e22_packet_len_t)(reg1 >> 6);
  out->rssi_env = ((reg1 & 0x20u) != 0u);
  out->soft_mode = ((reg1 & 0x04u) != 0u);
  out->tx_power = (e22_tx_power_t)(reg1 & 0x03u);

  out->rssi_byte = ((reg3 & 0x80u) != 0u);
  out->fixed_tx = ((reg3 & 0x40u) != 0u);
  out->relay = ((reg3 & 0x20u) != 0u);
  out->lbt = ((reg3 & 0x10u) != 0u);
  out->wor_role = (e22_wor_role_t)((reg3 >> 3) & 0x01u);
  out->wor_period = (e22_wor_period_t)(reg3 & 0x07u);

  /* 记下传输方式，e22_send() 据此限制定点模式下的单包长度 */
  inst->fixed_tx = out->fixed_tx;

  return E22_OK;
}

e22_status_t e22_write_params(e22_t *inst, const e22_params_t *params)
{
  uint8_t regs[7];
  e22_status_t st;
  unsigned air;

  if ((inst == NULL) || (params == NULL)) {
    return E22_ERR_PARAM;
  }
  if (params->channel > 83u) {
    return E22_ERR_PARAM; /* 400 频段只有 0~83 */
  }
  /* 本板 LPUART1 用 LSE 32.768 kHz，9600 已是上限，写成别的值会直接失联 */
  if ((params->uart_baud != E22_UART_BAUD_9600) ||
      (params->uart_parity != E22_UART_PARITY_8N1)) {
    return E22_ERR_PARAM;
  }
  if (((unsigned)params->packet_len > 3u) ||
      ((unsigned)params->tx_power > 3u) ||
      ((unsigned)params->wor_role > 1u) ||
      ((unsigned)params->wor_period > 7u)) {
    return E22_ERR_PARAM;
  }
  air = (unsigned)params->air_rate;
  if ((air > 7u) || ((air != 0u) && (air < 3u))) {
    return E22_ERR_PARAM; /* 枚举里 1/2 不是合法速率码 */
  }

  regs[0] = (uint8_t)(params->address >> 8);
  regs[1] = (uint8_t)(params->address & 0xFFu);
  regs[2] = params->netid;
  regs[3] = (uint8_t)(((unsigned)params->uart_baud << 5) |
                      ((unsigned)params->uart_parity << 3) |
                      e22_air_rate_encode(params->air_rate));
  regs[4] = (uint8_t)(((unsigned)params->packet_len << 6) |
                      (params->rssi_env ? 0x20u : 0x00u) |
                      (params->soft_mode ? 0x04u : 0x00u) |
                      (unsigned)params->tx_power);
  regs[5] = params->channel;
  regs[6] = (uint8_t)((params->rssi_byte ? 0x80u : 0x00u) |
                      (params->fixed_tx ? 0x40u : 0x00u) |
                      (params->relay ? 0x20u : 0x00u) |
                      (params->lbt ? 0x10u : 0x00u) |
                      ((unsigned)params->wor_role << 3) |
                      (unsigned)params->wor_period);

  st = e22_write_regs(inst, 0x00u, (uint8_t)sizeof(regs), regs);
  if (st == E22_OK) {
    inst->fixed_tx = params->fixed_tx;
  }
  return st;
}

e22_status_t e22_write_key(e22_t *inst, uint16_t key)
{
  uint8_t regs[2];

  regs[0] = (uint8_t)(key >> 8);
  regs[1] = (uint8_t)(key & 0xFFu);

  return e22_write_regs(inst, 0x07u, (uint8_t)sizeof(regs), regs);
}

e22_status_t e22_write_wor_delay(e22_t *inst, uint16_t delay_ms)
{
  uint8_t regs[2];

  regs[0] = (uint8_t)(delay_ms >> 8);
  regs[1] = (uint8_t)(delay_ms & 0xFFu);

  return e22_write_regs(inst, 0x09u, (uint8_t)sizeof(regs), regs);
}

e22_status_t e22_read_wor_delay(e22_t *inst, uint16_t *delay_ms)
{
  uint8_t regs[2];
  e22_status_t st;

  if (delay_ms == NULL) {
    return E22_ERR_PARAM;
  }

  st = e22_read_regs(inst, 0x09u, (uint8_t)sizeof(regs), regs);
  if (st == E22_OK) {
    *delay_ms = (uint16_t)(((uint16_t)regs[0] << 8) | regs[1]);
  }

  return st;
}

e22_status_t e22_read_pid(e22_t *inst, uint8_t pid[E22_PID_LEN])
{
  if (pid == NULL) {
    return E22_ERR_PARAM;
  }

  return e22_read_regs(inst, 0x80u, (uint8_t)E22_PID_LEN, pid);
}

/*---------------------------------------------------------------------------*/
/* AT 查询                                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief 读一行 AT 应答
 * @note  不以固定长度判断结束：收到字节后若连续 10 ms 没有新字节就认为一行结束，
 *        兼容带/不带 CR/LF 的固件版本（手册 7.1 节说明两种都存在）。
 */
static e22_status_t e22_read_line(e22_t *inst, char *out, uint16_t cap,
                                  uint32_t timeout_ms)
{
  uint16_t n = 0u;
  uint32_t t0 = e22_port_tick_ms();
  uint32_t last = t0;

  for (;;) {
    uint8_t b;
    uint32_t now;

    if (e22_ring_pop(inst, &b)) {
      last = e22_port_tick_ms();

      if ((b == (uint8_t)'\r') || (b == (uint8_t)'\n')) {
        if (n != 0u) {
          break;
        }
        continue; /* 跳过行首 CR/LF */
      }

      if (n < (uint16_t)(cap - 1u)) {
        out[n++] = (char)b;
      }
      continue;
    }

    now = e22_port_tick_ms();
    if ((n != 0u) && ((uint32_t)(now - last) >= E22_AT_IDLE_GAP_MS)) {
      break;
    }
    if ((uint32_t)(now - t0) >= timeout_ms) {
      if (n != 0u) {
        break;
      }
      out[0] = '\0';
      return E22_ERR_TIMEOUT;
    }
    e22_port_delay_ms(1u);
  }

  out[n] = '\0';

  if (strncmp(out, "=ERR", 4u) == 0) {
    return E22_ERR_PROTOCOL;
  }

  return E22_OK;
}

/**
 * @brief 在配置模式下做一次 AT 交互，取回第一行非回显应答
 * @note 有的固件会把命令原样回显一行（内含 key，直接匹配会误判），最多看 3 行。
 */
static e22_status_t e22_at_exchange(e22_t *inst, const char *command, char *line,
                                    uint16_t cap, uint32_t timeout_ms)
{
  e22_cfg_ctx_t ctx;
  e22_status_t st;
  uint8_t attempts;

  st = e22_cfg_enter(inst, &ctx);
  if (st != E22_OK) {
    return st;
  }

  e22_flush_rx(inst);
  line[0] = '\0';

  st = e22_cfg_send((const uint8_t *)command, (uint16_t)strlen(command));
  if (st == E22_OK) {
    for (attempts = 0u; attempts < 3u; attempts++) {
      st = e22_read_line(inst, line, cap, timeout_ms);
      if (st != E22_OK) {
        break;
      }
      if (strncmp(line, "AT+", 3u) == 0) {
        line[0] = '\0';
        continue;
      }
      break;
    }
  }

  {
    e22_status_t leave = e22_cfg_leave(inst, &ctx);
    if (st == E22_OK) {
      st = leave;
    }
  }

  return st;
}

e22_status_t e22_at_query(e22_t *inst, const char *command, const char *key,
                          char *out, uint16_t out_size, uint32_t timeout_ms)
{
  e22_status_t st;
  char line[96];
  const char *value;
  size_t value_len;

  if ((inst == NULL) || (command == NULL) || (key == NULL) || (out == NULL) ||
      (out_size < 2u) || (strlen(command) > 32u)) {
    return E22_ERR_PARAM;
  }

  st = e22_at_exchange(inst, command, line, (uint16_t)sizeof(line), timeout_ms);
  if (st != E22_OK) {
    return st;
  }

  value = strstr(line, key);
  if (value == NULL) {
    return E22_ERR_PROTOCOL;
  }
  value += strlen(key);

  while (*value == ' ') {
    value++;
  }

  value_len = strlen(value);
  while ((value_len > 0u) &&
         ((value[value_len - 1u] == ' ') || (value[value_len - 1u] == '\r') ||
          (value[value_len - 1u] == '\n'))) {
    value_len--;
  }
  if (value_len == 0u) {
    return E22_ERR_PROTOCOL;
  }

  if (value_len >= out_size) {
    value_len = (size_t)(out_size - 1u);
  }
  (void)memcpy(out, value, value_len);
  out[value_len] = '\0';

  return E22_OK;
}

e22_status_t e22_at_command(e22_t *inst, const char *command, uint32_t timeout_ms)
{
  e22_status_t st;
  char line[48];

  if ((inst == NULL) || (command == NULL) || (strlen(command) > 32u)) {
    return E22_ERR_PARAM;
  }

  st = e22_at_exchange(inst, command, line, (uint16_t)sizeof(line), timeout_ms);
  if (st != E22_OK) {
    return st;
  }

  /* 设置类命令正常回 "=OK"；老固件可能不回或回其它内容，只要不是 "=ERR" 就当成功 */
  if (strncmp(line, "=ERR", 4u) == 0) {
    return E22_ERR_PROTOCOL;
  }

  return E22_OK;
}
