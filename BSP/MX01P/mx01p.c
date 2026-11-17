/**
 * @file    mx01p.c
 * @author  YangKY
 * @brief   MX-01P 蓝牙 5.3 (BLE) 透传模块驱动核心实现
 *
 * 结构：
 *   - 生命周期：init / reset（RES 硬复位 + 等 +READY）/ reboot / restore
 *   - 数据面  ：RX 分流器（控制行 vs 透传数据）→ 环形缓冲；send 阻塞发送
 *   - 控制面  ：单槽应答邮箱 + 同步 AT 事务 + 手册里全套参数读写
 *   - 睡眠    ：BRTS 控制，发送/AT 前自动唤醒
 *
 * 为什么 RX 要做分流（本驱动的核心设计点）：
 *   手册里 CDS 脚悬空即"AT 命令有效，非 AT 命令透传"，本板 CDS 没接，所以 AT 应答
 *   与手机发来的透传数据**在同一根 RX 线上交错出现**。分流规则（见 mx01p_rx_feed）：
 *     1. 行首不是 '+'/'O'/'E' → 一定不是控制行，逐字节直接进数据缓冲（零延迟）；
 *     2. 行首是这三个字符 → 先攒成一行，收齐 CRLF 后按前缀判定：
 *        命中事件 → 分发；AT 事务进行中 → 投进应答邮箱；其余 → 回退成数据；
 *     3. 攒行时超过 MX01P_RX_IDLE_FLUSH_MS 没有新字节，或长度超过行缓冲 → 判定为
 *        数据并整体吐出，避免二进制载荷被吞掉。
 *   因此以 '+' 开头的二进制载荷最多晚 5 ms 到达，其余数据没有额外延迟。
 *
 * 时序依据（手册未给的数值都标注了来源）：
 *   - RES 低电平保持 MX01P_RESET_LOW_MS = 10 ms（手册只说"拉低复位"，取保守值）；
 *   - 复位/重启后必须等到 `+READY\r\n` 才允许操作（手册第 8 页备注）；
 *   - 唤醒延时 MX01P_WAKE_MS = 10 ms（手册未给，实测 10 ms 足够，见 README 第 8 节）。
 */

#include "mx01p.h"
#include "mx01p_port.h"

#include <stddef.h>
#include <string.h>

/** RES 低电平保持时间（ms）。手册只写"拉低复位"，没有下限，取足够大的保守值 */
#define MX01P_RESET_LOW_MS 10u

/** 候选行空闲多久判定为数据（ms）。115200 下 5 ms 约 57 个字节时间，够一包 BLE 数据 */
#define MX01P_RX_IDLE_FLUSH_MS 5u

/** 一次 AT 事务最多看多少行不符合预期就放弃（防止被垃圾行拖死） */
#define MX01P_AT_MAX_LINES 6u

/** 睡眠验证时等模块"没有应答"的观察窗口（ms） */
#define MX01P_SLEEP_WATCH_MS 200u

/** 唤醒后等模块把积压的处理完的时间（ms），第 8 节实测 */
#define MX01P_WAKE_SETTLE_MS 50u

/** 数据通路回环时等应答回来的窗口（ms） */
#define MX01P_LOOPBACK_MS 300u

/** 一条应答行的最大长度（含 CRLF）：最长的是 +AMDATA（58 个 HEX 字符）+ 前缀 */
#define MX01P_LINE_MAX ((uint16_t)sizeof(((mx01p_t *)0)->line_buf))

/* 手册里三条 UUID 通道的查询/前缀/设置命令，下标同 mx01p_uuid_kind_t */
static const char *const g_uuid_cmd[3] = {"AT+UUIDS?", "AT+UUIDN?", "AT+UUIDW?"};
static const char *const g_uuid_key[3] = {"+UUIDS:", "+UUIDN:", "+UUIDW:"};
static const char *const g_uuid_set[3] = {"AT+UUIDS=", "AT+UUIDN=", "AT+UUIDW="};

/*---------------------------------------------------------------------------*/
/* 小工具                                                                    */
/*---------------------------------------------------------------------------*/

/** @brief 是否十六进制字符 */
static bool mx01p_is_hex(char c)
{
  return (((c >= '0') && (c <= '9')) || ((c >= 'a') && (c <= 'f')) ||
          ((c >= 'A') && (c <= 'F')));
}

/** @brief 字符串是否全为十六进制字符 */
static bool mx01p_is_hex_str(const char *s)
{
  if ((s == NULL) || (*s == '\0')) {
    return false;
  }
  while (*s != '\0') {
    if (!mx01p_is_hex(*s)) {
      return false;
    }
    s++;
  }
  return true;
}

/** @brief 解析十进制整数（可带负号），成功返回 true */
static bool mx01p_parse_int(const char *s, int32_t *out)
{
  bool neg = false;
  uint32_t v = 0u;
  uint8_t digits = 0u;

  if ((s == NULL) || (out == NULL)) {
    return false;
  }

  while (*s == ' ') {
    s++;
  }
  if (*s == '-') {
    neg = true;
    s++;
  }
  while ((*s >= '0') && (*s <= '9')) {
    v = (v * 10u) + (uint32_t)(*s - '0');
    s++;
    digits++;
    if (digits > 9u) {
      return false; /* 明显不是参数值 */
    }
  }
  while (*s == ' ') {
    s++;
  }
  if ((digits == 0u) || (*s != '\0')) {
    return false;
  }

  *out = neg ? -(int32_t)v : (int32_t)v;
  return true;
}

/**
 * @brief 把无符号数追加成十进制字符串
 * @return 写入后的字符串长度
 */
static size_t mx01p_put_uint(char *dst, uint32_t value)
{
  char tmp[10];
  uint8_t n = 0u;
  size_t i;

  do {
    tmp[n++] = (char)('0' + (value % 10u));
    value /= 10u;
  } while ((value != 0u) && (n < (uint8_t)sizeof(tmp)));

  for (i = 0u; i < (size_t)n; i++) {
    dst[i] = tmp[n - 1u - (uint8_t)i];
  }
  dst[n] = '\0';
  return (size_t)n;
}

/** @brief 拼接 "prefix + value" 成一条 AT 命令；装不下返回 false */
static bool mx01p_cmd_join(const char *prefix, const char *value, char *out,
                           uint16_t cap)
{
  size_t plen;
  size_t vlen;

  if ((prefix == NULL) || (value == NULL) || (out == NULL)) {
    return false;
  }
  plen = strlen(prefix);
  vlen = strlen(value);
  if ((plen + vlen) > (size_t)MX01P_CMD_MAX) {
    return false;
  }
  if ((plen + vlen + 1u) > (size_t)cap) {
    return false;
  }

  (void)memcpy(out, prefix, plen);
  (void)memcpy(&out[plen], value, vlen + 1u);
  return true;
}

/** @brief 拼接 "prefix + 十进制数" 成一条 AT 命令 */
static bool mx01p_cmd_join_int(const char *prefix, int32_t value, char *out,
                               uint16_t cap)
{
  char num[12];
  size_t len = 0u;

  if (value < 0) {
    num[len++] = '-';
  }
  len += mx01p_put_uint(&num[len], (value < 0) ? (uint32_t)(-value)
                                               : (uint32_t)value);

  return mx01p_cmd_join(prefix, num, out, cap);
}

/*---------------------------------------------------------------------------*/
/* 环形缓冲（SPSC：RX 中断写 head，消费线程写 tail）                           */
/*---------------------------------------------------------------------------*/

static uint16_t mx01p_ring_mask(const mx01p_t *inst)
{
  return (uint16_t)(inst->rx_ring_size - 1u);
}

/** @brief 压入一个字节；缓冲满丢弃新数据（不覆盖未消费数据） */
static void mx01p_ring_push(mx01p_t *inst, uint8_t byte)
{
  uint16_t mask;
  uint16_t head;
  uint16_t next;

  if (inst->rx_ring == NULL) {
    return;
  }

  mask = mx01p_ring_mask(inst);
  head = inst->rx_head;
  next = (uint16_t)((head + 1u) & mask);

  if (next == inst->rx_tail) {
    inst->rx_dropped++;
    return;
  }

  inst->rx_ring[head] = byte;
  inst->rx_head = next; /* 先写数据，后移游标 */
}

uint16_t mx01p_available(const mx01p_t *inst)
{
  if ((inst == NULL) || (inst->rx_ring == NULL)) {
    return 0u;
  }

  return (uint16_t)((inst->rx_head - inst->rx_tail) & mx01p_ring_mask(inst));
}

/*---------------------------------------------------------------------------*/
/* RX 分流器（中断上下文）                                                    */
/*---------------------------------------------------------------------------*/

/** @brief 把候选行里的字节整体当数据吐出，并清空行缓冲 */
static void mx01p_line_to_payload(mx01p_t *inst)
{
  uint16_t i;

  for (i = 0u; i < inst->line_len; i++) {
    mx01p_ring_push(inst, (uint8_t)inst->line_buf[i]);
  }
  inst->line_len = 0u;
}

/** @brief 把一行投进应答邮箱；邮箱已有内容则丢弃新行并计数
 *  @note 存进去的是**去掉行尾 CR/LF** 的文本：上层要拿它和 "OK"/"ERROR" 直接比，
 *        数据通路那边则保留原始字节（见 mx01p_line_to_payload）。
 */
static void mx01p_line_to_mailbox(mx01p_t *inst)
{
  uint16_t len = inst->line_len;

  if (inst->resp_ready) {
    inst->resp_overrun++;
    inst->line_len = 0u;
    return;
  }

  while ((len > 0u) &&
         ((inst->line_buf[len - 1u] == '\r') || (inst->line_buf[len - 1u] == '\n'))) {
    len--;
  }

  if (len >= (uint16_t)sizeof(inst->resp)) {
    len = (uint16_t)(sizeof(inst->resp) - 1u);
  }

  (void)memcpy(inst->resp, inst->line_buf, len);
  inst->resp[len] = '\0';
  inst->resp_len = len;
  inst->resp_ready = true;
  inst->line_len = 0u;
}

/**
 * @brief 解析 "TYP,MAC" 形式的连接信息
 * @param s 指向前缀之后的位置，可带 "CONN" 与空白
 * @retval true 解析成功
 */
static bool mx01p_parse_conn(const char *s, uint8_t *link_type, char *mac,
                             uint16_t mac_size, uint16_t *mac_len)
{
  uint32_t typ = 0u;
  uint16_t n = 0u;

  if ((s == NULL) || (mac == NULL) || (mac_size == 0u)) {
    return false;
  }

  while (*s == ' ') {
    s++;
  }
  if (strncmp(s, "CONN", 4u) == 0) {
    s += 4;
  }
  while (*s == ' ') {
    s++;
  }
  if ((*s < '0') || (*s > '9')) {
    return false;
  }
  while ((*s >= '0') && (*s <= '9')) {
    typ = (typ * 10u) + (uint32_t)(*s - '0');
    s++;
  }
  if (typ > 0xFFu) {
    return false;
  }
  while (*s == ' ') {
    s++;
  }
  if (*s == ',') {
    s++;
  }
  while (*s == ' ') {
    s++;
  }

  while (mx01p_is_hex(*s)) {
    if ((n + 1u) >= mac_size) {
      return false;
    }
    mac[n++] = *s;
    s++;
  }
  mac[n] = '\0';

  /* 手册里 MAC 固定 12 个十六进制字符 */
  if (n != (uint16_t)MX01P_MAC_STR_LEN) {
    return false;
  }

  if (link_type != NULL) {
    *link_type = (uint8_t)typ;
  }
  if (mac_len != NULL) {
    *mac_len = n;
  }
  return true;
}

/** @brief 分发 +CONNECTED / +DISCONN 事件并更新链路状态 */
static void mx01p_dispatch_link(mx01p_t *inst, bool connected, const char *value)
{
  uint8_t typ = 0u;
  char mac[MX01P_MAC_STR_LEN + 1u];
  bool ok;

  mac[0] = '\0';
  ok = mx01p_parse_conn(value, &typ, mac, (uint16_t)sizeof(mac), NULL);

  if (ok) {
    inst->link_type = typ;
    (void)memcpy(inst->link_mac, mac, sizeof(mac));
    inst->link_up = connected;
  }

  if (inst->evt_cb != NULL) {
    inst->evt_cb(inst, connected ? MX01P_EVENT_CONNECTED
                                 : MX01P_EVENT_DISCONNECTED,
                 typ, ok ? mac : NULL);
  }
}

/**
 * @brief 收齐一行后判定归属：事件 / AT 应答 / 数据
 * @note 判定顺序很关键：事件必须在"AT 事务进行中"之前判定，否则重启时的 +READY
 *       会被当成 AT 应答塞进邮箱，永远不会有人取走。
 */
static void mx01p_line_complete(mx01p_t *inst)
{
  uint16_t text_len = inst->line_len;
  const char *line = inst->line_buf;

  /* 去掉行尾的 CR/LF，便于和手册里的字符串直接比较 */
  while ((text_len > 0u) &&
         ((line[text_len - 1u] == '\r') || (line[text_len - 1u] == '\n'))) {
    text_len--;
  }

  if ((text_len == 0u) || (line[0] != '+')) {
    /* OK / ERROR / 或干脆是空行：只有 AT 事务在等的时候才当应答 */
    if (inst->at_pending) {
      mx01p_line_to_mailbox(inst);
    } else {
      mx01p_line_to_payload(inst);
    }
    return;
  }

  /* 从这里开始都是 '+' 开头的行 */
  if (text_len >= 6u && (memcmp(line, "+READY", 6u) == 0)) {
    inst->ready_edge = true;
    inst->line_len = 0u;
    if (inst->evt_cb != NULL) {
      inst->evt_cb(inst, MX01P_EVENT_READY, 0u, NULL);
    }
    return;
  }

  /* ---- 连接/断开事件 ----
     名字必须匹配到 ':' 之前，且不能要求冒号位置：手册写 `+CONNECTED:TYP,MAC` /
     `+DISCONN:TYP,MAC`，而实测 V0.0.8 固件发的是 `+CONNECTED:` / `+DISCONNECTED:`。
     按名字前缀（"CONNECT"/"DISCONN"）匹配，两种拼写都认；
     早期实现写死 "+DISCONN:"（含冒号），结果漏判了 +DISCONNECTED:，见 README 第 10 节。 */
  if (line[0] == '+') {
    uint16_t colon = 0u;
    uint16_t i;

    for (i = 1u; i < text_len; i++) {
      if (line[i] == ':') {
        colon = i;
        break;
      }
    }

    if ((colon >= 8u) && (memcmp(&line[1], "DISCONN", 7u) == 0)) {
      mx01p_dispatch_link(inst, false, &line[colon + 1u]);
      inst->line_len = 0u;
      return;
    }
    if ((colon >= 8u) && (memcmp(&line[1], "CONNECT", 7u) == 0)) {
      mx01p_dispatch_link(inst, true, &line[colon + 1u]);
      inst->line_len = 0u;
      return;
    }
  }

  if (inst->at_pending) {
    mx01p_line_to_mailbox(inst);
    return;
  }

  /* 空闲时收到不认识的 '+' 行：可能是以 '+' 开头的二进制载荷，退回数据通路 */
  mx01p_line_to_payload(inst);
}

void mx01p_rx_feed(mx01p_t *inst, const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if ((inst == NULL) || (data == NULL) || (len == 0u)) {
    return;
  }

  for (i = 0u; i < len; i++) {
    uint8_t byte = data[i];

    inst->line_tick = mx01p_port_tick_ms();

    if (inst->line_raw) {
      /* 已判定为数据：直通到行尾，遇到 '\n' 回到行首状态 */
      mx01p_ring_push(inst, byte);
      if (byte == (uint8_t)'\n') {
        inst->line_raw = false;
      }
      continue;
    }

    if (inst->line_len == 0u) {
      /* 行首：不像控制行的首字节立刻进数据缓冲，零延迟 */
      if ((byte == (uint8_t)'+') || (byte == (uint8_t)'O') ||
          (byte == (uint8_t)'E')) {
        inst->line_buf[0] = (char)byte;
        inst->line_len = 1u;
      } else {
        mx01p_ring_push(inst, byte);
      }
      continue;
    }

    if ((inst->line_len + 1u) >= MX01P_LINE_MAX) {
      /* 候选行超长：手册里最长的应答也装得下，所以这段一定是数据 */
      mx01p_line_to_payload(inst);
      inst->line_raw = true;
      mx01p_ring_push(inst, byte);
      if (byte == (uint8_t)'\n') {
        inst->line_raw = false;
      }
      continue;
    }

    inst->line_buf[inst->line_len] = (char)byte;
    inst->line_len++;

    if (byte == (uint8_t)'\n') {
      mx01p_line_complete(inst);
    }
  }
}

void mx01p_poll(mx01p_t *inst)
{
  uint32_t key;

  if (inst == NULL) {
    return;
  }

  mx01p_port_poll();

  key = mx01p_port_irq_lock();
  if ((inst->line_len != 0u) && !inst->line_raw &&
      ((uint32_t)(mx01p_port_tick_ms() - inst->line_tick) >= MX01P_RX_IDLE_FLUSH_MS)) {
    /* 一行攒了半天没有下文：不可能是完整控制行，当数据交出去 */
    mx01p_line_to_payload(inst);
  }
  mx01p_port_irq_unlock(key);
}

/*---------------------------------------------------------------------------*/
/* 数据面                                                                    */
/*---------------------------------------------------------------------------*/

uint16_t mx01p_read(mx01p_t *inst, uint8_t *buf, uint16_t max_len)
{
  uint16_t n = 0u;

  if ((inst == NULL) || (buf == NULL) || (inst->rx_ring == NULL)) {
    return 0u;
  }

  /* 先把可能停在候选行里的尾巴冲进缓冲，再取数据 */
  mx01p_poll(inst);

  while (n < max_len) {
    uint16_t tail = inst->rx_tail;

    if (tail == inst->rx_head) {
      break;
    }

    buf[n++] = inst->rx_ring[tail];
    inst->rx_tail = (uint16_t)((tail + 1u) & mx01p_ring_mask(inst));
  }

  return n;
}

void mx01p_flush_rx(mx01p_t *inst)
{
  uint32_t key;

  if (inst == NULL) {
    return;
  }

  key = mx01p_port_irq_lock();
  inst->rx_tail = inst->rx_head;
  inst->line_len = 0u;
  inst->line_raw = false;
  inst->resp_ready = false;
  inst->resp_len = 0u;
  mx01p_port_irq_unlock(key);
}

/*---------------------------------------------------------------------------*/
/* 睡眠                                                                      */
/*---------------------------------------------------------------------------*/

/** @brief 内部用：确保模块处于唤醒状态（睡过就唤醒并等恢复） */
static void mx01p_wake(mx01p_t *inst)
{
  if (!inst->asleep) {
    return;
  }

  mx01p_port_set_sleep(false);
  inst->asleep = false;
  mx01p_port_delay_ms(MX01P_WAKE_MS);
}

bool mx01p_is_asleep(const mx01p_t *inst)
{
  return (inst != NULL) && inst->asleep;
}

bool mx01p_is_connected(const mx01p_t *inst)
{
  return (inst != NULL) && inst->link_up;
}

void mx01p_get_link(const mx01p_t *inst, uint8_t *link_type, char *mac)
{
  uint32_t key;

  if (inst == NULL) {
    if (link_type != NULL) {
      *link_type = 0u;
    }
    if (mac != NULL) {
      mac[0] = '\0';
    }
    return;
  }

  key = mx01p_port_irq_lock();
  if (link_type != NULL) {
    *link_type = inst->link_type;
  }
  if (mac != NULL) {
    (void)memcpy(mac, inst->link_mac, sizeof(inst->link_mac));
  }
  mx01p_port_irq_unlock(key);
}

mx01p_status_t mx01p_set_sleep(mx01p_t *inst, bool sleep)
{
  if (inst == NULL) {
    return MX01P_ERR_PARAM;
  }
  if (!inst->ready) {
    return MX01P_ERR_STATE;
  }
  /* AT 事务中途睡眠必然超时，直接拒绝比让调用者等 500 ms 好 */
  if (inst->at_pending) {
    return MX01P_ERR_STATE;
  }
  if (sleep == inst->asleep) {
    return MX01P_OK;
  }

  mx01p_port_set_sleep(sleep);
  inst->asleep = sleep;

  if (!sleep) {
    mx01p_port_delay_ms(MX01P_WAKE_MS);
  }

  return MX01P_OK;
}

/*---------------------------------------------------------------------------*/
/* AT 事务                                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief 发一条命令并取回第一行应答
 * @param line 输出应答原文（含 CRLF 之外的内容，未去空白）
 * @note 调用前必须 ready 且没有其它 AT 事务在跑。超时/发送失败都会清 at_pending。
 */
static mx01p_status_t mx01p_exchange(mx01p_t *inst, const char *cmd, char *line,
                                     uint16_t cap, uint32_t timeout_ms)
{
  char buf[MX01P_CMD_MAX + 2u];
  size_t len;
  uint32_t t0;
  uint32_t key;
  uint16_t take;

  if ((inst == NULL) || (cmd == NULL) || (line == NULL) || (cap < 2u)) {
    return MX01P_ERR_PARAM;
  }
  if (!inst->ready) {
    return MX01P_ERR_STATE;
  }
  if (inst->at_pending) {
    return MX01P_ERR_STATE; /* 同步事务，不允许嵌套 */
  }
  if (timeout_ms == 0u) {
    timeout_ms = MX01P_AT_TIMEOUT_MS;
  }

  len = strlen(cmd);
  if ((len == 0u) || (len > (size_t)MX01P_CMD_MAX)) {
    return MX01P_ERR_NOMEM;
  }
  (void)memcpy(buf, cmd, len);
  buf[len] = '\r';
  buf[len + 1u] = '\n';

  mx01p_wake(inst);
  mx01p_poll(inst);

  key = mx01p_port_irq_lock();
  inst->resp_ready = false;
  inst->resp_len = 0u;
  inst->at_pending = true;
  mx01p_port_irq_unlock(key);

  if (!mx01p_port_uart_send((const uint8_t *)buf, (uint16_t)(len + 2u),
                            timeout_ms)) {
    inst->at_pending = false;
    return MX01P_ERR_IO;
  }

  t0 = mx01p_port_tick_ms();
  for (;;) {
    if (inst->resp_ready) {
      break;
    }
    if ((uint32_t)(mx01p_port_tick_ms() - t0) >= timeout_ms) {
      inst->at_pending = false;
      return MX01P_ERR_TIMEOUT;
    }
    mx01p_port_delay_ms(1u);
    mx01p_poll(inst);
    if (inst->resp_ready) {
      break;
    }
  }

  key = mx01p_port_irq_lock();
  take = inst->resp_len;
  if (take > (uint16_t)(cap - 1u)) {
    take = (uint16_t)(cap - 1u);
  }
  (void)memcpy(line, inst->resp, take);
  line[take] = '\0';
  inst->resp_ready = false;
  inst->resp_len = 0u;
  inst->at_pending = false;
  mx01p_port_irq_unlock(key);

  return MX01P_OK;
}

/** @brief 去掉首尾空白，返回去空白后的长度 */
static uint16_t mx01p_trim(const char **s)
{
  const char *p = *s;
  size_t len;

  while ((*p == ' ') || (*p == '\r') || (*p == '\n')) {
    p++;
  }
  len = strlen(p);
  while ((len > 0u) &&
         ((p[len - 1u] == ' ') || (p[len - 1u] == '\r') ||
          (p[len - 1u] == '\n'))) {
    len--;
  }

  *s = p;
  return (uint16_t)len;
}

/** @brief 查询类命令的公共实现：发命令 → 匹配 key → 取出值 */
static mx01p_status_t mx01p_get_str(mx01p_t *inst, const char *cmd,
                                    const char *key, char *out,
                                    uint16_t out_size, uint32_t timeout_ms)
{
  char line[MX01P_LINE_MAX];
  mx01p_status_t st;
  const char *value;
  uint16_t value_len;

  if ((out == NULL) || (out_size < 2u) || (key == NULL)) {
    return MX01P_ERR_PARAM;
  }

  st = mx01p_exchange(inst, cmd, line, (uint16_t)sizeof(line), timeout_ms);
  if (st != MX01P_OK) {
    return st;
  }

  if (strcmp(line, "ERROR") == 0) {
    return MX01P_ERR_PROTOCOL;
  }

  value = strstr(line, key);
  if (value == NULL) {
    return MX01P_ERR_PROTOCOL;
  }
  value += strlen(key);

  value_len = mx01p_trim(&value);
  if (value_len == 0u) {
    return MX01P_ERR_PROTOCOL;
  }
  if (value_len >= out_size) {
    value_len = (uint16_t)(out_size - 1u);
  }

  (void)memcpy(out, value, value_len);
  out[value_len] = '\0';

  return MX01P_OK;
}

/** @brief 查询类命令取十进制整数 */
static mx01p_status_t mx01p_get_int(mx01p_t *inst, const char *cmd,
                                    const char *key, int32_t *out)
{
  char text[12];
  mx01p_status_t st;

  if (out == NULL) {
    return MX01P_ERR_PARAM;
  }

  st = mx01p_get_str(inst, cmd, key, text, (uint16_t)sizeof(text),
                     MX01P_AT_TIMEOUT_MS);
  if (st != MX01P_OK) {
    return st;
  }

  if (!mx01p_parse_int(text, out)) {
    return MX01P_ERR_PROTOCOL;
  }

  return MX01P_OK;
}

mx01p_status_t mx01p_at_query(mx01p_t *inst, const char *command, const char *key,
                              char *out, uint16_t out_size, uint32_t timeout_ms)
{
  return mx01p_get_str(inst, command, key, out, out_size, timeout_ms);
}

mx01p_status_t mx01p_at_exec(mx01p_t *inst, const char *command,
                             uint32_t timeout_ms)
{
  char line[MX01P_LINE_MAX];
  mx01p_status_t st;

  st = mx01p_exchange(inst, command, line, (uint16_t)sizeof(line), timeout_ms);
  if (st != MX01P_OK) {
    return st;
  }

  if (strcmp(line, "ERROR") == 0) {
    return MX01P_ERR_PROTOCOL;
  }

  /* 手册里像 AT+DISCONN=NUM 这类命令回的是 +DISCONN:...，也视为成功 */
  return MX01P_OK;
}

/*---------------------------------------------------------------------------*/
/* 生命周期                                                                  */
/*---------------------------------------------------------------------------*/

mx01p_status_t mx01p_reset(mx01p_t *inst)
{
  uint32_t t0;
  uint32_t key;

  if (inst == NULL) {
    return MX01P_ERR_PARAM;
  }
  if (inst->rx_ring == NULL) {
    return MX01P_ERR_STATE;
  }

  inst->ready = false;

  /* 先让模块醒着，再拉复位：RES 期间 BRTS 高会省电，但复位完成后马上要等 +READY */
  mx01p_port_set_sleep(false);
  inst->asleep = false;

  key = mx01p_port_irq_lock();
  inst->ready_edge = false;
  inst->at_pending = false;
  inst->resp_ready = false;
  inst->resp_len = 0u;
  inst->line_len = 0u;
  inst->line_raw = false;
  inst->link_up = false;
  inst->link_type = 0u;
  inst->link_mac[0] = '\0';
  inst->rx_head = 0u;
  inst->rx_tail = 0u;
  mx01p_port_irq_unlock(key);

  mx01p_port_reset(true);
  mx01p_port_delay_ms(MX01P_RESET_LOW_MS);
  mx01p_port_attach(inst); /* HAL_Delay 期间可能停摆，重新挂一次最稳 */
  mx01p_port_reset(false);

  t0 = mx01p_port_tick_ms();
  while (!inst->ready_edge) {
    if ((uint32_t)(mx01p_port_tick_ms() - t0) >= inst->ready_timeout_ms) {
      return MX01P_ERR_TIMEOUT;
    }
    mx01p_port_delay_ms(1u);
  }

  inst->ready = true;

  /* +READY 之后模块可能还吐别的内容，清一次避免污染第一条命令的应答 */
  mx01p_poll(inst);

  return MX01P_OK;
}

mx01p_status_t mx01p_init(mx01p_t *inst, const mx01p_cfg_t *cfg)
{
  if ((inst == NULL) || (cfg == NULL) || (cfg->rx_ring == NULL)) {
    return MX01P_ERR_PARAM;
  }
  /* 容量必须是 2 的幂，否则环形缓冲的掩码算法会越界 */
  if ((cfg->rx_ring_size < 8u) ||
      ((cfg->rx_ring_size & (uint16_t)(cfg->rx_ring_size - 1u)) != 0u)) {
    return MX01P_ERR_PARAM;
  }

  (void)memset(inst, 0, sizeof(*inst));
  inst->rx_ring = cfg->rx_ring;
  inst->rx_ring_size = cfg->rx_ring_size;
  inst->evt_cb = cfg->evt_cb;
  inst->user = cfg->user;
  inst->ready_timeout_ms = (cfg->ready_timeout_ms != 0u)
                               ? cfg->ready_timeout_ms
                               : MX01P_READY_TIMEOUT_MS;

  /* 上电默认让模块醒着（BRTS 低），否则 +READY 永远出不来 */
  mx01p_port_set_sleep(false);

  mx01p_port_attach(inst);

  return mx01p_reset(inst);
}

mx01p_status_t mx01p_reboot(mx01p_t *inst)
{
  mx01p_status_t st;
  uint32_t t0;
  uint32_t key;

  if (inst == NULL) {
    return MX01P_ERR_PARAM;
  }
  if (!inst->ready) {
    return MX01P_ERR_STATE;
  }

  /* 必须在发命令前清标志：否则上一轮的 +READY 会被当成本次重启完成 */
  key = mx01p_port_irq_lock();
  inst->ready_edge = false;
  mx01p_port_irq_unlock(key);

  st = mx01p_at_exec(inst, "AT+REBOOT=1", MX01P_AT_TIMEOUT_MS);
  if ((st != MX01P_OK) && (st != MX01P_ERR_TIMEOUT)) {
    /* 少数固件先重启再回 OK；只有明确的 ERROR/通讯失败才算失败 */
    return st;
  }

  t0 = mx01p_port_tick_ms();
  while (!inst->ready_edge) {
    if ((uint32_t)(mx01p_port_tick_ms() - t0) >= inst->ready_timeout_ms) {
      inst->ready = false;
      return MX01P_ERR_TIMEOUT;
    }
    mx01p_port_delay_ms(1u);
  }

  inst->link_up = false;
  inst->link_type = 0u;
  inst->link_mac[0] = '\0';
  inst->ready = true;
  mx01p_flush_rx(inst);

  return MX01P_OK;
}

mx01p_status_t mx01p_restore(mx01p_t *inst)
{
  return mx01p_at_exec(inst, "AT+RESET=1", MX01P_AT_TIMEOUT_MS);
}

/*---------------------------------------------------------------------------*/
/* 数据发送                                                                  */
/*---------------------------------------------------------------------------*/

mx01p_status_t mx01p_send(mx01p_t *inst, const uint8_t *data, uint16_t len,
                          uint32_t timeout_ms)
{
  if ((inst == NULL) || (data == NULL) || (len == 0u) ||
      (len > (uint16_t)MX01P_PORT_TX_MAX)) {
    return MX01P_ERR_PARAM;
  }
  if (!inst->ready) {
    return MX01P_ERR_STATE;
  }

  /* 睡眠期间模块只能收不能发，发送前必须唤醒；唤醒后保持醒着，省电由调用者决定 */
  mx01p_wake(inst);
  mx01p_poll(inst);

  if (!mx01p_port_uart_send(data, len, timeout_ms)) {
    return MX01P_ERR_IO;
  }

  return MX01P_OK;
}

/*---------------------------------------------------------------------------*/
/* 参数读写                                                                  */
/*---------------------------------------------------------------------------*/

mx01p_status_t mx01p_read_mac(mx01p_t *inst, char *out, uint16_t out_size)
{
  if ((out == NULL) || (out_size < (uint16_t)(MX01P_MAC_STR_LEN + 1u))) {
    return MX01P_ERR_PARAM;
  }

  return mx01p_get_str(inst, "AT+MAC?", "+MAC:", out, out_size,
                       MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_write_mac(mx01p_t *inst, const char *mac)
{
  char cmd[MX01P_CMD_MAX + 1u];

  if ((mac == NULL) || (strlen(mac) != (size_t)MX01P_MAC_STR_LEN) ||
      !mx01p_is_hex_str(mac)) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join("AT+MAC=", mac, cmd, (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_name(mx01p_t *inst, char *out, uint16_t out_size)
{
  if ((out == NULL) || (out_size < 2u)) {
    return MX01P_ERR_PARAM;
  }

  return mx01p_get_str(inst, "AT+NAME?", "+NAME:", out, out_size,
                       MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_write_name(mx01p_t *inst, const char *name)
{
  char cmd[MX01P_CMD_MAX + 1u];

  if ((name == NULL) || (strlen(name) > (size_t)MX01P_NAME_STR_LEN)) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join("AT+NAME=", name, cmd, (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_adv(mx01p_t *inst, bool *on)
{
  int32_t value;
  mx01p_status_t st;

  if (on == NULL) {
    return MX01P_ERR_PARAM;
  }

  st = mx01p_get_int(inst, "AT+ADV?", "+ADV:", &value);
  if (st == MX01P_OK) {
    *on = (value != 0);
  }

  return st;
}

mx01p_status_t mx01p_set_adv(mx01p_t *inst, bool on)
{
  char cmd[MX01P_CMD_MAX + 1u];

  if (!mx01p_cmd_join_int("AT+ADV=", on ? 1 : 0, cmd, (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_baud(mx01p_t *inst, mx01p_baud_t *baud)
{
  int32_t value;
  mx01p_status_t st;

  if (baud == NULL) {
    return MX01P_ERR_PARAM;
  }

  st = mx01p_get_int(inst, "AT+UART?", "+UART:", &value);
  if (st != MX01P_OK) {
    return st;
  }
  if ((value < 0) || (value > (int32_t)MX01P_BAUD_921600)) {
    return MX01P_ERR_PROTOCOL;
  }

  *baud = (mx01p_baud_t)value;
  return MX01P_OK;
}

mx01p_status_t mx01p_write_baud(mx01p_t *inst, mx01p_baud_t baud)
{
  char cmd[MX01P_CMD_MAX + 1u];

  if ((unsigned)baud > (unsigned)MX01P_BAUD_921600) {
    return MX01P_ERR_PARAM;
  }
  /* 本板 USART1 固定 115200：写成别的码值会让模块立刻失联 */
  if ((unsigned)baud != (unsigned)MX01P_BAUD_115200) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join_int("AT+UART=", (int32_t)baud, cmd,
                          (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_adv_interval(mx01p_t *inst, uint16_t *interval_ms)
{
  int32_t value;
  mx01p_status_t st;

  if (interval_ms == NULL) {
    return MX01P_ERR_PARAM;
  }

  st = mx01p_get_int(inst, "AT+AINTVL?", "+AINTVL:", &value);
  if (st != MX01P_OK) {
    return st;
  }
  if ((value < 0) || (value > 0xFFFF)) {
    return MX01P_ERR_PROTOCOL;
  }

  *interval_ms = (uint16_t)value;
  return MX01P_OK;
}

mx01p_status_t mx01p_write_adv_interval(mx01p_t *inst, uint16_t interval_ms)
{
  char cmd[MX01P_CMD_MAX + 1u];

  if ((interval_ms < 20u) || (interval_ms > 10000u)) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join_int("AT+AINTVL=", (int32_t)interval_ms, cmd,
                          (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_version(mx01p_t *inst, char *out, uint16_t out_size)
{
  if ((out == NULL) || (out_size < 2u)) {
    return MX01P_ERR_PARAM;
  }

  return mx01p_get_str(inst, "AT+VER?", "+VER:", out, out_size,
                       MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_tx_power(mx01p_t *inst, mx01p_tx_power_t *power)
{
  int32_t value;
  mx01p_status_t st;

  if (power == NULL) {
    return MX01P_ERR_PARAM;
  }

  st = mx01p_get_int(inst, "AT+TXPOWER?", "+TXPOWER:", &value);
  if (st != MX01P_OK) {
    return st;
  }
  if ((value < 0) || (value > (int32_t)MX01P_POWER_N18DBM)) {
    return MX01P_ERR_PROTOCOL;
  }

  *power = (mx01p_tx_power_t)value;
  return MX01P_OK;
}

mx01p_status_t mx01p_write_tx_power(mx01p_t *inst, mx01p_tx_power_t power)
{
  char cmd[MX01P_CMD_MAX + 1u];

  if ((unsigned)power > (unsigned)MX01P_POWER_N18DBM) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join_int("AT+TXPOWER=", (int32_t)power, cmd,
                          (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_uuid(mx01p_t *inst, mx01p_uuid_kind_t kind, char *out,
                               uint16_t out_size)
{
  if ((out == NULL) || (out_size < 5u) ||
      ((unsigned)kind > (unsigned)MX01P_UUID_WRITE)) {
    return MX01P_ERR_PARAM;
  }

  return mx01p_get_str(inst, g_uuid_cmd[(unsigned)kind], g_uuid_key[(unsigned)kind],
                       out, out_size, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_write_uuid(mx01p_t *inst, mx01p_uuid_kind_t kind,
                                const char *uuid)
{
  char cmd[MX01P_CMD_MAX + 1u];
  size_t len;

  if ((uuid == NULL) || ((unsigned)kind > (unsigned)MX01P_UUID_WRITE)) {
    return MX01P_ERR_PARAM;
  }
  len = strlen(uuid);
  /* 手册只接受 16 bit（4 字符）或 128 bit（32 字符）两种形式 */
  if (((len != 4u) && (len != (size_t)MX01P_UUID_STR_LEN)) ||
      !mx01p_is_hex_str(uuid)) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join(g_uuid_set[(unsigned)kind], uuid, cmd,
                      (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_adv_data(mx01p_t *inst, char *out, uint16_t out_size)
{
  if ((out == NULL) ||
      (out_size < (uint16_t)(MX01P_ADV_DATA_HEX_LEN + 1u))) {
    return MX01P_ERR_PARAM;
  }

  return mx01p_get_str(inst, "AT+AMDATA?", "+AMDATA:", out, out_size,
                       MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_write_adv_data(mx01p_t *inst, const char *hex)
{
  char cmd[MX01P_CMD_MAX + 1u];
  size_t len;

  if (hex == NULL) {
    return MX01P_ERR_PARAM;
  }
  len = strlen(hex);
  if ((len > (size_t)MX01P_ADV_DATA_HEX_LEN) || ((len % 2u) != 0u) ||
      !mx01p_is_hex_str(hex)) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join("AT+AMDATA=", hex, cmd, (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_disconnect(mx01p_t *inst, uint8_t mode)
{
  char cmd[MX01P_CMD_MAX + 1u];

  if (mode > 1u) {
    return MX01P_ERR_PARAM;
  }
  if (!mx01p_cmd_join_int("AT+DISCONN=", (int32_t)mode, cmd,
                          (uint16_t)sizeof(cmd))) {
    return MX01P_ERR_NOMEM;
  }

  return mx01p_at_exec(inst, cmd, MX01P_AT_TIMEOUT_MS);
}

mx01p_status_t mx01p_read_link(mx01p_t *inst, uint8_t *link_type, char *mac)
{
  char line[MX01P_LINE_MAX];
  mx01p_status_t st;
  const char *value;
  uint8_t typ = 0u;
  char tmp[MX01P_MAC_STR_LEN + 1u];

  if (mac == NULL) {
    return MX01P_ERR_PARAM;
  }

  st = mx01p_exchange(inst, "AT+DEV?", line, (uint16_t)sizeof(line),
                      MX01P_AT_TIMEOUT_MS);
  if (st != MX01P_OK) {
    return st;
  }

  value = strstr(line, "+DEV:");
  if (value == NULL) {
    return MX01P_ERR_PROTOCOL;
  }
  value += 5;

  if (!mx01p_parse_conn(value, &typ, tmp, (uint16_t)sizeof(tmp), NULL)) {
    return MX01P_ERR_PROTOCOL; /* 没有连接设备时没有 CONN 行 */
  }

  if (link_type != NULL) {
    *link_type = typ;
  }
  (void)memcpy(mac, tmp, sizeof(tmp));

  inst->link_up = true;
  inst->link_type = typ;
  (void)memcpy(inst->link_mac, tmp, sizeof(tmp));

  return MX01P_OK;
}

/*---------------------------------------------------------------------------*/
/* 上电自检                                                                  */
/*---------------------------------------------------------------------------*/

/** @brief 自检里一次 AT 查询的统一记账 */
static void mx01p_probe_note(mx01p_probe_t *out, int32_t st, bool mandatory)
{
  out->at_total++;
  if ((st != (int32_t)MX01P_OK) && mandatory) {
    out->at_fail++;
  }
}

bool mx01p_probe(mx01p_t *inst, const mx01p_cfg_t *cfg, mx01p_probe_t *out)
{
  mx01p_status_t st;
  uint32_t t0;
  char text[16];
  uint8_t typ = 0u;
  char mac[MX01P_MAC_STR_LEN + 1u];

  if ((inst == NULL) || (cfg == NULL) || (out == NULL)) {
    return false;
  }

  (void)memset(out, 0, sizeof(*out));
  out->stage = (uint32_t)MX01P_PROBE_STAGE_NONE;
  out->link_st = (int32_t)MX01P_ERR_STATE;

  /* ---- 1. 初始化：失败重试一次，覆盖上电时序敏感的板子 ---- */
  t0 = mx01p_port_tick_ms();
  st = mx01p_init(inst, cfg);
  if (st != MX01P_OK) {
    out->reset_retry = 1u;
    st = mx01p_init(inst, cfg);
  }
  out->init_st = (int32_t)st;
  out->ready_ms = (uint32_t)(mx01p_port_tick_ms() - t0);

  if (st != MX01P_OK) {
    out->rx_dropped = inst->rx_dropped;
    out->resp_overrun = inst->resp_overrun;
    return false;
  }
  out->stage = (uint32_t)MX01P_PROBE_STAGE_INIT;

  /* ---- 2. 一轮参数查询（对应手册每一条 AT 指令） ---- */
  out->mac_st = (int32_t)mx01p_read_mac(inst, out->mac, (uint16_t)sizeof(out->mac));
  mx01p_probe_note(out, out->mac_st, true);

  out->name_st = (int32_t)mx01p_read_name(inst, out->name,
                                          (uint16_t)sizeof(out->name));
  mx01p_probe_note(out, out->name_st, true);

  out->version_st = (int32_t)mx01p_read_version(inst, out->version,
                                                (uint16_t)sizeof(out->version));
  mx01p_probe_note(out, out->version_st, true);

  {
    mx01p_baud_t baud = MX01P_BAUD_115200;

    out->baud_st = (int32_t)mx01p_read_baud(inst, &baud);
    out->baud_code = (int32_t)baud;
    mx01p_probe_note(out, out->baud_st, true);
  }

  {
    bool on = false;

    out->adv_st = (int32_t)mx01p_read_adv(inst, &on);
    out->adv_on = on ? 1 : 0;
    mx01p_probe_note(out, out->adv_st, true);
  }

  {
    uint16_t interval = 0u;

    out->interval_st = (int32_t)mx01p_read_adv_interval(inst, &interval);
    out->interval_ms = (int32_t)interval;
    mx01p_probe_note(out, out->interval_st, true);
  }

  {
    mx01p_tx_power_t power = MX01P_POWER_0DBM;

    out->power_st = (int32_t)mx01p_read_tx_power(inst, &power);
    out->power_code = (int32_t)power;
    mx01p_probe_note(out, out->power_st, true);
  }

  /* 三条 UUID 通道分别读，任一失败算整体失败 */
  {
    int32_t s1;
    int32_t s2;
    int32_t s3;

    s1 = (int32_t)mx01p_read_uuid(inst, MX01P_UUID_SERVICE, out->uuid_service,
                                  (uint16_t)sizeof(out->uuid_service));
    s2 = (int32_t)mx01p_read_uuid(inst, MX01P_UUID_NOTIFY, out->uuid_notify,
                                  (uint16_t)sizeof(out->uuid_notify));
    s3 = (int32_t)mx01p_read_uuid(inst, MX01P_UUID_WRITE, out->uuid_write,
                                  (uint16_t)sizeof(out->uuid_write));
    out->uuid_st = ((s1 == (int32_t)MX01P_OK) && (s2 == (int32_t)MX01P_OK) &&
                    (s3 == (int32_t)MX01P_OK))
                       ? (int32_t)MX01P_OK
                       : ((s1 != (int32_t)MX01P_OK) ? s1
                                                    : ((s2 != (int32_t)MX01P_OK) ? s2 : s3));
    mx01p_probe_note(out, out->uuid_st, true);
  }

  /* 连接状态：手边没有手机时"无连接"是正常结果，不计入失败 */
  out->link_st = (int32_t)mx01p_read_link(inst, &typ, mac);
  mx01p_probe_note(out, out->link_st, false);

  out->stage = (uint32_t)MX01P_PROBE_STAGE_AT;

  /* ---- 3. 睡眠脚验证：睡下后模块不能再发串口，AT 必然无应答 ---- */
  out->sleep_st = (int32_t)mx01p_set_sleep(inst, true);
  out->at_total++;

  if (out->sleep_st == (int32_t)MX01P_OK) {
    mx01p_flush_rx(inst);
    (void)mx01p_port_uart_send((const uint8_t *)"AT+MAC?\r\n", 9u, 100u);

    out->sleep_blocked = 1;
    t0 = mx01p_port_tick_ms();
    while ((uint32_t)(mx01p_port_tick_ms() - t0) < MX01P_SLEEP_WATCH_MS) {
      if (mx01p_available(inst) != 0u) {
        out->sleep_blocked = 0; /* 睡眠期间还能回数据 → BRTS 没接对 */
        break;
      }
      mx01p_port_delay_ms(1u);
    }
    if (out->sleep_blocked == 0) {
      out->at_fail++;
    }

    /* 唤醒并等模块把积压处理完，再确认一次 AT 通畅 */
    out->wake_st = (int32_t)mx01p_set_sleep(inst, false);
    mx01p_port_delay_ms(MX01P_WAKE_SETTLE_MS);
    mx01p_flush_rx(inst);

    st = mx01p_read_version(inst, text, (uint16_t)sizeof(text));
    mx01p_probe_note(out, (int32_t)st, true);
    if (st != MX01P_OK) {
      out->wake_st = (int32_t)st;
    }
  }

  out->stage = (uint32_t)MX01P_PROBE_STAGE_SLEEP;

  /* ---- 4. 数据通路回环：从透传口发 AT+VER?，再从 mx01p_read() 收回来 ---- */
  mx01p_flush_rx(inst);
  st = mx01p_send(inst, (const uint8_t *)"AT+VER?\r\n", 9u, 100u);
  out->loop_st = (int32_t)st;

  if (st == MX01P_OK) {
    uint16_t got = 0u;

    t0 = mx01p_port_tick_ms();
    while ((uint32_t)(mx01p_port_tick_ms() - t0) < MX01P_LOOPBACK_MS) {
      got = (uint16_t)(got + mx01p_read(inst, (uint8_t *)&out->loopback[got],
                                        (uint16_t)(sizeof(out->loopback) - 1u - got)));
      out->loopback[got] = '\0';
      if (strstr(out->loopback, "+VER:") != NULL) {
        break;
      }
      if (got >= (uint16_t)(sizeof(out->loopback) - 1u)) {
        break;
      }
      mx01p_port_delay_ms(1u);
    }
    out->loop_len = got;

    /* 期望回 "AT+VER?" 的应答，走的是"不认识的 '+' 行退回数据通路"那条分支 */
    if (strstr(out->loopback, "+VER:") == NULL) {
      out->loop_st = (int32_t)MX01P_ERR_PROTOCOL;
    }
  }
  mx01p_probe_note(out, out->loop_st, true);

  out->stage = (uint32_t)MX01P_PROBE_STAGE_LOOP;

  /* ---- 5. 软复位：AT+REBOOT=1 之后必须还能重新等到 +READY 并查询 ---- */
  out->reboot_st = (int32_t)mx01p_reboot(inst);
  out->at_total++;
  if (out->reboot_st == (int32_t)MX01P_OK) {
    st = mx01p_read_version(inst, text, (uint16_t)sizeof(text));
    out->at_total++;
    if (st != MX01P_OK) {
      out->reboot_st = (int32_t)st;
    }
  }
  if (out->reboot_st != (int32_t)MX01P_OK) {
    out->at_fail++;
  }

  out->stage = (uint32_t)MX01P_PROBE_STAGE_REBOOT;
  out->rx_dropped = inst->rx_dropped;
  out->resp_overrun = inst->resp_overrun;

  return (out->at_fail == 0u) && (out->sleep_blocked != 0) &&
         (out->wake_st == (int32_t)MX01P_OK) &&
         (out->loop_st == (int32_t)MX01P_OK) &&
         (out->reboot_st == (int32_t)MX01P_OK);
}
