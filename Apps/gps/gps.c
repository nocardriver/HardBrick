/**
 * @file    gps.c
 * @author  YangKY
 * @brief   GPS 应用层：NMEA 字节流 -> 按行交给 minmea 解析 -> 定位快照
 *
 * 通路：HAL_UARTEx_RxEventCallback (ISR)  --gps_rx_push()-->  环形缓冲
 *       --事件标志-->  gps 线程  -->  切行  -->  minmea  -->  g_nav 快照
 *
 * 为什么解析不放在中断里：minmea 一次只吃一条完整语句，而一次 DMA burst 是十几条
 * 语句拼接的字节流。中断只负责搬运，丢字节只毁掉当前一行，切行器会在下一个 '$'
 * 处自动重新同步。
 */

#include "gps.h"

#include "tx_api.h"
#include "minmea.h"
#include "SEGGER_RTT.h"

/* 环形缓冲容量：2 的幂（用掩码取模），能装下一次 DMA burst（最大 1024 B） */
#define GPS_RING_SIZE     2048u
#define GPS_RING_MASK     (GPS_RING_SIZE - 1u)

/* 单行缓冲：minmea 上限 + '\0'，超长行整行丢弃 */
#define GPS_LINE_MAX      (MINMEA_MAX_SENTENCE_LENGTH + 1u)

#define GPS_EVENT_RX_DATA 0x1u   /* 事件标志：缓冲里有新数据 */
#define GPS_WAIT_TICKS    10u    /* 线程等待超时：10 tick = 100 ms */

/**
 * @brief 单生产者（中断）/ 单消费者（线程）环形缓冲
 * @note  buf 与游标都要 volatile：保证"先写数据、后移游标"不被编译器重排
 */
typedef struct {
  volatile uint8_t  buf[GPS_RING_SIZE];
  volatile uint16_t head;   /* 仅中断写 */
  volatile uint16_t tail;   /* 仅线程写 */
} gps_ring_t;

/** 行切分状态（线程私有） */
typedef struct {
  char     line[GPS_LINE_MAX];
  uint16_t len;
  bool     overflow;        /* 本行超长，作废到行尾 */
} gps_line_t;

static gps_ring_t g_ring;

static TX_THREAD            g_thread;
static TX_EVENT_FLAGS_GROUP g_events;
static uint8_t              g_stack[GPS_THREAD_STACK_SIZE] __attribute__((aligned(8)));

/* 最新定位快照：解析线程写，调试器/其他模块读 */
static gps_nav_t g_nav;

/* ------------------------------------------------------------------------- */
/* 环形缓冲：中断写入，线程读出                                                */
/* ------------------------------------------------------------------------- */

void gps_rx_push(const uint8_t *data, uint16_t len)
{
  uint16_t head = g_ring.head;
  uint16_t used = (uint16_t)((head - g_ring.tail) & GPS_RING_MASK);
  uint16_t free_bytes = (uint16_t)(GPS_RING_MASK - used);
  uint16_t n = (len <= free_bytes) ? len : free_bytes;   /* 满则丢弃新数据 */
  uint16_t i;

  for (i = 0u; i < n; i++) {
    g_ring.buf[head] = data[i];
    head = (uint16_t)((head + 1u) & GPS_RING_MASK);   /* 先写数据、后移游标 */
  }
  g_ring.head = head;

  (void)tx_event_flags_set(&g_events, GPS_EVENT_RX_DATA, TX_OR);
}

/** @brief 从环形缓冲取一个字节，缓冲空时返回 false */
static bool gps_rx_pop(uint8_t *byte)
{
  uint16_t tail = g_ring.tail;

  if (tail == g_ring.head) {
    return false;
  }

  *byte = g_ring.buf[tail];
  g_ring.tail = (uint16_t)((tail + 1u) & GPS_RING_MASK);
  return true;
}

/**
 * @brief 逐字节喂入串口字节流，凑齐一整行时返回 true
 * @note  从 '$' 开始收集（丢字节造成的半帧残码会被跳过），剥掉 '\r'，遇 '\n' 结束。
 *        超长行（丢字节使两条语句粘连）整行丢弃：截断出来的半条语句过不了校验
 */
static bool gps_line_feed(gps_line_t *ctx, uint8_t byte)
{
  if (byte == '\n') {
    bool complete = (ctx->len != 0u) && (!ctx->overflow);

    if (complete) {
      ctx->line[ctx->len] = '\0';   /* minmea 要求 NUL 结尾 */
    }
    ctx->len      = 0u;
    ctx->overflow = false;
    return complete;
  }

  if (byte == '\r') {
    return false;
  }

  if (ctx->len == 0u) {
    if (byte != '$') {
      return false;              /* 只认 '$' 开头的语句 */
    }
    ctx->overflow = false;
  }

  if (ctx->len >= (GPS_LINE_MAX - 1u)) {
    ctx->overflow = true;        /* 超长：本行作废，等 '\n' 复位 */
    return false;
  }

  ctx->line[ctx->len++] = (char)byte;
  return false;
}

/* ------------------------------------------------------------------------- */
/* 解析：一次喂一条语句，按类型分发                                             */
/* ------------------------------------------------------------------------- */

/**
 * @brief 用 minmea 解析一条完整语句，合并更新快照
 * @param sentence '$' 开头、'\0' 结尾、已剥掉 CR/LF 的单条语句
 */
static void gps_parse_line(const char *sentence)
{
  switch (minmea_sentence_id(sentence, true)) {   /* true：校验和不符直接丢弃 */
  case MINMEA_SENTENCE_RMC: {
    struct minmea_sentence_rmc rmc;

    if (!minmea_parse_rmc(&rmc, sentence)) {
      break;
    }

    g_nav.date    = rmc.date;
    g_nav.time    = rmc.time;
    g_nav.lat_deg = minmea_tocoord(&rmc.latitude);
    g_nav.lon_deg = minmea_tocoord(&rmc.longitude);
    if (!rmc.valid) {
      g_nav.fix = 0u;   /* RMC 报无效就清零；有效时定位质量以 GGA 为准 */
    }

    /* 唯一一条观测输出，其余变量直接看调试器里的 g_nav。
       坐标按 1e-6 度整数打印（RTT printf 不支持 %f） */
    SEGGER_RTT_printf(0, "[GPS] %c %02d-%02d %02d:%02d:%02d lat=%d lon=%d fix=%d sats=%d\r\n",
                      rmc.valid ? 'A' : 'V',
                      g_nav.date.month, g_nav.date.day,
                      g_nav.time.hours, g_nav.time.minutes, g_nav.time.seconds,
                      (int)(g_nav.lat_deg * 1000000.0f), (int)(g_nav.lon_deg * 1000000.0f),
                      (int)g_nav.fix, (int)g_nav.numsat);
  } break;

  case MINMEA_SENTENCE_GGA: {
    struct minmea_sentence_gga gga;

    if (!minmea_parse_gga(&gga, sentence)) {
      break;
    }

    g_nav.fix    = (uint8_t)gga.fix_quality;   /* 0=无效 1=单点 2=差分 */
    g_nav.numsat = (uint8_t)gga.satellites_tracked;
    g_nav.alt_m  = minmea_tofloat(&gga.altitude);
  } break;

  default:
    break;   /* INVALID / UNKNOWN（$CRC、$GNTXT、GLL、GSA、GSV…）一律忽略 */
  }
}

/* ------------------------------------------------------------------------- */
/* 线程与对外接口                                                              */
/* ------------------------------------------------------------------------- */

static void gps_thread_entry(ULONG thread_input)
{
  gps_line_t ctx = {0};
  uint8_t    byte;
  ULONG      actual;

  (void)thread_input;

  for (;;) {
    /* 先排空缓冲（内核启动前收到的数据此时已经在里面了） */
    while (gps_rx_pop(&byte)) {
      if (gps_line_feed(&ctx, byte)) {
        gps_parse_line(ctx.line);   /* 一次只喂 minmea 一条语句 */
      }
    }

    /* 等中断唤醒；超时兜底。事件标志在下次排空前会一直保持置位，不会丢数据 */
    (void)tx_event_flags_get(&g_events, GPS_EVENT_RX_DATA, TX_OR_CLEAR,
                             &actual, GPS_WAIT_TICKS);
  }
}

void gps_init(void)
{
  /* 不清零 g_ring：UART 接收在 tx_kernel_enter() 之前就已启动 */
  (void)tx_event_flags_create(&g_events, "gps rx");
  (void)tx_thread_create(&g_thread, "gps", gps_thread_entry, 0u,
                         g_stack, sizeof(g_stack),
                         4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);
}

void gps_get_nav(gps_nav_t *out)
{
  *out = g_nav;
}
