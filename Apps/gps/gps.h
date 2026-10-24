/**
 * @file    gps.h
 * @author  YangKY
 * @brief   GPS 应用层：把 LC760Z 发来的 NMEA 字节流按行交给 minmea 解析
 *
 * 通路：HAL_UARTEx_RxEventCallback(ISR) --gps_rx_push()--> 环形缓冲
 *       --事件标志--> gps 线程 --> 切行 --> minmea --> g_nav 快照
 *
 * 坐标/海拔全部用定点整数保存（1e-7 度 / 毫米）：
 *   float 在经纬度量级（31°/121°）的 ULP 约 2e-6 ~ 1.2e-5 度，
 *   直接乘 1e6 打印时末位会抖动；定点值由 minmea 的原始 value/scale
 *   一次换算，显示层不再做浮点运算。
 *   快照由 gps 线程写、显示线程读，读写都经过 g_nav_mutex。
 */

#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#include "minmea.h"   /* 快照直接复用 minmea 的日期/时间类型 */

/** GPS 线程栈大小（字节，静态分配） */
#define GPS_THREAD_STACK_SIZE 2048u

/** 定位快照：RMC 填时间/日期/经纬度，GGA 填定位质量/卫星数/海拔 */
typedef struct {
  struct minmea_date date;   /**< UTC 日期（RMC，year 为两位，如 26 表示 2026） */
  struct minmea_time time;   /**< UTC 时间（RMC） */
  int32_t  lat_e7;           /**< 纬度，1e-7 度，北正南负（RMC） */
  int32_t  lon_e7;           /**< 经度，1e-7 度，东正西负（RMC） */
  int32_t  alt_mm;           /**< 海拔，毫米（GGA） */
  uint8_t  fix;              /**< 0=无效 1=单点 2=差分 */
  uint8_t  numsat;           /**< 参与定位的卫星数（GGA） */
  bool     pos_valid;        /**< 经纬度有效（RMC 字段非空） */
  bool     alt_valid;        /**< 海拔有效（GGA 字段非空） */
  uint32_t tick;             /**< 最后一次解析成功的 ThreadX tick，0=尚未收到 */
} gps_nav_t;

/**
 * @brief 创建 GPS 解析线程与快照互斥锁（在 tx_application_define 里调用）
 * @note  不清零环形缓冲：UART 接收在 tx_kernel_enter() 之前就已启动
 */
void gps_init(void);

/**
 * @brief 把串口收到的字节搬进环形缓冲，并唤醒解析线程
 * @param data DMA 接收缓冲地址
 * @param len  本次收到的字节数（HAL 回调的 Size，不要传整个缓冲长度）
 * @note  在中断里调用：只拷贝 + 置事件标志，不解析、不打印
 */
void gps_rx_push(const uint8_t *data, uint16_t len);

/**
 * @brief 取定位快照副本（线程安全，调用前必须先 gps_init）
 * @param out 由调用者提供的缓冲；out 为空时直接返回
 */
void gps_get_nav(gps_nav_t *out);

#endif /* GPS_H */
