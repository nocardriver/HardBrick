/**
 * @file    gps.h
 * @author  YangKY
 * @brief   GPS 应用层：把 LC760Z 发来的 NMEA 字节流按行交给 minmea 解析
 *
 * 通路：HAL_UARTEx_RxEventCallback(ISR) --gps_rx_push()--> 环形缓冲
 *       --事件标志--> gps 线程 --> 切行 --> minmea --> g_nav 快照
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
  struct minmea_date date;   /**< UTC 日期（RMC，年是两位数） */
  struct minmea_time time;   /**< UTC 时间（RMC） */
  float   lat_deg;           /**< 纬度，十进制度，北正南负（RMC） */
  float   lon_deg;           /**< 经度，十进制度，东正西负（RMC） */
  float   alt_m;             /**< 海拔，米（GGA，字段为空时为 NAN） */
  uint8_t fix;               /**< 0=无效 1=单点 2=差分 */
  uint8_t numsat;            /**< 参与定位的卫星数（GGA） */
} gps_nav_t;

/**
 * @brief 创建 GPS 解析线程（在 tx_application_define 里调用）
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
 * @brief 取定位快照副本
 * @param out 由调用者提供的缓冲
 */
void gps_get_nav(gps_nav_t *out);

#endif /* GPS_H */
