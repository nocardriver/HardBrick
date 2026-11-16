/**
 * @file    compass.h
 * @author  YangKY
 * @brief   地磁罗盘应用层：QMC5883P 25 Hz 采样 + 硬磁自动标定 + 航向角
 *
 * 分工：BSP/QMC5883P 只管寄存器、量程和 LSB→mG 换算；这一层决定"多久采一次、
 * 怎么把三轴磁场变成航向角、什么时候认为数据可信"。
 *
 * 采样率 25 Hz（40 ms）。内核 tick 是 100 Hz，所以 COMPASS_PERIOD_TICKS = 4；
 * 传感器 ODR 设成 100 Hz，保证每 40 ms 读之前 DRDY 一定已经置位——读取不阻塞、
 * 相位固定。想省电可把 ODR 降到 50 Hz，代价是偶发要等下一次转换。
 *
 * 标定：硬磁偏移用"水平旋转过程中的 min/max 中点"自动估计，不需要用户输入。
 * 上电后水平转一圈，x/y 跨度都超过 COMPASS_CALIB_MIN_SPAN_MG 后自动生效并一直
 * 保持（继续旋转还能继续修正）。软磁标定与倾斜补偿不在本层。
 *
 * 坐标假设：模块水平放置，传感器 X 轴朝前、Y 轴朝左、Z 轴朝上；航向角以北为 0、
 * 顺时针增大（0.1° 分辨率）。实际安装方向不同的话，用
 * compass_set_heading_offset() 一次性补掉安装角。
 *
 * 线程：25 Hz 采集 + 1 Hz RTT 摘要；快照用互斥量保护，供显示线程 / 上位机读取。
 */

#ifndef COMPASS_H
#define COMPASS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 采样率（Hz）：25 Hz 即 40 ms 一次 */
#define COMPASS_SAMPLE_HZ 25u

/** 采集线程栈（字节，静态分配）；含一次 SEGGER_RTT_printf，留足余量 */
#define COMPASS_THREAD_STACK_SIZE 1536u

/** 采集线程优先级：比 GPS(4)/显示(5)/背光(6) 都低，属于"有空再采"的传感器任务 */
#define COMPASS_THREAD_PRIORITY 8u

/** 硬磁标定生效的门槛：x/y 跨度都超过这个值（mG）才认为转过一圈 */
#define COMPASS_CALIB_MIN_SPAN_MG 100

/** @brief 罗盘快照（线程安全副本，由调用者提供空间） */
typedef struct {
  int32_t heading_deg10; /**< 航向角，0.1° 单位，0~3599（0=北，顺时针增大） */
  int32_t field_mg;      /**< 标定后的磁场矢量模，mG；正常地磁约 450~550 */
  int16_t x_mg;          /**< 原始（未标定）三轴磁场，mG */
  int16_t y_mg;
  int16_t z_mg;
  int16_t off_x_mg;      /**< 当前硬磁偏移估计，mG；未标定时为 0 */
  int16_t off_y_mg;
  int16_t off_z_mg;
  bool calibrated;       /**< 硬磁偏移已生效 */
  uint32_t sequence;     /**< 每成功更新一次 +1，可用来核对采样率 */
  uint32_t tick;         /**< 本次更新时的 ThreadX tick（100 Hz） */
  uint32_t missed;       /**< 累计"DRDY 未就绪或读取失败"而跳过的次数 */
} compass_snapshot_t;

/**
 * @brief 传感器上电配置（阻塞，只在 tx_kernel_enter() 之前调用）
 * @note 内部会做一次完整的 qmc5883p_init（含软复位），配置为 100 Hz / ±8 G /
 *       OSR1=8 / Set-Reset 开。失败时置 g_hw_ready=false，采集线程只累计 missed，
 *       不会卡死启动流程。
 */
void compass_hw_init(void);

/**
 * @brief 创建快照互斥量与 25 Hz 采集线程（在 tx_application_define 里调用）
 * @note 必须在 compass_hw_init() 之后；线程创建失败只会打印一行 RTT，不影响其它任务。
 */
void compass_init(void);

/**
 * @brief 取最新快照（线程安全）
 * @param out 输出缓冲；为 NULL 时直接返回
 * @note 拿不到锁也不会阻塞调用者——用的是 TX_WAIT_FOREVER，但持锁区间只有一次
 *       结构体拷贝，实际等待是微秒级。
 */
void compass_get(compass_snapshot_t *out);

/**
 * @brief 重新开始硬磁标定（清掉 min/max 与当前偏移）
 * @note 采集线程在下一个周期处理该请求；调用后需要水平转一圈才会重新生效。
 */
void compass_calib_reset(void);

/**
 * @brief 设置安装角补偿，叠加到航向角上
 * @param deg10 0.1° 单位，可正可负；例如传感器 X 轴比车头偏左 90°，传 -900
 */
void compass_set_heading_offset(int16_t deg10);

#ifdef __cplusplus
}
#endif

#endif /* COMPASS_H */
