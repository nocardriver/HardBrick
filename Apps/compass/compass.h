/**
 * @file    compass.h
 * @author  YangKY
 * @brief   地磁罗盘应用层：QMC5883P 25 Hz 采样 + 硬磁圆拟合标定 + 航向角
 *
 * 分工：BSP/QMC5883P 只管寄存器、量程和 LSB→mG 换算；这一层决定"多久采一次、
 * 怎么把三轴磁场变成航向角、什么时候认为数据可信"。
 *
 * 采样率 25 Hz（40 ms）。内核 tick 是 100 Hz，所以 COMPASS_PERIOD_TICKS = 4；
 * 传感器 ODR 设成 100 Hz，保证每 40 ms 读之前 DRDY 一定已经置位——读取不阻塞、
 * 相位固定。想省电可把 ODR 降到 50 Hz，代价是偶发要等下一次转换。
 *
 * 标定：对最近 10 s 的样本做圆拟合，圆心就是硬磁零偏。采纳条件同时要求
 * 覆盖 12 个 30° 扇区里的 10 个以上、半径在 100~1000 mG、残差 RMS ≤ 15 mG，
 * 因此"只转一小段角度"不会误触发（min/max 中点那种做法会）。标定成功后继续
 * 每秒重算，转得更多就继续修正。软磁标定与倾斜补偿不在本层。
 *
 * 坐标假设：模块水平放置；航向角是传感器 +X 轴相对磁北的方位（0=北、顺时针
 * 增大、0.1° 分辨率）。+X 轴在板子上的实际朝向由安装决定，用
 * compass_set_heading_offset() 一次性补掉安装角。磁北与真北差磁偏角（上海约 -5.5°），
 * 与手机指南针对比时要留意这一点。
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

/** 采集线程栈（字节，静态分配）；含 SEGGER_RTT_printf 与浮点调用，留足余量 */
#define COMPASS_THREAD_STACK_SIZE 1536u

/** 采集线程优先级：比 GPS(4)/显示(5)/背光(6) 都低，属于"有空再采"的传感器任务 */
#define COMPASS_THREAD_PRIORITY 8u

/** @brief 罗盘快照（线程安全副本，由调用者提供空间） */
typedef struct {
  int32_t heading_deg10;  /**< 航向角，0.1° 单位，0~3599（0=磁北，顺时针增大） */
  int32_t field_mg;       /**< 标定后的磁场矢量模，mG；正常地磁约 450~550 */
  int32_t radius_mg;      /**< 圆拟合半径（水平分量），mG；正常约 300~400；未标定为 0 */
  int32_t fit_rms_mg;     /**< 圆拟合残差 RMS，mG；越小说明点云越"圆" */
  uint32_t sequence;      /**< 每成功更新一次 +1，可用来核对采样率 */
  uint32_t tick;          /**< 本次更新时的 ThreadX tick（100 Hz） */
  uint32_t missed;        /**< 累计"DRDY 未就绪或读取失败"而跳过的次数 */
  uint32_t calib_samples; /**< 参与当前拟合的样本数（封顶 256） */
  int16_t x_mg;           /**< 原始（未标定）三轴磁场，mG */
  int16_t y_mg;
  int16_t z_mg;
  int16_t off_x_mg;       /**< 当前硬磁零偏估计，mG；未标定时为 0 */
  int16_t off_y_mg;
  uint16_t coverage_bins; /**< 拟合时点云覆盖到的 30° 扇区数（0~12），<10 不会采纳 */
  bool calibrated;        /**< 半径/残差/覆盖度都达标，零偏已生效 */
  int8_t rotation_sign;   /**< 拟合区间内的旋转方向：+1 磁场矢量逆时针、-1 顺时针、0 几乎没动 */
  int16_t z_span_mg;      /**< 窗口内 Z 轴跨度（mG）；超过 60 认为转动时板子在上下翻，标定被拒 */
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
 * @note 持锁区间只有一次结构体拷贝，实际等待是微秒级；内核起来前调用返回全零。
 */
void compass_get(compass_snapshot_t *out);

/**
 * @brief 重新开始硬磁标定：清掉样本窗口与当前零偏
 * @note 采集线程在下一个周期处理该请求；之后需要水平慢慢转一整圈（≥300°）才会重新生效。
 *       换电池、换安装位置、附近新增铁磁物体后应该调用一次。
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
