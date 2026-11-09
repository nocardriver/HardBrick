/**
 * @file    usb_connect.h
 * @brief   USB 连接线控制（BSP 层）：用 GPIO 拉低 D+ 模拟"拔出"
 *
 * 为什么需要这一层：调试阶段反复插拔 USB 线很烦，但直接复位 MCU 时主机不一定能
 * 看到"设备断开"——
 *   - 板子上如果有对 D+ 硬上拉的 1.5k 电阻，MCU 复位后 D+ 依然是高电平，主机会
 *     认为设备一直在线，于是不重新枚举（Windows 上表现为"设备无响应"）。
 *   - 只有 OTG 内部上拉时，复位后 D+ 虽然会掉下来，但重新接入可能只隔几十毫秒，
 *     主机来不及判定断开。
 *
 * 做法：把 D+（PA12）临时切成推挽输出并输出低电平，总线被压在 SE0，主机判定为
 * 拔出；保持一段时间后再切回 AF10（USB），OTG 内核内部上拉重新接上，主机会看到
 * 一次干净的"插入"并重新枚举。
 *
 * @note 调用顺序有约束：**必须先让 OTG 停止驱动总线**（HAL_PCD_Stop() 会置
 *       DCTL.SDIS=1，PHY 进入软断开），再调用 usb_connect_hold_off()，否则
 *       GPIO 推挽输出会和 PHY 抢同一根线。本模块不做这个判断，由 App 层保证。
 */

#ifndef USB_CONNECT_H
#define USB_CONNECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* D+ 引脚：OTG_FS 的 DP 固定是 PA12 / AF10，具体定义在 usb_connect.c。
   本头文件不引入 HAL，方便在纯逻辑代码里引用 */

/**
 * @brief  把 D+ 拉成输出低电平（模拟 USB 拔出）
 * @note   调用前 OTG 必须已经软断开（HAL_PCD_Stop()）
 */
void usb_connect_hold_off(void);

/**
 * @brief  释放 D+，恢复 AF10（USB）复用（模拟 USB 插入）
 * @note   只恢复引脚复用，PHY 上拉由随后的 HAL_PCD_Start() 打开；OTG 处于软断开
 *         （SDIS=1）时 PHY 不驱动总线，所以此时切 AF 不会和 GPIO 抢线。
 */
void usb_connect_release(void);

/**
 * @brief  当前是否处于"模拟拔出"状态
 * @retval true  D+ 正被 GPIO 拉低
 * @retval false D+ 已交还给 USB 外设
 * @note   给 RTT 日志 / J-Link 观察用
 */
bool usb_connect_is_held_off(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CONNECT_H */
