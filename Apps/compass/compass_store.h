/**
 * @file    compass_store.h
 * @author  YangKY
 * @brief   罗盘标定的文件持久化：TF 卡上的 0:/COMPASS.CAL
 *
 * 为什么放 TF 卡而不是内部 Flash：本机标配卡、FATFS 已经由 Apps/storage 挂好，
 * 卡上有磨损均衡，也不存在"擦错页把代码擦了"的风险。
 *
 * 文件内容（16 字节二进制，小端）：
 *   [0..3]   魔数 0x50415241（'PARA'）
 *   [4..11]  off_x, off_y, off_z, radius，各 int16（mG）
 *   [12..15] CRC32（覆盖前 12 字节）
 *
 * 访问约束（与 Apps/storage 的约定一致）：
 *   - 只在 storage_ready() 且 PC 没占用卡（usb_msc_host_active() == false）时访问；
 *   - 读写都要拿 usb_msc_media_lock()，避免和 USB MSC 同时发 SD 命令；
 *   - FatFs 自身的卷锁（FF_FS_REENTRANT）由 f_* 内部处理，不用再加外层锁。
 */

#ifndef COMPASS_STORE_H
#define COMPASS_STORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 标定文件路径（8.3 文件名，不占 LFN 目录项） */
#define COMPASS_STORE_PATH "0:/COMPASS.CAL"

/**
 * @brief 卡当前能否用于存取标定
 * @retval true 卷已挂载且 PC 没有占用
 */
bool compass_store_available(void);

/**
 * @brief 从卡上读回标定
 * @retval true 读到且校验通过
 * @retval false 文件不存在 / 内容损坏 / 卡不可用
 */
bool compass_store_load(int32_t *off_x, int32_t *off_y, int32_t *off_z,
                        int32_t *radius);

/**
 * @brief 把标定写到卡上（覆盖写 + 回读校验）
 * @retval true 写入并校验通过
 * @retval false 卡不可用 / 写失败
 */
bool compass_store_save(int32_t off_x, int32_t off_y, int32_t off_z,
                        int32_t radius);

/**
 * @brief 删除卡上的标定文件
 * @retval true 已删除（文件本来就不存在也算成功）
 */
bool compass_store_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* COMPASS_STORE_H */
