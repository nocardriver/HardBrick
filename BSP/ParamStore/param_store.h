/**
 * @file    param_store.h
 * @author  YangKY
 * @brief   小参数持久化：内部 Flash 日志式存储（BSP 层，掉电不丢）
 *
 * 只面向"几字节、改得很少"的配置（罗盘零偏这类）：保存一次是一次追加，
 * 页写满才整页擦除，因此不会频繁擦写同一页。
 */

#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 每次读写的负载字节数；魔数/CRC 由存储层维护，不占负载 */
#define PARAM_STORE_PAYLOAD_SIZE 8u

/**
 * @brief 扫描存储页，确定追加位置
 * @retval true  存储可用（页里没有异物）
 * @retval false 页里存在非本模块的数据，出于安全原因不使用
 * @note 必须在任何 load/save 之前调用一次；每上电调用一次即可。
 */
bool param_store_init(void);

/**
 * @brief 读回最后一条有效记录
 * @param payload 输出缓冲，长度 PARAM_STORE_PAYLOAD_SIZE
 * @retval true 有记录
 * @retval false 没有记录 / 记录损坏 / 未初始化
 */
bool param_store_load(uint8_t payload[PARAM_STORE_PAYLOAD_SIZE]);

/**
 * @brief 追加一条记录（页满时自动擦除后从头写）
 * @param payload 待保存负载，长度 PARAM_STORE_PAYLOAD_SIZE
 * @retval true 写入并回读校验通过
 * @retval false 擦除/编程失败（Flash 被保护、地址不可写等）
 * @note 内部在临界区里执行，最长约几十毫秒（整页擦除），调用者别在 ISR 里用。
 */
bool param_store_save(const uint8_t payload[PARAM_STORE_PAYLOAD_SIZE]);

/**
 * @brief 清空存储（整页擦除）
 * @retval true 已擦除
 * @note 只在"恢复出厂/重新标定"这类操作里调用。
 */
bool param_store_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_STORE_H */
