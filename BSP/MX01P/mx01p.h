/**
 * @file    mx01p.h
 * @author  YangKY
 * @brief   MX-01P 蓝牙 5.3 (BLE) 透传模块驱动核心（BSP 层，纯 C，不依赖 HAL/RTOS）
 *
 * 模块通过 USART1 与 MCU 相连，另有两根控制线：
 *   - RES ：低电平复位。本板上电后 CubeMX 把 PC3 初始化为低，即模块一直处在复位态，
 *           必须由 mx01p_init() 释放；
 *   - BRTS：睡眠脚。高电平/悬空 = 进入睡眠，低电平 = 正常工作。睡眠期间模块串口
 *           只能收数据、**不能发数据**（手册 Pin15 注明），因此发数据前必须先唤醒。
 *
 * 手册：`C:\myProjects\PCB_Proj\docs\MX-01P规格书+V0.0.5.pdf`
 *   - 第 3 页   出厂默认参数（115200-8-N-1、广播开、功率 0 dBm、FFFF0/FFF1/FFF2）
 *   - 第 4 页   引脚定义（CDS/LINK/FC/BRTS/RES 语义）
 *   - 第 5~15 页 AT 指令集与逐条应答格式
 *
 * 关键协议事实（决定了本驱动为什么长这样）：
 *   1. 上电/重启成功后模块必须先在串口吐 `+READY\r\n`，主机才能发指令或数据。
 *   2. AT 与透传数据**共用同一条串口且同时有效**：本板 CDS 悬空 = 高电平，AT 命令
 *      始终有效，非 AT 数据走透传。所以 RX 方向必须自己做「控制行 / 数据」分流。
 *   3. 应答行都以 '+' 开头（`+MAC:` / `+NAME:` …），设置类命令回 `OK` / `ERROR`；
 *      另有三种异步事件：`+READY` / `+CONNECTED:TYP,MAC` / `+DISCONN:TYP,MAC`。
 *
 * 使用约束：
 *   1. mx01p_init() 前模块 VCC 必须已上电（本板由 PW_EN 控制），
 *      且已调用 MX_GPIO_Init() / MX_USART1_UART_Init()；
 *   2. USART1 固定 115200-8-N-1，必须与模块当前波特率一致，否则 +READY 收不到、
 *      init 超时（见 README「波特率翻车」）；
 *   3. 驱动不加锁：AT 类 API 与数据类 API 不得在两个线程里并发调用（同一时刻只允许
 *      一个 AT 事务）；建议由 App 的一个蓝牙线程独占驱动。
 */

#ifndef MX01P_H
#define MX01P_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** MAC 地址字符串长度（12 个十六进制字符，无分隔符），缓冲区需 +1 放 '\0' */
#define MX01P_MAC_STR_LEN 12u

/** 设备名最大字节数（手册「支持参数：总长度不超过 20 字节」） */
#define MX01P_NAME_STR_LEN 20u

/** UUID 字符串最大长度（128 bit 形式为 32 个十六进制字符） */
#define MX01P_UUID_STR_LEN 32u

/** 自定义广播数据最大字节数（手册「HEX 为 0-29 字节」） */
#define MX01P_ADV_DATA_MAX 29u

/** 自定义广播数据的 HEX 字符串最大长度（2 字节/字节） */
#define MX01P_ADV_DATA_HEX_LEN (MX01P_ADV_DATA_MAX * 2u)

/** 单条 AT 命令（含 "AT+" 与参数）的最大长度，不含 CRLF */
#define MX01P_CMD_MAX 48u

/** 会话超时：一条 AT 命令从发出到收到应答的上限 */
#define MX01P_AT_TIMEOUT_MS 500u

/** 复位到 +READY 的默认超时（手册未给数值，实测远小于此值） */
#define MX01P_READY_TIMEOUT_MS 3000u

/** 唤醒后等待模块恢复的延时（手册未给数值，取保守值，见 README） */
#define MX01P_WAKE_MS 10u

/**
 * @brief 返回值。全部错误为负值，便于 `if (st != MX01P_OK)` 与 RTT 直接打印
 */
typedef enum {
  MX01P_OK = 0,           /**< 成功 */
  MX01P_ERR_PARAM = -1,   /**< 参数非法（空指针、长度越界、环形缓冲不是 2 的幂等） */
  MX01P_ERR_TIMEOUT = -2, /**< 等 +READY 或等应答超时（含波特率不匹配） */
  MX01P_ERR_IO = -3,      /**< UART 发送失败（HAL 返回错误/超时） */
  MX01P_ERR_PROTOCOL = -4,/**< 应答不符合手册格式，或模块回 ERROR */
  MX01P_ERR_STATE = -5,   /**< 当前状态不允许（未初始化 / 未收到 +READY） */
  MX01P_ERR_NOMEM = -6,   /**< 命令串超长，装不进内部缓冲 */
} mx01p_status_t;

/** 串口波特率码值，编码与 `AT+UART=NUM` 的 NUM 一致（手册第 8 页） */
typedef enum {
  MX01P_BAUD_9600 = 0,
  MX01P_BAUD_14400 = 1,
  MX01P_BAUD_19200 = 2,
  MX01P_BAUD_38400 = 3,
  MX01P_BAUD_57600 = 4,
  MX01P_BAUD_115200 = 5, /**< 出厂默认，与本板 USART1 一致 */
  MX01P_BAUD_230400 = 6,
  MX01P_BAUD_460800 = 7,
  MX01P_BAUD_921600 = 8,
} mx01p_baud_t;

/** 发射功率码值，编码与 `AT+TXPOWER=NUM` 的 NUM 一致（手册第 11 页，0 最大） */
typedef enum {
  MX01P_POWER_4DBM = 0,   /**< +4 dBm */
  MX01P_POWER_3P5DBM = 1, /**< +3.5 dBm */
  MX01P_POWER_3DBM = 2,   /**< +3 dBm */
  MX01P_POWER_2DBM = 3,   /**< +2 dBm */
  MX01P_POWER_1DBM = 4,   /**< +1 dBm */
  MX01P_POWER_0DBM = 5,   /**< 0 dBm（出厂默认） */
  MX01P_POWER_N1DBM = 6,  /**< -1 dBm */
  MX01P_POWER_N5DBM = 7,  /**< -5 dBm */
  MX01P_POWER_N9DBM = 8,  /**< -9 dBm */
  MX01P_POWER_N18DBM = 9, /**< -18 dBm */
} mx01p_tx_power_t;

/** BLE 属性表里三条可配置 UUID 的编号（手册第 12~14 页） */
typedef enum {
  MX01P_UUID_SERVICE = 0, /**< 主服务 UUID，出厂 FFF0 */
  MX01P_UUID_NOTIFY = 1,  /**< 读/通知通道，出厂 FFF1（MCU → APP） */
  MX01P_UUID_WRITE = 2,   /**< 写通道，出厂 FFF2（APP → MCU） */
} mx01p_uuid_kind_t;

/** 异步事件类型（在中断上下文回调，或在 AT 等待期间顺带分发） */
typedef enum {
  MX01P_EVENT_READY = 0,     /**< 上电/重启完成，可以发指令了 */
  MX01P_EVENT_CONNECTED = 1, /**< 手机/主机已连上（`+CONNECTED:TYP,MAC`） */
  MX01P_EVENT_DISCONNECTED = 2, /**< 连接已断开（`+DISCONN:TYP,MAC`） */
} mx01p_event_t;

typedef struct mx01p mx01p_t;

/**
 * @brief 异步事件回调
 * @param inst    驱动实例
 * @param event   事件类型
 * @param link_type TYP 字段（1 = 连接方为主机），无该字段时为 0
 * @param mac     MAC 字符串（12 个十六进制字符），**指针仅在回调期间有效**；
 *                事件不带 MAC 时为 NULL
 * @note  READY 事件在 RX 中断里产生；CONNECTED/DISCONNECTED 可能在中断里，也可能在
 *        AT 等待循环里产生。回调里只做置标志 / 唤醒线程这类最小工作。
 */
typedef void (*mx01p_event_cb_t)(mx01p_t *inst, mx01p_event_t event,
                                 uint8_t link_type, const char *mac);

/** @brief 驱动装配参数，由调用者提供（跨模块数据约定） */
typedef struct {
  uint8_t *rx_ring;          /**< 有效载荷环形缓冲，调用者分配，容量必须是 2 的幂 */
  uint16_t rx_ring_size;     /**< 环形缓冲字节数，建议 512 或 1024 */
  mx01p_event_cb_t evt_cb;   /**< 可选的事件回调，NULL 表示不关心 */
  void *user;                /**< 透传给回调的上下文 */
  uint32_t ready_timeout_ms; /**< 等 +READY 的超时，0 表示用 MX01P_READY_TIMEOUT_MS */
} mx01p_cfg_t;

/** @brief 驱动实例。字段由驱动维护，调用者只负责分配 storage */
struct mx01p {
  /* ---- 有效载荷通路 ---- */
  uint8_t *rx_ring;              /**< 调用者提供的环形缓冲 */
  uint16_t rx_ring_size;         /**< 容量（2 的幂） */
  volatile uint16_t rx_head;     /**< 仅 RX 中断写 */
  volatile uint16_t rx_tail;     /**< 仅消费线程写 */
  volatile uint32_t rx_dropped;  /**< 缓冲满时丢弃的字节数（累计） */

  /* ---- RX 分流：控制行收集器（ISR 写，线程只在临界区里读改） ---- */
  char line_buf[96];             /**< 候选行缓冲（可能是控制行，也可能是数据） */
  volatile uint16_t line_len;    /**< 已收字节数，0 表示行首 */
  volatile uint32_t line_tick;   /**< 本行最后一个字节的时间戳，用于空闲冲刷 */
  volatile bool line_raw;        /**< true = 本行已判定为数据，逐字节直通直到 '\n' */

  /* ---- 应答邮箱：单槽，够同步 AT 用 ---- */
  char resp[96];                 /**< 完整的一行应答（含 CRLF 之外的原始字节） */
  volatile uint16_t resp_len;
  volatile bool resp_ready;      /**< 有待取走的应答行 */
  volatile uint32_t resp_overrun;/**< 应答邮箱被覆盖的次数（说明有人在并发用驱动程序） */
  volatile bool at_pending;      /**< 有 AT 事务在等应答 */

  /* ---- 事件与状态 ---- */
  volatile bool ready_edge;      /**< 收到过 +READY（由 init 清零后再等） */
  volatile bool link_up;         /**< 最近一次已知的连接状态 */
  volatile uint8_t link_type;    /**< 最近一次事件的 TYP */
  char link_mac[MX01P_MAC_STR_LEN + 1u]; /**< 最近一次事件的 MAC */
  bool ready;                    /**< init 成功，可以收发 AT */
  bool asleep;                   /**< 模块当前被 BRTS 拉入睡眠 */
  volatile uint32_t rx_errors;   /**< ORE/帧错误计数。模块复位、BRTS 唤醒的瞬间
                                     各可能出现 1 次，属正常；运行中持续增长说明
                                     接线/波特率/电平有问题 */

  mx01p_event_cb_t evt_cb;       /**< 事件回调 */
  void *user;                    /**< 回调上下文 */
  uint32_t ready_timeout_ms;     /**< 等 +READY 的超时 */
};

/*---------------------------------------------------------------------------*/
/* 生命周期                                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief 装配驱动：绑定板级资源，硬复位模块并等它上报 +READY
 * @param inst 调用者分配的实例
 * @param cfg  装配参数（rx_ring 必须非空，容量必须是 2 的幂）
 * @retval MX01P_OK 模块已就绪
 * @retval MX01P_ERR_PARAM 参数非法
 * @retval MX01P_ERR_TIMEOUT 复位后未在超时内收到 +READY（查供电/RES 接线/波特率）
 * @note 必须在模块 VCC 上电后调用；本工程在 tx_kernel_enter() 之前调用，因此内部
 *       只用 HAL_Delay 与轮询，不依赖 RTOS。失败时 inst->ready 保持 false，所有需
 *       要 ready 的 API 会返回 MX01P_ERR_STATE，可反复调用 init 重试。
 */
mx01p_status_t mx01p_init(mx01p_t *inst, const mx01p_cfg_t *cfg);

/**
 * @brief 硬复位模块（RES 拉低 MX01P_RESET_LOW_MS → 释放 → 等 +READY）
 * @retval MX01P_OK 复位完成并收到 +READY
 * @retval MX01P_ERR_TIMEOUT 超时未收到 +READY
 * @retval MX01P_ERR_STATE 未初始化
 * @note 复位会清空接收环形缓冲与分流状态；已建立的蓝牙连接随复位断开。
 */
mx01p_status_t mx01p_reset(mx01p_t *inst);

/**
 * @brief 软复位（`AT+REBOOT=1`，模块回 OK 后重新吐 +READY）
 * @retval MX01P_OK 重启完成并收到 +READY
 * @retval MX01P_ERR_PROTOCOL 模块回 ERROR
 * @retval MX01P_ERR_TIMEOUT 没等回 +READY
 */
mx01p_status_t mx01p_reboot(mx01p_t *inst);

/**
 * @brief 恢复出厂设置（`AT+RESET=1`）
 * @retval MX01P_OK 已接受（模块提示"重启生效"，需要重启请接着调用 mx01p_reboot）
 * @note **MAC 地址改动不可恢复**；本函数不自动重启，避免把上层正在用的连接静默打断。
 */
mx01p_status_t mx01p_restore(mx01p_t *inst);

/*---------------------------------------------------------------------------*/
/* 睡眠与唤醒（BRTS）                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief 进入/退出睡眠
 * @param sleep true = BRTS 拉高进入睡眠（模块串口只能收、不能发）；false = 唤醒
 * @retval MX01P_OK 已切换
 * @retval MX01P_ERR_STATE 未初始化
 * @note 唤醒后内部会等 MX01P_WAKE_MS 再返回，保证调用者接着发数据不会丢。
 *       睡眠期间模块**不能**往 MCU 发数据（包括 AT 应答），所以不要在 AT 事务中途
 *       调用本函数；mx01p_send()/AT 类 API 会自动唤醒。
 */
mx01p_status_t mx01p_set_sleep(mx01p_t *inst, bool sleep);

/** @brief 当前是否处于睡眠（驱动记录，不访问硬件） */
bool mx01p_is_asleep(const mx01p_t *inst);

/** @brief 蓝牙链路最近一次已知状态（来自 +CONNECTED/+DISCONN/AT+DEV?） */
bool mx01p_is_connected(const mx01p_t *inst);

/** @brief 最近一次事件的 TYP 与 MAC；无事件时 TYP=0、mac 为空串 */
void mx01p_get_link(const mx01p_t *inst, uint8_t *link_type, char *mac);

/*---------------------------------------------------------------------------*/
/* 数据面（透传）                                                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief 透传发送：把数据交给模块，由它通过 FFF1 通知推给手机
 * @param data 待发送字节
 * @param len  字节数（BLE 单包受 MTU 限制，模块自行分包，驱动的上限是 512 B/次）
 * @param timeout_ms HAL 发送超时
 * @retval MX01P_OK 字节已写入串口
 * @retval MX01P_ERR_STATE 未初始化/未就绪
 * @retval MX01P_ERR_PARAM 空指针或 len 为 0 或 len > 512
 * @retval MX01P_ERR_IO UART 发送失败
 * @note 睡眠中会先自动唤醒（多花 MX01P_WAKE_MS），且**保持唤醒**，需要省电请显式
 *       再调用 mx01p_set_sleep(true)。未连接手机时数据会被模块丢弃，驱动无法感知。
 */
mx01p_status_t mx01p_send(mx01p_t *inst, const uint8_t *data, uint16_t len,
                          uint32_t timeout_ms);

/**
 * @brief 从接收环形缓冲取数据（非阻塞）
 * @param buf 目标缓冲
 * @param max_len 最多取多少字节
 * @return 实际取出的字节数
 * @note 缓冲满时丢弃新数据（不覆盖未消费数据），丢弃量见 inst->rx_dropped。
 *       取数据前会先把"停在候选行里超过 MX01P_RX_IDLE_FLUSH_MS 的尾巴"冲进缓冲，
 *       所以以 '+'/'O'/'E' 开头的二进制载荷最多晚一个空闲门限到达。
 */
uint16_t mx01p_read(mx01p_t *inst, uint8_t *buf, uint16_t max_len);

/** @brief 环形缓冲中待读字节数（不含还在候选行里的尾巴） */
uint16_t mx01p_available(const mx01p_t *inst);

/** @brief 丢弃环形缓冲与候选行里的所有未消费数据 */
void mx01p_flush_rx(mx01p_t *inst);

/*---------------------------------------------------------------------------*/
/* AT 通用接口                                                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief 发一条查询类 AT 命令并取出应答值
 * @param inst 驱动实例
 * @param command 完整命令（不含 CRLF），如 "AT+MAC?"
 * @param key     应答前缀，如 "+MAC:"
 * @param out     输出值（去掉前缀与首尾空白）
 * @param out_size 输出缓冲大小
 * @param timeout_ms 等待应答的超时
 * @retval MX01P_OK 成功
 * @retval MX01P_ERR_PROTOCOL 没找到 key，或模块回 ERROR
 * @retval MX01P_ERR_TIMEOUT 无应答
 * @note 命令里带参数时由调用者拼好；长度上限 MX01P_CMD_MAX。
 */
mx01p_status_t mx01p_at_query(mx01p_t *inst, const char *command, const char *key,
                              char *out, uint16_t out_size, uint32_t timeout_ms);

/**
 * @brief 发一条设置类 AT 命令
 * @param command 完整命令（不含 CRLF），如 "AT+NAME=HardBrick"
 * @retval MX01P_OK 模块回 OK
 * @retval MX01P_ERR_PROTOCOL 模块回 ERROR
 * @retval MX01P_ERR_TIMEOUT 无应答
 * @note 像 `AT+DISCONN=NUM` 这种回 `+DISCONN:...` 而不是 OK 的命令，本函数同样返回
 *       MX01P_OK（只要不是 ERROR），需要细节请用 mx01p_at_query()。
 */
mx01p_status_t mx01p_at_exec(mx01p_t *inst, const char *command,
                             uint32_t timeout_ms);

/*---------------------------------------------------------------------------*/
/* 参数读写（内部拼接 AT 命令，超时统一 MX01P_AT_TIMEOUT_MS）                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief 读模块 MAC 地址
 * @param out 至少 MX01P_MAC_STR_LEN + 1 字节，形如 "001122334455"
 * @note 这是模块自己的蓝牙地址，不是手机地址；`+MAC:000102030405` 会被原样取出。
 */
mx01p_status_t mx01p_read_mac(mx01p_t *inst, char *out, uint16_t out_size);

/**
 * @brief 写模块 MAC 地址（**重启后生效**，写入不可撤销）
 * @param mac 12 个十六进制字符
 */
mx01p_status_t mx01p_write_mac(mx01p_t *inst, const char *mac);

/** @brief 读设备名（≤20 字节） */
mx01p_status_t mx01p_read_name(mx01p_t *inst, char *out, uint16_t out_size);

/** @brief 写设备名（≤20 字节，立即生效） */
mx01p_status_t mx01p_write_name(mx01p_t *inst, const char *name);

/** @brief 读广播开关 */
mx01p_status_t mx01p_read_adv(mx01p_t *inst, bool *on);

/** @brief 写广播开关（立即生效） */
mx01p_status_t mx01p_set_adv(mx01p_t *inst, bool on);

/** @brief 读串口波特率码值 */
mx01p_status_t mx01p_read_baud(mx01p_t *inst, mx01p_baud_t *baud);

/**
 * @brief 写串口波特率码值（立即生效）
 * @retval MX01P_ERR_PARAM 目标码值与本板 USART1 实际波特率不同
 * @note 本板 USART1 由 CubeMX 固定为 115200，写别的码值会让模块立刻失联（只能靠
 *       断电重上电 + 改 host 波特率救回来），所以驱动直接拒绝。
 */
mx01p_status_t mx01p_write_baud(mx01p_t *inst, mx01p_baud_t baud);

/** @brief 读广播间隔，单位 ms（20~10000） */
mx01p_status_t mx01p_read_adv_interval(mx01p_t *inst, uint16_t *interval_ms);

/** @brief 写广播间隔（20~10000 ms，**重启后生效**） */
mx01p_status_t mx01p_write_adv_interval(mx01p_t *inst, uint16_t interval_ms);

/** @brief 读软件版本，如 "V0.0.1" */
mx01p_status_t mx01p_read_version(mx01p_t *inst, char *out, uint16_t out_size);

/** @brief 读发射功率码值 */
mx01p_status_t mx01p_read_tx_power(mx01p_t *inst, mx01p_tx_power_t *power);

/** @brief 写发射功率（**重启后生效**） */
mx01p_status_t mx01p_write_tx_power(mx01p_t *inst, mx01p_tx_power_t power);

/**
 * @brief 读某个 BLE 通道 UUID（16 bit 形如 "FFF0"，128 bit 为 32 字符）
 * @param kind 通道编号
 */
mx01p_status_t mx01p_read_uuid(mx01p_t *inst, mx01p_uuid_kind_t kind,
                               char *out, uint16_t out_size);

/**
 * @brief 写某个 BLE 通道 UUID（**重启后生效**，16 bit 或 128 bit 均可）
 * @param uuid 4 或 32 个十六进制字符
 */
mx01p_status_t mx01p_write_uuid(mx01p_t *inst, mx01p_uuid_kind_t kind,
                                const char *uuid);

/**
 * @brief 读自定义广播数据（HEX 字符串，形如 "3132333435"）
 * @param out 至少 MX01P_ADV_DATA_HEX_LEN + 1 字节
 * @note 该数据放在 BLE 广播的 Manufacturer Specific Data 段，出厂默认 8 字节
 *       （前两字节固定 00 00，后 6 字节为 MAC）。
 */
mx01p_status_t mx01p_read_adv_data(mx01p_t *inst, char *out, uint16_t out_size);

/**
 * @brief 写自定义广播数据
 * @param hex 偶数个十六进制字符，长度 ≤ MX01P_ADV_DATA_HEX_LEN
 */
mx01p_status_t mx01p_write_adv_data(mx01p_t *inst, const char *hex);

/**
 * @brief 断开蓝牙连接
 * @param mode 0 = 断开所有从设备；1 = 主动断开与主机端设备的连接
 * @retval MX01P_OK 已下发（随即会收到 +DISCONN 事件）
 */
mx01p_status_t mx01p_disconnect(mx01p_t *inst, uint8_t mode);

/**
 * @brief 查询当前已连接设备（取第一条 `+DEV:CONN TYP,MAC`）
 * @param link_type 输出 TYP（无连接时保持原值）
 * @param mac 输出 12 字符 MAC，缓冲区至少 13 字节
 * @retval MX01P_OK 有连接设备
 * @retval MX01P_ERR_PROTOCOL 模块无连接（应答里没有 CONN 行）
 * @note 模块支持多连接时 `AT+DEV?` 会回多行，这里只取第一行；需要跟踪全部连接请用
 *       事件回调。查询成功后同步更新 mx01p_is_connected()。
 */
mx01p_status_t mx01p_read_link(mx01p_t *inst, uint8_t *link_type, char *mac);

/*---------------------------------------------------------------------------*/
/* 上电自检                                                                  */
/*---------------------------------------------------------------------------*/

/** @brief 自检阶段号，J-Link 直接读 g_bt_probe.stage 就能判断跑到哪一步 */
typedef enum {
  MX01P_PROBE_STAGE_NONE = 0,   /**< 还没开始 */
  MX01P_PROBE_STAGE_INIT = 1,   /**< mx01p_init() 已返回 */
  MX01P_PROBE_STAGE_AT = 2,     /**< 全部 AT 查询已跑完 */
  MX01P_PROBE_STAGE_SLEEP = 3,  /**< 睡眠脚验证已跑完 */
  MX01P_PROBE_STAGE_LOOP = 4,   /**< 数据通路回环也跑完了 */
  MX01P_PROBE_STAGE_REBOOT = 5, /**< 软复位（AT+REBOOT）也验证完了 */
} mx01p_probe_stage_t;

/**
 * @brief 自检结果快照
 * @note 每个 `*_st` 都是对应 API 的返回值（0 = 成功，负值 = 失败原因）；驱动只填结构
 *       体、不打印，方便放在全局变量里让 J-Link 直接读（不依赖 RTT 是否在连）。
 */
typedef struct {
  uint32_t stage;                            /**< 见 mx01p_probe_stage_t */
  int32_t init_st;                           /**< mx01p_init() 返回值 */
  uint32_t ready_ms;                         /**< 复位到 +READY 的耗时 */
  uint32_t reset_retry;                      /**< init 失败后重试次数 */
  int32_t mac_st;                            /**< mx01p_read_mac() */
  int32_t name_st;                           /**< mx01p_read_name() */
  int32_t version_st;                        /**< mx01p_read_version() */
  int32_t baud_st;                           /**< mx01p_read_baud() */
  int32_t adv_st;                            /**< mx01p_read_adv() */
  int32_t interval_st;                       /**< mx01p_read_adv_interval() */
  int32_t power_st;                          /**< mx01p_read_tx_power() */
  int32_t uuid_st;                           /**< mx01p_read_uuid() 三条 */
  int32_t link_st;                           /**< mx01p_read_link() */
  int32_t sleep_st;                          /**< mx01p_set_sleep(true) */
  int32_t wake_st;                           /**< mx01p_set_sleep(false) */
  int32_t sleep_blocked;                     /**< 1 = 睡眠期间 AT 确实无应答（BRTS 有效） */
  int32_t loop_st;                           /**< 回环验证：发 "AT+VER?" 再从数据通路收回来 */
  uint16_t loop_len;                         /**< 回环收到多少字节 */
  char loopback[24];                         /**< 回环收到的原文（只读前 loop_len 字节有效） */
  int32_t reboot_st;                         /**< 软复位 AT+REBOOT=1 后能否重新等到 +READY */
  char mac[MX01P_MAC_STR_LEN + 1u];          /**< "001122334455" */
  char name[MX01P_NAME_STR_LEN + 1u];        /**< 设备名 */
  char version[16];                          /**< 软件版本 */
  char uuid_service[MX01P_UUID_STR_LEN + 1u];
  char uuid_notify[MX01P_UUID_STR_LEN + 1u];
  char uuid_write[MX01P_UUID_STR_LEN + 1u];
  int32_t baud_code;                         /**< 串口波特率码值 */
  int32_t adv_on;                            /**< 1 = 正在广播 */
  int32_t interval_ms;                       /**< 广播间隔 */
  int32_t power_code;                        /**< 发射功率码值 */
  uint32_t at_total;                         /**< 发起的 AT 事务数 */
  uint32_t at_fail;                          /**< 失败的 AT 事务数 */
  uint32_t rx_dropped;                       /**< 接收缓冲丢弃字节数 */
  uint32_t resp_overrun;                     /**< 应答邮箱被覆盖次数 */
} mx01p_probe_t;

/**
 * @brief 上电自检：init + 一轮 AT 查询 + 睡眠脚验证 + 数据通路回环，结果写进 out
 * @param inst 驱动实例
 * @param cfg  与正常使用时相同的装配参数（本函数内部会自己调 mx01p_init）
 * @param out  结果快照，调用者分配
 * @retval true 全部必检项成功
 * @retval false 至少一项失败，细节看 out->*_st
 * @note 1) 睡眠验证：在睡眠状态下直接发一次 `AT+MAC?\r\n`，预期模块无应答（睡眠时不
 *          能发串口）。这是唯一能证明 BRTS 接线正确的方法；
 *       2) 回环验证：从数据通路发一次 `AT+VER?`（本板 CDS 悬空，AT 命令始终有效），
 *          再从 mx01p_read() 把 `+VER:...` 当载荷收回来，证明「发送 → 分流 → 接收」
 *          三段都通；若手边有手机连着，这份数据会被模块同时透传给手机，属正常；
 *       3) `link_st` 需要手机在旁边才算成功，不计入失败项。
 */
bool mx01p_probe(mx01p_t *inst, const mx01p_cfg_t *cfg, mx01p_probe_t *out);

/*---------------------------------------------------------------------------*/
/* 中断/轮询入口：由 mx01p_port.c 与 HAL 回调调用                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief 把一段收到的字节喂进分流器（USART1 RX 事件里调用，ISR 安全）
 * @note 内部完成「控制行 → 应答邮箱/事件」与「数据 → 环形缓冲」的分流；缓冲满时
 *       丢弃放不下的部分，绝不覆盖未消费数据。
 */
void mx01p_rx_feed(mx01p_t *inst, const uint8_t *data, uint16_t len);

/**
 * @brief 线程侧维护：把长时间没动静的候选行当作数据冲进缓冲
 * @note 由 mx01p_read()/mx01p_send()/AT 等待循环自动调用，App 一般不必手动调用；
 *       若 App 在别处也要看数据，可在自己的周期线程里调一次。
 */
void mx01p_poll(mx01p_t *inst);

#ifdef __cplusplus
}
#endif

#endif /* MX01P_H */
