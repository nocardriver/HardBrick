/**
 * @file    e22.h
 * @author  YangKY
 * @brief   E22-400T22S LoRa 模块驱动核心（BSP 层，纯 C，不依赖 HAL/RTOS）
 *
 * 模块通过 UART 与 MCU 相连，另有 4 根控制线：
 *   - M1/M0：工作模式选择（00 透传、01 WOR、10 配置、11 深度休眠）；
 *   - NRST ：低电平复位（拉低 >100 us）；
 *   - AUX  ：忙闲指示，低=忙、高=空闲；模块从串口输出数据前约 3 ms 拉低。
 *
 * 本层职责：模式切换、复位、AUX 等待、透传收发、HEX/AT 参数配置。
 * 板级资源（LPUART1、GPIO）全部收敛在 e22_port.c，核心不 include 任何 HAL 头文件。
 *
 * 出厂默认（E22-400T22S）：9600-8-N-1、透传、地址 0x0000、网络 0、信道 0x17
 * （433.125 MHz）、空速 2.4 kbps、分包 240 B、22 dBm。
 *
 * 使用约束：
 *   1. e22_init() 前模块 VCC 必须已上电（本板由 PW_EN 控制）；
 *   2. LPUART1 已由 CubeMX 初始化（9600-8-N-1，LSE 32.768 kHz 时钟，中断接收）；
 *   3. 驱动不加锁：e22_send() 只允许一个写者；配置类 API（会切到配置模式并从
 *      环形缓冲读响应）与 e22_read() 不得在两个线程里并发调用。
 */

#ifndef E22_H
#define E22_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 单次空中包的最大有效载荷（出厂分包长度 240 B），e22_send() 按此值分块 */
#define E22_PACKET_PAYLOAD_MAX 240u

/** 产品信息寄存器 0x80~0x86 的长度（7 字节二进制，厂家未公开逐字节含义） */
#define E22_PID_LEN 7u

/**
 * @brief 返回值。全部错误为负值，便于 `if (st != E22_OK)` 与 RTT 直接打印
 */
typedef enum {
  E22_OK = 0,           /**< 成功 */
  E22_ERR_PARAM = -1,   /**< 参数非法（空指针、长度 0、环形缓冲不是 2 的幂等） */
  E22_ERR_TIMEOUT = -2, /**< 等待 AUX 空闲或模块应答超时 */
  E22_ERR_IO = -3,      /**< UART 发送失败（HAL 返回错误/超时） */
  E22_ERR_PROTOCOL = -4,/**< 应答格式错误（FF FF FF）或与写入内容不一致 */
  E22_ERR_STATE = -5,   /**< 当前状态不允许该操作（未初始化、模式不对） */
} e22_status_t;

/** 工作模式，编码与 M1/M0 一致：M1=bit1、M0=bit0 */
typedef enum {
  E22_MODE_TRANSMISSION = 0, /**< M1=0 M0=0 透传 */
  E22_MODE_WOR = 1,          /**< M1=0 M0=1 空中唤醒 */
  E22_MODE_CONFIG = 2,       /**< M1=1 M0=0 配置（串口强制 9600-8-N-1） */
  E22_MODE_SLEEP = 3,        /**< M1=1 M0=1 深度休眠 */
} e22_mode_t;

/** REG0[7:5] 串口波特率码值 */
typedef enum {
  E22_UART_BAUD_1200 = 0,
  E22_UART_BAUD_2400 = 1,
  E22_UART_BAUD_4800 = 2,
  E22_UART_BAUD_9600 = 3,
  E22_UART_BAUD_19200 = 4,
  E22_UART_BAUD_38400 = 5,
  E22_UART_BAUD_57600 = 6,
  E22_UART_BAUD_115200 = 7,
} e22_uart_baud_t;

/** REG0[4:3] 串口校验码值 */
typedef enum {
  E22_UART_PARITY_8N1 = 0,
  E22_UART_PARITY_8O1 = 1,
  E22_UART_PARITY_8E1 = 2,
} e22_uart_parity_t;

/**
 * REG0[2:0] 空中速率码值（按 E22-400T22S 的映射）
 * @note 400/900 频段码值 0/1/2 都是 2.4 kbps，驱动统一按默认码值 2 写入；
 *       230 频段 6/7 的速率不同，本工程不使用。
 */
typedef enum {
  E22_AIR_RATE_2K4 = 0,
  E22_AIR_RATE_4K8 = 3,
  E22_AIR_RATE_9K6 = 4,
  E22_AIR_RATE_19K2 = 5,
  E22_AIR_RATE_38K4 = 6,
  E22_AIR_RATE_62K5 = 7,
} e22_air_rate_t;

/** REG1[7:6] 串口分包长度码值 */
typedef enum {
  E22_PACKET_240B = 0,
  E22_PACKET_128B = 1,
  E22_PACKET_64B = 2,
  E22_PACKET_32B = 3,
} e22_packet_len_t;

/** REG1[1:0] 发射功率（22 dBm 模块的码值；30 dBm 模块同样码值对应 30/27/24/21） */
typedef enum {
  E22_TX_POWER_22DBM = 0,
  E22_TX_POWER_17DBM = 1,
  E22_TX_POWER_14DBM = 2,
  E22_TX_POWER_10DBM = 3,
} e22_tx_power_t;

/** REG3[3] WOR 角色 */
typedef enum {
  E22_WOR_ROLE_RX = 0, /**< WOR 接收方（低功耗监听） */
  E22_WOR_ROLE_TX = 1, /**< WOR 发送方（数据前插唤醒码） */
} e22_wor_role_t;

/** REG3[2:0] WOR 周期，实际周期 = (码值 + 1) × 500 ms */
typedef enum {
  E22_WOR_PERIOD_500MS = 0,
  E22_WOR_PERIOD_1000MS = 1,
  E22_WOR_PERIOD_1500MS = 2,
  E22_WOR_PERIOD_2000MS = 3,
  E22_WOR_PERIOD_2500MS = 4,
  E22_WOR_PERIOD_3000MS = 5,
  E22_WOR_PERIOD_3500MS = 6,
  E22_WOR_PERIOD_4000MS = 7,
} e22_wor_period_t;

/**
 * @brief 模块参数快照，对应寄存器 0x00~0x06（7 字节）
 * @note  0x07/0x08 是只写密钥、0x09/0x0A 是 WOR 应答窗口，三者不进本结构体，
 *        分别用 e22_write_key() / e22_write_wor_delay() 单独写。
 */
typedef struct {
  uint16_t address;             /**< 0x00/0x01 通信地址（0xFFFF = 广播/监听） */
  uint8_t netid;                /**< 0x02 网络码 */
  uint8_t channel;              /**< 0x05 信道，400 频段 0~83，频率 = 410.125 + ch MHz */
  e22_uart_baud_t uart_baud;    /**< 0x03[7:5] 串口波特率（切换模式后生效） */
  e22_uart_parity_t uart_parity;/**< 0x03[4:3] 串口校验 */
  e22_air_rate_t air_rate;      /**< 0x03[2:0] 空中速率（收发双方必须一致） */
  e22_packet_len_t packet_len;  /**< 0x04[7:6] 串口分包长度 */
  bool rssi_env;                /**< 0x04[5] 环境噪声 RSSI 使能 */
  bool soft_mode;               /**< 0x04[2] 软件切换模式使能 */
  e22_tx_power_t tx_power;      /**< 0x04[1:0] 发射功率分档 */
  bool rssi_byte;               /**< 0x06[7] 接收数据包末尾附加 RSSI 字节 */
  bool fixed_tx;                /**< 0x06[6] 定点传输（数据前 3 字节 = 目标地址高/低 + 信道） */
  bool relay;                   /**< 0x06[5] 中继转发 */
  bool lbt;                     /**< 0x06[4] 先听后发 */
  e22_wor_role_t wor_role;      /**< 0x06[3] WOR 角色 */
  e22_wor_period_t wor_period;  /**< 0x06[2:0] WOR 周期 */
} e22_params_t;

typedef struct e22 e22_t;

/**
 * @brief AUX 边沿回调（在 EXTI 中断上下文执行）
 * @param inst 驱动实例
 * @param busy true = AUX 低（模块开始忙），false = AUX 高（空闲）
 * @note  回调里只做"置事件标志/唤醒线程"这类最小工作，禁止阻塞、打印、HAL 长调用。
 */
typedef void (*e22_aux_cb_t)(e22_t *inst, bool busy);

/** @brief 驱动装配参数，由调用者提供（跨模块数据约定） */
typedef struct {
  uint8_t *rx_ring;        /**< 接收环形缓冲，调用者分配，容量必须是 2 的幂 */
  uint16_t rx_ring_size;   /**< 环形缓冲字节数，建议 1024 */
  e22_aux_cb_t aux_cb;     /**< 可选的 AUX 边沿回调，NULL 表示不关心 */
  void *user;              /**< 透传给回调的上下文 */
} e22_cfg_t;

/** @brief 驱动实例。字段由驱动维护，调用者只负责分配 storage */
struct e22 {
  uint8_t *rx_ring;          /**< 调用者提供的环形缓冲 */
  uint16_t rx_ring_size;     /**< 容量（2 的幂） */
  volatile uint16_t rx_head; /**< 仅 RX 中断写 */
  volatile uint16_t rx_tail; /**< 仅消费线程写 */
  volatile uint32_t rx_dropped; /**< 缓冲满时丢弃的字节数（累计） */
  volatile bool aux_busy;    /**< 最近一次 AUX 边沿的状态，供快速判断 */
  e22_mode_t mode;           /**< 驱动记录的当前模式 */
  bool ready;                /**< init 成功（复位后确认过 AUX 空闲） */
  bool fixed_tx;             /**< 最近一次读写参数得到的定点传输标志（决定能否 >240 B 分块） */
  e22_aux_cb_t aux_cb;       /**< AUX 回调 */
  void *user;                /**< 回调上下文 */
};

/*---------------------------------------------------------------------------*/
/* 生命周期与模式                                                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief 装配驱动：绑定板级资源、复位模块、等待启动完成并启动 UART 接收
 * @param inst 调用者分配的实例
 * @param cfg  装配参数（rx_ring 必须非空，容量必须是 2 的幂）
 * @retval E22_OK 模块已就绪（AUX 回到空闲）
 * @retval E22_ERR_PARAM 参数非法
 * @retval E22_ERR_TIMEOUT 复位后 1 s 内 AUX 仍未回到空闲（查供电/NRST/AUX 接线）
 * @note  必须在模块 VCC 上电后调用；本工程在 tx_kernel_enter() 之前调用，
 *        因此内部只用 HAL_Delay，不依赖 RTOS。
 */
e22_status_t e22_init(e22_t *inst, const e22_cfg_t *cfg);

/**
 * @brief 硬件复位模块（NRST 低 2 ms → 释放 → 等待 16 ms 启动 + AUX 空闲）
 * @retval E22_OK 复位完成且 AUX 空闲
 * @retval E22_ERR_TIMEOUT AUX 超时未空闲
 * @note 复位后模块回到透传模式，参数保持掉电前的值。
 */
e22_status_t e22_reset(e22_t *inst);

/**
 * @brief 切换工作模式
 * @param mode 目标模式
 * @param timeout_ms 每次 AUX 等待的超时（切换前后各等一次）
 * @retval E22_OK 已切换完成
 * @retval E22_ERR_TIMEOUT AUX 一直忙（模块还在发上一包/切换）
 * @retval E22_ERR_STATE 未初始化
 * @note 切换到配置模式后串口被模块强制为 9600-8-N-1，本板 LPUART1 恰好匹配。
 *       从休眠模式唤醒时不等 AUX（休眠期间模块可能不再驱动该脚）。
 */
e22_status_t e22_set_mode(e22_t *inst, e22_mode_t mode, uint32_t timeout_ms);

/** @brief 读取驱动记录的当前模式（不访问硬件） */
e22_mode_t e22_get_mode(const e22_t *inst);

/** @brief 读 AUX 引脚实时电平：true = 忙（低电平） */
bool e22_is_busy(const e22_t *inst);

/**
 * @brief 等待模块空闲（AUX 高）
 * @param timeout_ms 超时（ms）
 * @retval E22_OK 已空闲
 * @retval E22_ERR_TIMEOUT 超时
 */
e22_status_t e22_wait_idle(e22_t *inst, uint32_t timeout_ms);

/*---------------------------------------------------------------------------*/
/* 透传收发                                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief 透传发送（阻塞，按 240 B 分块）
 * @param data 待发送字节
 * @param len  字节数；超过 240 B 时按 240 B 分块，每块前都等模块空闲
 * @param timeout_ms 每次等待 AUX 空闲的超时
 * @retval E22_OK 全部字节已写入模块串口
 * @retval E22_ERR_STATE 未初始化，或当前不是透传/WOR 模式
 * @retval E22_ERR_TIMEOUT 模块一直忙
 * @retval E22_ERR_IO UART 发送失败
 * @retval E22_ERR_PARAM 定点传输模式下 len > 240（分块会破坏 3 字节目标头）
 * @note 返回时只保证字节已交给模块，不保证空中发送完成（默认 UAUX 关闭）。
 *       下一包会在发送前等 AUX，天然形成背压。
 *       定点传输时前 3 字节由调用者按「目标地址高、目标地址低、目标信道」拼好；
 *       一次调用只能是一个目标包（<= E22_PACKET_PAYLOAD_MAX），因为驱动按 240 B
 *       分块时不会给后续块补目标头。定点标志来自最近一次 read/write_params。
 */
e22_status_t e22_send(e22_t *inst, const uint8_t *data, uint16_t len,
                      uint32_t timeout_ms);

/**
 * @brief 从接收环形缓冲取数据（非阻塞）
 * @param buf 目标缓冲
 * @param max_len 最多取多少字节
 * @return 实际取出的字节数
 * @note 缓冲满时驱动丢弃新数据（不覆盖未消费数据），丢弃量见 rx_dropped。
 */
uint16_t e22_read(e22_t *inst, uint8_t *buf, uint16_t max_len);

/** @brief 环形缓冲中待读字节数 */
uint16_t e22_available(const e22_t *inst);

/** @brief 丢弃环形缓冲中所有待读数据 */
void e22_flush_rx(e22_t *inst);

/*---------------------------------------------------------------------------*/
/* 参数配置（内部自动切到配置模式，完成后恢复原模式）                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief 读模块参数（寄存器 0x00~0x06）
 * @param out 输出快照
 * @retval E22_OK 读取成功
 * @retval E22_ERR_PROTOCOL 模块返回 FF FF FF 或应答头不匹配
 * @retval E22_ERR_TIMEOUT 切换模式或应答超时
 */
e22_status_t e22_read_params(e22_t *inst, e22_params_t *out);

/**
 * @brief 写模块参数（寄存器 0x00~0x06），写入后切回原模式生效
 * @param params 目标参数
 * @retval E22_OK 写入并校验回显成功
 * @retval E22_ERR_PARAM 参数越界，或串口参数不是 9600-8-N-1
 * @retval E22_ERR_PROTOCOL 模块回显与写入内容不一致
 * @note 本板 LPUART1 用 LSE 32.768 kHz 作时钟，最高只能到约 10.9 kbps，
 *       因此只接受 9600-8-N-1；写别的波特率/校验会让模块失联。
 *       信道范围按 400 频段限制为 0~83。
 */
e22_status_t e22_write_params(e22_t *inst, const e22_params_t *params);

/**
 * @brief 写通信密钥（寄存器 0x07/0x08，只写不可读）
 * @param key 16 位密钥，0 表示不加密
 * @retval E22_OK 写入并校验回显成功
 */
e22_status_t e22_write_key(e22_t *inst, uint16_t key);

/** @brief 写 WOR 接收方应答窗口（寄存器 0x09/0x0A，单位 ms，0 = 关闭） */
e22_status_t e22_write_wor_delay(e22_t *inst, uint16_t delay_ms);

/** @brief 读 WOR 接收方应答窗口（寄存器 0x09/0x0A，单位 ms） */
e22_status_t e22_read_wor_delay(e22_t *inst, uint16_t *delay_ms);

/**
 * @brief 读产品信息（寄存器 0x80~0x86，7 字节二进制，如 00 22 10 16 0A 00 00）
 * @param pid 输出缓冲，至少 E22_PID_LEN 字节
 * @note 这 7 字节不是 ASCII 字符串，含义由厂家定义；要识别型号请用
 *       e22_at_query(..., "AT+DEVTYPE=?", "DEVTYPE=", ...)。
 */
e22_status_t e22_read_pid(e22_t *inst, uint8_t pid[E22_PID_LEN]);

/**
 * @brief 读写任意寄存器（配置模式 HEX 指令的底层接口）
 * @param start 起始地址；读允许 0x00~0x86，写只允许 0x00~0x0A
 * @param len   字节数 1~16
 * @param buf   读：输出；写：输入
 * @retval E22_OK 成功（写会校验回显）
 * @note 上层一般用 e22_read_params()/e22_write_params()；本接口用于 PID、
 *       寄存器级的特殊配置。
 */
e22_status_t e22_read_regs(e22_t *inst, uint8_t start, uint8_t len, uint8_t *buf);
e22_status_t e22_write_regs(e22_t *inst, uint8_t start, uint8_t len,
                            const uint8_t *buf);

/**
 * @brief 在配置模式下执行一条 AT 查询并取出响应值
 * @param command  完整命令，如 "AT+FWCODE=?"（不带 CR/LF）
 * @param key      响应前缀，如 "FWCODE="
 * @param out      输出值（去掉前缀与首尾空白）
 * @param out_size 输出缓冲大小
 * @param timeout_ms 等待响应的超时
 * @retval E22_OK 成功
 * @retval E22_ERR_PROTOCOL 没找到 key，或模块回 "=ERR"
 * @retval E22_ERR_TIMEOUT 无应答
 * @note 命令必须能在配置模式下执行；驱动自动切模式并在结束后恢复。
 */
e22_status_t e22_at_query(e22_t *inst, const char *command, const char *key,
                          char *out, uint16_t out_size, uint32_t timeout_ms);

/**
 * @brief 在配置模式下执行一条设置类 AT 命令（如 "AT+UAUX=1"）
 * @param command    完整命令，不带 CR/LF
 * @param timeout_ms 等待应答超时
 * @retval E22_OK 命令已执行（应答不是 "=ERR"）
 * @retval E22_ERR_PROTOCOL 模块回 "=ERR"
 * @retval E22_ERR_TIMEOUT 无应答（老固件对个别命令可能不回）
 * @note 命令必须能在配置模式下执行；驱动自动切模式并在结束后恢复。
 */
e22_status_t e22_at_command(e22_t *inst, const char *command,
                            uint32_t timeout_ms);

/*---------------------------------------------------------------------------*/
/* 中断入口：由 e22_port.c / HAL 回调调用，ISR 安全                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief 把一段收到的字节写入环形缓冲（RX 中断调用）
 * @note 空间不足时丢弃本段放不下的部分，绝不覆盖未消费数据。
 */
void e22_rx_feed(e22_t *inst, const uint8_t *data, uint16_t len);

/**
 * @brief 上报 AUX 边沿（EXTI 中断调用），更新状态并转发给用户回调
 */
void e22_aux_event(e22_t *inst, bool busy);

#ifdef __cplusplus
}
#endif

#endif /* E22_H */
