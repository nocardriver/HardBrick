/**
 * @file    qmc5883p.h
 * @author  YangKY
 * @brief   QMC5883P 三轴地磁传感器驱动核心（BSP 层，纯 C，不依赖 HAL/RTOS）
 *
 * 器件要点（QST 数据手册 QST-PD-B002-22 Rev.E）：
 *   - I2C 从机地址固定为 7 位 0x2C（写 0x58 / 读 0x59），**不是** QMC5883L 的 0x0D；
 *   - 速率上限 400 kHz（Fast mode），100 kHz 亦可；总线需外部上拉（2.7 k / 4.7 k）；
 *   - 单电源 2.5~3.6 V，VDD 同时也是 IO 电平参考；
 *   - 上电复位后进入 Suspend 模式，必须写控制寄存器 0x0A 的 MODE 位才会出数据；
 *   - 数据寄存器 0x01~0x06 依次是 X/Y/Z 的 LSB、MSB（小端、16 位补码）；
 *   - 状态寄存器 0x09 的 bit0 = DRDY（读该寄存器即清零）、bit1 = OVFL。
 *
 * 本层职责：寄存器读写协议、模式/量程/滤波配置、数据读取与单位换算。
 * 板级资源（I2C2、HAL 调用、毫秒延时）全部收敛在 qmc5883p_port.c。
 *
 * 使用约束：
 *   1. qmc5883p_init() 前必须已完成 MX_I2C2_Init()（CubeMX 生成，PB13/PB14）；
 *   2. 驱动不加锁：同一实例不得在两个线程里并发调用（App 侧用单线程采集即可）；
 *   3. 所有 API 都是阻塞的（每次事务最坏几十毫秒），禁止在中断里调用。
 */

#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 7 位从机地址；HAL 接口需要左移一位（0x58 写 / 0x59 读），左移在 port 层完成 */
#define QMC5883P_I2C_ADDR_7BIT  0x2Cu

/** CHIPID（寄存器 0x00）的固定值，用于确认器件与接线正确 */
#define QMC5883P_CHIP_ID        0x80u

/** 软复位后等待器件重新就绪的时间（数据手册未给，实测 5 ms 不够） */
#define QMC5883P_RESET_DELAY_MS 20u

/**
 * @brief 返回值。错误为负值，便于 `if (st != QMC5883P_OK)` 与 RTT 直接打印
 */
typedef enum {
  QMC5883P_OK = 0,           /**< 成功 */
  QMC5883P_ERR_PARAM = -1,   /**< 参数非法（空指针、枚举越界） */
  QMC5883P_ERR_IO = -2,      /**< I2C 事务失败（NACK、总线错误、HAL 超时） */
  QMC5883P_ERR_ID = -3,      /**< CHIPID 不是 0x80：器件不存在或地址/接线错误 */
  QMC5883P_ERR_TIMEOUT = -4, /**< 等待 DRDY 超时（模式没开、ODR 太慢或器件掉电） */
  QMC5883P_ERR_STATE = -5,   /**< 当前状态不允许该操作（未初始化） */
} qmc5883p_status_t;

/** 寄存器 0x0A[1:0] 工作模式 */
typedef enum {
  QMC5883P_MODE_SUSPEND = 0,    /**< 挂起：不上电测量，寄存器仍可读写（POR 默认） */
  QMC5883P_MODE_NORMAL = 1,     /**< 连续测量，按 ODR 更新，测量之间有休眠 */
  QMC5883P_MODE_SINGLE = 2,     /**< 单次测量，完成后自动回 Suspend */
  QMC5883P_MODE_CONTINUOUS = 3, /**< 全速连续测量（自检只能用这个模式） */
} qmc5883p_mode_t;

/** 寄存器 0x0A[3:2] 输出数据率 */
typedef enum {
  QMC5883P_ODR_10HZ = 0,
  QMC5883P_ODR_50HZ = 1,
  QMC5883P_ODR_100HZ = 2,
  QMC5883P_ODR_200HZ = 3,
} qmc5883p_odr_t;

/** 寄存器 0x0A[5:4] 过采样率 1（数字滤波器带宽） */
typedef enum {
  QMC5883P_OSR1_8 = 0, /**< 过采样 8 次：噪声最低、功耗最高 */
  QMC5883P_OSR1_4 = 1,
  QMC5883P_OSR1_2 = 2,
  QMC5883P_OSR1_1 = 3, /**< 不做平均：功耗最低 */
} qmc5883p_osr1_t;

/** 寄存器 0x0A[7:6] 降采样率（第二级滤波器深度） */
typedef enum {
  QMC5883P_OSR2_1 = 0,
  QMC5883P_OSR2_2 = 1,
  QMC5883P_OSR2_4 = 2,
  QMC5883P_OSR2_8 = 3,
} qmc5883p_osr2_t;

/** 寄存器 0x0B[3:2] 量程；量程越小灵敏度越高 */
typedef enum {
  QMC5883P_RANGE_30G = 0, /**< ±30 G，1000 LSB/G */
  QMC5883P_RANGE_12G = 1, /**< ±12 G，2500 LSB/G */
  QMC5883P_RANGE_8G = 2,  /**< ±8 G，3750 LSB/G（地磁约 ±0.6 G，够用） */
  QMC5883P_RANGE_2G = 3,  /**< ±2 G，15000 LSB/G（精度最高，易饱和） */
} qmc5883p_range_t;

/** 寄存器 0x0B[1:0] Set/Reset 模式；关闭后偏移不再每次测量更新，零漂变差 */
typedef enum {
  QMC5883P_SET_RESET_ON = 0,  /**< 每次测量都做 Set 和 Reset（推荐） */
  QMC5883P_SET_ONLY = 1,      /**< 只做 Set */
  QMC5883P_SET_RESET_OFF = 2, /**< 关闭（码值 2 和 3 等价） */
} qmc5883p_setreset_t;

/** @brief 器件配置（对应寄存器 0x0A / 0x0B 的全部有效位） */
typedef struct {
  qmc5883p_mode_t mode;        /**< 工作模式，上电后驱动会切到该模式 */
  qmc5883p_odr_t odr;          /**< 输出数据率 */
  qmc5883p_osr1_t osr1;        /**< 第一级过采样 */
  qmc5883p_osr2_t osr2;        /**< 第二级降采样 */
  qmc5883p_range_t range;      /**< 量程 */
  qmc5883p_setreset_t set_reset; /**< Set/Reset 模式 */
} qmc5883p_cfg_t;

/** 默认配置：Normal 100 Hz、OSR1=8、OSR2=1、±8 G、Set/Reset 开（地磁罗盘常用档） */
#define QMC5883P_CFG_DEFAULT                        \
  {                                                 \
    QMC5883P_MODE_NORMAL, QMC5883P_ODR_100HZ,       \
    QMC5883P_OSR1_8, QMC5883P_OSR2_1,               \
    QMC5883P_RANGE_8G, QMC5883P_SET_RESET_ON,       \
  }

/** @brief 一次原始读数（传感器坐标，单位 LSB） */
typedef struct {
  int16_t x;        /**< X 轴原始计数 */
  int16_t y;        /**< Y 轴原始计数 */
  int16_t z;        /**< Z 轴原始计数 */
  bool fresh;       /**< 读取前 DRDY=1（false 表示这是上一轮的旧数据） */
  bool overflow;    /**< OVFL=1：某轴输出超出 ±30000 LSB，读数可能已经削顶 */
} qmc5883p_raw_t;

/** @brief 一次换算后的读数（毫高斯，1 mG = 0.1 µT） */
typedef struct {
  int32_t x_mg;     /**< X 轴磁场，单位 mG */
  int32_t y_mg;     /**< Y 轴磁场，单位 mG */
  int32_t z_mg;     /**< Z 轴磁场，单位 mG */
  bool fresh;       /**< 同 qmc5883p_raw_t::fresh */
  bool overflow;    /**< 同 qmc5883p_raw_t::overflow */
} qmc5883p_mg_t;

/** @brief 驱动实例。字段由驱动维护，调用者只负责分配 storage */
typedef struct {
  qmc5883p_cfg_t cfg;       /**< 当前生效的配置（set_mode 会同步更新 mode 字段） */
  int32_t lsb_per_gauss;    /**< 当前量程的灵敏度，用于 LSB → mG 换算 */
  bool ready;               /**< init 成功（CHIPID 校验通过） */
  uint32_t samples;         /**< 累计成功读数次数（诊断用） */
  uint32_t timeouts;        /**< 累计等待 DRDY 超时次数 */
  uint32_t io_errors;       /**< 累计 I2C 事务失败次数 */
  uint32_t overflows;       /**< 累计观察到 OVFL 的次数 */
} qmc5883p_t;

/*---------------------------------------------------------------------------*/
/* 生命周期与配置                                                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief 初始化器件：校验 CHIPID、软复位、写轴符号与配置、切到目标模式
 * @param inst 调用者分配的实例
 * @param cfg  目标配置；传 NULL 使用 QMC5883P_CFG_DEFAULT
 * @retval QMC5883P_OK 已就绪并开始测量
 * @retval QMC5883P_ERR_PARAM cfg 里有越界枚举
 * @retval QMC5883P_ERR_ID 读不到 0x80（查供电、上拉、地址、PB13/PB14 接线）
 * @retval QMC5883P_ERR_IO I2C 事务失败
 * @note 必须在 MX_I2C2_Init() 之后调用；内部用 port 层的 HAL_Delay，不依赖 RTOS，
 *       因此可以在 tx_kernel_enter() 之前调用。
 */
qmc5883p_status_t qmc5883p_init(qmc5883p_t *inst, const qmc5883p_cfg_t *cfg);

/**
 * @brief 软复位（写 0x0B[7]=1），所有寄存器恢复默认值、器件回到 Suspend
 * @note 复位后需要重新 qmc5883p_init() 或 qmc5883p_apply_config() 才能出数据。
 */
qmc5883p_status_t qmc5883p_soft_reset(qmc5883p_t *inst);

/**
 * @brief 写入完整配置（0x0B 之后再写 0x0A，避免先启动测量再用错量程）
 * @retval QMC5883P_OK 已写入并更新实例缓存
 * @retval QMC5883P_ERR_STATE 未初始化
 */
qmc5883p_status_t qmc5883p_apply_config(qmc5883p_t *inst,
                                        const qmc5883p_cfg_t *cfg);

/**
 * @brief 切换工作模式
 * @param mode 目标模式
 * @note 数据手册要求 Normal/Single/Continuous 之间切换要先经过 Suspend，
 *       本函数已按该顺序处理（多一次 0x0A 写入 + 1 ms 等待）。
 */
qmc5883p_status_t qmc5883p_set_mode(qmc5883p_t *inst, qmc5883p_mode_t mode);

/** @brief 进入 Suspend（寄存器值保持，I2C 仍可用），用于间歇采样省电 */
qmc5883p_status_t qmc5883p_suspend(qmc5883p_t *inst);

/** @brief 读 CHIPID（寄存器 0x00），正常应为 QMC5883P_CHIP_ID */
qmc5883p_status_t qmc5883p_read_chip_id(qmc5883p_t *inst, uint8_t *id);

/*---------------------------------------------------------------------------*/
/* 数据读取                                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief 读状态寄存器 0x09（读操作本身会清零 DRDY 与 OVFL）
 * @param drdy 输出：有新数据
 * @param ovfl 输出：上一轮数据溢出（可为 NULL）
 */
qmc5883p_status_t qmc5883p_read_status(qmc5883p_t *inst, bool *drdy,
                                       bool *ovfl);

/**
 * @brief 直接连读 0x01~0x06 六个字节（不等待 DRDY，拿到的是最新一轮数据）
 * @param out 输出原始读数
 * @note 必须一次连读：分三次读 X/Y/Z 会跨采样点，姿态解算会看到"撕裂"的向量。
 */
qmc5883p_status_t qmc5883p_read_raw(qmc5883p_t *inst, qmc5883p_raw_t *out);

/**
 * @brief 等待 DRDY 后读一轮数据（轮询状态寄存器，间隔 1 ms）
 * @param timeout_ms 最长等待时间；按 ODR 估：100 Hz 时一轮约 10 ms，给 20 ms 足够
 * @retval QMC5883P_OK 拿到新数据
 * @retval QMC5883P_ERR_TIMEOUT 超时（MODE 仍是 Suspend 是最常见原因）
 */
qmc5883p_status_t qmc5883p_read_sample(qmc5883p_t *inst, qmc5883p_raw_t *out,
                                       uint32_t timeout_ms);

/**
 * @brief 等待 DRDY 后读一轮数据并换算成毫高斯
 * @param timeout_ms 同 qmc5883p_read_sample()
 */
qmc5883p_status_t qmc5883p_read_mg(qmc5883p_t *inst, qmc5883p_mg_t *out,
                                   uint32_t timeout_ms);

/** @brief 量程对应的灵敏度（LSB/G）：30G→1000、12G→2500、8G→3750、2G→15000 */
int32_t qmc5883p_range_sensitivity(qmc5883p_range_t range);

/**
 * @brief 把原始计数换算成毫高斯：mg = raw * 1000 / sensitivity
 * @note 用整数运算，避免引入浮点（RTT 打印也不支持 %f）。
 *       8 G 量程下 1 LSB ≈ 0.27 mG，舍入误差可忽略。
 */
int32_t qmc5883p_raw_to_mg(const qmc5883p_t *inst, int16_t raw);

/*---------------------------------------------------------------------------*/
/* 上电自检（填 g_qmc_probe，可用 J-Link 直接读内存，不依赖 RTT）             */
/*---------------------------------------------------------------------------*/

/** 自检结构体魔数，用来判断 g_qmc_probe 是否已被填过 */
#define QMC5883P_PROBE_MAGIC 0x51C3A001u

/** 自检阶段：失败时 stage 停在出问题的那一步 */
typedef enum {
  QMC5883P_STAGE_IDLE = 0,          /**< 还没跑 */
  QMC5883P_STAGE_NO_ACK = 1,        /**< 地址 0x2C 无应答：没供电/没上拉/接错线 */
  QMC5883P_STAGE_ID_FAIL = 2,       /**< 有应答但 CHIPID 不是 0x80 */
  QMC5883P_STAGE_ID_OK = 3,         /**< CHIPID 正常 */
  QMC5883P_STAGE_INIT_FAIL = 4,     /**< 配置写入失败 */
  QMC5883P_STAGE_CONFIGURED = 5,    /**< 配置完成，开始采样 */
  QMC5883P_STAGE_SAMPLE_FAIL = 6,   /**< 采样超时/失败 */
  QMC5883P_STAGE_SAMPLED = 7,       /**< 采样完成 */
  QMC5883P_STAGE_SELFTEST_FAIL = 8, /**< 内部自检（Set/Reset 激励）失败 */
  QMC5883P_STAGE_DONE = 9,          /**< 全部通过 */
} qmc5883p_stage_t;

/** @brief 上电自检结果，布局固定，便于 J-Link 直接按字段看 */
typedef struct {
  uint32_t magic;   /**< QMC5883P_PROBE_MAGIC */
  uint32_t stage;   /**< qmc5883p_stage_t */
  uint8_t chip_id;  /**< 寄存器 0x00 */
  uint8_t status;   /**< 最近一次读到的寄存器 0x09 */
  uint8_t reg_ctrl1; /**< 读回的寄存器 0x0A（期望 ODR=3、MODE=1 → 0x0D） */
  uint8_t reg_ctrl2; /**< 读回的寄存器 0x0B（期望 RNG=2、Set/Reset=0 → 0x08） */
  uint16_t reserved;
  uint32_t samples; /**< 本轮采到的样本数 */
  int16_t x_min;    /**< 采样窗口内各轴最小/最大值，用于确认数据确实在变 */
  int16_t x_max;
  int16_t y_min;
  int16_t y_max;
  int16_t z_min;
  int16_t z_max;
  int16_t x_last;   /**< 最后一个样本 */
  int16_t y_last;
  int16_t z_last;
  int16_t st_dx;    /**< 自检增量（打开内部激励前后之差） */
  int16_t st_dy;
  int16_t st_dz;
  uint32_t errors;  /**< 采集过程中的失败次数 */
  int32_t last_mg_l1; /**< 最后一个样本换算成 mG 后的 |x|+|y|+|z|（验证 LSB→mG 换算） */
} qmc5883p_probe_t;

extern volatile qmc5883p_probe_t g_qmc_probe;

/**
 * @brief 上电自检：地址探测 → CHIPID → 配置 → 采 16 个样本 → 内部自检
 * @note 阻塞约 120 ms（含软复位等待、16 个样本与一次内部自检），只在启动阶段调用一次；
 *       全程不打印，结果写 g_qmc_probe，由调用者决定用 RTT 输出还是 J-Link 查看。
 */
void qmc5883p_probe(void);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883P_H */
