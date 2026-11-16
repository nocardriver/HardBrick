# QMC5883P 三轴地磁传感器（BSP）

QST QMC5883P，I2C 接口，±2/8/12/30 G 量程，16 位输出，ODR 10~200 Hz。
数据手册：`QST-PD-B002-22 Rev.E`。

## 文件划分

| 文件 | 职责 |
|---|---|
| `qmc5883p.c` | 驱动核心：寄存器协议、配置、连读、单位换算、内部自检。不依赖 HAL/RTOS |
| `qmc5883p_port.c` | 平台层：I2C2 阻塞收发、`HAL_GetTick` / `HAL_Delay` |
| `qmc5883p.h` | 对外 API、枚举、实例与自检结构体 |

## 硬件连接（本板）

| 传感器 | STM32U575 | 说明 |
|---|---|---|
| SCK (pin 1) | PB13 / I2C2_SCL | 需外部上拉，2.7 kΩ（总线 <10 cm）或 4.7 kΩ（<5 cm）到 3.3 V |
| SDA (pin 16) | PB14 / I2C2_SDA | 同上 |
| VDD (pin 2) | 3.3 V | 2.5~3.6 V，单电源，IO 电平也以它为参考 |
| GND (pin 9/11) | GND | |
| C1 (pin 10) | — | 4.7 µF 低 ESR 陶瓷电容到地 |

7 位地址 **0x2C**（写 0x58 / 读 0x59）。**不是 0x0D** —— 0x0D 是 QMC5883L，两者寄存器也不兼容。

CubeMX 侧：I2C2 = Fast Mode 400 kHz、7-bit、模拟滤波开、`NoStretch = Disable`。

## 用法

```c
#include "qmc5883p.h"

static qmc5883p_t g_mag;

void mag_start(void)
{
  /* cfg 传 NULL 使用 QMC5883P_CFG_DEFAULT：Normal 100 Hz / ±8 G / Set-Reset 开 */
  if (qmc5883p_init(&g_mag, NULL) != QMC5883P_OK) {
    return;                       /* CHIPID 不对或 I2C 不通，查供电/上拉/地址 */
  }
}

void mag_poll(void)
{
  qmc5883p_mg_t mg;

  if (qmc5883p_read_mg(&g_mag, &mg, 20u) == QMC5883P_OK) {
    /* mg.x_mg / y_mg / z_mg 单位毫高斯；mg.overflow 为 true 时该轮接近满量程 */
  }
}

void mag_stop(void)
{
  qmc5883p_suspend(&g_mag);       /* 进 Suspend：寄存器保持，I2C 仍可访问 */
}
```

要点：

- 读到的是**原始矢量**，硬磁/软磁标定属于应用层（App），不在 BSP 里做。
- `qmc5883p_read_sample()` 会先轮询 `0x09` 的 DRDY 再连读 6 字节；100 Hz 下
  一轮约 10 ms，超时给 20 ms 足够。
- 一次连读是必须的：分三次读 X/Y/Z 会跨采样点，姿态解算会看到撕裂的向量。

## 上电自检

`qmc5883p_probe()` 会依次做：地址探测 → CHIPID → 配置 → 采 16 个样本 → 换算 mG →
Set/Reset 内部自检，结果写进 `g_qmc_probe`（带魔数，可直接用 J-Link 读内存），不依赖 RTT。

`stage` 停在哪一步就说明卡在哪：

| stage | 含义 | 排查方向 |
|---|---|---|
| 1 `NO_ACK` | 地址 0x2C 没应答 | 供电、上拉电阻、PB13/PB14 是否接反 |
| 2 `ID_FAIL` | 有应答但 CHIPID≠0x80 | 地址误用了 0x0D，或挂在总线上的不是 P 型号 |
| 4 `INIT_FAIL` | 配置写入失败 | 总线被拉死、时序参数不对 |
| 6 `SAMPLE_FAIL` | 16 个样本全超时 | MODE 没生效、器件掉电 |
| 8 `SELFTEST_FAIL` | 内部激励读数失败 | 磁阻桥/模拟链路异常 |

正常时 `stage = 9 (DONE)`，且 `x_min != x_max`（数据确实在变）。

本板（HardBrick）实测结果，可当验收基线：

| 字段 | 实测值 | 说明 |
|---|---|---|
| `chip_id` | 0x80 | |
| `reg_ctrl1` / `reg_ctrl2` | 0x0D / 0x08 | ODR=200 Hz、Normal、RNG=±8 G |
| `x/y/z` | 约 650~700 / 1200~1260 / 1000~1120 LSB | ±8 G 档 |
| `last_mg_l1` | 约 777 mG | `\|x\|+\|y\|+\|z\|`，矢量模约 0.46 G ≈ 46 µT |
| `st_dx/dy/dz` | 约 −3850 / −3650 / −3520 LSB | 内部激励约 1 G，三轴均明显非零 |

## 实测确认的两件事

1. **RNG 码与灵敏度按手册排列**。四个码各采 8 个样本，同一磁场下
   `|x|+|y|+|z|` 之比为 1 : 2.5 : 3.75 : 15，与 1000/2500/3750/15000 LSB/G 完全对应
   （码值 0=±30 G、1=±12 G、2=±8 G、3=±2 G）。
2. **软复位后第一次配置会被器件还原**：写完 0x0B=0x08 再读回是 0x00，
   同时 0x0A 又写进去了——表现为量程停在 ±30 G、读数只有应有值的 1/3.75。
   驱动在 `qmc5883p_init()` 里做了「回读 0x0B + 不一致就再走一遍 apply_config」的兜底，
   之后每次换配置都正常。第二次一定生效，原因未知（NVM 重载时序的可能性最大）。

## 已知坑

- 上电复位后是 **Suspend** 模式，不写 `0x0A` 就永远读不到数据。
- 读 `0x09` 会清零 DRDY/OVFL，所以"读状态"和"读数据"不能各自读一次状态。
- 软复位会把 `0x29`（轴符号）也清掉，`qmc5883p_init()` 里在复位后重写了 `0x06`。
- Normal/Single/Continuous 之间切换必须先经过 Suspend（手册 §9.2.3），驱动已处理。
- 软复位后的第一次配置见上文，驱动已用回读校验兜住；自己手写初始化流程时要小心。
