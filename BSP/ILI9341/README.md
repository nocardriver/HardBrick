# ILI9341 BSP 模块设计文档

## 1. 模块概述

本模块驱动 STARTEK **KD035QVRMA223** 3.5" TFT 模组（240×320，控制器 **ILI9341**，4-line SPI）。
职责限定为**控制器寄存器操作 + 像素搬运**：初始化、地址窗口、单色填充、RGB565 像素块写入。
业务画面（布局、文字、刷新策略）全部在 `Apps/display`，本模块不持有帧缓冲、不依赖 RTOS。

**资料**：

- 本地规格书：`C:\myProjects\资料\屏幕资料\KD035QVRMA223 规格书 V1.0.pdf`
  （第 4 页确认 240×320 / ILI9341 / 3-4 线 SPI；第 7~8 页为引脚定义）
- 4-line SPI 初始化参考代码：`KD035VRMA223-4SPI-MCU CODE .txt`（厂商例程）
- 已验证的 STM32F4 演示工程：`C:\myProjects\STM32proj\Brick`
  （10 MHz SPI、GPIO 手动 CS/DC、阻塞发送；本模块的初始化序列与 MADCTL 值取自该工程）

## 2. 硬件说明

### 2.1 接口与引脚

| 信号 | 引脚 | 说明 |
|---|---|---|
| SPI1_SCK | PA5 (AF5) | 时钟，Mode 0（CPOL=0, CPHA=1Edge） |
| SPI1_MOSI | PA7 (AF5) | 数据（面板 SDO 未使用，本工程只写） |
| SPI1_CS | PA4 (GPIO) | 片选，低有效；软件 NSS |
| DIS_DC | PC4 (GPIO) | 1=数据，0=命令 |
| DIS_RST | PB0 (GPIO) | 复位，低有效 |

> **PW_EN（PB3）是整机电源控制，不是面板背光使能，也不属于本模块。**
> 它在 `main.c` 启动流程里由用户代码拉高（本模块不碰它），显示驱动只使用上表的 5 根信号线。
> 面板背光由 `BSP/Backlight` 通过 TIM17_CH1（PB9）PWM 控制，不属于本模块。

- SPI1 时钟源 PCLK2 = 160 MHz，CubeMX 中 `BaudRatePrescaler=4` → **40 MHz**，8 bit、MSB first。
- DMA：SPI1_TX 走 GPDMA1 **Channel 2**（Channel 0/1 已被 USART2 RX/TX 占用），
  中断优先级 10（与 USART2 相同）。CubeMX 生成在 `HAL_SPI_MspInit()` 里。
- 数据格式：RGB565，**每个像素高字节先发**（大端）。
- 初始化时序：RST 高 1 ms → 低 10 ms → 高 120 ms → `0x01` 软复位等待 1000 ms → 其余寄存器 → `0x11` 等待 120 ms → `0x29`。
  总耗时约 1.3 s，因此 `ili9341_init()` 应在 `tx_kernel_enter()` 之前调用。

### 2.2 需要留意的点

- **电源**：`PW_EN`（PB3）是**整机电源控制**，由 `main.c` 启动时拉高，不是屏幕背光，也不由显示驱动管理。本模块和 `Apps/display` 都不读写该引脚；背光由独立的 `BSP/Backlight`（TIM17_CH1/PB9）管理，按键点亮策略在 `Apps/backlight`。
- **40 MHz 超出面板规格**：模块规格书 4-line SPI 写周期 tSCYCW ≥ 66 ns → SCL ≤ 15.15 MHz
  （ILI9341 手册给出的串行写上限是 10 MHz），当前按 40 MHz 跑，短排线下实测可用，
  是**已知并接受的超规格使用**。若出现花屏、错位或颜色异常，先把 `SPI1.BaudRatePrescaler`
  改为 8（20 MHz）或 16（10 MHz）再复测；DMA 不改变总线时序，不会让这个风险变大。
- **默认方向**：由 `ili9341.h` 的 `ILI9341_DEFAULT_ROTATION` 决定，当前是
  `ILI9341_ROT_PORTRAIT_2`（MADCTL=0x88，MX=0 MY=1 BGR=1）——本机面板倒装，
  逻辑画面相对 `ILI9341_ROT_PORTRAIT_1`（0x48，MX=1）旋转 180°。
  换装配方向只改这个宏：`PORTRAIT_1` 正向、`PORTRAIT_2` 旋转 180°。
- **方向只定义一处**：`g_init_seq` 里的 MADCTL 只写 BGR 位（`ILI9341_MADCTL_BGR`），
  方向位由 `ili9341_init()` 末尾的 `ili9341_set_rotation()` 统一写入，
  不再需要同步初始化表与 `set_rotation()` 两个 0x48。
- **红蓝互换**：把 `ILI9341_MADCTL_BGR` 与 `ili9341_set_rotation()` 各枚举值的
  0x08 位去掉（BGR→RGB）。
- **SPE=0 期间必须保持 IO 驱动（`AFCNTR=1`）**：HAL 在每笔传输结束（EOT）都会把 SPI 关掉；
  如果 `MasterKeepIOState=DISABLE`（`AFCNTR=0`），SPI 会**放开 SCK/MOSI**，悬空引脚上的毛刺
  会被面板当成一个时钟沿，每笔传输让像素位相位漂移 1 bit —— 表现为纯色填充变成彩色横条、
  文字因会话短而基本正常，且**与 SPI 时钟无关**（40 MHz 和 10 MHz 一样）。
  已在 `Core/Src/spi.c` 的 `USER CODE BEGIN SPI1_Init 2` 里置位，并在 `.ioc` 中设为
  `SPI1.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE`。
- **发送方式**：命令/参数（< 64 字节）走 `HAL_SPI_Transmit` 轮询；像素数据在内核启动并
  `ili9341_rtos_attach()` 之后走 SPI1_TX DMA（GPDMA1 Channel 2），线程在信号量上睡眠等 EOT。
  DMA 完成链路是 HAL 定的：GPDMA 传输完成中断只负责打开 SPI 的 EOT 中断，真正收尾
  （`SPE=0`、清 `TXDMAEN`、调用 `HAL_SPI_TxCpltCallback`）在 **SPI1 全局中断**里，
  所以 CubeMX 里 `SPI1 global interrupt` 必须勾选，否则 DMA 永远"不完成"。
- **不需要 cache 维护**：内部 SRAM（本驱动的 `g_text_buf`、`g_fill_chunk` 都在 `.bss`）走
  Cortex-M33 S-bus 的独立通路，不经 DCACHE1（RM0456 Table 1 / §2.1.4），GPDMA 读到的就是
  CPU 写入的内容。Cortex-M33 也没有核心 D-Cache，`SCB_CleanDCache_by_Addr()` 不适用；
  只有以后把帧缓冲放到外部 PSRAM（OCTOSPI）时才需要 `HAL_DCACHE_CleanByAddr()`。
- **错误处理**：DMA 启动失败（`HAL_BUSY` 等，数据一个字节都没发出）会自动重试 2 次；
  超时或下溢这类"可能已发出一部分"的错误**不重发**——像素流是连续的，重发会让后续像素
  整体后移，比停画更糟，只统计并让驱动停画。超时时若查到 EOT 其实已置位（数据已发完、
  只是收尾中断没到）则按成功处理；连续 3 次都靠这种方式救回，说明收尾中断不可用，
  自动**永久回退轮询**，避免每笔都等满超时。
- 面板 SDO 未接回 MCU，ILI9341 的读寄存器/读 GRAM 功能不可用；驱动只做写方向，不做读回校验。
- SPI1 是全双工 2LINES 配置，但本工程只写不读；MISO 上的无效数据不会影响发送（HAL 只轮询 TXP/EOT）。

## 3. 文件划分

| 文件 | 职责 |
|---|---|
| `ili9341.h` | 对外 API、尺寸/颜色宏、方向枚举。不含 HAL 类型 |
| `ili9341.c` | 初始化序列表、`0x2A/0x2B/0x2C` 窗口、填充、blit、方向切换。**不 include HAL** |
| `ili9341_port.h/.c` | 平台移植层：SPI1 轮询 / DMA 发送、DMA 完成信号量、CS/DC/RST、`HAL_Delay`。**唯一允许 include HAL / ThreadX 的文件** |
| `CMakeLists.txt` | target `ili9341`，注册进 `BSP_LIBS` |

## 4. API 契约

```c
bool     ili9341_init(void);                 /* 复位 + 初始化序列 + 清黑；失败后绘制接口变空操作 */
bool     ili9341_is_ready(void);
bool     ili9341_rtos_attach(void);          /* 内核启动后绑定 DMA（tx_application_define 内调用） */
void     ili9341_set_rotation(ili9341_rotation_t rot);
uint16_t ili9341_width(void);                /* 当前方向下的逻辑宽度 */
uint16_t ili9341_height(void);
void     ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void     ili9341_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels);
void     ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
```

- `ili9341_init()` 必须在 `tx_kernel_enter()` **之前**调用（内部约 1.3 s 阻塞延时，且此时 DMA 还不可用）。
- `ili9341_rtos_attach()` 必须在 `tx_application_define()` 内、显示线程启动前调用；返回 false 表示
  CubeMX 没配 SPI1_TX DMA，发送会自动保持轮询，功能不受影响。

- `ili9341_fill_rect()` 自动裁剪到屏幕内；`ili9341_blit()` **不裁剪**（裁剪会改变源缓冲的行跨度），调用者必须保证矩形完全在屏幕内。
- `pixels` 必须是大端 RGB565：`pixels[2*i]=color>>8`、`pixels[2*i+1]=color&0xFF`。
- 颜色常量用 `ILI9341_RGB565(r,g,b)` 生成，例如 `ILI9341_RGB565(0, 200, 255)`。
- 方向：竖屏 `ILI9341_ROT_PORTRAIT_1/2`（240×320；默认 `PORTRAIT_2`，即面板倒装、旋转 180°），
  横屏 `ILI9341_ROT_LANDSCAPE_1/2`（320×240）。

## 5. 与 LVGL 的衔接

后续接 LVGL 时不需要改本模块的驱动核心：

1. 在 LVGL 的 `flush_cb` 里调用 `ili9341_blit(area->x1, area->y1, w, h, buf)`；
2. LVGL 的 RGB565 输出是小端，`flush_cb` 里需要做一次字节交换（或用 LVGL 的 `LV_COLOR_16_SWAP`）；
3. 发送已经是 SPI1 TX DMA，`flush_cb` 里调用 `ili9341_blit()` 会在 DMA 完成后返回，
   LVGL 可以安全地复用/切换自己的 draw buffer；若以后要做"边发边渲染"，再在
   `ili9341_port.c` 里加异步完成回调，`ili9341.c` 不需要动。
