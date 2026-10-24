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
> 面板背光由硬件电路直接供电，本工程没有软件调光通路。

- SPI1 时钟源 PCLK2 = 160 MHz，CubeMX 中 `BaudRatePrescaler=4` → **40 MHz**，8 bit、MSB first。
- 数据格式：RGB565，**每个像素高字节先发**（大端）。
- 初始化时序：RST 高 1 ms → 低 10 ms → 高 120 ms → `0x01` 软复位等待 1000 ms → 其余寄存器 → `0x11` 等待 120 ms → `0x29`。
  总耗时约 1.3 s，因此 `ili9341_init()` 应在 `tx_kernel_enter()` 之前调用。

### 2.2 需要留意的点

- **电源**：`PW_EN`（PB3）是**整机电源控制**，由 `main.c` 启动时拉高，不是屏幕背光，也不由显示驱动管理。本模块和 `Apps/display` 都不读写该引脚；面板背光无软件调光通路。
- **40 MHz 超出 ILI9341 手册的串行写时序上限（10 MHz）**。Brick 演示工程用 10 MHz 验证；本工程按当前 CubeMX 配置跑 40 MHz，短排线下通常可用。若出现花屏、错位或颜色异常，先把 `SPI1.BaudRatePrescaler` 改为 8（20 MHz）或 16（10 MHz）再复测。
- **MADCTL=0x48**（MX=1、BGR=1）沿用 Brick / Adafruit 的标准竖屏值。若上电后发现：
  - 红蓝互换：把 `0x48` 改成 `0x40`（BGR=0）；
  - 画面左右镜像：把 `0x48` 改成 `0x08`（MX=0）；
  - 画面上下镜像：把 `0x48` 改成 `0x88`（MY=1）。
  这三个值都在 `ili9341.c` 的 `g_init_seq` 与 `ili9341_set_rotation()` 里。
- 本驱动使用**阻塞发送**，不做 DMA。DCache 已使能，但阻塞模式下 CPU 直接读源缓冲，不存在 cache 一致性问题。将来改 DMA 时必须对源缓冲做 `SCB_CleanDCache_by_Addr()`。
- 面板 SDO 未接回 MCU，ILI9341 的读寄存器/读 GRAM 功能不可用；驱动只做写方向，不做读回校验。
- SPI1 是全双工 2LINES 配置，但本工程只写不读；MISO 上的无效数据不会影响发送（HAL 只轮询 TXP/EOT）。

## 3. 文件划分

| 文件 | 职责 |
|---|---|
| `ili9341.h` | 对外 API、尺寸/颜色宏、方向枚举。不含 HAL 类型 |
| `ili9341.c` | 初始化序列表、`0x2A/0x2B/0x2C` 窗口、填充、blit、方向切换。**不 include HAL** |
| `ili9341_port.h/.c` | 平台移植层：SPI1 阻塞发送、CS/DC/RST、`HAL_Delay`。**唯一允许 include HAL 的文件** |
| `CMakeLists.txt` | target `ili9341`，注册进 `BSP_LIBS` |

## 4. API 契约

```c
bool     ili9341_init(void);                 /* 复位 + 初始化序列 + 清黑；失败后绘制接口变空操作 */
bool     ili9341_is_ready(void);
void     ili9341_set_rotation(ili9341_rotation_t rot);
uint16_t ili9341_width(void);                /* 当前方向下的逻辑宽度 */
uint16_t ili9341_height(void);
void     ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void     ili9341_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels);
void     ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
```

- `ili9341_fill_rect()` 自动裁剪到屏幕内；`ili9341_blit()` **不裁剪**（裁剪会改变源缓冲的行跨度），调用者必须保证矩形完全在屏幕内。
- `pixels` 必须是大端 RGB565：`pixels[2*i]=color>>8`、`pixels[2*i+1]=color&0xFF`。
- 颜色常量用 `ILI9341_RGB565(r,g,b)` 生成，例如 `ILI9341_RGB565(0, 200, 255)`。
- 方向：竖屏 `ILI9341_ROT_PORTRAIT_1`（240×320，默认），横屏 `ILI9341_ROT_LANDSCAPE_1/2`（320×240）。

## 5. 与 LVGL 的衔接

后续接 LVGL 时不需要改本模块的驱动核心：

1. 在 LVGL 的 `flush_cb` 里调用 `ili9341_blit(area->x1, area->y1, w, h, buf)`；
2. LVGL 的 RGB565 输出是小端，`flush_cb` 里需要做一次字节交换（或用 LVGL 的 `LV_COLOR_16_SWAP`）；
3. 若届时改用 SPI1 TX DMA，只改 `ili9341_port.c` 的发送实现（增加 DMA 完成信号量），`ili9341.c` 与 App 层不需要动。
