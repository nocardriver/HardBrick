# 屏幕 SPI 发送改 DMA —— 设计研究

> 状态：**已实施**（2026-11-08）。研究结论、实施方案与最终代码一致，§8 记录了已确认的决策。
>
> 对象：`BSP/ILI9341`（KD035QVRMA223 / ILI9341，4-line SPI，RGB565）
> 平台：STM32U575RG + ThreadX + HAL，SPI1 @ 40 MHz，GPDMA1

---

## 0. 结论摘要（TL;DR）

1. **改动面很小**：只有移植层 `ili9341_port.c` 的发送实现换成 `HAL_SPI_Transmit_DMA` +
   ThreadX 信号量阻塞等待；`ili9341.c` 与 `Apps/display` 的调用方式不变。
   这与 `BSP/ILI9341/README.md` 第 5 节的预判一致。
2. **必须同时开 SPI1 全局中断**。U5 的 HAL 在 DMA 传输完成回调里只是打开 EOT 中断，
   真正收尾（关 SPI、清 TXDMAEN、调用用户 `TxCpltCallback`）发生在 SPI 的 EOT 中断里。
   不开这个中断，DMA 传输永远不会"完成"，`hspi1.State` 会卡在 `BUSY_TX`。
3. **不需要 cache 维护**。内部 SRAM 走 Cortex-M33 S-bus 的独立通路，不经 DCACHE1
   （RM0456 Table 1 / §2.1.4）；本驱动的缓冲都在 `.bss`，GPDMA 读到的就是 CPU 写入的数据。
   README 里"改 DMA 时必须 `SCB_CleanDCache_by_Addr()`"的说法对 M33 + U5 不成立，应一并修正。
4. **初始化路径要保留轮询**：`ili9341_init()` 在 `tx_kernel_enter()` 之前执行，此时
   信号量还不存在，无法等待 DMA；且初始化命令都是 1~16 字节，DMA 开销反而更大。
5. **收益要客观看**：当前 UI 最坏一轮刷新 56.9 KB ≈ 11.4 ms/200 ms（约 5.7% CPU），
   改成 DMA 后这部分 CPU 时间被释放，但对现有画面提升有限；真正价值是给 LVGL
   全屏刷新（153.6 KB ≈ 30.7 ms/帧）和动画铺路，并消除显示线程忙等造成的尾延迟。
6. **面板时钟是既有风险**：规格书写周期 tSCYCW ≥ 66 ns → SCL ≤ 15.15 MHz，当前跑 40 MHz
   已超规格。DMA 不改变总线时序，但建议借这次机会复测 20 MHz。

---

## 1. 现状

### 1.1 硬件与 SPI 配置

| 项目 | 值 | 来源 |
|---|---|---|
| 控制器 / 面板 | ILI9341 / KD035QVRMA223，240×320，RGB565 | 规格书 p.4 |
| SPI | SPI1，Master，8 bit，CPOL=0/CPHA=1Edge，MSB first，NSS 软件 | `Core/Src/spi.c` |
| 时钟 | PCLK2=160 MHz，Prescaler=4 → **40 MHz** | `.ioc` |
| FIFO | 16×8 bit，阈值 1 data（`SPI_FIFO_THRESHOLD_01DATA`） | RM0456 Table 696 p.2903 |
| 引脚 | PA5 SCK / PA7 MOSI / PA4 CS(GPIO) / PC4 DC / PB0 RST | `Core/Src/spi.c`、`gpio.c` |
| DMA | USART2 占用 GPDMA1 ch0(RX)/ch1(TX)，SPI1 无 DMA | `usart.c` MspInit、`.ioc` |
| DCACHE1 | 已使能（`MX_DCACHE1_Init`），只服务外部存储通路 | `Core/Src/dcache.c`、RM0456 §2.1.4 |

### 1.2 当前调用链

```
display thread (prio 5)
  └─ ili9341_blit() / ili9341_fill_rect()          BSP/ILI9341/ili9341.c
       └─ ili9341_port_write()                      BSP/ILI9341/ili9341_port.c
            └─ HAL_SPI_Transmit()  ← CPU 在整段传输期间忙等
```

`HAL_SPI_Transmit()` 结束时同样走 `SPI_CloseTransfer()`（SPE=0、清 TXDMAEN），
所以"每笔传输后 SPI 被关闭、下次重新使能"这一行为在 DMA 方案里保持一致，不会引入新的
CS/DC 时序差异（见 `stm32u5xx_hal_spi.c:1046` 与 `:3933`）。

### 1.3 现状耗时（按 40 MHz ≈ 5 MB/s）

| 场景 | 字节数 | 纯 SPI 时间 |
|---|---|---|
| 全屏清屏 240×320 | 153,600 | 30.7 ms |
| 时间字段 240×15 | 7,200 | 1.44 ms |
| 日期字段 240×10 | 4,800 | 0.96 ms |
| 状态条底色 216×40 | 17,280 | 3.46 ms |
| 最坏一轮全字段刷新 | 56,880 | 11.4 ms |

这 11.4 ms 现在是**纯 CPU 忙等**。线程优先级：GPS=4、display=5、backlight=6，
所以 GPS 解析不会被显示阻塞，被阻塞的是 backlight 线程和 idle。

---

## 2. 资料依据（关键结论）

### 2.1 SPI 的 DMA 行为（RM0456）

| 结论 | 位置 |
|---|---|
| 发送时 TXP 置位触发 DMA 请求，DMA 写 `SPI_TXDR` | §68.4.14 p.2925 |
| 启动顺序：RXDMAEN → DMA 通道使能 → TXDMAEN → SPE | §68.4.14 p.2926 |
| `TSIZE` 是硬件数据计数器，归零时置 EOT；建议每笔传输后 disable/enable SPI 复位状态机 | §68.4.12 p.2922–2923 |
| 主模式关闭流程必须先等 EOT/TXC | §68.4.13 p.2924 |
| 主模式 TxFIFO 空 → **自动暂停 SCK**，DMA 补数据后继续（不是 UDR，不丢字节） | §68.4.12 p.2923 |
| `COMM=01` 是 simplex transmitter（HAL 发送时自动切到该模式） | §68.8.4 p.2941 |
| SPI1 FIFO 16×8 bit，阈值 1–16，计数上限 65535 | Table 696 p.2903 |

> `COMM` 这一点很重要：HAL 的 `SPI_2LINES_TX()` 在每次发送时把 SPI 切成"只发"，
> RX FIFO 不参与，因此不会出现"全双工只发不收 → OVR"的错误。现有轮询路径和 DMA 路径一致。

### 2.2 GPDMA 请求映射（RM0456）

| 结论 | 位置 |
|---|---|
| `REQSEL=7` → `spi1_tx_dma` | Table 137 p.687 |
| ch0–11：8 字节 FIFO、连续地址（ch12–15 才是 32 字节 + 2D） | Table 135 p.687 |

本工程 ch0/ch1 已给 USART2，**SPI1_TX 选 ch2** 即可（`GPDMA1_Channel2_IRQn = 31`）。

### 2.3 HAL 的完成链路（必须理解，否则会踩坑）

```
HAL_SPI_Transmit_DMA()                    hal_spi.c:2039
  置 TSIZE=len、TXDMAEN、SPE、CSTART，返回 HAL_OK
        │
GPDMA TC 中断 → SPI_DMATransmitCplt()     hal_spi.c:3486
  ** 只是 __HAL_SPI_ENABLE_IT(SPI_IT_EOT) **   ← 注意：不调用用户回调
        │
SPI EOT 中断 → HAL_SPI_IRQHandler()       hal_spi.c:3099
  清 EOT/TXTF/SUSP → SPI_CloseTransfer()  hal_spi.c:3933（SPE=0、清 TXDMAEN）
  → hspi->State = READY → HAL_SPI_TxCpltCallback(hspi)
```

**⇒ SPI1 全局中断（`SPI1_IRQn`，编号 59）必须使能**，否则上面的最后一步永远不发生。
症状：`HAL_SPI_Transmit_DMA` 返回 `HAL_OK`，但 CS 一直不释放、显示线程卡到超时、
`hspi1.State` 停在 `HAL_SPI_STATE_BUSY_TX`。

### 2.4 缓存一致性：本项目不需要

RM0456 Table 1（p.129）与 §2.1.4（p.132）：Cortex-M33 的 S-bus 分成两条通路——

- 内部 SRAM（SRAM1/2/3/5/6）：直连，零等待，**不经过 DCACHE1**；
- 外部存储（FSMC/HSPI1/OCTOSPI）：经 DCACHE1。

本驱动的 `g_text_buf`（11.5 KB）、`g_fill_chunk`（480 B）都在 `.bss`（0x2000_xxxx），
属于内部 SRAM，所以 GPDMA 读到的就是 CPU 写进去的内容，**不需要 clean/invalidate**。

另外两点需要纠正 README 的旧说法：

- Cortex-M33 没有核心 D-Cache，CMSIS `core_cm33.h` 里也没有 `SCB_CleanDCache_by_Addr()`
  （那是 Cortex-M7 的 API）；U5 的 DCACHE 是独立外设，对应 `HAL_DCACHE_CleanByAddr()`。
- 只有将来把帧缓冲放到外部 PSRAM（OCTOSPI）时，才需要"DMA 读前 clean、DMA 写后 invalidate"。

### 2.5 面板 4-line SPI 时序（规格书 p.18 §6.3）

| 参数 | 最小值 | 含义 |
|---|---|---|
| tSCYCW | 66 ns | 写周期 → **f_SCL ≤ 15.15 MHz** |
| tSHW / tSLW | 15 ns | SCL 高/低电平宽度 |
| tCSS / tCSH | 15 ns | CS 建立/保持 |
| tDCS / tDCH | 10 ns | D/C 建立/保持 |

当前 40 MHz 约是规格上限的 2.6 倍（Brick 例程用 10 MHz 验证过）。这与是否用 DMA 无关，
但改造后建议做一次降频对比测试。面板 FPC 没有 TE 引脚（p.7–8），无法做撕裂同步，本阶段也不需要。

---

## 3. 方案对比

| 方案 | 做法 | 优点 | 缺点 | 结论 |
|---|---|---|---|---|
| **A. 同步 API + DMA + 信号量** | `port_write()` 内部启动 DMA，`tx_semaphore_get` 等到 EOT 再返回 | 调用方零改动；CS/DC 时序完全不变；错误路径可控 | 线程仍要等整段传输（但会让出 CPU） | ✅ 推荐 |
| B. 异步 + 双缓冲 | 驱动立即返回，完成回调里拉 CS，上层边发边渲染 | 极限吞吐更高 | 缓冲所有权、双缓冲、CS 生命周期都要重做；当前 UI 收益很小 | 暂不做 |
| C. 小包轮询 + 大包 DMA | 即方案 A 的一部分：命令走轮询，像素块走 DMA | 兼顾开销与简单 | —— | ✅ 包含在 A 中 |

**选择 A**：`ILI9341_*` 的公开 API 与语义保持不变，`ili9341.c` 一行都不用改（连 chunk 都不用改），
风险最低，也符合 AGENTS.md"平台差异收敛到 `*_port.c`"的分层要求。

---

## 4. 详细设计

### 4.1 改动清单

| 文件 | 改动 | 是否 CubeMX 生成区 |
|---|---|---|
| `HaradBrick.ioc` | SPI1 增加 TX DMA → GPDMA1 ch2；使能 SPI1 与 GPDMA1 ch2 中断 | 配置源 |
| `Core/Src/spi.c` | `HAL_SPI_MspInit` 增加 DMA 初始化 + `__HAL_LINKDMA` + NVIC | 生成区 |
| `Core/Src/gpdma.c` | 增加 `GPDMA1_Channel2_IRQn` 使能 | 生成区 |
| `Core/Src/stm32u5xx_it.c` | `SPI1_IRQHandler`、`GPDMA1_Channel2_IRQHandler` | 生成区 |
| `Core/Src/main.c` | `tx_application_define()` 开头调用 `ili9341_rtos_attach()` | USER CODE |
| `BSP/ILI9341/ili9341_port.c` | DMA 发送、完成/错误回调、信号量、超时与 abort | 手写 |
| `BSP/ILI9341/ili9341_port.h` | 新增 `ili9341_port_dma_attach()` / 错误计数 | 手写 |
| `BSP/ILI9341/ili9341.c/.h` | 新增 `ili9341_rtos_attach()` 薄封装（core 仍不依赖 RTOS） | 手写 |
| `BSP/ILI9341/CMakeLists.txt` | 链接 `threadx`（拿 `tx_api.h`） | 手写 |
| `BSP/ILI9341/README.md` | 更新 DMA / cache 说明 | 文档 |
| `Apps/display/*` | 仅更新注释（发送方式说明），无行为改动 |

### 4.2 数据流

```
display thread                     ili9341_port.c                  SPI1 / GPDMA1 / NVIC
    │                                   │
    ├─ ili9341_blit() ────────────────► │
    │                            len≥64?│
    │                            ┌──────┴───────┐
    │                        否  │              │ 是
    │                            ▼              ▼
    │                     HAL_SPI_Transmit  HAL_SPI_Transmit_DMA
    │                     （命令/小包）      ├─ tx_semaphore_get()  ← 线程睡眠
    │                                       │        │
    │                                       │   GPDMA TC IRQ → 打开 EOT 中断
    │                                       │   SPI EOT IRQ  → CloseTransfer
    │                                       │                → TxCpltCallback
    │                                       │                → tx_semaphore_put()
    │◄──────────────── 返回 true ───────────┴─ 唤醒
    │
    └─ 继续下一个 chunk / 拉高 CS
```

关键点：`port_write()` 返回时 DMA 已经 EOT 完成、SPI 已被 HAL 关闭，所以
`ili9341.c` 里"整段 fill/blit 期间 CS 保持低、结束后拉高"的时序完全不变。

### 4.3 触发条件与回退

```c
if (g_dma_ready && (len >= ILI9341_PORT_DMA_MIN_BYTES)) {
    return write_dma(data, len);      /* 大块数据 */
}
return write_polling(data, len);      /* 命令、参数、初始化、以及 DMA 未就绪 */
```

- `g_dma_ready` 只在 `ili9341_port_dma_attach()` 成功后置位；
- attach 在 `tx_application_define()` 里调用（此时内核对象可以创建，线程还没跑）；
- 如果 CubeMX 没配 DMA（`hspi1.hdmatx == NULL`）或信号量创建失败，attach 返回 false，
  驱动永久走轮询 —— 行为与今天完全一致，不会变砖。

### 4.4 关键代码骨架（`ili9341_port.c`）

> 下面是研究阶段的骨架；实际落地版本在超时分支额外做了 EOT 复查（收尾中断没到但数据已发完时
> 按成功处理）并累计异常计数，最终规则见 §9。其余结构一致。

```c
#include "tx_api.h"

/** 小于该长度的写走轮询：命令/参数只有 1~16 字节，DMA 的两次中断开销不划算 */
#define ILI9341_PORT_DMA_MIN_BYTES      64u
/** 单笔超时基数（ms），与轮询路径保持一致 */
#define ILI9341_PORT_TIMEOUT_BASE_MS    10u

static TX_SEMAPHORE g_dma_sem;      /**< EOT 完成信号，ISR 里 put */
static volatile bool g_dma_error;   /**< 本次传输是否出错 */
static bool          g_dma_ready;   /**< 信号量已创建、可走 DMA */
static uint32_t      g_dma_errors;  /**< 累计错误数，供 RTT 打印 */

/** 毫秒 → ThreadX tick（100 Hz，向上取整） */
static ULONG timeout_ticks(uint16_t len)
{
  uint32_t ms = ILI9341_PORT_TIMEOUT_BASE_MS + (uint32_t)len / 1000u;
  return (ULONG)((ms * TX_TIMER_TICKS_PER_SECOND + 999u) / 1000u);
}

/** 清掉可能残留的令牌（超时 abort 后 ISR 仍可能补发一个） */
static void dma_drain(void)
{
  while (tx_semaphore_get(&g_dma_sem, TX_NO_WAIT) == TX_SUCCESS) {
  }
}

bool ili9341_port_dma_attach(void)
{
  if (g_dma_ready) {
    return true;
  }
  if (hspi1.hdmatx == NULL) {          /* CubeMX 未配置 SPI1_TX DMA */
    return false;
  }
  if (tx_semaphore_create(&g_dma_sem, "lcd dma", 0u) != TX_SUCCESS) {
    return false;
  }
  g_dma_ready = true;
  return true;
}

static bool write_dma(const uint8_t *data, uint16_t len)
{
  dma_drain();
  g_dma_error = false;

  if (HAL_SPI_Transmit_DMA(&hspi1, data, len) != HAL_OK) {
    g_dma_errors++;
    return false;
  }
  if (tx_semaphore_get(&g_dma_sem, timeout_ticks(len)) != TX_SUCCESS) {
    /* EOT 没来：把总线收回来，否则后面所有传输都会卡死 */
    (void)HAL_SPI_Abort(&hspi1);
    dma_drain();
    g_dma_errors++;
    return false;
  }
  if (g_dma_error) {
    g_dma_errors++;
    return false;
  }
  return true;
}

bool ili9341_port_write(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0u)) {
    return false;
  }
  if (g_dma_ready && (len >= ILI9341_PORT_DMA_MIN_BYTES)) {
    return write_dma(data, len);
  }
  return write_polling(data, len);     /* 现有的 HAL_SPI_Transmit 实现 */
}

/* ---- HAL 回调：EOT 收尾后由 SPI 中断调用 ---- */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI1) {
    return;
  }
  (void)tx_semaphore_put(&g_dma_sem);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI1) {
    return;
  }
  g_dma_error = true;
  (void)tx_semaphore_put(&g_dma_sem);
}
```

`ili9341.c` 里只加一个转发函数（保持驱动核心不含 RTOS）：

```c
bool ili9341_rtos_attach(void)
{
  return ili9341_port_dma_attach();
}
```

`main.c` 的 `tx_application_define()` 开头：

```c
if (!ili9341_rtos_attach()) {
  SEGGER_RTT_WriteString(0, "[LCD] SPI DMA unavailable, polling mode\r\n");
}
```

### 4.5 CubeMX 配置清单

| 位置 | 设置 |
|---|---|
| `Connectivity → SPI1 → Parameter Settings` | 保持现状：Master / 8 bit / CPOL Low / CPHA 1Edge / NSS Software / Prescaler 4（40 MHz）/ FIFO threshold 1 data |
| `SPI1 → DMA Settings` | Add → `SPI1_TX`；GPDMA1 **Channel 2**；Direction Memory→Peripheral；Mode Normal；Priority Low；**Source 地址自增 ✓ / Destination 自增 ✗**；Data Width Byte/Byte；Burst 1 |
| `SPI1 → NVIC Settings` | ✅ **SPI1 global interrupt**（关键）；优先级建议 5~10 |
| `GPDMA1 → NVIC` | ✅ `GPDMA1 Channel 2 global interrupt` |
| 重新生成后检查 | `spi.c` 里出现 `handle_GPDMA1_Channel2` + `__HAL_LINKDMA(hspi, hdmatx, ...)`；`stm32u5xx_it.c` 里出现 `SPI1_IRQHandler` 与 `GPDMA1_Channel2_IRQHandler` |

> 生成代码会落在 `HAL_SPI_MspInit()` 里（与 `usart.c` 的 USART2 完全同构），
> 属于生成区，不需要手改。若不想重新生成，也可以只把这几段抄进
> `/* USER CODE BEGIN SPI1_MspInit 1 */`，但后续再生成容易漏，**不推荐**。

### 4.6 超时、错误与恢复

| 情况 | 处理 |
|---|---|
| EOT 超时（中断没开 / DMA 卡住） | `HAL_SPI_Abort()` → 清残留令牌 → 返回 false，上层停画并打日志 |
| UDR（发送下溢） | **主模式基本不会出现**：TxFIFO 空时硬件自动暂停 SCK，DMA 补上数据后继续；UDR 主要发生在从模式。回调仍保留兜底 |
| OVR/MODF/FRE | 同上 |
| `HAL_SPI_Transmit_DMA` 返回 `HAL_BUSY` | 说明上一笔状态未清；建议 `tx_thread_sleep(1)` 后重试一次 |
| 中断里调 `tx_semaphore_put` | ThreadX 允许在 ISR 里调用，安全 |

### 4.7 分块策略

- `ili9341_blit`：4 KB/块（0.82 ms/块），DMA 开销 <2%，**保持**。
- `ili9341_fill_rect`：480 B/块（96 µs/块），每块一次 DMA 往返，开销约 10–20%。
  全屏填充 320 块 → 多 3~4 ms；但填充只用于初始化清屏和状态条（8640 px ≈ 36 块），
  实际影响可忽略。若后续介意，把 `ILI9341_FILL_CHUNK_PIXELS` 调到 1024（2 KB）即可，
  属于可选优化，不影响本次方案成立。
- 单笔 DMA 上限 65535 B（GPDMA BNDT 16 位），现有分片已满足。

### 4.8 时钟取舍

| SCL | 全屏 153.6 KB | 是否符合面板规格 | 说明 |
|---|---|---|---|
| 40 MHz（现状） | 30.7 ms | ❌ 超 2.6 倍 | 与改造前可比，先验证功能 |
| 20 MHz | 61.4 ms | ❌ 仍超 | 折中，建议做长稳测试 |
| 10 MHz | 122.9 ms | ✅ 规格内 | 最稳，但全屏刷新变慢 4 倍 |

建议：**本次先保持 40 MHz**（只换传输机制，变量最少），功能验证通过后单独做一轮降频对比。

---

## 5. 风险清单

| # | 现象 | 原因 | 对策 |
|---|---|---|---|
| 1 | DMA 永不完成，显示卡死 | 忘记使能 `SPI1_IRQn` | 本文 §2.3；CubeMX 勾选；attach 时可检查 NVIC 使能位 |
| 2 | 花屏 / 错位 / 颜色异常 | 40 MHz 超面板规格（与 DMA 无关）；DMA 供给不足只会让 SCK 暂停变慢，不丢数据 | 降到 20/10 MHz 复测；对比改造前后波形 |
| 3 | 偶发 CS 不释放 | 超时后未 abort | `HAL_SPI_Abort()` + 令牌清理 |
| 4 | 第二次传输立刻返回、数据错位 | 超时后残留信号量令牌 | 起始 `dma_drain()` |
| 5 | 初始化阶段卡死 | 内核未启动就等信号量 | 初始化走轮询；attach 在 `tx_application_define()` |
| 6 | 与 USART2 DMA 抢通道 | 误用 ch0/ch1 | 固定用 ch2，并在生成后核对 `GPDMA1_Channel2` |
| 7 | 以后换 PSRAM 帧缓冲后花屏 | 外部存储经 DCACHE1 | 届时 DMA 前 `HAL_DCACHE_CleanByAddr()` |
| 8 | 错误后永久停画（现状行为） | `g_ready=false` 后所有绘制变空操作 | 给超时/`HAL_BUSY` 增加 1 次重试；或改为计数+告警 |

---

## 6. 验证计划

1. **逻辑分析仪 / 示波器**：抓 CS、SCK、MOSI、DC
   - 最后一个 bit 的上升沿之后 CS 才拉高（tCSH ≥ 15 ns，实际远大于）；
   - D/C 在 CS 拉低前或第一个时钟前建立（现有代码已满足）；
   - 对比改造前后 SCK 波形：时钟频率不变，只是不再有 CPU 造成的字节间空隙。
2. **RTT 统计**：每 10 s 打印累计传输次数、字节数、`g_dma_errors`；
   正常运行应为 0，且字节数与画面刷新量吻合。
3. **性能**：用 GPIO 翻转包住 `ili9341_blit()`，测显示线程占用时间；
   预期最坏一轮从 ~11.4 ms 降到 <1 ms（不等价于总线时间，总线仍是 11.4 ms）。
4. **压力**：连续 30 min 每 200 ms 全字段刷新 + 旋转 + 重复初始化；UART 并发不丢帧。
5. **回滚**：编译开关 `ILI9341_PORT_DMA_ENABLE`，或直接不调用 `ili9341_rtos_attach()`。

---

## 7. 实施步骤（估时）

| 步骤 | 内容 | 估时 |
|---|---|---|
| 1 | CubeMX 加 SPI1_TX DMA + 两个 NVIC；重新生成；编译通过 | 0.5 h |
| 2 | `ili9341_port.c/.h`：DMA 路径、回调、超时/abort、计数 | 1.5 h |
| 3 | `ili9341.c/.h` 薄封装 + `main.c` 调用 + CMake 链接 threadx | 0.5 h |
| 4 | 示波器/RTT 验证 + 参数微调 | 1–2 h |
| 5 | README / 文档更新 | 0.5 h |

合计约半天，代码增量约 150 行（不含生成代码）。

---

## 8. 已确认的决策

1. **时钟**：保持 40 MHz（实测可用，不在本次改动范围）。DMA 不改变总线时序。
2. **API 语义**：同步 —— `port_write()` 内部起 DMA 并等 EOT，调用方零改动。
3. **配置方式**：CubeMX 配置并重新生成（已生成 `GPDMA1 Channel 2 = SPI1_TX`、
   `SPI1_IRQn` 与 `GPDMA1_Channel2_IRQn` 均为优先级 10）。
4. **错误策略**：自动重试。落地时的具体规则：
   - `HAL_SPI_Transmit_DMA` 启动失败（`HAL_BUSY` 等，一个字节都没发出）→ 最多重试 2 次；
   - EOT 超时 → 先查 EOT 标志：硬件其实已发完的（收尾中断没到）按成功处理并计入异常；
   - 可能已发出部分数据的失败 → **不重发**，避免像素流整体后移造成更严重的花屏；
   - 连续 3 次靠 EOT 复查救回 → 判定收尾中断不可用，自动**永久回退轮询**。

---

## 9. 实施记录

| 文件 | 实际改动 |
|---|---|
| `HaradBrick.ioc` / `Core/Src/spi.c` / `gpdma.c` / `stm32u5xx_it.c` | CubeMX 生成：GPDMA1 ch2 = SPI1_TX、`__HAL_LINKDMA`、SPI1 与 ch2 中断 |
| `BSP/ILI9341/ili9341_port.c` | 轮询/DMA 双路径、EOT 信号量、超时 abort、自动重试、异常计数 |
| `BSP/ILI9341/ili9341_port.h` | `ili9341_port_dma_attach()` / `_dma_error_count()` / `_dma_retry_count()` |
| `BSP/ILI9341/ili9341.h/.c` | `ili9341_rtos_attach()` 薄封装（驱动核心仍不含 RTOS） |
| `Core/Src/main.c` | `tx_application_define()` 开头调用 `ili9341_rtos_attach()` |
| `BSP/ILI9341/CMakeLists.txt` | 链接 `threadx` |
| `BSP/ILI9341/README.md` | 发送方式、cache 结论、错误策略、超频说明 |

运行时统计：`ili9341_port_dma_error_count()` / `ili9341_port_dma_retry_count()`，
可在 RTT 或调试器里观察；正常应为 0（异常计数含"已自动恢复"的情况，用于暴露配置问题）。

### 9.1 上板调试：纯色填充变彩色横条（根因与修复）

**现象**：DMA 改造后文字形状/位置正常，但背景纯色填充变成彩色横向条纹，肉眼可见；
把 SPI 从 40 MHz 降到 10 MHz 后**毫无变化**。

**证据**（J-Link 直读 + 照片定量分析）：

- `g_dma_ready=1`、`g_dma_errors/retries/recover=0`、`hspi1.State=READY`、`ErrorCode=0`，
  `TSIZE` 与最后一笔缓冲区吻合 —— DMA/SPI 软件链路完全正常，不是丢传输；
- `g_fill_chunk` 内容正确（如 `29 C9 29 C9…`），即填充源数据没问题；
- 照片逐行采样：行内颜色沿 x **缓慢漂移**、行间跳变，颜色呈约 5 行周期的循环；
  这是 16 位像素**位相位持续滑移**的典型特征（每 16 bit 色相转一圈）；
- 时钟降到 10 MHz 无任何改善（GPS 时间在走，时间字段每秒确实重绘）—— 排除采样速率。

**根因**：`SPI_CFG2.AFCNTR=0`（`hspi1.Init.MasterKeepIOState = DISABLE`）。
HAL 在每笔传输的 EOT 中断里执行 `SPI_CloseTransfer()` → `SPE=0`；AFCNTR=0 时
SPI **放开 SCK/MOSI 的 AF 输出**，引脚悬空。悬空 SCK 上的毛刺被面板当成一个时钟沿，
每笔传输让面板的位计数多走 1 bit，像素的高/低字节边界逐行漂移 → 纯色被"旋转"成彩虹。
RM0456 §68.8.4 对 AFCNTR 的说明就是"disable 时防止输出毛刺"。
这也解释了为什么与时钟无关、为什么轮询（间隙更短、悬空时间更短）不明显。

**修复**：`AFCNTR=1`（CubeMX：`SPI1 → Master Keep IO State = Enable`）。

- `Core/Src/spi.c` 的 `USER CODE BEGIN SPI1_Init 2`：`SET_BIT(hspi1.Instance->CFG2, SPI_CFG2_AFCNTR);`
- `.ioc`：`SPI1.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE`（重新生成后保持一致）

**验证**：运行中单独置 AFCNTR=1 后，重绘区域立即恢复正常；重新编译烧录后
`CFG1=0x10070007`（40 MHz）、`CFG2=0xC4420000`（AFCNTR=1）、DMA 计数为 0，整屏正常。

> 如果以后有人把 `MasterKeepIOState` 改回 DISABLE，这个花屏会立刻复现。

---

## 附录：资料页码索引

| 内容 | 出处 |
|---|---|
| S-bus 与 DCACHE 的分工 | RM0456 Table 1 p.129、§2.1.3/§2.1.4 p.132 |
| DCACHE 只服务外部存储 | RM0456 §9.4 p.384 |
| GPDMA 通道能力 / 请求映射 | RM0456 Table 135/137 p.687 |
| SPI FIFO 与数据计数 | RM0456 Table 696 p.2903 |
| TSIZE / EOT / 状态机复位 | RM0456 §68.4.12 p.2922–2923 |
| SPI 关闭流程 | RM0456 §68.4.13 p.2924 |
| SPI DMA 启动/关闭顺序 | RM0456 §68.4.14 p.2925–2926 |
| COMM 通信模式编码 | RM0456 §68.8.4 p.2941 |
| HAL DMA 发送与 EOT 收尾 | `stm32u5xx_hal_spi.c:2039 / 3099 / 3486 / 3933` |
| 4-line SPI 时序 | 规格书 §6.3 p.18 |
| 面板引脚（无 TE） | 规格书 §3 p.7–8 |
