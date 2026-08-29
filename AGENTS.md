# HardBrick 工程规范

本文件是 HardBrick（STM32U575 + ThreadX RTOS + SEGGER RTT）工程**始终生效**的统一编码与架构规范。
在本仓库内由 AI 生成/修改代码时，必须遵守本规范；若与规范冲突，先按规范执行，有疑问再询问。

---

## 1. 代码风格（Code Style）

- **缩进**：2 空格，不用 Tab。
- **语言**：C11（`CMAKE_C_STANDARD 11`，扩展开启）。
- **注释**：以中文为主；文件头用 Doxygen 风格（`@file` / `@brief`）；公开 API 用 `@param` / `@retval` / `@note`；复杂逻辑写「为什么这么做」，不要复述代码。
- **命名**：
  - 函数 / 变量：`snake_case`。
  - 模块级公共符号统一加模块前缀（`uart_dma_*`、`lc760z_*`）。
  - 静态全局变量：`g_` 前缀（如 `g_instances`）。
  - 类型：`typedef struct xxx { ... } xxx_t;`。
  - 宏 / 常量：`UPPER_SNAKE_CASE` + 模块前缀（如 `UART_DMA_MAX_INSTANCES`）。
  - 无符号字面量加 `u` 后缀（`0u`、`1u`）。
  - 硬件相关一律用 stdint 定宽类型（`uint8_t` / `uint16_t` / ...），不用裸 `int` 表示寄存器宽度。
- **头文件**：
  - 用 `#ifndef XXX_H` 保护宏（与 STM32 生态一致），不用 `#pragma once`。
  - 需要被 C++ 引用时加 `extern "C"` 包裹。
- **跨模块数据**：采用「调用者提供实例与缓冲」模式（参照 `uart_dma_t`），模块自身不持有大块静态缓冲，便于多实例。

## 2. 架构分层（Architecture）

- **四层结构**：
  - `Core/`：CubeMX 生成的 HAL 代码，不改生成区。
  - `BSP/`：**底层驱动**（板级外设驱动，如 `uart_dma`、`LC760Z`）。
  - `Services/`：**中间件 / 基础服务**（`threadx`、`SEGGER_RTT`）。
  - `App/`：**应用层**（业务任务模块，如 `gps`）。
- **依赖方向单向**：`Core` ← `BSP` / `Services`，`BSP` / `Services` ← `App`。BSP 模块之间不互相依赖；`App` 可依赖 `BSP`、`Services` 与 `Core`。
- **BSP 模块两种形态**：
  - 通用基础设施（如 `uart_dma`）：与具体器件无关，可复用到任何串口。
  - 器件驱动（如 `LC760Z`）：拆为「驱动核心 `lc760z.c` + 平台移植层 `lc760z_port.c` + 协议解析 `lc760z_nmea.c` / `lc760z_bin.c`」。
- **驱动分层原则**：
  - 驱动核心**不直接调用 HAL / 外设 API**，平台差异（接收回调、发送、GPIO、延时）收敛到 `*_port.c`。
  - 每个 BSP / Services / App 模块一个目录，含 `.c/.h + CMakeLists.txt`；禁止堆巨型单文件。
- **应用层**：业务逻辑按模块放在 `App/`（如 `app_gps`）。`main.c` 的 USER CODE 区只做启动装配（初始化外设、创建线程、启动应用入口），**不堆业务逻辑**。
- **并发模型**：ISR 只写缓冲，消费线程只读；多线程共享数据一律通过缓冲 / 队列 / 信号量，禁止裸全局变量跨线程读写。

## 3. 错误处理与健壮性（Error Handling）

- 用 `bool` / 状态码表达成败，调用方必须检查返回值（尤其 HAL 返回值）。
- 缓冲满策略：环形缓冲写满时**丢弃新数据**（不覆盖未消费数据），并在注释中说明。
- 串口错误（如 ORE 溢出）：在 `HAL_UART_ErrorCallback` 中复位接收（rearm），不 panic。
- ISR 内禁止阻塞 / 延时 / 打印，只做最小工作（拷贝数据 + rearm）。
- 公开 API 入口做必要参数校验，但避免过度防御堆砌，保持可读性。

## 4. 嵌入式 / STM32 约定

- **CubeMX 生成文件**：用户代码只写在 `/* USER CODE BEGIN x */` 与 `/* USER CODE END x */` 之间，**不改生成区**。
- **RTOS**：任务栈用**静态数组分配**（`static uint8_t xxx_stack[N] __attribute__((aligned(8)));`），放 `.bss` 由链接器核算总量，RAM 溢出在链接期即报错；不用 `first_unused_memory` 手动切分（无溢出保护，越界会静默写坏内存）。任务间通信用队列 / 信号量。
- **调试输出**：用 SEGGER RTT（`SEGGER_RTT_printf` 不支持 `%f`，浮点按 1e6 缩放成整数打印）；不占用业务串口打印调试信息。
- **延时**：周期 / 超时用 `tx_thread_sleep`，不用裸 `while` 忙等；上电时序等需要精确延时的按数据手册用 `HAL_Delay`。

## 5. 构建（Build）

- 构建：`cmake --build build/ATfE-Debug`（Ninja + arm-none-eabi-gcc）。
- **新增模块**：在 `BSP/`、`Services/`、`App/` 的子目录加 `CMakeLists.txt`，并把 target 名追加到父目录导出的 `BSP_LIBS` / `SERVICES_LIBS`（`App/` 层建好后导出 `APP_LIBS`，`PARENT_SCOPE`）。
- 遵循 C11，不引入工程外的额外工具链依赖。
