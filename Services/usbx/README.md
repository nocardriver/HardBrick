# USBX（设备侧）移植说明

本目录是 **Eclipse ThreadX USBX 6.5.2.202603_rel** 的裁剪副本，只保留本工程需要的
设备侧部分。上层用法见 `Apps/usb_msc`。

## 目录来源

| 目录 | 来源 | 说明 |
|------|------|------|
| `common/core/` | `usbx/common/core` | 设备栈 + 工具函数（`ux_host_*.c` / `ux_hcd_*.c` 已排除） |
| `common/usbx_device_classes/` | `usbx/common/usbx_device_classes` | 只保留 MSC（storage）类 |
| `common/usbx_stm32_device_controllers/` | `STM32Cube_FW_U5_V1.9.0/Middlewares/ST/usbx` | ST 的 PCD HAL 胶水层，驱动版本 V1.3.4 |
| `ports/cortex_m33/gnu/inc/` | `usbx/ports/cortex_m33/gnu/inc` | Cortex-M33 端口头文件 |
| `config/` | 本工程编写 | `ux_user.h`、`ux_stm32_config.h` |

升级 USBX 时按上表整目录替换，**保留 `config/`**；`common/usbx_stm32_device_controllers/`
必须与 Cube 包里的驱动一起更新（该驱动依赖 USBX 内部接口，官方要求 USBX ≥ V6.1.10）。

## 关键配置

`config/ux_user.h`：

- `UX_DEVICE_SIDE_ONLY`：只编设备栈。
- `UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT`：STM32 DCD 按方向独立维护端点对象，
  MSC 使用 EP1 IN(0x81) / EP1 OUT(0x01)。
- `UX_SLAVE_REQUEST_DATA_MAX_LENGTH = 512`：与 TF 卡扇区同大小，同时决定 MSC 单次
  SCSI 数据传输长度。
- `UX_THREAD_STACK_SIZE = 2048`：MSC 类线程栈（-O0 下 1 KB 默认值偏紧）。
- `UX_THREAD_PRIORITY_CLASS = 6`：USBX 默认 20，会被 gps(4)/display(5)/backlight(6)
  压住，块传输变慢且不稳定。

`config/ux_stm32_config.h`：

- `UX_DCD_STM32_MAX_ED = 6`：与 CubeMX 里 `dev_endpoints = 6` 保持一致。

## 构建要点

`CMakeLists.txt` 里 `add_library(usbx OBJECT)` **必须是 OBJECT 库，不能是 STATIC**：
`ux_dcd_stm32_callback.c` 只定义 `HAL_PCD_*Callback()`，这些符号在 HAL 里是 `__weak`
空实现，静态库里没人引用它们，链接器只会保留 HAL 的弱符号，结果设备能枚举但永远不
应答控制传输。OBJECT 库保证全部目标文件进链接，没用的再由 `--gc-sections` 丢掉。

## 应用侧还必须做的事

USBX 之外有两处硬件初始化是 HAL 不会自动做、CubeMX 在没选中间件时也不会生成的：

1. **OTG FIFO 分配**（`HAL_PCDEx_SetRxFiFo` / `SetTxFiFo`）——没有 EP1 IN 的 TX FIFO
   时批量 IN 永远发不出去，现象是 Windows"设备代码 10"。位置见
   `Core/Src/usb_otg.c` 的 `USER CODE BEGIN USB_OTG_FS_Init 2`。
2. **NVIC 里的 OTG 中断**要等 DCD 就绪后再开（`_ux_dcd_stm32_initialize()` 之前进来
   的中断会在 DCD 回调里解引用空指针）。

更多细节（时序、坑、诊断）见 `docs/usb-msc-design.md`。

## 与 ThreadX 的关系

USBX 的线程 / 互斥量 / 延时全部走 ThreadX，`UX_PERIODIC_RATE` 默认取
`TX_TIMER_TICKS_PER_SECOND`（本工程 100 Hz）。改 tick 频率时两边自动一致，
不需要额外定义。

## 中断

USB 中断不需要在 `stm32u5xx_it.c` 里加 USBX 的钩子：`OTG_FS_IRQHandler()` 调
`HAL_PCD_IRQHandler()`，而 USBX 的 ST DCD 驱动已经实现了 `HAL_PCD_*Callback()`
（见 `common/usbx_stm32_device_controllers/ux_dcd_stm32_callback.c`）。
