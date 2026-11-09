# USB MSC（TF 卡当 U 盘）实现说明

把板上的 TF 卡通过 USB 暴露给 PC，PC 上出现一个可读写的可移动磁盘；同时解决调试阶段
频繁插拔 USB 线的问题。本文记录**怎么搭的、为什么这么搭、踩过哪些坑**。

## 1. 总体结构

```
┌──────────────────── Apps/usb_msc ────────────────────┐
│ usb_msc.c            USBX 设备栈启动 / 接入 / 重新枚举线程 │
│ usb_msc_descriptors.c 设备 + 配置 + 字符串描述符（全速）   │
│ usb_msc_media.c      SCSI 回调 → 块设备、媒体互斥量、主机门控 │
└───────────┬──────────────────────────────┬───────────┘
            │                              │
   ┌────────▼────────┐            ┌────────▼────────┐
   │ Services/usbx   │            │ BSP/USBConnect  │
   │ USBX 6.5.2 设备栈│            │ D+ (PA12) 拉低   │
   │ + MSC 类 + ST DCD│            │ 模拟拔出/插入     │
   └────────┬────────┘            └─────────────────┘
            │
   ┌────────▼────────┐   ┌──────────────────┐   ┌──────────────┐
   │ Services/threadx│   │ BSP/SDCard       │   │ Apps/storage │
   │ 线程/互斥量/节拍 │   │ SDMMC1 + IDMA    │   │ FatFs 自检    │
   └─────────────────┘   └──────────────────┘   └──────────────┘
```

数据通路（一次 SCSI 读）：

```
PC → OTG_FS EP1 OUT(CBW) → USBX MSC 类线程 → media_read()
   → 媒体互斥量 → sdcard_read_blocks()（SDMMC + IDMA）
   → USBX 传输缓冲 → EP1 IN(数据 + CSW) → PC
```

USBX 内存（设备栈、MSC 类实例、类线程栈、端点缓冲）来自 `usb_msc.c` 里 16 KB 的静态
内存池；任务栈同样是静态数组，RAM 由链接器核算（当前整个固件 57 KB / 768 KB）。

## 2. 启动时序

| 阶段 | 位置 | 动作 |
|------|------|------|
| 1 | `main()` 外设初始化 | CubeMX：`MX_USB_OTG_FS_PCD_Init()` + **FIFO 分配**（见 §5） |
| 2 | `main()` USER CODE 2 第一句 | `usb_msc_boot_detach()`：关 OTG 中断 → `HAL_PCD_Stop()`（SDIS=1）→ D+ 拉低 |
| 3 | 内核启动前 | 屏幕 / GPS 等初始化（约 1.5 s，这段时间主机一直认为"设备已拔出"） |
| 4 | `tx_application_define()` | `usb_msc_init()`：建媒体互斥量、事件组、`usb_msc` 线程；**必须早于 `storage_init()`** |
| 5 | `usb_msc` 线程 | `ux_system_initialize` → `ux_device_stack_initialize` → 注册 MSC 类 → `ux_dcd_stm32_initialize` |
| 6 | 同上，末尾 | 再等 `USB_MSC_ATTACH_DELAY_MS`(300 ms) → D+ 交还 AF10 → `HAL_PCD_Start()` → 开 OTG 中断 |

之所以要等到第 6 步才接入：主机判定"断开"只需要几微秒，但 Windows 重新枚举前有去抖，
D+ 保持低电平几百毫秒最稳；同时保证 USBX 已经把控制端点、描述符都准备好，主机一问就能答。

### 运行中的重新枚举

`usb_msc_reconnect()` 置事件标志，`usb_msc` 线程收到后执行同一套动作：
`ux_device_stack_disconnect()` → `HAL_PCD_Stop()` → D+ 拉低 → 保持 300 ms → D+ 交还 →
`HAL_PCD_Start()`。改了描述符 / 换了卡容量之后，不用重新上电就能让 PC 重新识别。

## 3. 用 GPIO 模拟拔出的细节

- 拉低的是 **D+（PA12）**，OTG_FS 的 DP 固定在这个引脚上（AF10）。
- 前提是 **OTG 内核必须先软断开**（`HAL_PCD_Stop()` 置 `DCTL.SDIS=1`），此时 PHY 不再
  驱动总线，把引脚切成推挽输出低电平不会和 PHY 打架。`usb_connect.c` 不管这个顺序，
  由 `Apps/usb_msc` 保证「先停 PCD 再拉低」。
- 板子上如果有对 D+ 的硬上拉，靠 `SDIS` 是断不开的（上拉电阻一直把 D+ 顶在高电平），
  只有把 D+ 主动拉低才能让主机看到 SE0 断开——这正是本方案的价值。
- 接入时相反：先把引脚恢复成 AF10，再 `HAL_PCD_Start()` 打开内部上拉，最后开 NVIC。
  顺序反了会在上拉建立前就进中断。

## 4. 与固件自己 FatFs 的共存

同一张卡不能同时被 PC 和固件改 FAT，所以做了两层保护：

1. **主机门控**（粗粒度）：MSC 类激活回调把 `g_host_active` 置位，`Apps/storage` 在动
   文件系统前用 `usb_msc_host_active()` 检查，PC 占用期间只打印一行日志并等它释放。
2. **媒体互斥量**（细粒度）：`usb_msc_media_lock()` / `usb_msc_media_unlock()` 保护
   **每一条** SD 命令（含 `sdcard_is_ready()` 里的 CMD13）。USB 侧每次块传输拿一次，
   storage 侧自检时拿一次；ThreadX 互斥量对持有者可重入，所以 MSC 回调里嵌套调用也安全。

PC 侧改动过 FAT 之后，固件本地的 `FATFS` 缓存会失效，重新访问前要
`storage_resync()`（内部 `f_mount(&g_fs, "0:", 1)` 强制重新挂载）。当前 storage 线程
自检完就挂起，这个接口留给后续"PC 断开后继续写日志"之类的功能。

## 5. 关键坑：OTG FIFO 必须由应用分配

`HAL_PCD_Init()` 里的 `USB_DevInit()` 会把 `DIEPTXF0..n` **全部清零**，HAL 不会自动给
各端点分配 TX FIFO。CubeMX 只有在勾选了 USB 中间件时才会生成分配代码；本工程是手工
接入 USBX 的，所以分配写在 `Core/Src/usb_otg.c` 的 `USER CODE BEGIN USB_OTG_FS_Init 2`：

```c
HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);        /* 128 字 RX */
HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 64 / 4);   /* EP0 IN：16 字 */
HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 64 / 2);   /* EP1 IN：32 字 */
```

漏掉 EP1 那一行的现象非常迷惑：

- 设备能正常枚举（EP0 有默认 FIFO），Windows 里能看到 "USB Mass Storage Device"；
- 但设备代码 10（无法启动），`USBSTOR\Disk` 永远不出现；
- 因为批量 IN 没有 FIFO，INQUIRY 的数据阶段根本发不出去，USBX 每次等 50 s 超时，
  Windows 重试两轮（约 2 分钟）后放弃。

排查依据：`hpcd` 里 `IN_ep[1].maxpacket == 0`、`DIEPTXF[0] == 0`。
**单位是 32 位字，不是字节；顺序必须是 RX → EP0 → EP1**（`HAL_PCDEx_SetTxFiFo()`
用前几级的起始地址累加偏移）。

## 6. 描述符与可配置点

| 想改什么 | 改哪里 |
|----------|--------|
| VID / PID / 厂商 / 型号 / 序列号 | `Apps/usb_msc/usb_msc.h` 的 `USB_MSC_*` 宏 |
| 端点号 / 包长 | 同上 `USB_MSC_EPIN_ADDR` / `USB_MSC_EPOUT_ADDR` / `USB_MSC_EP_MPS` |
| 只读 U 盘 | `usb_msc_media.c` 里 `media_read_only_flag` 改成 `UX_TRUE` |
| 当固定磁盘上报（Windows 允许写缓存） | `MEDIA_REMOVABLE_FLAG` 改成 `0` |
| 重新枚举保持时间 | `USB_MSC_ATTACH_DELAY_MS` |
| USB 类线程优先级 | `Services/usbx/config/ux_user.h` 的 `UX_THREAD_PRIORITY_CLASS` |

描述符是手写的静态数组（设备 18 B + 配置 32 B 连续存放），没有用 ST 例程那套 builder
函数：单配置单接口双端点，直接写出来更容易核对。字符串表按 USBX 的格式
（`lang_lo, lang_hi, index, len, chars...`）在首次调用时生成。

## 7. 诊断

- **RTT**：`[USB] usb_msc thread started` / `MSC device attached, last LBA ...` /
  `host configured, TF card is now owned by the PC` / `PC released the card` /
  `re-enumerating (simulated unplug/replug)`。
- **J-Link 直接读内存**：`g_usb_msc`（`usb_msc_status_t`），字段含义见 `usb_msc.h`：

| 字段 | 正常值 / 含义 |
|------|----------------|
| `magic` | `0x4D534355`，说明模块跑过 |
| `stage` | 5 = `USB_MSC_STAGE_READY`；<5 表示 USBX 初始化卡在某一步 |
| `last_lba` | 卡容量减一（32 GB 卡 = 61132799） |
| `configure_count` | 主机 SET_CONFIGURATION 次数；0 说明 PC 没枚举成功 |
| `read_count` / `write_count` | PC 实际读写的扇区数 |
| `read_errors` / `write_errors` | 块传输失败次数 |
| `media_busy` | 抢媒体锁超时次数（正常应为 0） |

## 8. 已知限制

- **OTG_FS 只有全速（12 Mbps）**，实测顺序读约 700 KB/s～1 MB/s，拷大文件偏慢。
  U5 没有 OTG_HS，这是硬件上限。
- **PC 占用期间固件不碰卡**；如果卡是 PC 枚举之后才插上的，容量不会自动刷新到 Windows，
  需要调用一次 `usb_msc_reconnect()`（或在 PC 上"安全删除"后重新枚举）。
- 只支持 Bulk-Only Transport，没有 UASP。
- 卡在 PC 上被格式化成其他文件系统后，固件侧 `storage` 自检会失败（FatFs 只认 FAT）。

## 9. 验证记录（2026-11-09，实机）

- 硬件：STM32U575RGT6 + 32 GB TF（FAT32，MBR，分区起始 LBA 8192）。
- Windows 11 识别为 `USBSTOR\Disk&Ven_HardBric&Prod_TF_Card_Storage&Rev_0100`，
  磁盘 2 / 卷 E:，29.1 GB，DriveType = Removable。
- 目录列举、复制文件进卡、读回内容均正常；期间 `read_count` 7674、`write_count` 40、
  错误计数 0。
- 用调试器复位（等价于软件复位）后，D+ 保持低电平约 1.5 s，Windows 自动重新枚举，
  盘符恢复，无需拔插 USB 线；固件同时在 `HARDBRICK.TXT` 末尾追加一行启动记录，
  PC 侧可见（198 B → 242 B）。
