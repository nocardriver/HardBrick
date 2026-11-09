# FatFs — SD 卡文件系统（Services 层）

ChaN FatFs **R0.16**，跑在 `BSP/SDCard` 之上，给应用层提供 `f_open` / `f_read` /
`f_write` / `f_opendir` 这套标准接口。

## 目录里什么是上游、什么是本项目写的

| 文件 | 来源 | 说明 |
|------|------|------|
| `ff.c` / `ff.h` | 上游 | 文件系统核心，**未改动** |
| `ffunicode.c` | 上游 | 码表（LFN 用），**未改动** |
| `diskio.h` | 上游 | 磁盘接口声明，**未改动** |
| `ffconf.h` | 上游 | 只改了 6 个开关，文件头逐条写了原因 |
| `diskio.c` | **本项目** | `disk_*` → `sdcard_*`，块设备到文件系统的桥 |
| `fatfs_port.c` | **本项目** | `FF_FS_REENTRANT` 的互斥量（ThreadX mutex） |

上游的 `diskio.c` 骨架和 `ffsystem.c` 示例没有被使用，也没有编译——它们只是模板。

升级 FatFs 时：覆盖 `ff.c` / `ff.h` / `ffunicode.c` / `diskio.h`，然后重新把
`ffconf.h` 的 6 处改动打回去、对一遍 `FFCONF_DEF` 版本号即可。

## 数据通路

```
Apps/storage  f_open / f_read / f_write
     |
Services/FatFs/ff.c            FAT 解析、簇分配、目录项
     |
Services/FatFs/diskio.c        sector + 缓冲区
     |
BSP/SDCard/sdcard.c            sdcard_read_blocks() / sdcard_write_blocks()
     |
HAL_SD_*_DMA → SDMMC1 IDMA     PC8-PC12 / PD2，4-bit，20 MHz
```

依赖方向单向：`App → Services(FatFs) → BSP(SDCard) → Core(HAL)`。FatFs 层不碰任何
HAL 句柄，`sdcard` 层不认识文件系统，两边可以各自替换。

## 配置（`ffconf.h` 相对上游的 6 处改动）

| 选项 | 上游 → 本项目 | 原因 |
|------|----------------|------|
| `FF_USE_MKFS` | 0 → 1 | 允许板上 `f_mkfs()` 格式化新卡 |
| `FF_CODE_PAGE` | 932 → **936** | 简体中文，Windows 按 CP936 解析 OEM 名 |
| `FF_USE_LFN` | 0 → 2 | 长文件名；缓冲放**调用者栈**上（静态缓冲与 `FF_FS_REENTRANT` 互斥，`ff.c` 里有 `#error`） |
| `FF_LFN_UNICODE` | 0 → 2 | API 侧 UTF-8：源码是 UTF-8，中文路径可以直接写字面量 |
| `FF_FS_NORTC` | 0 → 1 | 板上无 RTC，时间戳固定（`FF_NORTC_YEAR 2026`） |
| `FF_FS_REENTRANT` | 0 → 1 | 多线程共用一个卷，互斥量在 `fatfs_port.c` |

保持上游默认、值得知道的两项：

- `FF_MULTI_PARTITION = 0`：FatFs 自己扫 MBR，把分区起始 LBA 加到扇区号上再调
  `disk_read()`，所以 `diskio.c` 收到的就是整张卡的绝对扇区号，不用自己算偏移。
- `FF_MAX_SS == FF_MIN_SS == 512`：扇区大小固定，`disk_ioctl(GET_SECTOR_SIZE)`
  其实不会被调用（实现了只是为了将来改配置）。

## 用法

```c
#include "storage.h"

/* tx_application_define() 里创建 storage 线程，线程内挂载并自检 */
storage_init();

/* 其它线程要用文件 API 时：拿已挂载的卷，直接用 f_* —— 卷锁是 FatFs 内部取的，
   不需要外面再加互斥量 */
FATFS *fs = storage_volume();
if (fs != NULL) {
  FIL fil;
  if (f_open(&fil, "0:/LOG.TXT", FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
    UINT bw;
    (void)f_write(&fil, buf, len, &bw);
    (void)f_close(&fil);
  }
}
```

## 使用时要注意的几点

### 1. 线程栈要留 512 B 给 LFN

`FF_USE_LFN = 2` 每次解析路径都在**调用者栈**上开 `(FF_MAX_LFN + 1) * 2 = 512 B`。
调用 `f_open` / `f_stat` / `f_readdir` 的线程栈至少 2048，本项目 `storage` 线程给
了 4096（还要装 `FIL` 的 512 B 扇区缓冲和 `FILINFO` 的 256 B 名字）。

### 2. 卡可以热插拔，但要重新挂载

`diskio.c` 在传输失败时把盘打回 `STA_NOINIT`，这样再调一次 `f_mount(&fs, "0:", 1)`
就会重新走 `disk_initialize()` → `sdcard_init()` → `HAL_SD_Init()`，重新识别卡。
应用层收到 `FR_DISK_ERR` 后重新挂载即可，不用重启。

`disk_status()` 用缓存值（`g_sd_stat`），不是每次都发 CMD13——`ff.c` 几乎每个文件
操作都会问一次盘状态，真去问卡会让每个 `f_read` 多几十微秒。

### 3. 时间戳是固定的

没有 RTC，所有对象时间戳都是 2026-01-01 00:00。将来接了 RTC：把
`FF_FS_NORTC` 改回 0，实现 `get_fattime()`（返回值格式见上游 `ff.h` 注释，
`(year-1980) << 25 | mon << 21 | mday << 16 | hour << 11 | min << 5 | sec >> 1`）。

### 4. 如果以后打开 D-Cache

当前 `MX_DCACHE1_Init()` 只配了 `ReadBurstType`，全工程没有调 `HAL_DCACHE_Enable()`，
D-Cache 是**关着**的，所以 `sdcard.c` 的 DMA 缓冲没有做 clean/invalidate。
一旦打开 D-Cache，`sdcard_read_blocks()` 返回前要 `HAL_DCACHE_InvalidateByAddr()`，
`sd_transfer()` 写方向发 DMA 前要 `HAL_DCACHE_CleanByAddr()`，否则会读到旧数据。

### 5. Flash 占用（约 196 KB，其中 176 KB 是中文码表）

| 目标文件 | text |
|----------|------|
| `ff.c` | 20 KB |
| `ffunicode.c`（CP936 码表） | **176 KB** |
| `diskio.c` + `fatfs_port.c` | 1 KB |

觉得码表太大时有两个降级手段，改 `ffconf.h` 一处即可：

- 只支持英文名：`FF_CODE_PAGE` 改 437，码表降到几 KB（非 ASCII 字符会变成 `_`）
- 不要长文件名：`FF_USE_LFN` 改 0，`ffunicode.c` 整个不参与编译，只剩 20 KB，
  代价是文件名只能 8.3（`20261109.CSV` 这种）

当前 `ROM: 299 KB / 2 MB`，还宽裕，所以默认两者都开。

## 验证

上电后 RTT 应该看到（下面是 32 GB FAT32 卡实测的输出，`Apps/storage` 的
`run_selftest()` 打的）：

```
System Boot OK
[SD] stage=6 cd=0 type=1 ver=1 rca=21580 err=0x00000000          <- BSP 探测通过
[SD] 61132800 x 512 B, mbr=1 part=0x0C lba=8192 fat32=1
[SD] bps=512 spc=64 rsv=1464 fats=2 fatsz=7460 label=NO NAME
[LCD] SPI DMA enabled
[LCD] UI thread started
[SD]   <DIR> System Volume Information  0 B                      <- 26 字符长文件名，
[SD]   HARDBRICK.TXT  22 B                                         证明 LFN 读通路正常
[SD] stage=7 rc=0 fs=FAT32
[SD] total=29838 MB free=29837 MB, file=22 B
[SD] fatfs self-test PASS, wrote "boot 0 tick FATFS ok"
```

`stage=7` 是 `STORAGE_STAGE_VERIFY_OK`，含义是"写进去的内容读回来一模一样"。
`HARDBRICK.TXT` 每上电一次追加一行，所以 `file=` 的字节数会逐次变大——想验证
追加语义，重启一次看这个数有没有涨即可。

如果不方便接 RTT，可以用 J-Link 直接读 `g_storage`（结构体定义见
`Apps/storage/storage.h`，魔数 `0x46415458` = `'FATX'`）。

失败时 `stage` 停在哪一步，对应关系：

| stage | 含义 | 先查什么 |
|-------|------|----------|
| 1 | `f_mount` 失败 | `g_sd_probe.stage`：卡/链路有没有问题；`rc`：`FR_NO_FILESYSTEM` 说明卡上没有 FAT 卷，可以 `f_mkfs` |
| 3 | 容量查询完成 | 停在这里说明读写都没开始，但 `f_getfree` 没返回成功（FSINFO 异常） |
| 4 | 写失败 | `rc` + `g_sdcard_last_error`：CRC / 超时 / 写保护 |
| 6 | 读回比对失败 | 写成功但读回不一致：先怀疑缓冲对齐或卡本身 |
