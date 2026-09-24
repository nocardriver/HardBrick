# HaradBrick

STM32U575 项目（STM32CubeMX 生成 + CMake 构建）。

## 目录结构

- `Core/` — 应用代码（STM32CubeMX 生成）
- `Drivers/` — STM32 HAL 驱动
- `cmake/` — 工具链文件与 CubeMX 子工程

## 构建

需要 CMake 3.22+、Ninja，以及以下任一工具链：

| 工具链 | 配置预设 | 说明 |
|--------|----------|------|
| Arm GNU Toolchain（arm-none-eabi-gcc） | `Debug` / `Release` | 需在 PATH 中 |
| Arm Toolchain for Embedded（ATfE，LLVM/clang） | `ATfE-Debug` / `ATfE-Release` | 见下方说明 |

### ATfE 工具链定位

工具链文件 `cmake/atfe-arm-none-eabi.cmake` 按以下优先级查找 ATfE 安装位置：

1. `ATFE_ROOT` 命令行变量：`cmake --preset ATfE-Debug -DATFE_ROOT=<路径>`
2. `ATFE_ROOT` 环境变量：`setx ATFE_ROOT "<路径>"`（Windows，新终端生效）
3. PATH 回退：将 ATfE 的 `bin` 目录加入 PATH

> **注意**：`setx` / 修改系统 PATH 只对**新启动的进程**生效。已打开的 VS Code 及其终端仍使用旧环境，
> 必须**完全退出并重启 VS Code**（不是新建终端）才能识别到新配置。

### 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 配置报错 `Cannot find clang from ATfE` | ATfE 的 `bin` 不在当前进程 PATH，且 `ATFE_ROOT` 未设置 | 完全重启 VS Code；或临时用 `-DATFE_ROOT=<路径>` 配置 |

### 构建命令

```powershell
cmake --preset ATfE-Debug     # 配置（Debug）
cmake --build --preset ATfE-Debug
```

产物位于 `build/ATfE-Debug/`（`HaradBrick.elf`、`HaradBrick.map`）。GCC 预设用法相同。
