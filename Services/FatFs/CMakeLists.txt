cmake_minimum_required(VERSION 3.22)

#
# FatFs module: FatFs R0.16 文件系统 (Services layer)
#
# 目录里分成"上游原样"和"本项目移植"两类，升级 FatFs 时只覆盖上游那部分：
#
#   ff.c / ff.h / ffunicode.c   上游 ChaN FatFs R0.16，未改动
#   ffconf.h                    上游配置，只改了 6 处，文件头逐条记录了原因
#   diskio.h                    上游磁盘接口声明，未改动
#
#   diskio.c                    移植层：disk_* 接口 -> BSP/SDCard 的裸块读写
#   fatfs_port.c                移植层：FF_FS_REENTRANT 的互斥量（ThreadX mutex）
#
# 上游的 diskio.c 骨架和 ffsystem.c 示例没有被使用，这里不编译它们。
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(FATFS_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(FATFS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/ff.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ffunicode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/diskio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/fatfs_port.c
)

# Create fatfs object library
add_library(fatfs OBJECT)
target_sources(fatfs PRIVATE ${FATFS_Src})
target_include_directories(fatfs PUBLIC ${FATFS_Inc_Dirs})

# 依赖方向（AGENTS.md）：文件系统层在上面，块设备在下面。
#   sdcard    只经由 sdcard_*() 访问硬件，fatfs 不碰任何 HAL 句柄
#   threadx   只被 fatfs_port.c 用来实现卷互斥量
target_link_libraries(fatfs PUBLIC
    sdcard
    threadx
)

# 上游 ff.c / ffunicode.c 是第三方源码，-Wall 下会有一些风格类告警；只对这两个
# 文件关告警，本项目自己的 diskio.c / fatfs_port.c 仍然按全工程告警级别编译。
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/ff.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ffunicode.c
    PROPERTIES COMPILE_OPTIONS "-w"
)
