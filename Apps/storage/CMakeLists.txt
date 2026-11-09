cmake_minimum_required(VERSION 3.22)

#
# Storage application: 挂载 TF 卡上的 FAT 卷并做一次读写自检 (App layer)
#
#   storage.c   一个 storage 线程：f_mount -> 读容量 -> 追加记录 -> 读回比对
#               -> 列根目录，结果同时写 RTT 和 g_storage（J-Link 可读）
#
# FatFs 本身在 Services/FatFs，这一层只决定"挂哪个卷、什么时候挂、挂了干什么"。
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(STORAGE_App_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(STORAGE_App_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/storage.c
)

# Create storage application object library
add_library(storage OBJECT)
target_sources(storage PRIVATE ${STORAGE_App_Src})
target_include_directories(storage PUBLIC ${STORAGE_App_Inc_Dirs})

# App layer may depend on Services (AGENTS.md): fatfs for the file API, threadx
# for the thread, segger_rtt for the self-test log.
target_link_libraries(storage PUBLIC
    fatfs
    threadx
    segger_rtt
)
