cmake_minimum_required(VERSION 3.22)

#
# ParamStore module: 小参数持久化（BSP 层）
#
#   param_store.c  8 KB Flash 页内的日志式记录：追加写、页满才擦、CRC 校验、
#                  擦除前检查页内异物。只保存几字节的配置（如罗盘零偏）。
#
# Sources are listed explicitly (no file(GLOB)) so a renamed or missing file
# fails at configure time instead of silently dropping out of the build.
#

set(PARAM_STORE_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(PARAM_STORE_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/param_store.c
)

# Create ParamStore object library
add_library(param_store OBJECT)
target_sources(param_store PRIVATE ${PARAM_STORE_Src})
target_include_directories(param_store PUBLIC ${PARAM_STORE_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (USE_HAL_DRIVER, STM32U575xx);
# needed by param_store.c for stm32u5xx_hal.h / main.h.
target_link_libraries(param_store PUBLIC stm32cubemx)
