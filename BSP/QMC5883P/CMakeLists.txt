cmake_minimum_required(VERSION 3.22)

#
# QMC5883P module: QST 三轴地磁传感器驱动（BSP 层）
#
#   qmc5883p.c      驱动核心：寄存器协议、模式/量程/滤波配置、6 字节连读、
#                   LSB→mG 换算、Set/Reset 自检。纯 C，不 include HAL/RTOS，
#                   换 MCU 只需重写 port 层。
#   qmc5883p_port.c 平台胶水：I2C2（PB13/PB14）阻塞收发、HAL tick/delay。
#                   唯一允许 include HAL 头文件的翻译单元。
#
# 显式列出源文件（不用 file(GLOB)）：文件改名或漏提交时在 configure 阶段就报错。
#

set(QMC5883P_Inc_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(QMC5883P_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/qmc5883p.c
    ${CMAKE_CURRENT_SOURCE_DIR}/qmc5883p_port.c
)

# Create QMC5883P object library
add_library(qmc5883p OBJECT)
target_sources(qmc5883p PRIVATE ${QMC5883P_Src})
target_include_directories(qmc5883p PUBLIC ${QMC5883P_Inc_Dirs})

# Inherit STM32CubeMX include paths and symbols (USE_HAL_DRIVER, STM32U575xx);
# needed by qmc5883p_port.c for stm32u5xx_hal.h / i2c.h / main.h.
target_link_libraries(qmc5883p PUBLIC stm32cubemx)
