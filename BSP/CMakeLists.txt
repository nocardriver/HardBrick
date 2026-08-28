cmake_minimum_required(VERSION 3.22)

#
# BSP layer: aggregates all board support modules.
# A new module is registered by adding its subdirectory here and
# appending its target name to BSP_LIBS below.
#

# Register modules
add_subdirectory(uart_dma)
# add_subdirectory(LC760Z)   # 阶段1加入：LC760Z GPS 驱动

# Export list of BSP libraries to the parent (top-level) scope
set(BSP_LIBS
    uart_dma
    # Add new BSP library targets here
    PARENT_SCOPE
)
