cmake_minimum_required(VERSION 3.22)

#
# BSP layer: aggregates all board support modules.
# A new module is registered by adding its subdirectory here and
# appending its target name to BSP_LIBS below.
#

# Register modules
add_subdirectory(LC760Z)

# Export list of BSP libraries to the parent (top-level) scope
set(BSP_LIBS
    lc760z
    # Add new BSP library targets here
    PARENT_SCOPE
)
