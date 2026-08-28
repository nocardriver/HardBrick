cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME HaradBrick)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add user-defined service modules (exports SERVICES_LIBS)
add_subdirectory(Services)

# Add user-defined BSP modules (exports BSP_LIBS)
add_subdirectory(BSP)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ThreadX integration: CubeMX regenerates stub handlers for the Cortex-M
# system exceptions in Core/Src/stm32u5xx_it.c on every code generation.
# The ThreadX port (Services/threadx) provides the real implementations of
# PendSV_Handler (scheduler), SysTick_Handler (kernel tick) and
# HardFault/UsageFault (stack overflow handling). Rename the CubeMX stubs
# at compile time so they never clash with the ThreadX definitions; the
# renamed stubs are dead code and are removed by --gc-sections.
set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32u5xx_it.c
    PROPERTIES COMPILE_DEFINITIONS
    "PendSV_Handler=MX_Unused_PendSV_Handler;SysTick_Handler=MX_Unused_SysTick_Handler;HardFault_Handler=MX_Unused_HardFault_Handler;UsageFault_Handler=MX_Unused_UsageFault_Handler"
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    ${SERVICES_LIBS}
    ${BSP_LIBS}
)
