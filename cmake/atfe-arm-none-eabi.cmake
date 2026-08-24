# Toolchain file for the Arm Toolchain for Embedded (ATfE).
#
# ATfE is an LLVM/clang based bare-metal toolchain for Arm targets
# (https://github.com/arm/arm-toolchain). It uses clang + lld, with
# picolibc as the C library and compiler-rt as the runtime library.
#
# The ATfE installation is located in the following order of priority:
#   1. ATFE_ROOT CMake cache variable:  cmake -DATFE_ROOT=path/to/ATfE ...
#   2. ATFE_ROOT environment variable:  setx ATFE_ROOT "path/to/ATfE"
#   3. PATH lookup:                      add <ATfE>/bin to PATH
# No machine-specific path is stored in this file or in the presets.
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID Clang)
set(CMAKE_CXX_COMPILER_ID Clang)

# ---------------------------------------------------------------------------
# Locate the ATfE installation
# ---------------------------------------------------------------------------
set(ATFE_ROOT "" CACHE PATH "Root directory of the Arm Toolchain for Embedded (ATfE) installation")

# Forward ATFE_ROOT to try_compile sub-projects (compiler feature checks),
# otherwise it is lost there and the toolchain falls back to the PATH lookup.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ATFE_ROOT)

if(NOT ATFE_ROOT AND DEFINED ENV{ATFE_ROOT})
    set(ATFE_ROOT "$ENV{ATFE_ROOT}")
endif()

if(ATFE_ROOT)
    if(NOT EXISTS "${ATFE_ROOT}/bin/clang.exe")
        message(FATAL_ERROR "ATfE not found at '${ATFE_ROOT}'. "
                            "Please check the ATFE_ROOT variable (env var or -DATFE_ROOT=...).")
    endif()
    set(ATFE_BIN_DIR "${ATFE_ROOT}/bin")
    set(CMAKE_C_COMPILER                "${ATFE_BIN_DIR}/clang.exe")
    set(CMAKE_ASM_COMPILER              "${CMAKE_C_COMPILER}")
    set(CMAKE_CXX_COMPILER              "${ATFE_BIN_DIR}/clang++.exe")
    set(CMAKE_LINKER                    "${CMAKE_C_COMPILER}")
    set(CMAKE_AR                        "${ATFE_BIN_DIR}/llvm-ar.exe")
    set(CMAKE_RANLIB                    "${ATFE_BIN_DIR}/llvm-ranlib.exe")
    set(CMAKE_OBJCOPY                   "${ATFE_BIN_DIR}/llvm-objcopy.exe")
    set(CMAKE_SIZE                      "${ATFE_BIN_DIR}/llvm-size.exe")
else()
    # Fall back to tools available on PATH (add <ATfE>/bin to PATH).
    find_program(ATFE_CLANG clang)
    if(NOT ATFE_CLANG)
        message(FATAL_ERROR "Cannot find clang from ATfE. Set the ATFE_ROOT "
                            "environment variable, pass -DATFE_ROOT=..., or add "
                            "the ATfE bin directory to PATH.")
    endif()
    set(CMAKE_C_COMPILER                clang)
    set(CMAKE_ASM_COMPILER              clang)
    set(CMAKE_CXX_COMPILER              clang++)
    set(CMAKE_LINKER                    clang)
    set(CMAKE_AR                        llvm-ar)
    set(CMAKE_RANLIB                    llvm-ranlib)
    set(CMAKE_OBJCOPY                   llvm-objcopy)
    set(CMAKE_SIZE                      llvm-size)
endif()

# Target triple (STM32U575 = Cortex-M33, Armv8-M Mainline).
# Passing the triple on the command line also lets the clang driver
# resolve the correct picolibc multilib when linking.
set(TARGET_TRIPLE                    armv8m.main-none-eabi)
set(CMAKE_C_COMPILER_TARGET          ${TARGET_TRIPLE})
set(CMAKE_CXX_COMPILER_TARGET        ${TARGET_TRIPLE})
set(CMAKE_ASM_COMPILER_TARGET        ${TARGET_TRIPLE})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "--target=${TARGET_TRIPLE} -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
# Dependency tracking (-MD -MT -MF) is added automatically by CMake for
# Ninja, so do not add -MMD/-MP here (they conflict with -MD).
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

# Link through the clang driver so that picolibc is located automatically.
# The STM32CubeMX startup file (startup_stm32u575xx.s) provides Reset_Handler,
# so picolibc's crt0 is skipped with -nostartfiles. An LLD compatible copy of
# the STM32CubeMX linker script is used (see STM32U575xx_FLASH_ATfE.ld).
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -nostartfiles")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32U575xx_FLASH_ATfE.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
