# Toolchain file for the Arm Toolchain for Embedded (ATfE).
#
# ATfE is an LLVM/clang based bare-metal toolchain for Arm targets
# (https://github.com/arm/arm-toolchain). It uses clang + lld, with
# picolibc as the C library and compiler-rt as the runtime library.
#
# The installation root can be overridden at configure time:
#   cmake -DATFE_ROOT=path/to/ATfE-<version>-Windows-x86_64 ...
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID Clang)
set(CMAKE_CXX_COMPILER_ID Clang)

# Root of the ATfE installation
set(ATFE_ROOT "C:/Users/Forwork/AppData/Local/Programs/ATfE-22.1.0-Windows-x86_64" CACHE PATH "Path to the Arm Toolchain for Embedded installation root")
set(TOOLCHAIN_PREFIX "${ATFE_ROOT}/bin/")

set(CMAKE_C_COMPILER                "${TOOLCHAIN_PREFIX}clang.exe")
set(CMAKE_ASM_COMPILER              "${CMAKE_C_COMPILER}")
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_PREFIX}clang++.exe")
set(CMAKE_LINKER                    "${CMAKE_C_COMPILER}")
set(CMAKE_AR                        "${TOOLCHAIN_PREFIX}llvm-ar.exe")
set(CMAKE_RANLIB                    "${TOOLCHAIN_PREFIX}llvm-ranlib.exe")
set(CMAKE_OBJCOPY                   "${TOOLCHAIN_PREFIX}llvm-objcopy.exe")
set(CMAKE_SIZE                      "${TOOLCHAIN_PREFIX}llvm-size.exe")

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
