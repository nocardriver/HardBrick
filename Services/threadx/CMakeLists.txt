cmake_minimum_required(VERSION 3.22)

#
# ThreadX kernel module (Eclipse ThreadX 6.5.1.202602a)
#
# Layout mirrors the upstream source tree so future upgrades stay simple:
#   common/                     kernel sources and headers
#   ports/cortex_m33/gnu/       Cortex-M33 GNU/clang port (ATfE compatible)
#   ports/cortex_m33/gnu/src/tx_initialize_low_level.S  adapted for STM32CubeMX
#   tx_user.h                   user configuration (see tx_user_sample.h)
#

set(THREADX_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(THREADX_Port_Dir ${THREADX_DIR}/ports/cortex_m33/gnu)

# Cortex-M33 port specific sources. tx_misra.S is only needed with
# TX_MISRA_ENABLE and is not built here (matches upstream port CMakeLists).
set(THREADX_Port_Src
    ${THREADX_Port_Dir}/src/tx_initialize_low_level.S
    ${THREADX_Port_Dir}/src/tx_thread_context_restore.S
    ${THREADX_Port_Dir}/src/tx_thread_context_save.S
    ${THREADX_Port_Dir}/src/tx_thread_interrupt_control.S
    ${THREADX_Port_Dir}/src/tx_thread_interrupt_disable.S
    ${THREADX_Port_Dir}/src/tx_thread_interrupt_restore.S
    ${THREADX_Port_Dir}/src/tx_thread_schedule.S
    ${THREADX_Port_Dir}/src/tx_thread_secure_stack.c
    ${THREADX_Port_Dir}/src/tx_thread_secure_stack_allocate.S
    ${THREADX_Port_Dir}/src/tx_thread_secure_stack_free.S
    ${THREADX_Port_Dir}/src/tx_thread_secure_stack_initialize.S
    ${THREADX_Port_Dir}/src/tx_thread_stack_build.S
    ${THREADX_Port_Dir}/src/tx_thread_system_return.S
    ${THREADX_Port_Dir}/src/tx_timer_interrupt.S
    ${THREADX_Port_Dir}/src/txe_thread_secure_stack_allocate.c
    ${THREADX_Port_Dir}/src/txe_thread_secure_stack_free.c
)

# Kernel common sources
file(GLOB THREADX_Common_Src CONFIGURE_DEPENDS ${THREADX_DIR}/common/src/*.c)

add_library(threadx STATIC)
target_sources(threadx PRIVATE ${THREADX_Common_Src} ${THREADX_Port_Src})

target_include_directories(threadx
    PUBLIC
    ${THREADX_DIR}/common/inc
    ${THREADX_Port_Dir}/inc
    ${THREADX_DIR}          # tx_user.h
)

target_compile_definitions(threadx
    PUBLIC
    TX_INCLUDE_USER_DEFINE_FILE
    # NOTE: TX_SINGLE_MODE_NON_SECURE is defined in tx_user.h, not here --
    # it must not be defined twice (macro redefinition warning). It still
    # reaches every translation unit, because tx_port.h includes tx_user.h.
)

# ThreadX kernel sources are written in a MISRA-friendly style and trigger
# warnings with -Wall (e.g. unused parameters in stub functions); keep the
# module build quiet without affecting the global warning level.
target_compile_options(threadx PRIVATE -w)
