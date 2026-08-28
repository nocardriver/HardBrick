/***************************************************************************
 * tx_user.h - HardBrick ThreadX user configuration
 *
 * This file is included by tx_port.h when TX_INCLUDE_USER_DEFINE_FILE is
 * defined (set as a PUBLIC compile definition on the threadx target).
 * Every define here may alternatively be set on the command line.
 * Reference: common/inc/tx_user_sample.h
 **************************************************************************/

#ifndef TX_USER_H
#define TX_USER_H

/***************************************************************************
 * Project: HardBrick (STM32U575RGT6, Cortex-M33, 160 MHz, FPU hard-float)
 *
 * - TrustZone is DISABLED (Mcu.ContextProject=TrustZoneDisabled), so the
 *   port runs in "single mode" and all secure-stack machinery is excluded:
 *     --> TX_SINGLE_MODE_NON_SECURE
 * - HAL timebase will be moved off SysTick (to LPTIM/TIM) in CubeMX;
 *   SysTick is then owned by ThreadX at TX_TIMER_TICKS_PER_SECOND (100 Hz).
 * - FPU context (lazy stacking) is handled automatically by tx_port.h,
 *   detected via __ARM_FP.
 **************************************************************************/

/* TrustZone disabled: single (non-secure) mode, no secure stack support. */
#define TX_SINGLE_MODE_NON_SECURE

/* Enable thread stack checking (stack filled with 0xEF pattern, overflow
   detection via UsageFault/PSPLIM handling in tx_initialize_low_level.S).
   Recommended during development; can be removed for release builds. */
#define TX_ENABLE_STACK_CHECKING

/* --- Options left at ThreadX defaults, listed for documentation ---------
 * #define TX_MAX_PRIORITIES                32      // Thread priorities
 * #define TX_MINIMUM_STACK                 200     // Min thread stack size
 * #define TX_TIMER_TICKS_PER_SECOND        100     // Kernel tick rate
 * #define TX_TIMER_THREAD_PRIORITY         0       // Timer thread prio
 * #define TX_TIMER_THREAD_STACK_SIZE       1024    // Timer thread stack
 * // Timer processing runs in a dedicated (default) timer thread;
 * // TX_TIMER_PROCESS_IN_ISR is NOT defined.
 * // API error checking is enabled; TX_DISABLE_ERROR_CHECKING is NOT defined.
 * ---------------------------------------------------------------------- */

#endif /* TX_USER_H */
