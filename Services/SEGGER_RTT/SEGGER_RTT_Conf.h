/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************

---------------------------END-OF-HEADER------------------------------
Purpose : Project-specific RTT configuration for HardBrick
          (STM32U575AII6, Cortex-M33, ARMv8-M Mainline).

          This file overrides the factory defaults in
          SEGGER_RTT_ConfDefaults.h. Every value below must NOT be
          wrapped in #ifndef, so it always wins over the defaults.

          Interrupt locking (SEGGER_RTT_LOCK/UNLOCK): left undefined,
          the defaults in SEGGER_RTT_ConfDefaults.h will use the
          BASEPRI-based lock for Cortex-M (ARMv8-M Mainline), which is
          compatible with the 4-bit priority scheme of the STM32U5.
----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

//
// Number of RTT channels
//   Up-buffers  (Target -> Host): ch0 terminal, ch1 SystemView/log, ch2 spare
//   Down-buffers (Host -> Target): ch0 terminal input
//
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS       (3)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS     (3)

//
// Buffer sizes in bytes
//   ch0 up-buffer holds log strings before the host polls them.
//   With SEGGER_RTT_MODE_DEFAULT below, packets are skipped (not lost)
//   once the buffer is full, so size vs. poll interval is a trade-off.
//
#define BUFFER_SIZE_UP                      (1024)
#define BUFFER_SIZE_DOWN                    (16)
#define TERMINAL_BUFFER_SIZE                (64)

//
// Default transfer mode for channel 0.
//   NO_BLOCK_SKIP: never stall the target, drop data on overflow.
//
#define SEGGER_RTT_MODE_DEFAULT             SEGGER_RTT_MODE_NO_BLOCK_SKIP

//
// Temporary stack buffer used by SEGGER_RTT_printf()
//
#define SEGGER_RTT_PRINTF_BUFFER_SIZE       (64u)

//
// 0: use libc memcpy(), 1: simple byte loop (smaller, slower)
//
#define SEGGER_RTT_MEMCPY_USE_BYTELOOP      0

//
// Interrupts with numerical priority lower (more urgent) than this are
// NOT masked while writing RTT data on Cortex-M.
// NVIC priority bits = 4 on STM32U5 -> effective priorities are shifted
// left by 4. 0x20 == priority level 2: SysTick(15)/HAL(5) stay masked,
// only level 0-1 handlers run during an RTT write. Raise it carefully;
// nesting unsafe RTT calls corrupts buffer contents.
//
#define SEGGER_RTT_MAX_INTERRUPT_PRIORITY   (0x20)

#endif
/*************************** End of file ****************************/
