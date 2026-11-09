/***************************************************************************
 * ux_user.h - HardBrick USBX user configuration (device side only)
 *
 * Included by ux_port.h when UX_INCLUDE_USER_DEFINE_FILE is defined (exported
 * as a PUBLIC compile definition by the usbx CMake target).
 * Reference: common/core/inc/ux_user_sample.h
 **************************************************************************/

#ifndef UX_USER_H
#define UX_USER_H

/***************************************************************************
 * Project: HardBrick (STM32U575RGT6)
 *
 * The board is a USB device only (the OTG_FS core is wired to the PC and the
 * role never switches), so the whole host side of USBX is compiled out.
 * Exactly one device class is registered: the MSC (storage) class.
 **************************************************************************/

/* Device-only build: no host stack, no host classes. */
#define UX_DEVICE_SIDE_ONLY

/* The STM32 DCD driver keeps a separate endpoint object per direction; the
   OTG core manages IN and OUT halves of the same endpoint number
   independently, and the storage class uses EP1 both ways (0x81 / 0x01). */
#define UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT

/* One class driver (storage), one logical unit (the TF card). */
#define UX_MAX_SLAVE_CLASS_DRIVER    1
#define UX_MAX_SLAVE_LUN             1

/* Bulk transfer buffer. Every endpoint transfer request and the MSC class
   both use this value; the card works in 512 B sectors, so 512 is enough and
   saves ~3.5 KB per endpoint compared with the 4 KB default.
   NOTE: must stay >= the card block size (512). */
#define UX_SLAVE_REQUEST_DATA_MAX_LENGTH    512

/* Stack of the class thread created by ux_device_class_storage_initialize().
   The thread runs the SCSI command loop and calls into the BSP (HAL_SD +
   IDMA wait), so 1 KB (the USBX default) is tight for a -O0 build. */
#define UX_THREAD_STACK_SIZE          (2 * 1024)

/* Priority of the MSC class thread. USBX defaults to 20, which loses to every
   application thread (gps=4, display=5, backlight=6, storage=7) and makes block
   transfers crawl. Keep it just below the UI/GPS threads. */
#define UX_THREAD_PRIORITY_CLASS      6

/* UX_PERIODIC_RATE is left at its default (TX_TIMER_TICKS_PER_SECOND = 100),
   which is the tick rate configured in tx_user.h. USBX converts its
   millisecond timeouts with UX_MS_TO_TICK(), so both must stay in sync. */

#endif /* UX_USER_H */
