/***********************************************************************************

    Filename:     usb_interrupt.h

    Description:  USB library interrupt initialisation and ISR.

***********************************************************************************/

#ifndef USBINTERRUPT_H
#define USBINTERRUPT_H
/** \addtogroup module_usb_interrupt  USB Interrupt (usbirq)
 * \brief This module contains the USB interrupt handler, which converts USB interrupts to USBIRQ events.
 *
 * This module contains two interrupt service routines:
 * \li P0 ISR
 * \li P2 ISR
 * Both these are used by the USB part of the MCU. Hence it is recommended to only use P1 for
 * interrupts from any peripherals connected to the MCU. Unless running low on GPIO pins, one should
 * generally avoid using the P0 and P2 pins at all.
 *
 * The MCU contains three interrupt flag registers, USBCIE, USBIIE and USBOIE, which are all cleared
 * upon read access. The \ref module_usb_interrupt module encapsulates the USB interrupts, and saves the
 * three flag registers in a single 16-bit word. By doing that it becomes possible to process
 * high-priority events in the interrupt context, and low-priority events in the main loop.
 *
 * \section section_usbirq_initialization Initialization
 * After initializing the \ref module_usb_framework module, \c main() must call \ref usbirqInit(). The
 * \c irqMask parameter of this function shall contain all \c USBIRQ_EVENT bits that will be handled
 * either in the interrupt or in \c main(). Note, however, that the event reporting may not always be
 * necessary. For instance, there is usually no reason to enable \c USBIRQ_EVENT_EPxIN or
 * \c USBIRQ_EVENT_EPxOUT events when handling low-priority transfers in the \c main() loop. In these
 * cases it is simpler and more efficient to check the arming condition of the endpoint in question.
 *
 * The following example enables the setup and reset events (which must always be enabled!), and turns on
 * global interrupts:
 * \code
 * void main(void) {
 *
 *    ... Initialize the crystal oscillator and USB framework first ...
 *
 *    // Initialize the USB Interrupt module
 *    usbirqInit(USBIRQ_EVENT_RESET | USBIRQ_EVENT_SUSPEND | USBIRQ_EVENT_RESUME | USBIRQ_EVENT_SETUP);
 *
 *    // Turn on interrupts
 *    INT_GLOBAL_ENABLE();
 *
 *    // Main loop
 *    while (1) {
 *       ...
 *    }
 * \endcode
 *
 * \section section_usbirq_event_processing Event Processing
 * Regardless of whether the interrupt event is processed in the interrupt or in the main loop, the code
 * piece for doing it is the same (this example illustrates the processing of \ref USBIRQ_EVENT_RESET
 * events):
 * \code
 * // Let the framework handle reset events :)
 * if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESET) {
 *    USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESET);
 *    usbfwResetHandler();
 * }
 * \endcode
 *
 * \section Hooks
 * The following hook is called from the USB interrupt, and allows for event processing in the interrupt
 * context:
 * \code
 * void usbirqHookProcessEvents(void) {
 *    // Process high-priority events here, or simply return if there are none
 * }
 * \endcode
 * @{
 */

#include "usb_framework_structs.h"

#ifdef EXTERN
   #undef EXTERN
#endif
#ifdef USBINTERRUPT_C
   #define EXTERN ///< Definition used only for usb_interrupt.c to declare variable
#else
   #define EXTERN extern ///< Definition used in other source files to declare external
#endif


//-------------------------------------------------------------------------------------------------------
/// USBIRQ internal module data
/*typedef struct {
    uint16 eventMask; ///< Bit mask containing all pending events (see the \c USBIRQ_EVENT definitions)
    BOOL inSuspend; ///< Is currently in suspend?
    uint16 irqMask;   ///< USB interrupts to be enabled
} USBIRQ_DATA;*/
#ifdef USBIRQ_DATA_ADDR
    EXTERN __no_init __data USBIRQ_DATA usbirqData @ USBIRQ_DATA_ADDR; ///< USBIRQ internal module data at fixed address
#else
    EXTERN USBIRQ_DATA __data usbirqData; ///< USBIRQ internal module data
#endif
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name USB Interrupt Events
//@{

/// Suspend signaling detected on the USB bus
/// Note that the chip should not enter PM1 while still in the interrupt routine (inside the usbirqHookProcessEvents() function )
#define USBIRQ_EVENT_SUSPEND         0x0001
/// Resume signaling detected on the USB bus
#define USBIRQ_EVENT_RESUME          0x0002
/// Reset signaling detected on the USB bus (call \ref usbfwResetHandler() for processing)
#define USBIRQ_EVENT_RESET           0x0004
/// Start of frame token received (synthesized by hardware when the next SOF token is expected, so that missing or corrupted tokens have no effect)
#define USBIRQ_EVENT_START_OF_FRAME  0x0008
/// Endpoint 0 IN/OUT setup/data transfer complete / stall sent / premature completion (call \ref usbfwSetupHandler() for processing)
#define USBIRQ_EVENT_SETUP           0x0010
/// Endpoint 1 IN data successfully transmitted to host (FIFO disarmed) / FIFO flushed / stall sent
#define USBIRQ_EVENT_EP1IN           0x0020
/// Endpoint 2 IN data successfully transmitted to host (FIFO disarmed) / FIFO flushed / stall sent
#define USBIRQ_EVENT_EP2IN           0x0040
/// Endpoint 3 IN data successfully transmitted to host (FIFO disarmed) / FIFO flushed / stall sent
#define USBIRQ_EVENT_EP3IN           0x0080
/// Endpoint 4 IN data successfully transmitted to host (FIFO disarmed) / FIFO flushed / stall sent
#define USBIRQ_EVENT_EP4IN           0x0100
/// Endpoint 5 IN data successfully transmitted to host (FIFO disarmed) / FIFO flushed / stall sent
#define USBIRQ_EVENT_EP5IN           0x0200
/// Endpoint 1 OUT data received from host (FIFO disarmed) / stall sent
#define USBIRQ_EVENT_EP1OUT          0x0400
/// Endpoint 2 OUT data received from host (FIFO disarmed) / stall sent
#define USBIRQ_EVENT_EP2OUT          0x0800
/// Endpoint 3 OUT data received from host (FIFO disarmed) / stall sent
#define USBIRQ_EVENT_EP3OUT          0x1000
/// Endpoint 4 OUT data received from host (FIFO disarmed) / stall sent
#define USBIRQ_EVENT_EP4OUT          0x2000
/// Endpoint 5 OUT data received from host (FIFO disarmed) / stall sent
#define USBIRQ_EVENT_EP5OUT          0x4000
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Interrupt Mask Access Macros
//@{

/// Clears one or more events (use one or more \c USBIRQ_EVENT bits OR'ed together)
#define USBIRQ_CLEAR_EVENTS(mask)    (usbirqData.eventMask &= (mask ^ 0xFFFF))
/// Get the bit mask containing all pending events
#define USBIRQ_GET_EVENT_MASK()      (usbirqData.eventMask)
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Interrupt Event Hooks
//@{

/// Called upon all USB interrupts for high-priority event processing
void usbirqHookProcessEvents(void);

//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
// Function prototypes
void usbirqInit(uint16 irqMask);
#if defined HAL_SB_BOOT_CODE
void usbirqHandler(void);
#else
__interrupt void usbirqHandler(void);
#endif
//-------------------------------------------------------------------------------------------------------


//@}

/*
+------------------------------------------------------------------------------
|  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.
|
|  IMPORTANT: Your use of this Software is limited to those specific rights
|  granted under the terms of a software license agreement between the user who
|  downloaded the software, his/her employer (which must be your employer) and
|  Texas Instruments Incorporated (the "License"). You may not use this Software
|  unless you agree to abide by the terms of the License. The License limits
|  your use, and you acknowledge, that the Software may not be modified, copied
|  or distributed unless embedded on a Texas Instruments microcontroller or used
|  solely and exclusively in conjunction with a Texas Instruments radio
|  frequency transceiver, which is integrated into your product. Other than for
|  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
|  works of, modify, distribute, perform, display or sell this Software and/or
|  its documentation for any purpose.
|
|  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
|  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
|  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
|  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
|  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
|  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
|  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING
|  BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
|  CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
|  SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
|  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
|
|  Should you have any questions regarding your right to use this Software,
|  contact Texas Instruments Incorporated at www.TI.com.
|
+------------------------------------------------------------------------------
*/

#endif
