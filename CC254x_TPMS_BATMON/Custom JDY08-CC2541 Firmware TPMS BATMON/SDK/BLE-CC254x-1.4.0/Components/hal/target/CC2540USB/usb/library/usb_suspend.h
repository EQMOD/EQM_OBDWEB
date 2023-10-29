/***********************************************************************************

    Filename:     usb_suspend.h

    Description:  Handle the USB suspend state.

***********************************************************************************/

#ifndef USBSUSPEND_H
#define USBSUSPEND_H
/** \addtogroup module_usb_suspend USB Suspend (usbsusp)
 * \brief This module contains the functionality for USB suspend, USB resume and USB remote wakeup.
 *
 * All USB devices must support the suspended state to fully comply with the USB specification. Special
 * care must be taken to implement this functionality correctly, so follow the instructions below
 * carefully. Refer to the USB specification for detailed information on current consumption in suspend
 * mode (how power consumption shall be measured, averaging, peak value etc.).
 *
 * \section usb_suspend_resume USB Suspend and Resume
 * If there is no activity on the USB bus for a period longer than 3 ms, the MCU will generate a
 * \ref USBIRQ_EVENT_SUSPEND event. The USB device must then enter suspend mode within 10 ms, where it
 * draws no more than:
 * \li 500 uA for low-power devices or high-power devices operating at lower-power
 * \li 2.5 mA for high-power devices with remote wake-up enabled
 *
 * The library supports the USB suspend and resume functionality through a simple interface:
 * \li Make sure that the 48 MHz XOSC is never turned off anywhere in the application.
 * \li In the call to \ref usbirqInit(), add \ref USBIRQ_EVENT_SUSPEND and \ref USBIRQ_EVENT_RESUME
 *     (optional) to the interrupt mask.
 * \li Do NOT process \ref USBIRQ_EVENT_SUSPEND in \ref usbirqHookProcessEvents() or in any other
 *     interrupt service routine. This may (or in most cases will) prevent the USB device from getting
 *     out of suspend mode.
 *
 * \li In the main loop, add the code shown below. Make sure that this code is visited at least every
 *     10 ms. If the worst-case path through the main loop uses more than 10 ms, the code block can be
 *     inserted in multiple places in the loop until the requirement is met.
 * \code
 * // Process USB suspend events
 * if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SUSPEND) {
 *
 *    // Clear the suspend event
 *    USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SUSPEND);
 *
 *    ... Do what needs to be done before entering power mode 1 here (e.g. turn off the radio, configure
 *        I/O to minimize power consumption, start the sleep timer etc.) ...
 *
 *    // This function call will take the USB device into power mode 1. It will not return until resume
 *    // signaling has been detected on the bus, or the remote wake-up function has been used. Other
 *    // interrupts (for instance from I/O ports or the sleep timer) can be used during this period. When
 *    // returning from these interrupts, the \ref usbsuspEnter() function (running here) will put the
 *    // MCU back into power mode 1.
 *    usbsuspEnter();
 *
 *    // At this point the event handler is up and running again. Clear the resume event.
 *    USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESUME);
 *
 *    ... If a USBIRQ_EVENT_RESET event will affect the code that follows (before the event is processed
 *        elsewhere), then make sure to handle it here also ...
 *
 *    ... Do what needs to be done to wake up from suspend mode (e.g. turn on the radio, reactivate I/O
 *        and peripherals, turn off the sleep timer ...
 * }
 * \endcode
 *
 * \li All interrupts that run during suspension mode must start with the following code:
 * \code
 * while (!XOSC_STABLE);
 * \endcode
 *
 * \section usb_remote_wakeup USB Remote Wakeup:
 * Remote wakeup should be used when the USB device desires to initiate the resume process and wake up
 * the host. In a radio application this may happen when a particular radio packet is received, for
 * instance from a wireless keyboard or mouse.
 *
 * USB remote wakeup can only be performed if the host has given the device the privilege to do so. The
 * privilege to perform remote wakeup is requested by setting bit 5 in the \c bmAttributes field in
 * the \ref USB_CONFIGURATION_DESCRIPTOR. The host will then grant or recall the privilege through a
 * SET_FEATURE request, which is communicated through a \ref USBSR_EVENT_REMOTE_WAKEUP_ENABLED or
 * \ref USBSR_EVENT_REMOTE_WAKEUP_DISABLED event, respectively.
 *
 * The USB library handles the remote wakeup sequence automatically. Do the following to incorporate it
 * into the application:
 * \li Implement suspend and resume as described above.
 * \li In the USB descriptor, set bit 5 in bmAttributes in the configuration descriptor.
 * \li While the USB MCU is in USB suspend mode, remote wakeup can be performed from interrupt context
 *     (e.g. the sleep timer interrupt) by calling \ref usbsuspDoRemoteWakeup(). This function will
 *     return TRUE if successful or FALSE if remote wakeup is not permitted (by the host).
 * @{
 */
#include "usb_firmware_library_headers.h"

typedef void (*VFPTR)(void);

//-------------------------------------------------------------------------------------------------------
// Suspend enter/exit hooks
extern __xdata VFPTR pFnSuspendEnterHook;
extern __xdata VFPTR pFnSuspendExitHook;

//-------------------------------------------------------------------------------------------------------
// Function prototypes
void usbsuspEnter(void);
uint8 usbsuspDoRemoteWakeup(void);
void usbsuspStopPm1(void);
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
