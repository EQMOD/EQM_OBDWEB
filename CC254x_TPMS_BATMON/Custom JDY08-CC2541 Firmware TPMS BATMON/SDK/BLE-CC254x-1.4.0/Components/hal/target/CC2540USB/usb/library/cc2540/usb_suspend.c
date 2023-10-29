/***********************************************************************************

    Filename:     usb_suspend.c

    Description:  USB library common functionality.

***********************************************************************************/

/// \addtogroup module_usb_suspend
/// @{
#include "usb_firmware_library_headers.h"
#include "hal_board.h"
#include "hal_mcu.h"
#include "hal_led.h"

__xdata VFPTR pFnSuspendEnterHook=  NULL;
__xdata VFPTR pFnSuspendExitHook=   NULL;

#if HAL_UART_USB_SUSPEND
extern void halEnterPowerMode(void);

/** \brief Puts the chip into power mode 1 during USB suspend.
 *
 * This function must be called from main (i.e. not from interrupt context) upon the reception of a
 * \ref USBIRQ_EVENT_SUSPEND event. To comply with the USB specification, this must happen within 10 ms
 * after the event occurs. The chip will stay in power mode 1 until a USB resume or USB reset is detected
 * on the USB bus, or remote wakeup is used. During this period, the MCU can only run code from
 * interrupt context.
 */
void usbsuspEnter(void)
{
    if (pFnSuspendEnterHook!=NULL)
        pFnSuspendEnterHook();

      HAL_USB_INT_CLEAR();
      HAL_USB_INT_ENABLE();

      // Disable USB clock (PLL) before entering PM1
      HAL_USB_PLL_DISABLE();
    
    HAL_LED_CLR_1();

    do {
          // Enter PM1, in prescribed manner as explained in CC253x User's Guide
          SLEEPCMD = 0x05;
        halEnterPowerMode();
    } while ( usbirqData.inSuspend );

      // .... we are now up and running again

      // First make sure that the crystal oscillator is stable
    while (!CC2540_IS_XOSC_STABLE());

      // Restart the USB clock (PLL)
      HAL_USB_ENABLE();

    if (pFnSuspendExitHook!=NULL)
        pFnSuspendExitHook();
} // usbsuspEnter
#endif



/** \brief Attempts USB remote wakeup.
 *
 * This function can be called from interrupt context while the USB device is suspend mode. If the device
 * is privileged to do so (see \c usbfwData.remoteWakeup and the \ref USBSR_EVENT_REMOTE_WAKEUP_ENABLED
 * and \ref USBSR_EVENT_REMOTE_WAKEUP_DISABLED events), remote wakeup will be performed. Note that this
 * function will block for 10 ms while the resume signal is set on the bus. Note: This function can only
 * be called when the 48 MHz XOSC is stable.
 *
 * \return
 *     \c TRUE if the remote wakeup was performed (the privilege had been granted), otherwise \c FALSE
 *     (the device is still in suspend mode).
 */
uint8 usbsuspDoRemoteWakeup(void)
{
   extern void halMcuWaitMs(uint16 msec);
   halIntState_t   intState;

   // Make sure that it's OK
   if (!usbfwData.remoteWakeup) return FALSE;

   HAL_ENTER_CRITICAL_SECTION(intState);

   // Make sure that the suspend loop does not power down the chip again
   usbirqData.inSuspend = FALSE;

   // Perform remote wakeup by holding the USB resume signal for 10 ms
   USBPOW |= USBPOW_RESUME;
   halMcuWaitMs(10);
   USBPOW &= ~USBPOW_RESUME;

   // Clear the interrupt flag
   HAL_USB_INT_CLEAR();

   HAL_EXIT_CRITICAL_SECTION(intState);
   
   return TRUE;

} // usbsuspDoRemoteWakeup



//@}
/*
+------------------------------------------------------------------------------
|  Copyright 2004-2010 Texas Instruments Incorporated. All rights reserved.
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
