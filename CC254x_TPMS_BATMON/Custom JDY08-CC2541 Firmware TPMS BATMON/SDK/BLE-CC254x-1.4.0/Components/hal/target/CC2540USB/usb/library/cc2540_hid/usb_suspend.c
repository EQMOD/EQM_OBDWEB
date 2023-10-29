/***********************************************************************************

    Filename:     usb_suspend.c

    Description:  USB library common functionality.

***********************************************************************************/

/// \addtogroup module_usb_suspend
/// @{
#include "usb_firmware_library_headers.h"
#include "usb_board_cfg.h"
#include "hal_sleep.h"

__xdata VFPTR pFnSuspendEnterHook=  NULL;
__xdata VFPTR pFnSuspendExitHook=   NULL;

/***********************************************************************************
* @fn          halMcuWaitMs
*
* @brief       Busy wait function. Waits the specified number of milliseconds. Use
*              assumptions about number of clock cycles needed for the various
*              instructions.
*
*              NB! This function is highly dependent on architecture and compiler!
*
* @param       uint16 millisec - number of milliseconds delay
*
* @return      none
*/
#pragma optimize=none
static void halMcuWaitMs(uint16 msec)
{
  while(msec--)
  {
    halSleepWait(1000);
  }
}

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
   halIntState_t intState;
   uint8 exit = FALSE;

   if (!usbfwData.remoteWakeup)  return FALSE;  // Make sure that it's OK

   HAL_ENTER_CRITICAL_SECTION(intState);

   if (usbirqData.inSuspend)
   {
      HAL_USB_ENABLE();  // Never access the USB controller before PLL is stable.
      USBPOW |= USBPOW_RESUME;
      halMcuWaitMs(10);  // Perform remote wakeup by holding the USB resume signal for 10 ms.
      USBPOW &= ~USBPOW_RESUME;
      exit = TRUE;
   }

   HAL_EXIT_CRITICAL_SECTION(intState);

   if (exit && (pFnSuspendExitHook!=NULL))
   {
      pFnSuspendExitHook();
   }
   
   return TRUE;
}



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
