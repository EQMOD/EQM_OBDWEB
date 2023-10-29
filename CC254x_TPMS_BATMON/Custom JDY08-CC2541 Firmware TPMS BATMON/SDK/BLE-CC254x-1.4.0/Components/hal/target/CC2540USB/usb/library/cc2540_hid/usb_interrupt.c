/***********************************************************************************

    Filename:     usb_interrupt.c

    Description:  USB library interrupt initialisation and ISR.

***********************************************************************************/

/// \addtogroup module_usb_interrupt
/// @{
#define USBINTERRUPT_C ///< Modifies the behavior of "EXTERN" in usb_interrupt.h
#include "usb_firmware_library_headers.h"
#include "usb_board_cfg.h"


/** \brief Initializes the \ref module_usb_interrupt module
 *
 * This function should be called after the \ref module_usb_framework module has been initialized.
 * Use interrupt group priority control (refer to the CC2531 datasheet) to adjust the priority of the
 * USB interrupt relative to other interrupts.
 *
 * \param[in]       irqMask
 *     A bit mask containing USBIRQ_EVENT bits for all events that shall be reported
 */
void usbirqInit(uint16 irqMask)
{
    usbirqData.eventMask = 0x0000;
    usbirqData.irqMask = irqMask;
    usbirqData.inSuspend = FALSE;
}



/** \brief USB interrupt handler
 *
 * Clears the P2 interrupt flag and converts all USB interrupt flags into events.
 * The interrupt also lets \ref usbsuspEnter() break from the suspend loop.
 */
#if defined HAL_USB_BOOT_CODE
void usbirqHandler(void)
#else
#pragma vector=P2INT_VECTOR
__interrupt void usbirqHandler(void)
#endif
{
   uint16 eventMask;
   uint8 usbcif;

   HAL_ENTER_ISR();
   HAL_USB_ENABLE();  // Never access the USB controller before PLL is stable.
   usbcif = USBCIF;

   if ((usbcif & (USBCIF_RSTIF | USBCIF_RESUMEIF)) || (P2IFG & P2IFG_DPIF)) {
      if (usbcif & USBCIF_RSTIF) {
         // All interrupts (except suspend) are by default enabled by hardware, so
         // re-initialize the enable bits to avoid unwanted interrupts
         USBCIE = usbirqData.irqMask;
         USBIIE = usbirqData.irqMask >> 4;
         USBOIE = (usbirqData.irqMask >> 9) & 0x3E;

         // Enable suspend mode when suspend signaling is detected on the bus
         USBPOW |= USBPOW_SUSPEND_EN;
      }

      usbcif |= USBCIF_RESUMEIF;
      usbirqData.inSuspend = FALSE;
   }
   else if (usbcif & USBCIF_SUSPENDIF) {
      HAL_USB_PLL_DISABLE();  // Disable USB clock (PLL).
      usbirqData.inSuspend = TRUE;
   }

   eventMask  = usbcif;
   eventMask |= (uint16)USBIIF << 4;
   eventMask |= (uint16)USBOIF << 9;
   usbirqData.eventMask |= eventMask;  // Record events (keeping existing).

   HAL_USB_INT_CLEAR();
   HAL_EXIT_ISR();
}

//@}

/*
+------------------------------------------------------------------------------
|  Copyright 2008-2010 Texas Instruments Incorporated. All rights reserved.
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
