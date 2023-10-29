/***********************************************************************************

    Filename: usb_cdc_hooks.c

    Contains the necessary hook functions for various USB request processing
    that is featured from the USB firmware library. Some
    functions are empty.

***********************************************************************************/


/**********************************************************************************
 * INCLUDES
 */

#include "usb_cdc.h"
#include "usb_cdc_hooks.h"
#include "usb_firmware_library_headers.h"

#include "hal_types.h"

/* Global data */

CDC_LINE_CODING_STRUCTURE currentLineCoding;


// *********************************************************************************
// All Hooks and functions required by the USB library.
// *********************************************************************************

// **************** Process USB class requests with OUT data phase *****************
void usbcrHookProcessOut(void)
{
   // Process USB class requests with OUT data phase, or stall endpoint 0 when unsupported
   if (usbSetupHeader.request == CDC_SET_CONTROL_LINE_STATE) {
       // Control line state from host
      if(usbfwData.ep0Status == EP_IDLE)
      {
         usbfwData.ep0Status = EP_RX;
      }


   } else if(usbSetupHeader.request == CDC_SET_LINE_CODING) {

      if(usbfwData.ep0Status == EP_IDLE)
      {
         usbSetupData.pBuffer = (uint8 __xdata *) &currentLineCoding;
         usbfwData.ep0Status = EP_RX;
      }
      else if(usbfwData.ep0Status == EP_RX) { }
   }
   // Unknown request?
   else {
      usbfwData.ep0Status = EP_STALL;
   }
}

// **************** Process USB class requests with IN data phase ******************
void usbcrHookProcessIn(void)
{
   // Process USB class requests with IN data phase, or stall endpoint 0 when unsupported
   if (usbSetupHeader.request == CDC_GET_LINE_CODING) {
      // First the endpoint status is EP_IDLE...
      if (usbfwData.ep0Status == EP_IDLE) {
         usbSetupData.pBuffer = (uint8 __xdata *) &currentLineCoding;
         usbSetupData.bytesLeft = 7;
         usbfwData.ep0Status = EP_TX;
         // Then the endpoint status is EP_TX (remember: we did that here when setting up the buffer)
      } else if (usbfwData.ep0Status == EP_TX) {
         // usbfwData.ep0Status is automatically reset to EP_IDLE when returning to usbfwSetupHandler()
      }
   } else {
      usbfwData.ep0Status = EP_STALL;
   }
}

// ********************************  Unsupported USB hooks *************************
void usbvrHookProcessOut(void) {usbfwData.ep0Status = EP_STALL; }
void usbvrHookProcessIn(void) {usbfwData.ep0Status = EP_STALL; }

// ************************  unsupported/unhandled standard requests ***************
void usbsrHookSetDescriptor(void) { usbfwData.ep0Status = EP_STALL; }
void usbsrHookSynchFrame(void) { usbfwData.ep0Status = EP_STALL; }
void usbsrHookClearFeature(void) { usbfwData.ep0Status = EP_STALL; }
void usbsrHookSetFeature(void) { usbfwData.ep0Status = EP_STALL; }
void usbsrHookModifyGetStatus(uint8 recipient, uint8 index, uint16 __xdata *pStatus) { }


// ************************ USB standard request event processing ******************
void usbsrHookProcessEvent(uint8 event, uint8 index)
{
   // Process relevant events, one at a time.
   switch (event) {
   case USBSR_EVENT_CONFIGURATION_CHANGING : //(the device configuration is about to change)
      break;
   case USBSR_EVENT_CONFIGURATION_CHANGED :// (the device configuration has changed)
      break;
   case USBSR_EVENT_INTERFACE_CHANGING ://(the alternate setting of the given interface is about to change)
      break;
   case USBSR_EVENT_INTERFACE_CHANGED : //(the alternate setting of the given interface has changed)
      break;
   case USBSR_EVENT_REMOTE_WAKEUP_ENABLED ://(remote wakeup has been enabled by the host)
      break;
   case USBSR_EVENT_REMOTE_WAKEUP_DISABLED ://(remote wakeup has been disabled by the host)
      break;
   case USBSR_EVENT_EPIN_STALL_CLEARED ://(the given IN endpoint's stall condition has been cleared the host)
      break;
   case USBSR_EVENT_EPIN_STALL_SET ://(the given IN endpoint has been stalled by the host)
      break;
   case USBSR_EVENT_EPOUT_STALL_CLEARED ://(the given OUT endpoint's stall condition has been cleared the host)
      break;
   case USBSR_EVENT_EPOUT_STALL_SET ://(the given OUT endpoint has been stalled by the PC)
      break;
   }
}

// ************************ USB interrupt event processing *************************
void usbirqHookProcessEvents(void)
{
    // Handle events that require immediate processing here
}

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
