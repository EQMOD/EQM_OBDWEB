/***********************************************************************************

  Filename:	usb_hid.c

  Description:	Application support for the HID class

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/

#include "usb_firmware_library_headers.h"
#include "usb_hid_reports.h"
#include "usb_class_requests.h"
#include "usb_hid.h"

/***********************************************************************************
* GLOBAL DATA
*/

/** \brief Initializes the \ref module_usb_firmware_library_config module
*
* This function should be called first.
*/
void usbHidInit(void)
{
  // Initialize the USB interrupt handler with bit mask containing all processed USBIRQ events
  usbirqInit(0xFFFF);
  usbfwInit();  // Init USB library.
}

/// @}

void usbHidProcessEvents(void)
{
    // Handle USB resume
    if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESUME) {
        USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESUME);  // Clear USB resume interrupt
        if (pFnSuspendExitHook!=NULL)
        {
          pFnSuspendExitHook();
        }
    }

    // Handle USB reset
    if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESET) {
        USBIRQ_CLEAR_EVENTS(0xFFFF);
        usbfwResetHandler();
    }

    // Handle USB suspend
    if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SUSPEND) {
        USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SUSPEND);  // Clear USB suspend interrupt
        if (pFnSuspendEnterHook!=NULL)
        {
          pFnSuspendEnterHook();
        }
    }

    // Handle USB packets on EP0
    if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SETUP) {
        USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SETUP);
        usbfwSetupHandler();
    }
}

/*
+------------------------------------------------------------------------------
|  Copyright 2004-2013 Texas Instruments Incorporated. All rights reserved.
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





