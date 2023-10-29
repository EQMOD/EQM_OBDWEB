/***********************************************************************************

  Filename:		usb_hid_reports.h

  Description:	Implementation of HID reports

***********************************************************************************/

#include "usb_firmware_library_headers.h"
#include "usb_class_requests.h"
#include "usb_hid_reports.h"


HID_DATA hidData = {
    .keyboardOutReport.ledStatus= 0,
    .keyboardProtocol           = HID_PROTOCOL_REPORT,  // Default, as suggested by HID spec
    .mouseProtocol              = HID_PROTOCOL_REPORT,
    .keyboardIdleRate           = HID_IDLE_NOT_SET,     // Use this as init value, until it is set by USB host
    .mouseIdleRate              = HID_IDLE_NOT_SET,
};

#define halIntUnlock(HIS)  HAL_EXIT_CRITICAL_SECTION((HIS))
static halIntState_t halIntLock(void);
static halIntState_t halIntLock(void)
{
  halIntState_t intState = EA;
  HAL_DISABLE_INTERRUPTS();
  return intState;
}

/**************************************************************************************************
 * @fn      hidSendHidInReport
 *
 * @brief   Send HID Consumer Control report.
 *
 * input parameters
 *
 * @param   pReport - report to be sent
 * @param   endPoint - endPoint associated with report.
 * @param   len - length of report
 *
 * output parameters
 *
 * None.
 *
 * @return  TRUE if report was sent; FALSE otherwise.
 */
uint8 hidSendHidInReport(uint8 *pReport, uint8 endPoint, uint8 len)
{
  uint8 result = FALSE;

  if (endPoint < 6)
  {
    uint8 ea = halIntLock();

    USBFW_SELECT_ENDPOINT(endPoint);
    if (!(USBCSIL & USBCSIL_INPKT_RDY))
    {
      usbfwWriteFifo(((&USBF0) + (endPoint << 1)), len, pReport);
      USBCSIL |= USBCSIL_INPKT_RDY;
      result = TRUE;
    }
    halIntUnlock(ea);
  }

  return result;
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

