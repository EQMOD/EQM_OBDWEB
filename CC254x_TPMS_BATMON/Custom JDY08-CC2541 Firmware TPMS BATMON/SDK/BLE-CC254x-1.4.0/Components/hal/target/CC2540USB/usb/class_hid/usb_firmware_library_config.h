/***********************************************************************************

  Filename:		usb_firmware_library_config.h

  Description:	Configuration of USB firmware library.

***********************************************************************************/

#ifndef USB_FIRMWARE_LIBRARY_CONFIG_H
#define USB_FIRMWARE_LIBRARY_CONFIG_H


/*
 *
 * Description:
 * This file is used to configures the RF USB library.
 * The definitions not needed in a project can be commented out.
 *
 *
 *             ********** IMPORTANT! *************
 *
 * If the RF USB library are to be used, copy the rf_usb_library_config_template.h file
 * into the project catalog.
 * Rename it to rf_usb_library_config.h, and edit it to get the desired setup of the framework
 *
 *             ***********************************
 *
 */



//-------------------------------------------------------------------------------------------------------
// USB framework setup

// Enter the maximum number of interfaces that are used in the configurations (used to calculate the size
// of the table that stores the currently selected alternate setting for each interface)

#if !defined USB_SETUP_MAX_NUMBER_OF_INTERFACES
#define USB_SETUP_MAX_NUMBER_OF_INTERFACES          2
#endif

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

#endif //RF_USB_LIBRARY_CONFIG_H
