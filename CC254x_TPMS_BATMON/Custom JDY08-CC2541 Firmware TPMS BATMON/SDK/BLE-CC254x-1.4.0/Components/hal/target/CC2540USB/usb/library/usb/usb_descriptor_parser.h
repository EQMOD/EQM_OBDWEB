/***********************************************************************************

    Filename:     usb_descriptor_parser.h

    Description:  Parser for USB descriptor structures.

***********************************************************************************/

#ifndef USBDESCRIPTORPARSER_H
#define USBDESCRIPTORPARSER_H
/** \addtogroup module_usb_descriptor_parser  USB Descriptor Parser (usbdp)
 * \brief This module contains internally used functions for locating USB descriptors.
 *
 * The parsing mechanism supports all standard descriptors, i.e. DEVICE, CONFIGURATION, INTERFACE,
 * ENDPOINT and STRING, but also other types that use the standard descriptor format:
 * \code
 * typedef struct {
 *    uint8 bLength;                // Size of this descriptor (in bytes)
 *    uint8 bDescriptorType;        // Descriptor type
 *    ...
 * } USB_XXXXXXXX_DESCRIPTOR;
 * \endcode
 *
 * \section section_usbdp_requirements Requirements
 * The standard-formatted descriptors must be placed back-to-back in either XDATA or CODE memory.
 * In the current version of the library, the USB descriptors are assumed to reside in CODE.
 * Two markers (XDATA or CODE memory pointers), \ref usbDescStart and \ref usbDescEnd, define where
 * the first descriptor begins and where the last descriptor ends, respectively
 * (so that <tt>usbDescStart - usbDescEnd</tt> equals the total length of the descriptors).
 *
 * The markers can be dynamic, provided that they are not changed while the descriptor parser is in use.
 * However, in most cases the USB descriptor set will be static, hence the markers are also static.
 * The following example shows how static markers are declared and made public in 8051 assembler:
 * \code
 *                 ; Make the symbols public
 *                 PUBLIC usbDescStart;
 *                 PUBLIC usbDescEnd;
 *
 *                 ...
 *
 * usbDescStart:
 * deviceDesc:     ; Device descriptor (the first descriptor)
 *                 DB deviceDescEnd - deviceDesc
 *                 DB DESC_TYPE_DEVICE  ; bDescriptorType
 *                 DB 10H, 01H          ; bcdUSB
 *                 DB 00H               ; bDeviceClass
 *                 DB 00H               ; bDeviceSubClass
 *
 *                 ...
 *
 * string3Desc:    ; String descriptor: Serial number (the last descriptor)
 *                 DB string3DescEnd - string3Desc;
 *                 DB DESC_TYPE_STRING  ; bDescriptorType
 *                 DB '1', 0;
 *                 DB '2', 0;
 *                 DB '3', 0;
 * string3DescEnd:
 * usbDescEnd:
 * \endcode
 * @{
 */

#include "usb_descriptor.h"


//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Indexes Into USB Descriptors
//@{
#define DESC_LENGTH_IDX             0 ///< Index of the bLength field (all descriptors)
#define DESC_TYPE_IDX               1 ///< Index of the bDescriptorType field (all descriptors)
#define DESC_CONFIG_LENGTH_LSB_IDX  2 ///< Index of LOUINT8(USB_CONFIGURATION_DESCRIPTOR.wTotalLength)
#define DESC_CONFIG_LENGTH_MSB_IDX  3 ///< Index of HIUINT8(USB_CONFIGURATION_DESCRIPTOR.wTotalLength)
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
// Function prototypes
void usbdpInit(void);
void __code *usbdpFindNext(uint8 wantedType, uint8 haltAtType);

USB_DEVICE_DESCRIPTOR __code* usbdpGetDeviceDesc(void);
USB_CONFIGURATION_DESCRIPTOR __code* usbdpGetConfigurationDesc(uint8 cfgValue, uint8 cfgIndex);
USB_INTERFACE_DESCRIPTOR __code* usbdpGetInterfaceDesc(uint8 cfgValue, uint8 intNumber, uint8 altSetting);
USB_STRING_DESCRIPTOR *usbdpGetStringDesc(uint8 strIndex);
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
