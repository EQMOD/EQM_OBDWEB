/***********************************************************************************

    Filename:     usb_descriptor.h

    Description:  Interface to USB descriptors.

***********************************************************************************/
#ifndef USBDESCRIPTOR_H
#define USBDESCRIPTOR_H


#ifndef ASM_FILE
#include "usb_framework_structs.h"
#endif

/** \addtogroup module_usb_descriptor  USB Descriptor
 * \brief This module contains contains USB descriptor definitions, and guidelines on how to write
 * descriptor sets that work with the USB library.
 *
 * Information on the specific USB descriptor types is available in the USB 2.0 specification and
 * in device class documentation. Examples of complete descriptor sets can be found in the Chipcon USB
 * application examples.
 *
 * \section section_usbdsc_standard Standard Descriptors
 * The library requires the USB descriptor set to be organized as follows:
 * - Device descriptor (\ref USB_DEVICE_DESCRIPTOR)
 *     - Configuration descriptor (\ref USB_CONFIGURATION_DESCRIPTOR)
 *         - Interface descriptor (\ref USB_INTERFACE_DESCRIPTOR)
 *             - Endpoint descriptor (\ref USB_ENDPOINT_DESCRIPTOR)
 *             - More endpoint descriptors
 *         - More interface descriptors
 *     - More configuration descriptors
 * - String descriptor (\ref USB_STRING_DESCRIPTOR)
 * - More string descriptors
 *
 * Different USB device classes, such as "HID" and "Audio", may add other standard format descriptor
 * types to the hierarchy, and even extend the existing types. This is also supported by the library.
 *
 * Refer to the \ref module_usb_descriptor_parser module for information on
 * \li Where in memory the descriptor set can be placed
 * \li How to set the required markers (symbols), \ref usbDescStart and \ref usbDescEnd.
 *
 * \section section_usbdsc_other Other Descriptors
 * Differently formatted descriptors are not supported by the parsing mechanism, and are instead located
 * through a \ref DESC_LUT_INFO look-up table. Each entry in the \ref usbDescLut table contains the
 * index and value parameters for relevant \ref GET_DESCRIPTOR requests, and the locations and lengths
 * of the corresponding descriptors:
 * \code
 *                 ; Make the symbols public
 *                 PUBLIC usbDescLut;
 *                 PUBLIC usbDescLutEnd;
 *
 *                 ...
 *
 * usbDescLut:     DB HID_REPORT,  00H                         ; value (MSB, LSB)
 *                 DB 00H,         00H                         ; index (MSB, LSB)
 *                 DW hidReportDesc0Start                      ; pDescStart
 *                 DW hidReportDesc0End - hidReportDesc0Start  ; length
 *
 *                 DB HID_REPORT,  01H                         ; value (MSB, LSB)
 *                 DB 00H,         00H                         ; index (MSB, LSB)
 *                 DW hidReportDesc1Start                      ; pDescStart
 *                 DW hidReportDesc1End - hidReportDesc1Start  ; length
 * usbDescLutEnd:
 * \endcode
 *
 * An additional look-up table is needed configure endpoint double-buffering. The table must contain one
 * \ref DBLBUF_LUT_INFO entry for each interface descriptor with non-zero \c bNumEndpoints:
 * \code
 *                 ; Make the symbol public
 *                 PUBLIC usbDblbufLut;
 *
 *                 ...
 *
 * usbDblbufLut:   DW interface0Desc  ; pInterface
 *                 DB 02H             ; inMask   (example: EP1 IN is double-buffered)
 *                 DB 00H             ; outMask  (example: No double-buffered OUT endpoints)
 *
 *                 DW interface1Desc  ; pInterface
 *                 DB 10H             ; inMask   (example: EP4 IN is double-buffered)
 *                 DB 08H             ; outMask  (example: EP3 OUT is double-buffered)
 * \endcode
 * @{
 */


#ifdef EXTERN
   #undef EXTERN
#endif
#ifdef USBDESCRIPTORPARSER_C
   #define EXTERN ///< Definition used only for usb_descriptor_parser.c
#else
   #define EXTERN extern ///< Definition used in other source files to declare external
#endif


//-------------------------------------------------------------------------------------------------------
/// \name Sizes
//@{
#define EP0_PACKET_SIZE          32  ///< The maximum data packet size for endpoint 0
//@}

/// \name Standard Descriptor Types
//@{
#define DESC_TYPE_DEVICE       0x01  ///< Device
#define DESC_TYPE_CONFIG       0x02  ///< Configuration
#define DESC_TYPE_STRING       0x03  ///< String
#define DESC_TYPE_INTERFACE    0x04  ///< Interface
#define DESC_TYPE_ENDPOINT     0x05  ///< Endpoint
//@}

/// \name HID Class Descriptor Types
//@{
#define DESC_TYPE_HID          0x21  ///< HID descriptor (included in the interface descriptor)
#define DESC_TYPE_HIDREPORT    0x22  ///< Report descriptor (referenced in \ref usbDescLut)
//@}

/// \name Endpoint Types
//@{
#define EP_ATTR_CTRL           0x00  ///< Control (endpoint 0 only)
#define EP_ATTR_ISO            0x01  ///< Isochronous (not acknowledged)
#define EP_ATTR_BULK           0x02  ///< Bulk
#define EP_ATTR_INT            0x03  ///< Interrupt (guaranteed polling interval)
#define EP_ATTR_TYPE_BM        0x03  ///< Endpoint type bitmask
//@}
//-------------------------------------------------------------------------------------------------------


#ifndef ASM_FILE

//-------------------------------------------------------------------------------------------------------
/// \name USB Descriptor Marker
//@{
/// USB descriptor markers which the USB Firmware Library imports from the application
typedef struct {
    uint8 __code* const pUsbDescStart;           ///< USB descriptor start pointer
    uint8 __code* const pUsbDescEnd;             ///< USB descriptor end pointer
    DESC_LUT_INFO __code* const pUsbDescLut;     ///< Start of USB desc look-up table pointer
    DESC_LUT_INFO __code* const pUsbDescLutEnd;  ///< End of USB desc look-up table pointer
    DBLBUF_LUT_INFO __code* const pUsbDblbufLut; ///< Start of double-buffering look-up table pointer
    DBLBUF_LUT_INFO __code* const pUsbDblbufLutEnd; ///< End of double-buffering look-up table pointer
} USB_DESCRIPTOR_MARKER;

extern USB_DESCRIPTOR_MARKER __xdata usbDescriptorMarker; ///< USB descriptor marker

//-------------------------------------------------------------------------------------------------------

// Import marker symbols for the USB descriptor to use (from <app>_usb_descriptor.s51)
// They need to be defined here or in application FW
// The source file <app>_usb_descriptor.s51 need to use these names, or update
// the names used here with the ones used in <app>_usb_descriptor.s51.
extern void __code* usbDescStart;       ///< Pointer to start of (standard) USB descriptor
extern void __code* usbDescEnd;         ///< Pointer to end of (standard) USB descriptor
extern void __code* usbDescLut;         ///< Pointer to start of lookup table for non-standard USB descriptors
extern void __code* usbDescLutEnd;      ///< Pointer to end of lookup table for non-standard USB descriptors
extern void __code* usbDblbufLut;       ///< Pointer to start of lookup table for endpoints' double-buffer settings
extern void __code* usbDblbufLutEnd;    ///< Pointer to end of lookup table for endpoints' double-buffer settings
//@}


#endif // ASM_FILE
//@}

/*
+------------------------------------------------------------------------------
|  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.
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
