/***********************************************************************************

    Filename:     usb_framework_structs.h

    Description:  USB library common data structures.

***********************************************************************************/

#ifndef USBFRAMEWORKSTRUCTS_H
#define USBFRAMEWORKSTRUCTS_H
/** \addtogroup module_usb_framework USB Framework (usbfw)
 * \brief This module contains USB status and descriptor structs
 *
 *
 * @{
 */
#include "hal_types.h"
#include "usb_firmware_library_config.h"

#ifdef EXTERN
   #undef EXTERN
#endif
#ifdef USBFRAMEWORK_C
   #define EXTERN ///< Definition used only for usb_framework.c
#else
   #define EXTERN extern ///< Definition used in other source files to declare external
#endif


//-------------------------------------------------------------------------------------------------------
/// \name Module Data
//@{

/// Endpoint status (used with USB_INFO.ep0Status / pEpInStatus[] / pEpOutStatus[])
typedef enum {
    EP_IDLE      = 0x00,  ///< The endpoint is idle, or a setup token has been received
    EP_TX        = 0x01,  ///< Setup IN data is transmitted automatically by the framework
    EP_RX        = 0x02,  ///< Setup OUT data is received automatically by the framework
    EP_HALT      = 0x03,  ///< The endpoint is halted (returns stalls to the host)
    EP_STALL     = 0x04,  ///< Send procedural stall in the next status phase
    EP_MANUAL_TX = 0x05,  ///< Setup IN data is transmitted manually by the user application
    EP_MANUAL_RX = 0x06,  ///< Setup OUT data is received manually by the user application
    EP_CANCEL    = 0x07   ///< The current transfer was cancelled by the host
} EP_STATUS;

/// Device state (used with USB_INFO.usbState)
typedef enum {
   DEV_ATTACHED   = 0x00,  ///< Device attached (invisible state)
   DEV_POWERED    = 0x01,  ///< Device powered (invisible state)
   DEV_DEFAULT    = 0x02,  ///< Default state (the \c USBADDR register is 0)
   DEV_ADDRESS    = 0x03,  ///< Addressed state (the \c USBADDR register has been set)
   DEV_CONFIGURED = 0x04,  ///< Configured state (\c usbfwData.configurationValue != 0)
   DEV_SUSPENDED  = 0x05   ///< Suspended state (never set)
} USB_STATE;

/// USBFW internal module data
typedef struct {
    USB_STATE usbState;                                           ///< USB device state
    uint8 configurationValue;                                     ///< Current configuration value
    uint8 pAlternateSetting[USB_SETUP_MAX_NUMBER_OF_INTERFACES];  ///< Current alternate settings
    EP_STATUS ep0Status;                                          ///< Endpoint 0 status
    EP_STATUS pEpInStatus[5];                                     ///< Endpoint 1-5 IN status
    EP_STATUS pEpOutStatus[5];                                    ///< Endpoint 1-5 OUT status
    uint8 remoteWakeup;                                           ///< Remote wakeup allowed
    uint8 selfPowered;                                            ///< Is currently self-powered?
} USBFW_DATA;

//-------------------------------------------------------------------------------------------------------
/// Setup header (contains the 8 bytes received during the setup phase)
typedef struct {
    uint8 requestType;  ///< Request type (direction, type and recipient, see the \c RT_ definitions)
    uint8 request;      ///< Request ID
    uint16 value;       ///< Value field
    uint16 index;       ///< Index field
    uint16 length;      ///< Length of data phase
} USB_SETUP_HEADER;

/// Setup handler data phase configuration
typedef struct {
    uint8 __generic *pBuffer;  ///< Pointer to where IN/OUT data should be taken from/received
    uint16 bytesLeft;         ///< The number of bytes to transfer
} USB_SETUP_DATA;


//@}
//-------------------------------------------------------------------------------------------------------

// From usb_descriptor.h
/** \addtogroup module_usb_descriptor  USB Descriptor*/
//-------------------------------------------------------------------------------------------------------
/// USB device descriptor
typedef struct {
    uint8 bLength;             ///< Size of this descriptor (in bytes)
    uint8 bDescriptorType;     ///< Descriptor type = \ref DESC_TYPE_DEVICE
    uint16 bcdUSB;             ///< USB specification release number (in BCD, e.g. 0110 for USB 1.1)
    uint8 bDeviceClass;        ///< Device class code
    uint8 bDeviceSubClass;     ///< Device subclass code	
    uint8 bDeviceProtocol;     ///< Device protocol code
    uint8 bMaxPacketSize0;     ///< Maximum packet size for EP0
    uint16 idVendor;           ///< Vendor ID
    uint16 idProduct;          ///< Product ID
    uint16 bcdDevice;          ///< Device release number (in BCD)
    uint8 iManufacturer;       ///< Index of the string descriptor for manufacturer
    uint8 iProduct;            ///< Index of the string descriptor for product
    uint8 iSerialNumber;       ///< Index of the string descriptor for serial number
    uint8 bNumConfigurations;  ///< Number of possible configurations
} USB_DEVICE_DESCRIPTOR;

/// USB configuration descriptor
typedef struct {
    uint8 bLength;             ///< Size of this descriptor (in bytes)
    uint8 bDescriptorType;     ///< Descriptor type = \ref DESC_TYPE_CONFIG
    uint16 wTotalLength;       ///< Total length of data for this configuration
    uint8 bNumInterfaces;      ///< Number of interfaces supported by this configuration (one-based index)
    uint8 bConfigurationValue; ///< Designator value for this configuration
    uint8 iConfiguration;      ///< Index of the string descriptor for this configuration
    uint8 bmAttributes;        ///< Configuration characteristics
    uint8 bMaxPower;           ///< Maximum power consumption in this configuration (bMaxPower * 2 mA)
} USB_CONFIGURATION_DESCRIPTOR;

/// USB interface descriptor
typedef struct {
    uint8 bLength;             ///< Size of this descriptor (in bytes)
    uint8 bDescriptorType;     ///< Descriptor type = \ref DESC_TYPE_INTERFACE
    uint8 bInterfaceNumber;    ///< Number of *this* interface (zero-based index)
    uint8 bAlternateSetting;   ///< Alternate setting index for this descriptor (zero-based index)
    uint8 bNumEndpoints;       ///< Number of endpoints for this interface (excl. EP0)
    uint8 bInterfaceClass;     ///< Interface class code
    uint8 bInterfaceSubClass;  ///< Interface subclass code
    uint8 bInterfaceProtocol;  ///< Interface protocol code
    uint8 iInterface;          ///< Index of the string descriptor for this interface
} USB_INTERFACE_DESCRIPTOR;

/// USB endpoint descriptor
typedef struct {
    uint8 bLength;             ///< Size of this descriptor (in bytes)
    uint8 bDescriptorType;     ///< Descriptor type = \ref DESC_TYPE_ENDPOINT
    uint8 bEndpointAddress;    ///< Endpoint address (direction[7] + number[3:0])
    uint8 bmAttributes;        ///< Endpoint attributes (ISO / BULK / INT)
    uint16 wMaxPacketSize;      ///< Maximum endpoint packet size
    uint8 bInterval;           ///< \ref EP_ATTR_INT : Polling interval (in ms)
} USB_ENDPOINT_DESCRIPTOR;

/// USB string descriptor
typedef struct {
    uint8 bLength;             ///< Size of this descriptor (in bytes)
    uint8 bDescriptorType;     ///< Descriptor type = \ref DESC_TYPE_STRING
    uint16 pString[1];         ///< Unicode string
} USB_STRING_DESCRIPTOR;

/// USB HID descriptor
typedef struct {
    uint8 bLength;               ///< Size of this descriptor (in bytes)
    uint8 bDescriptorType;       ///< HID descriptor type
    uint16 bscHID;               ///< HID specification release number (in BCD)
    uint8 bCountryCode;          ///< Hardware target country
    uint8 bNumDescriptors;       ///< Number of HID class descriptors to follow
    uint8 bRDescriptorType;      ///< Report descriptor type
    uint16 wDescriptorLength;    ///< Total length of the associated report descriptor
} USB_HID_DESCRIPTOR;
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// Look-up table entry for non-standard descriptor types (used with \ref usbsrGetDescriptor)
typedef struct {
    uint8 valueMsb;            ///< LSB of the \ref USB_SETUP_HEADER.value request parameter
    uint8 valueLsb;            ///< MSB of the \ref USB_SETUP_HEADER.value request parameter
    uint8 indexMsb;            ///< LSB of the \ref USB_SETUP_HEADER.index request parameter
    uint8 indexLsb;            ///< MSB of the \ref USB_SETUP_HEADER.index request parameter
    uint8 __code *pDescStart;  ///< A pointer to the descriptor to be returned for the given index/value
    uint16 length;            ///< The length of the returned descriptor
} DESC_LUT_INFO;

/// Look-up table for double-buffer settings
typedef struct {
    USB_INTERFACE_DESCRIPTOR __code *pInterface; ///< Pointer to an interface descriptor
    uint8 inMask;                                 ///< Bitmask for IN endpoints (bit x maps to EPx IN)
    uint8 outMask;                                ///< Bitmask for OUT endpoints (bit x maps to EPx OUT)
} DBLBUF_LUT_INFO;

// From usb_interrupt.h
//-------------------------------------------------------------------------------------------------------
/// USBIRQ internal module data
typedef struct {
    uint16 eventMask; ///< Bit mask containing all pending events (see the \c USBIRQ_EVENT definitions)
    uint8 inSuspend; ///< Is currently in suspend?
    uint16 irqMask;   ///< USB interrupts to be enabled
} USBIRQ_DATA;

//-------------------------------------------------------------------------------------------------------
/// USBDP internal module data
typedef struct {
    const uint8 __code *pDesc; ///< Pointer to the current descriptor
} USBDP_DATA;
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
