/***********************************************************************************

    Filename:     usb_standard_request.h

    Description:  Handle USB standard requests.

***********************************************************************************/

#ifndef USBSTANDARDREQUESTS_H
#define USBSTANDARDREQUESTS_H
/** \addtogroup module_usb_standard_requests  USB Standard Requests (usbsr)
 * \brief This module contains automated functions for processing USB standard requests
 *
 * The processing functions are based on the \ref module_usb_framework, and access to the user-provided
 * USB descriptors through the \ref module_usb_descriptor_parser. All device classes and descriptor
 * combinations are supported, with no need to write or modify any source code. However, as described
 * below, some standard request must be fully or partially implemented by the user.
 *
 * \section section_usbsr_hooks Hooks
 * Standard requests that are not supported by the USB library or that refer to non-standard features,
 * are forwarded to the application via function hooks. This includes:
 * \li All \ref SET_DESCRIPTOR requests (see \ref usbsrHookSetDescriptor())
 * \li All \ref SYNCH_FRAME requests (see \ref usbsrHookSynchFrame())
 * \li \ref CLEAR_FEATURE requests that refer to unknown features (see \ref usbsrHookClearFeature())
 * \li \ref SET_FEATURE requests that refer to unknown features (see \ref usbsrHookSetFeature())
 *
 * These hooks must always be provided, however if the application does not either support the requests,
 * it should just stall endpoint 0. The processing uses the same mechanisms as for class and vendor
 * requests (refer to the \ref module_usb_framework module for detailed description and examples).
 *
 * When the \ref GET_STATUS request is received, the \ref usbsrHookModifyGetStatus() hook is always
 * called, so that additional status bits may be added.
 *
 * To have any practical purpose, "OUT data phase" standard requests need to notify the application of
 * certain events. This is done by passing the event via yet another function hook,
 * \ref usbsrHookProcessEvent(uint8 event, uint8 index). For events related to interfaces and endpoints,
 * the \c index parameter refers to an interface number or the least significant nibble of the endpoint
 * address. The following events can be generated:
 * \li \ref USBSR_EVENT_CONFIGURATION_CHANGING (the device configuration is about to change)
 * \li \ref USBSR_EVENT_CONFIGURATION_CHANGED (the device configuration has changed)
 * \li \ref USBSR_EVENT_INTERFACE_CHANGING (the alternate setting of the given interface is about to
 *     change)
 * \li \ref USBSR_EVENT_INTERFACE_CHANGED (the alternate setting of the given interface has changed)
 * \li \ref USBSR_EVENT_REMOTE_WAKEUP_ENABLED (remote wakeup has been enabled by the host)
 * \li \ref USBSR_EVENT_REMOTE_WAKEUP_DISABLED (remote wakeup has been disabled by the host)
 * \li \ref USBSR_EVENT_EPIN_STALL_CLEARED (the given IN endpoint's stall condition has been cleared the
 *     host)
 * \li \ref USBSR_EVENT_EPIN_STALL_SET (the given IN endpoint has been stalled by the host)
 * \li \ref USBSR_EVENT_EPOUT_STALL_CLEARED (the given OUT endpoint's stall condition has been cleared
 *     the host)
 * \li \ref USBSR_EVENT_EPOUT_STALL_SET (the given OUT endpoint has been stalled by the PC)
 * @{
 */


//-------------------------------------------------------------------------------------------------------
/// \name Standard Request Codes
//@{

/// Standard request that returns status for the specified recipient
#define GET_STATUS           0x00
/// Standard request that clears or disables a specific feature
#define CLEAR_FEATURE        0x01
/// Standard request that sets or enables a specific feature
#define SET_FEATURE          0x03
/// Standard request that sets the device address for all future device accesses
#define SET_ADDRESS          0x05
/// Standard request that returns the specified USB descriptor
#define GET_DESCRIPTOR       0x06
/// Standard request that may be used to update exitsting descriptors or new descriptors may be added
#define SET_DESCRIPTOR       0x07
/// Standard request that returns the current device configuration value
#define GET_CONFIGURATION    0x08
/// Standard request that sets the device configuration
#define SET_CONFIGURATION    0x09
/// Standard request that returns the selected alternate setting for the specified interface
#define GET_INTERFACE        0x0A
/// Standard request that selects an alternate setting for the specified interface
#define SET_INTERFACE        0x0B
/// Standard request that is used to set and then report an endpoint's synchronization frame
#define SYNCH_FRAME          0x0C
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Features Indexes
//@{

/// Endpoint feature: Halt
#define ENDPOINT_HALT        0x00
/// Device feature: Remote wakeup
#define DEVICE_REMOTE_WAKEUP 0x01
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Event Types
//@{

/// The device configuration is about to change
#define USBSR_EVENT_CONFIGURATION_CHANGING  0x01
/// The device configuration has changed
#define USBSR_EVENT_CONFIGURATION_CHANGED   0x02
/// The alternate setting of the given interface about to change (index = "interface index")
#define USBSR_EVENT_INTERFACE_CHANGING      0x03
/// The alternate setting of the given interface has changed (index = "interface index")
#define USBSR_EVENT_INTERFACE_CHANGED       0x04
/// Remote wakeup has been enabled by the host
#define USBSR_EVENT_REMOTE_WAKEUP_ENABLED   0x05
/// Remote wakeup has been disabled by the host
#define USBSR_EVENT_REMOTE_WAKEUP_DISABLED  0x06
/// The given IN endpoint has been unstalled by the PC (index = "endpoint address" & 0x0F)
#define USBSR_EVENT_EPIN_STALL_CLEARED      0x07 /* Endpoint index */
/// The given IN endpoint has been stalled by the PC (index = "endpoint address" & 0x0F)
#define USBSR_EVENT_EPIN_STALL_SET          0x08 /* Endpoint index */
/// The given OUT endpoint has been unstalled by the PC (index = "endpoint address" & 0x0F)
#define USBSR_EVENT_EPOUT_STALL_CLEARED     0x09 /* Endpoint index */
/// The given OUT endpoint has been stalled by the PC (index = "endpoint address" & 0x0F)
#define USBSR_EVENT_EPOUT_STALL_SET         0x0A /* Endpoint index */
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Standard Request Hooks
//@{
/// Refer to the \ref section_setup_handler_usage section for a description on how to process standard
/// requests.

/// Hook which is called upon reception of a \ref SET_DESCRIPTOR request
void usbsrHookSetDescriptor(void);
/// Hook which is called upon reception of a \ref SYNCH_FRAME request (unsupported).
void usbsrHookSynchFrame(void);
/// Hook which is called when a \ref CLEAR_FEATURE request refers to a an unsupported featureted.
void usbsrHookClearFeature(void);
/// Hook which is called when a \ref SET_FEATURE request refers to a an unsupported feature.
void usbsrHookSetFeature(void);
/// Hook for modifying a \ref GET_STATUS request before the status value is returned to the PC.
void usbsrHookModifyGetStatus(uint8 recipient, uint8 index, uint16 __xdata *pStatus);
/// Hook which is called upon a standard request generated event (unsupported).
void usbsrHookProcessEvent(uint8 event, uint8 index);
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Handled Standard Requests
//@{
void usbsrGetStatus(void);
void usbsrClearFeature(void);
void usbsrSetFeature(void);
void usbsrSetAddress(void);
void usbsrGetDescriptor(void);
void usbsrGetConfiguration(void);
void usbsrSetConfiguration(void);
void usbsrGetInterface(void);
void usbsrSetInterface(void);
//@}
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
