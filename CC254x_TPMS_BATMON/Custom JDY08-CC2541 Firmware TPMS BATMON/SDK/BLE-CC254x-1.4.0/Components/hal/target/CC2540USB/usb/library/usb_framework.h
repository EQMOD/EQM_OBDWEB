/***********************************************************************************

    Filename:     usb_framework.h

    Description:  USB library common functionality.

***********************************************************************************/

#ifndef USBFRAMEWORK_H
#define USBFRAMEWORK_H
/** \addtogroup module_usb_framework USB Framework (usbfw)
 * \brief This module contains USB status information, functions for initialization, USB device reset
 * handling, and most importantly, the framework for transfers on endpoint 0, FIFO access, and endpoint
 * control.
 *
 * \section section_init Framework Initialization
 * The framework and the USB peripheral unit can be initialized once the crystal oscillator (48 MHz for
 * CC1111/CC2511, 32 MHz for CC2531) is running. This is done by \ref usbfwInit(), which:
 * \li Initializes framework variables
 * \li Enables the USB peripheral unit by setting the \c SLEEP.USB_EN bit
 * \li Enables the pull-up resistor via a GPIO pin
 *
 * When completing the last step, the host will be recognize the USB device, and start the enumeration
 * process. To reply to the shortly incoming standard requests, the call to \ref usbfwInit() must be
 * followed immediately by USB Interrupt \ref section_usbirq_initialization.
 *
 * \section section_endpoint_0_transfers Endpoint 0 Transfers
 * The USB interface uses endpoint 0 to perform setup requests of the standard, vendor and class types.
 * Such transfers consist of three phases:
 * - A setup phase, where an 8-byte \ref USB_SETUP_HEADER is transferred to the device.
 * - An IN/OUT data phase, if the length field of the \ref USB_SETUP_HEADER structure is non-zero.
 * - A handshake phase, where the application can stall the endpoint to indicate error conditions.
 *
 * The setup handler, \ref usbfwSetupHandler(), takes care of the low-level parts of these transfers,
 * including the IN/OUT buffering during the data phase (when there is one). The transfers fall into two
 * categories:
 * - Most standard requests are processed internally, with little or no intervention form the user
 *   application. This is done by the \ref module_usb_standard_requests module.
 * - Vendor and class requests are application specific and must always be processed by the application.
 *   Whenever such a request is received, the following hooks will be called between the phases:
 *     - \ref usbcrHookProcessOut(): Class requests with OUT data phase
 *     - \ref usbcrHookProcessIn(): Class requests with IN data phase
 *     - \ref usbvrHookProcessOut(): Vendor requests with OUT data phase
 *     - \ref usbvrHookProcessIn(): Vendor requests with IN data phase
 *
 * \section section_setup_handler_usage Setup Handler Usage
 * This section describes what is required to make the vendor and class request hooks work. This
 * information also applies to the standard requests that needs application processing
 * (\ref usbsrHookSetDescriptor(), \ref usbsrHookSynchFrame(), and some cases of
 * \ref usbsrHookSetFeature() and \ref usbsrHookClearFeature()).
 *
 * The transactions are made using a clean and efficient interface, consisting of two data structures and
 * the endpoint 0 status byte:
 * - The endpoint status is initially \ref EP_IDLE, which basically means that the setup handler is ready
 *   for a new setup phase (a new request). Upon an incoming request, the processing hook is called, and
 *   the \ref usbSetupHeader structure contains the 8 bytes received during the setup phase. At this
 *   point there are four different outcomes:
 *     - If the request is unknown or contains errors, the endpoint should be stalled. This is done by
 *       setting the endpoint status to \ref EP_STALL.
 *     - If there is no data phase (the length field is zero), the endpoint status should just remain in
 *       it's current state, \ref EP_IDLE.
 *     - If the request has an IN data phase, the \ref usbSetupData structure must be prepared. This
 *       includes setting a pointer to where IN data should be taken from, and the number of bytes to be
 *       transferred (usually the same number as indicated by the length field, but it can also be a
 *       lower number). The endpoint state is then changed to \ref EP_TX.
 *     - If the request has an OUT data phase, the \ref usbSetupData structure must be prepared. This
 *       includes setting a pointer to where OUT data should be stored, and the number of bytes to be
 *       transferred (always the same number as indicated by the length field). The endpoint state is
 *       then changed to \ref EP_RX.
 * - When the data phase is complete, the processing hook function will be called a second time to notify
 *   the application. Under normal conditions the endpoint status will be either \ref EP_TX or \ref EP_RX,
 *   and does not need to be changed any further (as this is done automatically upon return). If the
 *   endpoint status is \ref EP_CANCEL, it means that the USB host cancelled the setup transfer.
 *
 * The following code examples illustrate practical usage (more code is available in the application
 * example projects):
 *
 * \par Example 1: Endpoint 0 Requests With OUT Data phase
 *
 * \code
 * uint8 pLcdBuffer[358];
 *
 * void usbvrHookProcessOut(void) {
 *
 *    // When this vendor request is received, we should either update a part of pLcdBuffer[] or leave
 *    // it as it is, and then refresh the LCD display. The index field of the setup header contains the
 *    // index of the first character to be updated, and the length field how many characters to update.
 *    if (usbSetupHeader.request == VR_LCD_UPDATE) {
 *
 *       // First the endpoint status is EP_IDLE... (we have just received the Setup packet)
 *       if (usbfwData.ep0Status == EP_IDLE) {
 *
 *          // There is no new data -> Just refresh the display
 *          if (usbSetupHeader.length == 0) {
 *             lcdRefresh();
 *             // There is no change to the endpoint status in this case
 *
 *          // The PC wants to send data that will be stored outside pLcdBuffer -> stall the endpoint!
 *          } else if ((usbSetupHeader.length > sizeof(pLcdBuffer) ||
 *                     (usbSetupHeader.index >= sizeof(pLcdBuffer) ||
 *                     ((usbSetupHeader.index + usbSetupHeader.length) > sizeof(pLcdBuffer)) {
 *             usbfwData.ep0Status = EP_STALL;
 *
 *          // Prepare for OUT data phase, setup the data buffer to receive the LCD data
 *          } else {
 *             usbSetupData.pBuffer = &pLcdBuffer[usbSetupHeader.index];
 *             usbSetupData.bytesLeft = usbSetupHeader.length;
 *             usbfwData.ep0Status = EP_RX;
 *          }
 *
 *       // Then the endpoint status is EP_RX (remember: we did that here when setting up the buffer)
 *       } else if (usbfwData.ep0Status == EP_RX) {
 *          // usbfwSetupHandler() has now updated pLcdBuffer, so all we need to do is refresh the LCD
 *          lcdRefresh();
 *          // usbfwData.ep0Status is automatically reset to EP_IDLE when returning to usbfwSetupHandler()
 *       }
 *
 *    // Unknown vendor request?
 *    } else {
 *       usbfwData.ep0Status = EP_STALL;
 *    }
 * }
 * \endcode
 *
 * \par Example 2: Endpoint 0 Requests With IN Data phase
 *
 * \code
 * uint8 keyBufferPos;
 * BOOL blockKeyboard;
 * char pKeyBuffer[150];
 *
 * void usbvrProcessIn(void) {
 *
 *    // When this vendor request is received, we should send all registered key-strokes, and reset the
 *    // position counter. New keystrokes are blocked during the transfer to avoid overwriting the buffer
 *    // before it has been sent to the host.
 *    if (usbSetupHeader.request == VR_GET_KEYS) {
 *
 *       // First the endpoint status is EP_IDLE...
 *       if (usbfwData.ep0Status == EP_IDLE) {
 *
 *          // Make sure that we do not send more than the PC asked for
 *          if (usbSetupHeader.length < keyBufferPos) {
 *             usbfwData.ep0Status = EP_STALL;
 *
 *          // Otherwise...
 *          } else {
 *             // Block for new keystrokes
 *             blockKeyboard = TRUE;
 *
 *             // Setup the buffer
 *             usbSetupData.pBuffer = pKeyBuffer;
 *             usbSetupData.bytesLeft = keyBufferPos;
 *             usbfwData.ep0Status = EP_TX;
 *
 *             // Reset the position counter
 *             keyBufferPos = 0;
 *          }
 *
 *       // Then the endpoint status is EP_TX (remember: we did that here when setting up the buffer)
 *       } else if (usbfwData.ep0Status == EP_TX) {
 *
 *          // pKeyBuffer has now been sent to the host, so new keystrokes can safely be registered
 *          blockKeyboard = FALSE;
 *
 *          // usbfwData.ep0Status is automatically reset to EP_IDLE when returning to usbfwSetupHandler()
 *       }
 *
 *    // Unknown request?
 *    } else {
 *       usbfwData.ep0Status = EP_STALL;
 *    }
 * }
 * \endcode
 *
 * If automated data transfer is not desired, the application should set \c usbfwData.ep0Status to
 * either \ref EP_MANUAL_RX or \ref EP_MANUAL_TX instead of \ref EP_RX or \ref EP_TX, respectively. Until
 * the transfer is completed, the processing hook function (e.g. \ref usbvrHookProcessIn()) will be called
 * at every endpoint 0 interrupt.
 * @{
 */
#include "usb_board_cfg.h"
#include "usb_framework_structs.h"

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
EXTERN USBFW_DATA __xdata usbfwData; ///< USBFW internal module data

//@}
//-------------------------------------------------------------------------------------------------------


/// \name Setup Handler Data
//@{
EXTERN USB_SETUP_DATA __xdata usbSetupData; ///< Setup handler data phase configuration
EXTERN USB_SETUP_HEADER __xdata usbSetupHeader; ///< Setup header
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Request Type Fields
//@{

// Field masks
#define RT_MASK_DIR       0x80  ///< Request direction bit mask
#define RT_MASK_TYPE      0x60  ///< Request type bit mask
#define RT_MASK_RECIP     0x1F  ///< Request recipient bit mask

// Direction field
#define RT_DIR_IN         0x80  ///< IN Request
#define RT_DIR_OUT        0x00  ///< OUT Request

// Type field
#define RT_TYPE_STD       0x00  ///< Standard Request
#define RT_TYPE_CLASS     0x20  ///< Class Request
#define RT_TYPE_VEND      0x40  ///< Vendor Request

// Recipient field
#define RT_RECIP_DEV      0x00  ///< Device Request
#define RT_RECIP_IF       0x01  ///< Interface Request
#define RT_RECIP_EP       0x02  ///< Endpoint Request
#define RT_RECIP_OTHER    0x03  ///< Other Request

// Type + direction
#define RT_STD_OUT        (RT_TYPE_STD | RT_DIR_OUT)    ///< Standard request, direction is OUT
#define RT_STD_IN         (RT_TYPE_STD | RT_DIR_IN)     ///< Standard request, direction is IN
#define RT_VEND_OUT       (RT_TYPE_VEND | RT_DIR_OUT)   ///< Vendor request, direction is OUT
#define RT_VEND_IN        (RT_TYPE_VEND | RT_DIR_IN)    ///< Vendor request, direction is IN
#define RT_CLASS_OUT      (RT_TYPE_CLASS | RT_DIR_OUT)  ///< Class request, direction is OUT
#define RT_CLASS_IN       (RT_TYPE_CLASS | RT_DIR_IN)   ///< Class request, direction is IN

// Direction + recepient
#define RT_OUT_DEVICE     (RT_DIR_OUT | RT_RECIP_DEV)   ///< Request made to device, direction is OUT
#define RT_IN_DEVICE      (RT_DIR_IN | RT_RECIP_DEV)    ///< Request made to device, direction is IN
#define RT_OUT_INTERFACE  (RT_DIR_OUT | RT_RECIP_IF)    ///< Request made to interface, direction is OUT
#define RT_IN_INTERFACE   (RT_DIR_IN | RT_RECIP_IF)     ///< Request made to interface, direction is IN
#define RT_OUT_ENDPOINT   (RT_DIR_OUT | RT_RECIP_EP)    ///< Request made to endpoint, direction is OUT
#define RT_IN_ENDPOINT    (RT_DIR_IN | RT_RECIP_EP)     ///< Request made to endpoint, direction is IN
//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Vendor and Class Request Hooks
/// Unused hooks must stall endpoint 0.
//@{

/// Hook which is called upon reception of a class request with OUT data phase
void usbcrHookProcessOut(void);
/// Hook which is called upon reception of a class request with IN data phase
void usbcrHookProcessIn(void);
/// Hook which is called upon reception of a vendor request with OUT data phase
void usbvrHookProcessOut(void);
/// Hook which is called upon reception of a vendor request with IN data phase
void usbvrHookProcessIn(void);

//@}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
/// \name Endpoint Access Macros
/// Note that the endpoint control registers are indexed, meaning that an endpoint must be selected
/// before the control operations listed below can be used. Interrupts using any of these macros, must
/// save the current selection and restore it upon return.
//@{

/// Selects which IN/OUT endpoint (by index 0 to 5) to operate on
#define USBFW_SELECT_ENDPOINT(n)            (USBINDEX = (n))
/// Gets the currently selected IN/OUT endpoint
#define USBFW_GET_SELECTED_ENDPOINT()       (USBINDEX)

/// Stalls the selected IN endpoint
#define USBFW_STALL_IN_ENDPOINT() st (\
    USBCSIL = USBCSIL_SEND_STALL; \
    usbfwData.pEpInStatus[USBINDEX - 1] = EP_HALT; )

/// Returns the stall condition for the selected IN endpoint
#define USBFW_IN_ENDPOINT_STALLED()         (USBCSIL & USBCSIL_SEND_STALL)
/// Flushes the FIFO for the selected IN endpoint (flush twice when using double-buffering)
#define USBFW_FLUSH_IN_ENDPOINT() st (\
    USBCSIL = USBCSIL_FLUSH_PACKET; \
    while (USBCSIL & USBCSIL_FLUSH_PACKET); )

/// Arms the selected IN endpoint, so that contents of the endpoint FIFO can be sent to the host
#define USBFW_ARM_IN_ENDPOINT()             (USBCSIL = USBCSIL_INPKT_RDY)
/// Is the selected IN endpoint disarmed?
#define USBFW_IN_ENDPOINT_DISARMED()        !(USBCSIL & USBCSIL_INPKT_RDY)
/// Is the FIFO for the selected IN endpoint empty?
#define USBFW_IN_ENDPOINT_FIFO_EMPTY()      !(USBCSIL & USBCSIL_PKT_PRESENT)

/// Stalls the selected OUT endpoint
#define USBFW_STALL_OUT_ENDPOINT() st ( \
    USBCSOL = USBCSOL_SEND_STALL; \
    usbfwData.pEpOutStatus[USBINDEX - 1] = EP_HALT; \
}

/// Returns the stall condition for the selected OUT endpoint
#define USBFW_OUT_ENDPOINT_STALLED()        (USBCSOL & USBCSOL_SEND_STALL)
/// Flushes the FIFO for the selected OUT endpoint (flush twice when using double-buffering)
#define USBFW_FLUSH_OUT_ENDPOINT() st(\
    USBCSOL = USBCSOL_FLUSH_PACKET; \
    while (USBCSOL & USBCSOL_FLUSH_PACKET); )

/// Arms the selected OUT endpoint, so that the FIFO can receive data from the host
#define USBFW_ARM_OUT_ENDPOINT()            (USBCSOL = 0)
/// Is the selected OUT endpoint disarmed? If so, there is data waiting in the FIFO
#define USBFW_OUT_ENDPOINT_DISARMED()       (USBCSOL & USBCSOL_OUTPKT_RDY)
/// Returns the number of bytes currently in the FIFO of the selected OUT endpoint, low byte
#define USBFW_GET_OUT_ENDPOINT_COUNT_LOW()  (USBCNTL)
/// Returns the number of bytes currently in the FIFO of the selected OUT endpoint, high byte
#define USBFW_GET_OUT_ENDPOINT_COUNT_HIGH() (USBCNTH)

//@}
//-------------------------------------------------------------------------------------------------------

// Little endian
#define LOBYTEPTR(w)  ( (uint8 __generic *)(&(w)) + 1 )
// Big endian
//#define LOBYTEPTR(w)  ( (uint8 __generic *)&(w) )



//-------------------------------------------------------------------------------------------------------
// Function prototypes
void usbfwInit(void);
void usbfwResetHandler(void);
void usbfwSetupHandler(void);
void usbfwSetAllEpStatus(EP_STATUS status);
void usbfwWriteFifo(uint8 volatile __xdata *pFifo, uint8 count, void __generic *pData);
void usbfwReadFifo(uint8 volatile __xdata *pFifo, uint8 count, void __generic *pData);
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
