/***********************************************************************************

    Filename:     usb_framework.c

    Description:  USB library common functionality.

***********************************************************************************/

/// \addtogroup module_usb_framework
/// @{
#define USBFRAMEWORK_C ///< Modifies the behavior of "EXTERN" in usb_framework.h
#include "usb_firmware_library_headers.h"
#include "usb_board_cfg.h"

// Function pointer used by usbfwSetupHandler()
static VFPTR __data ProcessFunc;

/** \brief Initializes the USB framework
 *
 * This function should be called when the microcontroller is ready to accept USB traffic. It enables the
 * USB peripheral unit and enables the pull-up resistor on the D+ line. Endpoint status, current
 * configuration value, etc. are initialized and evenetually re-initialized in the
 * \ref usbfwResetHandler() function.
 */
void usbfwInit(void)
{
    // Set default values
    usbfwData.selfPowered = (usbdpGetConfigurationDesc(1, 0)->bmAttributes & 0x40) ? TRUE : FALSE;
    usbfwData.remoteWakeup = FALSE;

    HAL_USB_ENABLE();

    // Enable Resume Interrupt
    HAL_USB_RESUME_INT_ENABLE();

} // usbfwInit




/** \brief Handles USB reset signalling
 *
 * This function should be called, either from the USB interrupt or the main loop, when the \c USBCIF.RST
 * flag has been set. Keep in mind that all bits in \c USBCIF register are cleared when the register is
 * read. The function puts the device into the default state (not yet addressed), and puts all endpoints
 * (except EP0) into the \ref EP_HALT state
 */
void usbfwResetHandler(void)
{

   // Reset the USB state
   usbfwData.usbState = DEV_DEFAULT;
   usbfwData.configurationValue = 0;

   // Reset all endpoints
   usbfwData.ep0Status = EP_IDLE;
   usbfwSetAllEpStatus(EP_HALT);

   // Reset last function pointer
   ProcessFunc = NULL;

} // usbfwResetHandler




/** \brief USB Setup Handler
 *
 * This function should be called either from the USB interrupt or the main loop when the \c USBIIF.EP0IF
 * flag has been set. Keep in mind that all bits in \c USBIIF register are cleared when the register is
 * read. A detailed description of the framework is found in the \ref section_setup_handler_usage
 * section.
 *
 * \note The USB header data is always little-endian, so if a big-endian compiler is used (such as Keil
 * C51), the 16-bit values in the \ref usbSetupHeader must be flipped before they are used.
 */
void usbfwSetupHandler(void)
{
   uint8 controlReg;
   uint8 bytesNow;
   uint8 oldEndpoint;

   // Save the old index setting, then select endpoint 0 and fetch the control register
   oldEndpoint = USBFW_GET_SELECTED_ENDPOINT();
   USBFW_SELECT_ENDPOINT(0);
   controlReg = USBCS0;

   // The last transfer was ended prematurely by a new SETUP packet
   if (controlReg & USBCS0_SETUP_END) {
      USBCS0 = USBCS0_CLR_SETUP_END;
      usbfwData.ep0Status = EP_CANCEL;
      if (ProcessFunc) ProcessFunc();
      usbfwData.ep0Status = EP_IDLE;
   }

   // A STALL handshake was transmitted to the PC
   if (controlReg & USBCS0_SENT_STALL) {
      USBCS0 = 0x00;
      usbfwData.ep0Status = EP_IDLE;
   }

   // Receive OUT packets
   if (usbfwData.ep0Status == EP_RX) {

      // Read FIFO
      bytesNow = USBCNT0;
      usbfwReadFifo(&USBF0, bytesNow, usbSetupData.pBuffer);
      usbSetupData.bytesLeft -= bytesNow;
      usbSetupData.pBuffer += bytesNow;

      // Arm the endpoint
      USBCS0 = usbSetupData.bytesLeft ? USBCS0_CLR_OUTPKT_RDY : (USBCS0_CLR_OUTPKT_RDY | USBCS0_DATA_END);

      // Make a call to the appropriate request handler when done
      if (usbSetupData.bytesLeft == 0) {
         if (ProcessFunc) ProcessFunc();
         usbfwData.ep0Status = EP_IDLE;
      }

      // Return here since nothing more will happen until the next interrupt
      USBFW_SELECT_ENDPOINT(oldEndpoint);
      return;

   // Let the application handle the reception
   } else if (usbfwData.ep0Status == EP_MANUAL_RX) {
      ProcessFunc();
   }

   // Receive SETUP header
   if (usbfwData.ep0Status == EP_IDLE) {
      if (controlReg & USBCS0_OUTPKT_RDY) {
         usbfwReadFifo(&USBF0, 8, (uint8 __xdata *) &usbSetupHeader);

         // Handle control transfers individually
         ProcessFunc = NULL;
         switch (usbSetupHeader.requestType & (RT_MASK_TYPE | RT_MASK_DIR)) {

            // Standard requests with data from the host (OUT)
         case RT_STD_OUT:
            switch (usbSetupHeader.request) {
            case SET_ADDRESS:       usbsrSetAddress(); break;
            case SET_FEATURE:       usbsrSetFeature(); break;
            case CLEAR_FEATURE:     usbsrClearFeature(); break;
            case SET_CONFIGURATION: usbsrSetConfiguration(); break;
            case SET_INTERFACE:     usbsrSetInterface(); break;
            case SET_DESCRIPTOR:    /*usbsrHookSetDescriptor(); break; - unsupported */
            default:                usbfwData.ep0Status = EP_STALL; break;
            }
            break;

            // Standard requests with data to the host (IN)
         case RT_STD_IN:
            switch (usbSetupHeader.request) {
            case GET_STATUS:        usbsrGetStatus(); break;
            case GET_DESCRIPTOR:    usbsrGetDescriptor(); break;
            case GET_CONFIGURATION: usbsrGetConfiguration(); break;
            case GET_INTERFACE:     usbsrGetInterface(); break;
            case SYNCH_FRAME:       /*usbsrHookSynchFrame(); break; - unsupported */
            default:                usbfwData.ep0Status = EP_STALL; break;
            }
            break;

            // Vendor requests
         case RT_VEND_OUT:
            ProcessFunc = usbvrHookProcessOut; usbvrHookProcessOut();
            break;
         case RT_VEND_IN:
            ProcessFunc = usbvrHookProcessIn; usbvrHookProcessIn();
            break;

            // Class requests
         case RT_CLASS_OUT:
            ProcessFunc = usbcrHookProcessOut; usbcrHookProcessOut();
            break;
         case RT_CLASS_IN:
            ProcessFunc = usbcrHookProcessIn; usbcrHookProcessIn();
            break;

            // Unrecognized request: Stall the endpoint
         default:
            usbfwData.ep0Status = EP_STALL;
            break;
         }

         // Arm/stall the endpoint
         USBCS0 = (usbfwData.ep0Status == EP_STALL) ? (USBCS0_CLR_OUTPKT_RDY | USBCS0_SEND_STALL) : USBCS0_CLR_OUTPKT_RDY;
      }
   }

   // Transmit IN packets
   if (usbfwData.ep0Status == EP_TX) {
      controlReg = USBCS0_INPKT_RDY;

      // The last frame should contain 0 to (EP0_PACKET_SIZE - 1) bytes
      if (usbSetupData.bytesLeft < EP0_PACKET_SIZE) {
         bytesNow = usbSetupData.bytesLeft;
         controlReg |= USBCS0_DATA_END;

         // All other packets should have the maximum length
      } else {
         bytesNow = EP0_PACKET_SIZE;
      }

      // Load the FIFO and move the pointer
      usbfwWriteFifo(&USBF0, bytesNow, usbSetupData.pBuffer);
      usbSetupData.pBuffer += bytesNow;
      usbSetupData.bytesLeft -= bytesNow;

      // Arm the FIFO (even for a zero-length packet)
      USBCS0 = controlReg;

      // Make a call to the appropriate request handler when done
      if (bytesNow < EP0_PACKET_SIZE) {
         if (ProcessFunc) ProcessFunc();
         usbfwData.ep0Status = EP_IDLE;
      }

   // Let the application handle the transmission
   } else if (usbfwData.ep0Status == EP_MANUAL_TX) {
      ProcessFunc();
   }

   // Restore the old index setting
   USBFW_SELECT_ENDPOINT(oldEndpoint);

} // usbfwSetupHandler




/** \brief Changes the state of endpoint 1-5 IN/OUT
 *
 * This is an internal function used by the library.
 *
 * \param[in]       status
 *     The new status for each endpoint
 */
void usbfwSetAllEpStatus(EP_STATUS status)
{
   uint8 n;
   for (n = 0; n < sizeof(usbfwData.pEpInStatus); n++)
       usbfwData.pEpInStatus[n] = status;
   for (n = 0; n < sizeof(usbfwData.pEpOutStatus); n++)
       usbfwData.pEpOutStatus[n] = status;
} // usbfwSetAllEpStatus




/** \brief Reads from the selected OUT endpoint FIFO, without using DMA
 *
 * The FIFO must be re-armed after reading it empty (using the \ref USBFW_ARM_OUT_ENDPOINT() macro). This
 * is not necessary when flushing the FIFO.
 *
 * \param[in]       *pFifo
 *     Pointer to the FIFO (\c &USBFx)
 * \param[in]       count
 *     The number of bytes to read
 * \param[in]       *pData
 *     A pointer to the storage location for the read data (in any memory space)
 */
void usbfwReadFifo(uint8 volatile __xdata *pFifo, uint8 count, void __generic *pData)
{
   uint8 __generic *pTemp = pData;
   if (count) {
      do {
         *(pTemp++) = *pFifo;
      } while (--count);
   }
} // usbfwReadFifo




/** \brief Writes to the selected IN endpoint FIFO, without using DMA
 *
 * Note that the FIFO must be armed in order to be transmitted (using the \ref USBFW_ARM_IN_ENDPOINT()
 * macro).
 *
 * \param[in]       *pFifo
 *     Pointer to the FIFO (\c &USBFx)
 * \param[in]       count
 *     The number of bytes to write
 * \param[in]       *pData
 *     A pointer to the data to be written (from any memory space)
 */
void usbfwWriteFifo(uint8 volatile __xdata *pFifo, uint8 count, void __generic *pData)
{
   uint8 __generic *pTemp = pData;
   if (count) {
      do {
         *pFifo = *(pTemp++);
      } while (--count);
   }
} // usbfwWriteFifo


/// @}
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
