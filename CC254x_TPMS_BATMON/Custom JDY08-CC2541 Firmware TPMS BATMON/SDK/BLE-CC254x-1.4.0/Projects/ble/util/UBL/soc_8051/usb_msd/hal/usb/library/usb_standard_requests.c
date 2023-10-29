/***********************************************************************************

    Filename:     usb_standard_request.c

    Description:  Handle USB standard requests.

***********************************************************************************/

/// \addtogroup module_usb_standard_requests
/// @{
#include "usb_firmware_library_headers.h"
#include "hal_types.h"
#include "hal_board.h"



/** \brief Processes the \ref GET_STATUS request (returns status for the specified recipient)
 *
 * The recipient bits in \ref USB_SETUP_HEADER.requestType specify the desired recipient. This is either the
 * (one and only) device, a specific interface, or a specific endpoint. Some of the status bits can be
 * changed with the SET_FEATURE and CLEAR_FEATURE requests.
 *
 * <b>Parameters</b>:
 * - VALUE: Always 0
 * - INDEX: Depends upon the recipient:
 *     - DEVICE: Always 0
 *     - INTERFACE: Interface number
 *     - ENDPOINT: Endpoint address
 * - LENGTH: Always 2
 *
 * <b>Data (IN)</b>:
 * Depends upon the recipient (the bit field illustrations are MSB first, LSB last):
 * - DEVICE: <tt>00000000.000000RS</tt>, where R(1) = DEVICE_REMOTE_WAKEUP and S(0) = SELF_POWERED
 * - INTERFACE: <tt>00000000.00000000</tt> (all bits are reserved)
 * - ENDPOINT: <tt>00000000.0000000H</tt>, where H(0) = ENDPOINT_HALT
 */
void usbsrGetStatus(void)
{
   uint8 endpoint;
   static uint16 __xdata status;

   // Common sanity check
   if (usbSetupHeader.value || HI_UINT16(usbSetupHeader.index) || (usbSetupHeader.length != 2)) {
      usbfwData.ep0Status = EP_STALL;

   // Return status for device, interface, or endpoint
   } else {
      switch (usbSetupHeader.requestType) {

         // Device status:
         //     Bit 0: Self powered
         //     Bit 1: Remote wake-up allowed
      case RT_IN_DEVICE:

         // Sanity check
         if (LO_UINT16(usbSetupHeader.index)) {
            usbfwData.ep0Status = EP_STALL;

         // Get the bit values from the USBFW_DATA struct
         } else {

            // Self powered?
            status = usbfwData.selfPowered ? 0x0001 : 0x0000;

            // Remote wakeup?
            if (usbfwData.remoteWakeup) status |= 0x0002;
         }
         break;

         // Interface status:
         //     All bits are reserved
      case RT_IN_INTERFACE:

         // Sanity check
         if (usbfwData.usbState != DEV_CONFIGURED) {
            usbfwData.ep0Status = EP_STALL;
         } else {
            status = 0x0000;
         }
         break;

         // Endpoint status:
         //     Bit 0: Endpoint halted
      case RT_IN_ENDPOINT:
         endpoint = LO_UINT16(usbSetupHeader.index) & 0x7F;

         // Sanity check
         if ((usbfwData.usbState != DEV_CONFIGURED) || (endpoint > 5)) {
            usbfwData.ep0Status = EP_STALL;

         // Translate endpoint address to status index and return the status
         } else {

            // IN
            if (LO_UINT16(usbSetupHeader.index) & 0x80) {
               status = (usbfwData.pEpInStatus[endpoint - 1] == EP_HALT) ? 0x0001 : 0x0000;

            // OUT
            } else {
               status = (usbfwData.pEpOutStatus[endpoint - 1] == EP_HALT) ? 0x0001 : 0x0000;
            }
         }
         break;

      default:
         usbfwData.ep0Status = EP_STALL;
         break;
      }

      if (usbfwData.ep0Status != EP_STALL) {
         // Send it
         usbSetupData.pBuffer = (uint8 __generic *)&status;
         usbSetupData.bytesLeft = 2;
         usbfwData.ep0Status = EP_TX;
      }
   }
} // usbsrGetStatus




/** \brief Internal function used for the very similar \ref SET_FEATURE and \ref CLEAR_FEATURE requests
 *
 * This function either sets or clears the specified feature on the specified recipient.
 *
 * \param[in]       set
 *     When TRUE, the feature is set. When FALSE, the feature is cleared.
 *
 * \return
 *     TRUE if the selected feature is supported by the USB library. FALSE to indicate that
 *     \ref usbsrHookClearFeature() or \ref usbsrHookSetFeature() must be called.
 */
static uint8 ChangeFeature(uint8 set)
{
   uint8 endpoint;

   // Sanity check
   if (usbSetupHeader.length || (usbfwData.usbState != DEV_CONFIGURED) && (usbSetupHeader.index != 0)) {
      usbfwData.ep0Status = EP_STALL;

      // Handle based on recipient
   } else {
      switch (usbSetupHeader.requestType & RT_MASK_RECIP) {

      // Device
      case RT_RECIP_DEV:

         // Sanity check
         if (LO_UINT16(usbSetupHeader.value) != DEVICE_REMOTE_WAKEUP) {
            return FALSE;
         } else {
            usbfwData.remoteWakeup = set;
            usbsrHookProcessEvent(set ? USBSR_EVENT_REMOTE_WAKEUP_ENABLED : USBSR_EVENT_REMOTE_WAKEUP_DISABLED, 0);
         }
         break;

      // Endpoint
      case RT_RECIP_IF:
         return FALSE;

      // Endpoint
      case RT_RECIP_EP:
         endpoint = LO_UINT16(usbSetupHeader.index) & 0x7F;

         // Sanity check
         if (LO_UINT16(usbSetupHeader.value) != ENDPOINT_HALT) {
            return FALSE;
         } else if (endpoint > 5) {
            usbfwData.ep0Status = EP_STALL;
         } else {
            USBFW_SELECT_ENDPOINT(endpoint);

            // IN
            if (LO_UINT16(usbSetupHeader.index) & 0x80) {
               USBCSIL = set ? USBCSIL_SEND_STALL : USBCSIL_CLR_DATA_TOG;
               usbfwData.pEpInStatus[endpoint - 1] = set ? EP_HALT : EP_IDLE;
               usbsrHookProcessEvent(set ? USBSR_EVENT_EPIN_STALL_SET : USBSR_EVENT_EPIN_STALL_CLEARED, endpoint);

            // OUT
            } else {
               USBCSOL = set ? USBCSOL_SEND_STALL : USBCSOL_CLR_DATA_TOG;
               usbfwData.pEpOutStatus[endpoint - 1] = set ? EP_HALT : EP_IDLE;
               usbsrHookProcessEvent(set ? USBSR_EVENT_EPOUT_STALL_SET : USBSR_EVENT_EPOUT_STALL_CLEARED, endpoint);
            }
            USBFW_SELECT_ENDPOINT(0);
         }
         break;

      default:
         usbfwData.ep0Status = EP_STALL;
         break;
      }
   }
   return TRUE;
} // ChangeFeature




/** \brief Processes the \ref CLEAR_FEATURE request (clears or disables a specific feature)
 *
 * The feature selector value must be appropriate to the recipient.
 *
 * <b>Parameters</b>:
 * - VALUE: Feature selector:
 *     - \c DEVICE_REMOTE_WAKEUP(1): Enable remote wakeup
 *     - \c ENDPOINT_HALT(0): Clear the halt feature for the specified endpoint (not endpoint 0!)
 * - INDEX: Depends upon the recipient:
 *     - DEVICE: Always 0
 *     - INTERFACE: Interface number
 *     - ENDPOINT: Endpoint address
 * - LENGTH: Always 0
 */
void usbsrClearFeature()
{
   if (!ChangeFeature(FALSE)) {
      usbsrHookClearFeature();
   }
} // usbsrClearFeature




/** \brief Processes the \ref SET_FEATURE request (sets or enables a specific feature)
 *
 * The feature selector value must be appropriate to the recipient.
 *
 * <b>Parameters</b>:
 * - VALUE: Feature selector:
 *     - \c DEVICE_REMOTE_WAKEUP(1): Enable remote wakeup
 *     - \c ENDPOINT_HALT(0): Set the halt feature for the specified endpoint (not endpoint 0!)
 * - INDEX: Depends upon the recipient:
 *     - DEVICE: Always 0
 *     - INTERFACE: Interface number
 *     - ENDPOINT: Endpoint address
 * - LENGTH: Always 0
 */
void usbsrSetFeature(void)
{
   if (!ChangeFeature(TRUE)) {
      usbsrHookSetFeature();
   }
} // usbsrSetFeature




/** \brief Processes the \ref SET_ADDRESS request (sets the device address for all future device
 * accesses)
 *
 * If the value is between 1 and 127 and the device is in the default state, it will enter the address
 * state. If it already is in the address state, it starts to use the newly-specified address.
 *
 * If the value is 0 and the device is in the address state, it will enter the default state. If it
 * already is in the default state, nothing happens.
 *
 * <b>Parameters</b>:
 * - VALUE: The device address (0-127)
 * - INDEX: Always 0
 * - LENGTH: Always 0
 */
void usbsrSetAddress(void)
{

   // Sanity check
   if (usbSetupHeader.index || usbSetupHeader.length || HI_UINT16(usbSetupHeader.value) || (LO_UINT16(usbSetupHeader.value) & 0x80)) {
      usbfwData.ep0Status = EP_STALL;

   // Update the device address
   } else {
      USBADDR = LO_UINT16(usbSetupHeader.value);
      if (LO_UINT16(usbSetupHeader.value) != 0) {
         if (usbfwData.usbState == DEV_DEFAULT) usbfwData.usbState = DEV_ADDRESS;
      } else {
         if (usbfwData.usbState == DEV_ADDRESS) usbfwData.usbState = DEV_DEFAULT;
      }
   }

} // usbsrSetAddress




/** \brief Processes the \ref GET_DESCRIPTOR request (returns the specified USB descriptor)
 *
 * The \ref module_usb_descriptor_parser module is used to locate device, configuration and string
 * descriptors. Note that configuration descriptors also include interface, endpoint and other
 * "similar" descriptor types (e.g. HID descriptor), with the total descriptor length specified by
 * the \ref USB_CONFIGURATION_DESCRIPTOR.wTotalLength field.
 *
 * Other descriptor types that are not returned with the configuration descriptor, must be defined in
 * the usbDescriptorMarker.pUsbDescLut lookup-table. This table specifies the values of the VALUE and INDEX fields, and
 * gives a pointer to the descriptor along with it's length.
 *
 * <b>Parameters</b>:
 * - VALUE.MSB: Descriptor type
 * - VALUE.LSB: Descriptor index
 * - INDEX: 0, or language ID for string descriptors (currently not supported)
 * - LENGTH: Descriptor length (either the requested number of bytes, or the length of the descriptor,
 *           whichever is the smallest)
 *
 * <b>Data (IN)</b>:
 * The descriptor(s)
 */
void usbsrGetDescriptor(void)
{
   uint8 n;

   // Which descriptor?
   switch (HI_UINT16(usbSetupHeader.value)) {

   // Device descriptor
   case DESC_TYPE_DEVICE:
      usbSetupData.pBuffer = (uint8 __code*) usbdpGetDeviceDesc();
      usbSetupData.bytesLeft = usbSetupData.pBuffer[DESC_LENGTH_IDX];
      break;

   // Configuration descriptor
   case DESC_TYPE_CONFIG:
      usbSetupData.pBuffer = (uint8 __code*) usbdpGetConfigurationDesc(0, LO_UINT16(usbSetupHeader.value));
      usbSetupData.bytesLeft = usbSetupData.pBuffer[DESC_CONFIG_LENGTH_LSB_IDX] +
                               usbSetupData.pBuffer[DESC_CONFIG_LENGTH_MSB_IDX] * 256;
      break;

   // String descriptor
   case DESC_TYPE_STRING:
      // TODO: Implement language ID
      usbSetupData.pBuffer = (uint8 *)usbdpGetStringDesc(LO_UINT16(usbSetupHeader.value));
      usbSetupData.bytesLeft = usbSetupData.pBuffer[DESC_LENGTH_IDX];
      break;

   // Other descriptor type
   default:
      // Perform a table search (on index and value)
      usbSetupData.pBuffer = NULL;
      for (n = 0; n < ((uint16)usbDescriptorMarker.pUsbDescLutEnd - (uint16)usbDescriptorMarker.pUsbDescLut) / sizeof(DESC_LUT_INFO); n++) {
         if ((usbDescriptorMarker.pUsbDescLut[n].valueMsb == HI_UINT16(usbSetupHeader.value))
             && (usbDescriptorMarker.pUsbDescLut[n].valueLsb == LO_UINT16(usbSetupHeader.value))
             && (usbDescriptorMarker.pUsbDescLut[n].indexMsb == HI_UINT16(usbSetupHeader.index))
             && (usbDescriptorMarker.pUsbDescLut[n].indexLsb == LO_UINT16(usbSetupHeader.index)) )
         {
            usbSetupData.pBuffer = usbDescriptorMarker.pUsbDescLut[n].pDescStart;
            usbSetupData.bytesLeft = usbDescriptorMarker.pUsbDescLut[n].length;
         }
      }
   }

   // Stall EP0 if no descriptor was found
   if (usbSetupData.pBuffer == NULL) usbfwData.ep0Status = EP_STALL;

   if (usbfwData.ep0Status != EP_STALL) {

      // Limit the returned descriptor size (the PC wants to know about sizes before
      // polling the complete descriptors)
      if (usbSetupData.bytesLeft > usbSetupHeader.length) {
         usbSetupData.bytesLeft = usbSetupHeader.length;
      }

      usbfwData.ep0Status = EP_TX;
   }

} // usbsrGetDescriptor



/** \brief Internally used function that configures all endpoints for the specified interface
 *
 * The new endpoint setup overwrites the old, without any warning. Unused endpoints keep their current
 * setup. The user is responsible for ensuring that no endpoint buffers overwrite each other, and that
 * interfaces do not cause conflicts. The pUsbDblbufLutInfo table must contain an entry for each
 * interface descriptor to define endpoint double-buffering.
 *
 * \param[in]       *pInterface
 *     A pointer to the interface descriptor
 */
static void ConfigureEndpoints(USB_INTERFACE_DESCRIPTOR __code *pInterface)
{
   uint8 n;
   uint16 maxpRegValue;
   uint8 csRegValue;
   uint8 endpoint;
   USB_ENDPOINT_DESCRIPTOR __code *pEndpoint;
   DBLBUF_LUT_INFO __code *pUsbDblbufLutInfo;

   // Locate the double buffer settings
   if (pInterface->bNumEndpoints) {
       pUsbDblbufLutInfo = (DBLBUF_LUT_INFO __code*) usbDescriptorMarker.pUsbDblbufLut;
       while (pUsbDblbufLutInfo->pInterface != pInterface) {
          pUsbDblbufLutInfo++;
       }
   }

   // For each endpoint in this interface
   for (n = 0; n < pInterface->bNumEndpoints; n++) {
      if (pEndpoint = usbdpFindNext(DESC_TYPE_ENDPOINT, 0)) {

         // Get the endpoint index
         endpoint = pEndpoint->bEndpointAddress & 0x0F;
         USBFW_SELECT_ENDPOINT(endpoint);

         csRegValue = 0x00;
         maxpRegValue = (pEndpoint->wMaxPacketSize + 7) >> 3;

         // For IN endpoints...
         if (pEndpoint->bEndpointAddress & 0x80) {

            // Clear data toggle, and flush twice (due to double buffering)
            USBCSIL = USBCSIL_CLR_DATA_TOG | USBCSIL_FLUSH_PACKET;
            USBCSIL = USBCSIL_FLUSH_PACKET;

            // USBCSIH
            if ((pEndpoint->bmAttributes & EP_ATTR_TYPE_BM) == EP_ATTR_ISO) csRegValue |= USBCSIH_ISO;  // ISO flag
            if (pUsbDblbufLutInfo->inMask & (1 << endpoint)) csRegValue |= USBCSIH_IN_DBL_BUF;          // Double buffering
            USBCSIH = csRegValue;

            // Max transfer size
            USBMAXI = maxpRegValue;

            // Endpoint status
            usbfwData.pEpInStatus[endpoint - 1] = EP_IDLE;

         // For OUT endpoints...
         } else {

            // Clear data toggle, and flush twice (due to double buffering)
            USBCSOL = USBCSOL_CLR_DATA_TOG | USBCSOL_FLUSH_PACKET;
            USBCSOL = USBCSOL_FLUSH_PACKET;

            // USBCSOH
            if ((pEndpoint->bmAttributes & EP_ATTR_TYPE_BM) == EP_ATTR_ISO) csRegValue |= USBCSOH_ISO;  // ISO flag
            if (pUsbDblbufLutInfo->outMask & (1 << endpoint)) csRegValue |= USBCSOH_OUT_DBL_BUF;        // Double buffering
            USBCSOH = csRegValue;

            // Max transfer size
            USBMAXO = maxpRegValue;

            // Endpoint status
            usbfwData.pEpOutStatus[endpoint - 1] = EP_IDLE;
         }
         USBFW_SELECT_ENDPOINT(0);
      }
   }
} // ConfigureEndpoints




/** \brief Processes the \ref GET_CONFIGURATION request (returns the current device configuration value)
 *
 * If the returned value is 0, the device is not configured (not in the configured state)
 *
 * <b>Parameters</b>:
 * - VALUE: Always 0
 * - INDEX: Always 0
 * - LENGTH: Always 1
 *
 * <b>Data (IN)</b>:
 * The non-zero \ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue of the currently selected
 * configuration.
 */
void usbsrGetConfiguration(void)
{

   // Sanity check
   if (usbSetupHeader.value || usbSetupHeader.index || (usbSetupHeader.length != 1)) {
      usbfwData.ep0Status = EP_STALL;

   // Return the current configuration
   } else {
      usbSetupData.pBuffer = &usbfwData.configurationValue;
      usbSetupData.bytesLeft = 1;
      usbfwData.ep0Status = EP_TX;
   }

} // usbsrGetConfiguration




/** \brief Processes the \ref SET_CONFIGURATION request (sets the device configuration)
 *
 * The configuration value must either be 0, in which case the device enters the address state, or it
 * must match a configuration value from one of the USB configuration descriptors. If there is a match,
 * the device enters the configured state.
 *
 * This request resets all interfaces to alternate setting 0, and uses the \c ConfigureEndpoints()
 * function to automatically setup all endpoint registers.
 *
 * <b>Parameters</b>:
 * - VALUE: The configuration value (0-255)
 * - INDEX: Always 0
 * - LENGTH: Always 0
 */
void usbsrSetConfiguration(void)
{
   uint8 n;
   USB_CONFIGURATION_DESCRIPTOR __code *pConfiguration;
   USB_INTERFACE_DESCRIPTOR __code *pInterface;

   // Sanity check
   if ((usbfwData.usbState == DEV_DEFAULT) || usbSetupHeader.index || usbSetupHeader.length || HI_UINT16(usbSetupHeader.value)) {
      usbfwData.ep0Status = EP_STALL;

   // Default endpoint setup
   } else {
      usbsrHookProcessEvent(USBSR_EVENT_CONFIGURATION_CHANGING, 0);

      // Configure relevant endpoints
      if (LO_UINT16(usbSetupHeader.value)) {

         // Find the correct configuration descriptor...
         pConfiguration = usbdpGetConfigurationDesc(LO_UINT16(usbSetupHeader.value), 0);

         // If it exists...
         if (pConfiguration) {
            usbfwData.usbState = DEV_CONFIGURED;
            usbfwData.configurationValue = LO_UINT16(usbSetupHeader.value);

            // For each interface...
            for (n = 0; n < pConfiguration->bNumInterfaces; n++) {
               usbfwData.pAlternateSetting[n] = 0x00;

               // Look only for alternate setting 0
               do {
                  pInterface = usbdpFindNext(DESC_TYPE_INTERFACE, 0);
               } while (pInterface->bAlternateSetting != usbfwData.pAlternateSetting[n]);

               // Configure all endpoints in this interface
               ConfigureEndpoints(pInterface);
            }

         // If not, then stall the endpoint
         } else {
            usbfwData.ep0Status = EP_STALL;
         }

      // Unconfigure endpoints
      } else {
         usbfwData.configurationValue = LO_UINT16(usbSetupHeader.value);
         usbfwData.usbState = DEV_ADDRESS;
         usbfwSetAllEpStatus(EP_HALT);
      }
      usbsrHookProcessEvent(USBSR_EVENT_CONFIGURATION_CHANGED, 0);
   }

} // usbsrSetConfiguration




/** \brief Processes the \ref GET_INTERFACE request (returns the selected alternate setting for the
 * specified interface)
 *
 * Some USB devices have configurations with mutually exclusive interface settings. This request allows
 * the host to determine the currently selected alternate setting.
 *
 * <b>Parameters</b>:
 * - VALUE: Always 0
 * - INDEX: Interface number
 * - LENGTH: Always 1
 *
 * <b>Data (IN)</b>:
 * The alternate setting for the selected interface
 */
void usbsrGetInterface(void)
{

   // Sanity check
   if ((usbfwData.usbState != DEV_CONFIGURED) || (usbSetupHeader.requestType != RT_IN_INTERFACE) || usbSetupHeader.value || (usbSetupHeader.length != 1)) {
      usbfwData.ep0Status = EP_STALL;

   // Return the current alternate setting
   } else {
      usbSetupData.pBuffer = &usbfwData.pAlternateSetting[usbSetupHeader.index];
      usbSetupData.bytesLeft = 1;
      usbfwData.ep0Status = EP_TX;
   }

} // usbsrGetInterface




/** \brief Processes the \ref SET_INTERFACE request (selects an alternate setting for the specified
 * interface)
 *
 * Some USB devices have configurations with mutually exclusive interface settings. This request allows
 * the host to select the desired alternate setting.
 *
 * This function uses the \c ConfigureEndpoints() to automatically setup the relevant endpoint
 * registers.
 *
 * <b>Parameters</b>:
 * - VALUE: Alternate setting
 * - INDEX: Interface number
 * - LENGTH: Always 0
 */
void usbsrSetInterface(void)
{
   USB_INTERFACE_DESCRIPTOR __code *pInterface;

   // Sanity check
   if ((usbfwData.usbState != DEV_CONFIGURED) || (usbSetupHeader.requestType != RT_OUT_INTERFACE) || usbSetupHeader.length) {
      usbfwData.ep0Status = EP_STALL;

   // Verify that the desired alternate setting is available, and then make the switch
   } else {
      if (pInterface = usbdpGetInterfaceDesc(usbfwData.configurationValue, usbSetupHeader.index, usbSetupHeader.value)) {
         usbsrHookProcessEvent(USBSR_EVENT_INTERFACE_CHANGING, usbSetupHeader.index);
         usbfwData.pAlternateSetting[usbSetupHeader.index] = usbSetupHeader.value;

         // Configure all endpoints in this interface
         ConfigureEndpoints(pInterface);
         usbsrHookProcessEvent(USBSR_EVENT_INTERFACE_CHANGED, usbSetupHeader.index);

      // This interface does not exist
      } else {
         usbfwData.ep0Status = EP_STALL;
      }
   }

} // usbsrSetInterface

//@}

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
