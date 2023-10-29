/***********************************************************************************

    Filename:     usb_descriptor_parser.c

    Description:  Parser for USB descriptor structures.

***********************************************************************************/

/// \addtogroup module_usb_descriptor_parser
/// @{
#define USBDESCRIPTORPARSER_C ///< Modifies the behavior of "EXTERN" in usb_descriptor_parser.h
#include "usb_firmware_library_headers.h"

#define BLE_INFO_PAGE_BDADDR       0x780C    // LSB..MSB
#define BLE_INFO_PAGE_BDADDR_LEN   8
#define BLE_INFO_PAGE_WAFER_INFO   0x7FF4

//-------------------------------------------------------------------------------------------------------
// USBDP internal module data
static USBDP_DATA __xdata usbdpData; ///< USBDP internal module data

//-------------------------------------------------------------------------------------------------------
// String descriptors (2-byte unicode data).

// Language ID.
static const uint8 languageId[4] = {
  4,
  DESC_TYPE_STRING,
  0x09, 0x04  /* US-EN */
};

// Manufacturer.
static const uint8 manufacturer[36] = {
  36,
  DESC_TYPE_STRING,
  'T', 0,
  'e', 0,
  'x', 0,
  'a', 0,
  's', 0,
  ' ', 0,
  'I', 0,
  'n', 0,
  's', 0,
  't', 0,
  'r', 0,
  'u', 0,
  'm', 0,
  'e', 0,
  'n', 0,
  't', 0,
  's', 0
};

// Product.
static const uint8 product[36] = {
  36,
  DESC_TYPE_STRING,
  'T', 0,
  'I', 0,
  ' ', 0,
  'C', 0,
  'C', 0,
  '2', 0,
  '5', 0,
  '4', 0,
  '0', 0,
  ' ', 0,
  'U', 0,
  'S', 0,
  'B', 0,
  ' ', 0,
  'C', 0,
  'D', 0,
  'C', 0
};

// Serial Number.
static uint8 serialNumber[42] = {
  0,
  DESC_TYPE_STRING,
  '0', 0,
  '1', 0,
  '2', 0,
  '3', 0,
  '4', 0,
  '5', 0,
  '6', 0,
  '7', 0,
  '8', 0,
  '9', 0,
  '0', 0,
  '1', 0,
  '2', 0,
  '3', 0,
  '4', 0,
  '5', 0,
  '6', 0,
  '7', 0,
  '8', 0,
  '9', 0
};

/** \brief	Initializes a search
*
* This function must be called before each new search to reset \ref USBDP_DATA.pDesc.
*/
void usbdpInit(void)
{
   usbdpData.pDesc = (const uint8 __code *) usbDescriptorMarker.pUsbDescStart;
} // usbdpInit




/** \brief	Locates the descriptor of the wanted type
*
* This function parses through the USB descriptors until:
* \li It hits one with <tt>bDescriptorType = wantedType</tt>, in which case it returns a pointer to
*     that descriptor, and exits. \ref USBDP_DATA.pDesc will then point to the next descriptor.
* \li It hits one with <tt>bDescriptorType = haltAtType</tt>, in which case it returns a NULL-pointer,
*     and exits. \ref USBDP_DATA.pDesc will then point to that descriptor.
* \li \ref USBDP_DATA.pDesc = \ref usbDescEnd, in which case it returns a NULL-pointer, and exits.
*     \ref USBDP_DATA.pDesc will continue to point to \ref usbDescEnd.
*
* \note To begin a search with this function, \ref usbdpInit should be called first. It should not be
*       called when continuing a search - for instance after a call to \ref usbdpGetConfigurationDesc().
*
* \param[in]       wantedType
*     The wanted descriptor type (e.g. \ref DESC_TYPE_CONFIG)
* \param[in]       haltAtType
*     The parser halts when it reaches this descriptor type, unless \c haltAtType is \c 0 (which in any
*     case is an invalid \c bDescriptorType value).
*
* \return
*     A pointer to the wanted descriptor type, or \c NULL if it was not found.
*/
void __code* usbdpFindNext(uint8 wantedType, uint8 haltAtType)
{
   void __code *pResult;
   pResult = NULL;

   // As long as we haven't reached the end...
   while (usbdpData.pDesc != (void __code *) usbDescriptorMarker.pUsbDescEnd) {

      // If we have a match on wantedType...
      if (usbdpData.pDesc[DESC_TYPE_IDX] == wantedType) {
         pResult = (void __code*) usbdpData.pDesc;
         usbdpData.pDesc += usbdpData.pDesc[DESC_LENGTH_IDX];
         break;

      // If we have a match on haltAtType...
      } else if (usbdpData.pDesc[DESC_TYPE_IDX] == haltAtType) {
         if (haltAtType) break;
      }

      // Move on to the next descriptor
      usbdpData.pDesc += usbdpData.pDesc[DESC_LENGTH_IDX];
   }

   return pResult;
} // usbdpFindNext




/** \brief	Locates the (one and only) device descriptor
*
* \note It is not necessary to call \ref usbdpInit() before this function.
*
* \return
*     A pointer to the \ref USB_DEVICE_DESCRIPTOR, or \c NULL if it was not found.
*/
USB_DEVICE_DESCRIPTOR __code* usbdpGetDeviceDesc(void)
{
   usbdpInit();
   return usbdpFindNext(DESC_TYPE_DEVICE, 0);
} // usbdpGetDeviceDesc




/** \brief	Locates a configuration descriptor
*
* The search will either look for a descriptor with a specific
* \ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue, or simply take the n'th descriptor (by "index")
*
* \note It is not necessary to call \ref usbdpInit() before this function.
*
* \param[in]       cfgValue
*     The configuration value to search for (\ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue), or
*     0 to find descriptor by index
* \param[in]       cfgIndex
*     A zero-based index for the configuration descriptor to find.
*     This value is ignored unless \c cfgValue is 0.
*
* \return
*     A pointer to the \ref USB_DEVICE_DESCRIPTOR, or \c NULL if it was not found.
*/
USB_CONFIGURATION_DESCRIPTOR __code* usbdpGetConfigurationDesc(uint8 cfgValue, uint8 cfgIndex)
{
   USB_CONFIGURATION_DESCRIPTOR __code *pConfigurationDesc;
   usbdpInit();

   // As long as there are more configuration descriptors...
   while (pConfigurationDesc = usbdpFindNext(DESC_TYPE_CONFIG, 0)) {

      // Search by value?
      if (cfgValue) {
         if (cfgValue == pConfigurationDesc->bConfigurationValue) break;

      // Search by index? (search cfgIndex+1 times)
      } else if (!cfgIndex--) {
         break;
      }
   }

   return pConfigurationDesc;
} // usbdpGetConfigurationDesc




/** \brief	Locates an interface descriptor
*
* The function will first go to the configuration descriptor that matches the supplied configuration
* value, and then locate the interface descriptor that matches the given interface number and alternate
* setting.
*
* \note It is not necessary to call \ref usbdpInit() before this function.
*
* \param[in]       cfgValue
*     The configuration value (\ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue)
* \param[in]       intNumber
*     The interface number (\ref USB_INTERFACE_DESCRIPTOR.bInterfaceNumber)
* \param[in]       altSetting
*     The alternate setting (\ref USB_INTERFACE_DESCRIPTOR.bAlternateSetting)
*
* \return
*     A pointer to the \ref USB_INTERFACE_DESCRIPTOR, or \c NULL if it was not found.
*/
USB_INTERFACE_DESCRIPTOR __code* usbdpGetInterfaceDesc(uint8 cfgValue, uint8 intNumber, uint8 altSetting)
{
   USB_INTERFACE_DESCRIPTOR __code *pInterfaceDesc;

   // First get to the correct configuration
   usbdpGetConfigurationDesc(cfgValue, 0);

   // Then find a match on the interface
   while (pInterfaceDesc = usbdpFindNext(DESC_TYPE_INTERFACE, DESC_TYPE_CONFIG)) {
      if ((pInterfaceDesc->bInterfaceNumber == intNumber) && (pInterfaceDesc->bAlternateSetting == altSetting)) {
         break;
      }
   }

   return pInterfaceDesc;
} // usbdpGetInterfaceDesc




/** \brief	Locates a string descriptor
*
* \note It is not necessary to call \ref usbdpInit() before this function.
*
* \param[in]       strIndex
*     A zero-based index that matches the "iXxxxxxxxxx" string indexes in the other descriptors
*
* \return
*     A pointer to the \ref USB_INTERFACE_DESCRIPTOR, or \c NULL if it was not found.
*/
USB_STRING_DESCRIPTOR *usbdpGetStringDesc(uint8 strIndex)
{
   USB_STRING_DESCRIPTOR *pStringDesc = NULL;

#ifdef MS_EXT_C_ID
    /* OPT: Find the Microsoft OS String Descriptor?
    usbdpInit();

    if (strIndex == 0xEE){
        // Find the Microsoft OS String Descriptor
        do{
            pStringDesc = usbdpFindNext(DESC_TYPE_STRING, 0);
        }while (pStringDesc != NULL && pStringDesc->bLength != 18);
    } else
   */
#endif
    {
      switch (strIndex)
      {
        case 0:
          pStringDesc = (USB_STRING_DESCRIPTOR *)languageId;
          break;

        case 1:
          pStringDesc = (USB_STRING_DESCRIPTOR *)manufacturer;
          break;

        case 2:
          pStringDesc = (USB_STRING_DESCRIPTOR *)product;
          break;

        case 3:
          if (serialNumber[0] == 0)
          {
            uint8  bdAddr[ 8 ], i;
            uint16 tmp;

            // get BDADDR from Info Page
            for (i=0; i<BLE_INFO_PAGE_BDADDR_LEN; i++)
            {
              bdAddr[i] = ((uint8 *)BLE_INFO_PAGE_BDADDR)[i];
            }

            // check if the permanent BDADDR address is invalid
            if ( (bdAddr[0] == 0xFF) && (bdAddr[1] == 0xFF) &&
                 (bdAddr[2] == 0xFF) && (bdAddr[3] == 0xFF) &&
                 (bdAddr[4] == 0xFF) && (bdAddr[5] == 0xFF) &&
                 (bdAddr[6] == 0xFF) && (bdAddr[7] == 0xFF) )
            {
              // grab something else like the wafer data
              for (i=0; i<BLE_INFO_PAGE_BDADDR_LEN; i++)
              {
                bdAddr[i] = ((uint8 *)BLE_INFO_PAGE_WAFER_INFO)[i];
              }
            }

            for (i=2; i<42; i+=2)
            {
              if (((i-2) % 10) == 0)
              {
                uint8 j = (i-2) / 5;
                tmp = BUILD_UINT16(bdAddr[j], bdAddr[j+1]);
              }

              serialNumber[i] = '0' + (tmp % 10);
              tmp /= 10;
            }
            serialNumber[0] = 42;
          }
          pStringDesc = (USB_STRING_DESCRIPTOR *)serialNumber;

        break;

      default:
        break;
      }
    }

   return pStringDesc;
} // usbdpGetStringDesc
/// @}
/*
+------------------------------------------------------------------------------
|  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.
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
