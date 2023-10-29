/**************************************************************************************************
  Filename:       sbl_app.c
  Revised:        $Date: 2012-09-07 14:26:14 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31498 $

  Description:

  This module implements the Application layer aspects for usability with a
  Serial Boot Loader for an 8051-based SOC using the UART/SPI transport.

  Note that this file is included only by the Application to be downloaded,
  not the SBL.


  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_flash.h"
#include "hal_types.h"
#include "sbl_app.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#pragma location="IMAGE_HEADER"
static const __code img_hdr_t _imgHdr = {
  { 2012, 0xFFFF },           // CRC and shadow.
  0x0000,                     // Version
  SBL_FULL_LEN,               // HAL_FLASH_WORD_SIZE units rounded up to fill a HAL_FLASH_PAGE_SIZE.
  { 0x08, 0x0A, 0x0C, 0x0E }, // User-Id
  { 0xFF, 0xFF, 0xFF, 0xFF }  // Reserved
};
#pragma required=_imgHdr

#pragma location="AES_HEADER"
static const __code aes_hdr_t _aesHdr = {
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
 { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B },  // Dummy Nonce
 { 0xFF, 0xFF, 0xFF, 0xFF }   // Spare
};
#pragma required=_aesHdr

/**************************************************************************************************
 * @fn          appForceBoot
 *
 * @brief       Force the boot loader to run.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void appForceBoot(void)
{
  uint16 crc[2];
  SBL_NVM_GET(SBL_ADDR_CRC, crc, sizeof(crc));

  // A simple check if the image is built for a boot loader is a valid CRC & shadow where expected.
  if ((crc[0] == 0xFFFF) || (crc[0] == 0x0000) ||
      (crc[1] == 0xFFFF) || (crc[1] == 0x0000) ||
      (crc[1] != crc[0]))
  {
    return;
  }

  HAL_DISABLE_INTERRUPTS();

  crc[0] ^= 0xFFFF;  // Only write to zero bits that are not already zero.
  crc[1] = 0xFFFF;   // No need to write any bits to zero.
  SBL_NVM_SET(SBL_ADDR_CRC, crc, sizeof(crc));

  HAL_SYSTEM_RESET();
}

/**************************************************************************************************
*/
