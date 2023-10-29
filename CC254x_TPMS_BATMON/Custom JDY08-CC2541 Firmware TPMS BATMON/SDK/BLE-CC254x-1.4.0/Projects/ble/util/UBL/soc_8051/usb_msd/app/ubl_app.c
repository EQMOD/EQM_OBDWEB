/**************************************************************************************************
  Filename:       ubl_app.c
  Revised:        $Date: 2013-09-13 12:00:00 -0700 (Fri, 13 Sep 2013) $
  Revision:       $Revision: 35310 $

  Description:

  This module implements the Application layer aspects for usability with a Universal Boot Loader
  for an 8051-based SOC using the USB transport by MSD.

  Note that this file is included only by the Application to be downloaded, not the UBL.


  Copyright 2011-2012 Texas Instruments Incorporated. All rights reserved.

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
#if defined MT_UBL_FUNC
#include "mt.h"
#include "mt_rpc.h"
#endif
#include "ubl_app.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#if defined HAL_DONGLE_NANO && !defined MT_UBL_FUNC
#warning Ensure that Application code has a way of being triggered to invoke ublAppForceBoot().
#endif

// Offset of meta-data, address 0x0E9C, from start of page.
#define UBL_MD_PG_OFFSET       0x069C

#pragma location="AES_CTRL_BLK"
static const __code aes_ctrl_blk_t _aesCtrlBlk = {
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFFFF, 0xFFFF},  // Dummy spares (simple EBL uses for CRC & Shadow).
   119,  // Image length in flash pages, not including SNV pages.
   0xFF, // Must be set to zero for downloading or to 0xFF for programming.
 { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09 }  // Dummy Nonce.
};
#pragma required=_aesCtrlBlk

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

#pragma location="UBL_FILENAME_SPACE"
const uint8 _ublFileName[256] = {  // Must be left as all 0xFF for UBL to fill-in for itself.
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#pragma required=_ublFileName

#pragma location="UBL_METADATA_SPACE"
const ublMetaData_t _ublMetaData = {
#if defined HAL_DONGLE_NANO  // No GPIO, so use 30-second delay in UBL mode on cold boot.
  // No crcRC, this detail is hidden from the Application
  0xFFFF,  // CRC-shadow must be left as 0xFFFF.
  0xFFFF,  // Checksum of all following bytes - not used since verify all by read-back after write.
  30000,   // Delay 30 seconds as Disk Drive before jumping to valid RC image and opening COM port.
  { // Security Key to prevent unauthorized access or Trojan Horse attack.
    #warning Put a meaningful security key here to protect your network security keys.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  },
  { // Disable erase of the NV: flash pages 120-121.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC
  },
  { // Allow over-write of the NV: flash pages 120-121.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  },
  { // Disable read of all flash pages to protect code from reverse-compilers & NV from access.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  },
  0xFF,  // No GPIO to force UBL mode.
  0xFF,  // GPIO Port/Pin don't care.
  { // Spare configuration bytes.
    0xFF, 0xFF
  },
  #warning Put a non-zero countdown here until the code has been proven to not lock-out the UBL.
  0x00,  // No count-down to force UBL mode - this is tested and  verified working code.
  { // Padding bytes.
    0xFF, 0xFF, 0xFF
  },
  0x00,  // Tolerate no attempted break-ins - erase all flash on first failed attempt.
  { // Padding bytes.
    0xFF, 0xFF, 0xFF
  }
// The DK-Dongle has keys, so no delay in UBL mode on cold boot, but look at outermost key, S1.
#else
  // No crcRC, this detail is hidden from the Application
  0xFFFF,  // CRC-shadow must be left as 0xFFFF.
  0xFFFF,  // Checksum of all following bytes - not used since verify all by read-back after write.
  0,       // No delay as a Disk Drive before jumping to valid RC image and opening COM port.
  { // Security Key to prevent unauthorized access or Trojan Horse attack.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  },
  { // Disable erase of the NV: flash pages 120-121.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x81
  },
  { // Allow over-write of the NV: flash pages 120-121.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x81
  },
  { // Disable read of all flash pages to protect code from reverse-compilers & NV from access.
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  },
  0xFC,  // GPIO to force UBL mode: pull-up, polarity zero.
  0x0A,  // GPIO Port/Pin P1_2.
  { // Spare configuration bytes.
    0xFF, 0xFF
  },
  0x00,  // No count-down to force UBL mode - this is tested and  verified working code.
  { // Padding bytes.
    0xFF, 0xFF, 0xFF
  },
  0x00,  // Tolerate no attempted break-ins - erase all flash on first failed attempt.
  { // Padding bytes.
    0xFF, 0xFF, 0xFF
  }
#endif
};
#pragma required=_ublMetaData

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

  // Make sure SBL is present.
  HalFlashRead(UBL_PAGE_FIRST, UBL_MD_PG_OFFSET, (uint8 *)crc, 4);

  if ((crc[0] == 0xFFFF) || (crc[0] == 0x0000) ||
      (crc[1] == 0xFFFF) || (crc[1] == 0x0000) ||
      (crc[1] != crc[0]))
  {
    return;
  }

  HAL_DISABLE_INTERRUPTS();

  crc[1] ^= 0xFFFF;  // Only write to zero bits that are not already zero.
  crc[0] = 0xFFFF;
  HalFlashWrite(((uint16)&_ublMetaData) / HAL_FLASH_WORD_SIZE, (uint8 *)crc, 1);

  HAL_SYSTEM_RESET();
}

#if defined MT_UBL_FUNC
/**************************************************************************************************
 * @fn          MT_UblCommandProcessing
 *
 * @brief       Force the boot loader to run via the MT API.
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
uint8 MT_UblCommandProcessing(uint8 *pBuf)
{
  if (pBuf[MT_RPC_POS_CMD1] == MT_UBL_FORCE)
  {
    ublAppForceBoot();
  }

  return MT_RPC_ERR_COMMAND_ID;
}
#endif

/**************************************************************************************************
*/
