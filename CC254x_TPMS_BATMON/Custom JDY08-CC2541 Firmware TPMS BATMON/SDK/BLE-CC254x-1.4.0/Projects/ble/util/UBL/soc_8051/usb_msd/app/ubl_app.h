/**************************************************************************************************
  Filename:       ubl_app.h
  Revised:        $Date: 2013-09-13 12:00:00 -0700 (Fri, 13 Sep 2013) $
  Revision:       $Revision: 35310 $

  Description:

  This module declares the Application layer aspects for usability with a Universal Boot Loader
  for an 8051-based SOC using the USB transport by MSD.

  Note that this file is included by both the UBL and the Application to be downloaded,
  thus any change to the data structures herewithin require a physical re-programming of the UBL.


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
#ifndef UBL_APP_H
#define UBL_APP_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_aes.h"
#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                         Constants
 * ------------------------------------------------------------------------------------------------
 */

// The SB page boundary since all SB addresses are "actual address / flash word size".
// Note for MSP - flash word size is 1, but 4 must be used for inter-compatibility w/ SoC.
#define UBL_PAGE_SIZE (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)

#define UBL_ADDR_OSET       (uint16)(UBL_PAGE_FIRST * HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)
// AES Control Block occupies the first 32 bytes of the RC Image.
#define UBL_ADDR_CTRL       (UBL_ADDR_OSET + 0x00)  // 0x800
#define UBL_ADDR_CRC        (UBL_ADDR_OSET + 0x10)  // 0x810
#define UBL_ADDR_IMG_LEN    (UBL_ADDR_OSET + 0x14)  // 0x814

// The UBL is in Bank0, page 0, and Bank7, pages 122-126.
// The Application image area first & last pages need to be carefully synchronized with the UBL and
// Application linker files as well as with the ASM IVED relays.
#define UBL_PAGE_FIRST               1
// Not including pages reserved for SNV for ease-of-use with encrypting-signing boot loader build.
#define UBL_PAGE_LAST                119
//efine UBL_PAGE_LAST                121

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

// The last address allowed for reading is determined by the 'imageLen' field of the Control Block
// in order to prevent access to the SNV pages after they contain a secret or security key.
#define UBL_RW_ADDR_END     ((ctrlBlk.imageLen * UBL_PAGE_SIZE) + UBL_ADDR_OSET - UBL_RW_BUF_LEN/4)
#define UBL_RW_ADDR_BEG       UBL_ADDR_OSET

// Convert an UBL_ADDR into the parameters for a call to HalFlashRead().
#define UBL_NVM_GET(UBL_ADDR, PBUF, BYTE_CNT) \
      HalFlashRead((UBL_ADDR) / UBL_PAGE_SIZE, ((UBL_ADDR) % UBL_PAGE_SIZE) << 2,\
                                                       (void *)(PBUF), (BYTE_CNT))

// Convert a BYTE_CNT into the HAL_FLASH_WORD_SIZE-length parameter for a call to HalFlashWrite().
#define UBL_NVM_SET(UBL_ADDR, PBUF, BYTE_CNT) \
     HalFlashWrite((UBL_ADDR), (void *)(PBUF), ((BYTE_CNT) / HAL_FLASH_WORD_SIZE))

/* ------------------------------------------------------------------------------------------------
 *                                         Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef struct {
  uint8  signature[KEY_BLENGTH];  // The AES-128 CBC-MAC signature.
  uint16 spare[2];       // Dummy spares (simple EBL uses for CRC & Shadow).
  uint8  imageLen;       // Image length in flash pages, must be 1-125.
  uint8  allowSignCmd;   // Must be zero in download binary, may be programmed by tool as non-zero.
  uint8  nonce10[10];    // 10 bytes to append to the crc[0] to create the 12-byte Nonce.
} aes_ctrl_blk_t;

static_assert(((sizeof(aes_ctrl_blk_t) % KEY_BLENGTH) == 0),
              "aes_ctrl_blk_t is not an even multiple of KEY_BLENGTH");

typedef struct {
#ifdef UBL_BUILD
  uint16 crcRC;         // CRC of the run-code (RC) image, not including this meta-data.
#endif //UBL_BUILD
  uint16 crcShdw;       // Shadow of the RC CRC.
  uint16 chkMD;         // Checksum of the meta-data, dlyJmp-cfgSpare, inclusive.
  uint16 dlyJmp;        // Delay in milliseconds before jumping to a valid RC image.
  uint8  secKey[32];    // Security key must match exactly; all 0xFF's allows all operations.
  uint8  eraseEn[16];   // Page-erase enable-bits: 1 = allow erase on mass erase command.
  uint8  writeEn[16];   // Page-write enable-bits: 1 = allow write (monolithic image) & incl in CRC.
  uint8  readLock[16];  // Page-read lock-bits: 1 = disallow read & return fill characters.
  uint8  cfgDiscs[1];   // Configuration discretes.
  uint8  gpioPort;      // GPIO Port/Pin value to use if the 'gpioUse' bit is cleared.
  uint8  cfgSpare[2];   // Spare discretes / bytes.

  // Enforce to be on a HAL_FLASH_WORD_SIZE boundary for easy writes.
  uint8  cntDnForced;   // Count down hard resets by bit clearing, if not zero, to force boot mode.
  uint8  wordPad1[3];   // Pad to HAL_FLASH_WORD_SIZE to handle the multiple writes to this word.

  // Enforce to be on a HAL_FLASH_WORD_SIZE boundary for easy writes.
  uint8  cntDnSecKey;   // Count down attempted cfg-file hacks with a bad 'secKey'.
  uint8  wordPad2[3];   // Pad to HAL_FLASH_WORD_SIZE to handle the multiple writes to this word.
} ublMetaData_t;
//static_assert((sizeof(ublMetaData_t) == 100), "Changing the ublMetaData_t structure requires \
//  corresponding changes to linker files and re-building the USB-MSD library.");

typedef enum {
  gpioUseNot,
  gpioPolarity,
  gpioPullTri,
  gpioPullUpDn,
  cfgDisc0_4,
  cfgDisc0_5,
  cfgDisc0_6,
  cfgDisc0_7
} cfgDisc0_t;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */

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
void appForceBoot(void);

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
uint8 MT_UblCommandProcessing(uint8 *pBuf);
#endif

#endif
/**************************************************************************************************
 */
