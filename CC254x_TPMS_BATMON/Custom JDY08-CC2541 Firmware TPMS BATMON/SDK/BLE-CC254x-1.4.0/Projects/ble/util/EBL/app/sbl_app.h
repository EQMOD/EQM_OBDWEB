/**************************************************************************************************
  Filename:       sbl_app.h
  Revised:        $Date: 2012-09-07 14:26:14 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31498 $

  Description:

  This module declares the Application layer aspects for usability with a
  Serial Boot Loader for an 8051-based SOC using the UART/SPI transport.

  Note that this file is included by both the UBL and the Application to be
  downloaded, thus any change to the data structures herewithin require a
  physical re-programming of the UBL.


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
#ifndef SBL_APP_H
#define SBL_APP_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_aes.h"
#include "hal_board_cfg.h"
#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                         Constants
 * ------------------------------------------------------------------------------------------------
 */

// Any changes to the SBL limits must be manually synchronized with the corresponding .xcl files.

#define SBL_PAGE_BEG                 1    // First page allowed for writing the RC Image.
#define SBL_PAGE_END                 124  // Last page allowed for writing the RC Image.

// The SB page boundary since all SB addresses are "actual address / flash word size".
#define SBL_PAGE_LEN        (uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

#define SBL_ADDR_MIN                (SBL_PAGE_BEG * SBL_PAGE_LEN)
#define SBL_ADDR_MAX                (SBL_PAGE_END * SBL_PAGE_LEN + SBL_PAGE_LEN)

#define SBL_ADDR_IMG_HDR             SBL_ADDR_MIN
#define SBL_ADDR_CRC                 SBL_ADDR_IMG_HDR

#define SBL_ADDR_AES_HDR            (SBL_ADDR_IMG_HDR + (16 / HAL_FLASH_WORD_SIZE))

#define SBL_ADDR_BEG                 SBL_ADDR_MIN
// The last address allowed for reading is determined by the 'len' field of the Image Header.
// This helps to prevent access to the SNV pages after they contain a secret or security key.
#define SBL_ADDR_END                (imgHdr.len + \
                                    (SBL_ADDR_MIN - (SBL_RW_BUF_LEN / HAL_FLASH_WORD_SIZE)))

#define SBL_FULL_LEN                (SBL_ADDR_MAX - SBL_ADDR_MIN)

// Convert an SBL_ADDR into the parameters for a call to HalFlashRead().
#define SBL_NVM_GET(SBL_ADDR, PBUF, BYTE_CNT) \
      HalFlashRead((SBL_ADDR) / SBL_PAGE_LEN, ((SBL_ADDR) % SBL_PAGE_LEN) << 2,\
                                                       (void *)(PBUF), (BYTE_CNT))

// Convert a BYTE_CNT into the HAL_FLASH_WORD_SIZE-length parameter for a call to HalFlashWrite().
#define SBL_NVM_SET(SBL_ADDR, PBUF, BYTE_CNT) \
     HalFlashWrite((SBL_ADDR), (void *)(PBUF), ((BYTE_CNT) / HAL_FLASH_WORD_SIZE))

#define SBL_READ_IMG_HDR() st ( \
  SBL_NVM_GET(SBL_ADDR_IMG_HDR, &imgHdr, sizeof(img_hdr_t)); \
  \
  if ((imgHdr.len == 0) || (imgHdr.len > SBL_FULL_LEN)) \
  { \
    imgHdr.len = SBL_FULL_LEN; \
  } \
)

/* ------------------------------------------------------------------------------------------------
 *                                         Typedefs
 * ------------------------------------------------------------------------------------------------
 */

// The Image Header will not be encrypted, but it will be included in a Signature.
typedef struct {
  uint16 crc[2];     // crc[0] must be non-zero and non-FFFF; crc[1] must be all-FFFF.
  // User-defined Image Version Number - default logic uses simple a '<' comparison to start an OAD.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} img_hdr_t;
static_assert((sizeof(img_hdr_t) == 16), "Bad SBL_ADDR_AES_HDR definition.");
static_assert(((sizeof(img_hdr_t) % KEY_BLENGTH) == 0),
                      "img_hdr_t is not an even multiple of KEY_BLENGTH");

// The AES Header must be encrypted and the Signature must include the Image Header.
typedef struct {
  uint8 signature[KEY_BLENGTH];  // The AES-128 CBC-MAC signature.
  uint8 nonce12[12];             // The 12-byte Nonce for calculating the signature.
  uint8 spare[4];
} aes_hdr_t;

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

#endif
/**************************************************************************************************
 */
