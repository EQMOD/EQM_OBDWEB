/**************************************************************************************************
  Filename:       ubl_exec.h
  Revised:        $Date: 2012-08-31 11:31:40 -0700 (Fri, 31 Aug 2012) $
  Revision:       $Revision: 31449 $

  Description:

  This module defines the executive functionality of a Universal Boot Loader
  for an 8051-based SOC using the USB transport by MSD.


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
#ifndef UBL_EXEC_H
#define UBL_EXEC_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_types.h"
#include "ubl_app.h"
#include "usb_msd.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

// Place the filename space on the last 256 bytes of the 1st available page in Bank-0.
#define UBL_FILE_NAME_PAGE                 UBL_RC_IMG_PG_BEG
#define UBL_FILE_NAME_IDX                 (HAL_FLASH_PAGE_SIZE - USB_MSD_FILENAME_MAX)

// Place the Meta Data immediately before the filename.
#define UBL_META_DATA_PAGE                 UBL_FILE_NAME_PAGE
#define UBL_META_DATA_IDX                 (UBL_FILE_NAME_IDX - sizeof(ublMetaData_t))

#define UBL_META_DATA_ADDR  \
  ((uint16)(UBL_META_DATA_PAGE * HAL_FLASH_PAGE_SIZE) + UBL_META_DATA_IDX)

// Define uint16 values for this UBL.
#define UBL_CRC_ERASED                     0xFFFF
#define UBL_CRC_ZEROED                     0x0000

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define UBL_RC_VALID     (                             \
  (((ublMD.crcRC == ublMD.crcShdw)  &&                 \
    (ublMD.crcRC != UBL_CRC_ERASED) &&                 \
    (ublMD.crcRC != UBL_CRC_ZEROED)) ?  TRUE : FALSE)  \
)

#define UBL_UNLOCKED  (ublMD.wordPad2[2] == 0)
#define UBL_UNLOCK()   ublMD.wordPad2[2] = 0
#define UBL_LOCK()     ublMD.wordPad2[2] = 255

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

extern const uint8 UBL_RC_IMG_PG_BEG;
extern const uint8 UBL_RC_IMG_PG_END;

extern uint8 pgBuf[HAL_FLASH_PAGE_SIZE];  // RAM (XDATA) buffer for an Rx/Tx flash page.
extern ublMetaData_t ublMD;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          ublAesAuth
 *
 * @brief       Run the AES CRC-MAC authentication calculation over the RC image according to the
 *              AES Control Block parameters and update the control block accordingly.
 *              Just set CRC shadow equal to CRC when no valid security key built into the UBL.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE whether the AES signature of the image in flash is valid.
 */
uint8 ublAesAuth(void);

/**************************************************************************************************
 * @fn          ublAesCrypt
 *
 * @brief       UBL AES encryption/decyption for the low-level transport driver.
 *              Just return when no valid security key built into the UBL.
 *
 * input parameters
 *
 * @param       pgNum - HAL Flash page number corresponding to the pgBuf.
 * @param       pgBuf - Pointer to the page buffer to crypt in place.
 *
 * output parameters
 *
 * @param       pgBuf - Pointer to the page buffer of crypted bytes.
 *
 * @return      TRUE or FALSE whether the AES image page is valid (i.e. the ctrl block on 1st page).
 */
uint8 ublAesCrypt(uint8 pgNum, uint8 *pgBuf);

/**************************************************************************************************
 * @fn          ublCfg
 *
 * @brief       Configure according to the received meta-data file if it checks out.
 *
 * input parameters
 *
 * @param       pMD - Pointer to the received meta-data file.
 *
 * output parameters
 *
 * None.
 *
 * @return      true if the cfg file is acceptable; false otherwise.
 */
bool ublCfg(ublMetaData_t *pMD);

/**************************************************************************************************
 * @fn          ublExec
 *
 * @brief       UBL executive loop for polling and managing environment.
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
void ublExec(void);

/**************************************************************************************************
 * @fn          ublInit
 *
 * @brief       UBL environment initialization in preparation for running.
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
void ublInit(void);

/**************************************************************************************************
 * @fn          ublJump
 *
 * @brief       Execute a simple long jump from non-banked UBL code to non-banked RC code space.
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
void ublJump(void);

/**************************************************************************************************
 * @fn          ublMassErase
 *
 * @brief       Erase all pages enabled for mass-erase (not including UBL pages or lock bits page).
 *
 * input parameters
 *
 * @param       eraseAll - Flag to override the 'eraseEn' bits of the Meta Data.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void ublMassErase(bool eraseAll);

#endif
/**************************************************************************************************
 */
