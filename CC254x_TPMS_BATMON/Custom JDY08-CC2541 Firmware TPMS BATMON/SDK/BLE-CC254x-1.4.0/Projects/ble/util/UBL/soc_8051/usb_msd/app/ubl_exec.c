/**************************************************************************************************
  Filename:       ubl_exec.c
  Revised:        $Date: 2012-09-07 14:26:14 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31498 $

  Description:

  This module implements the executive functionality of a Universal Boot Loader
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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include <stddef.h>
#include <string.h>

#include "hal_board_cfg.h"
#include "hal_dma.h"
#include "hal_flash.h"
#include "hal_types.h"
#include "ubl_app.h"
#include "ubl_exec.h"
#include "usb_msd.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined HAL_DONGLE_NANO
#define UBL_GPIO_USE
#endif

// Define the checksum-subset of the Meta Data for checksum calculation and copying so as not to
// overwrite ong.
#define UBL_MD_CHKLEN  (offsetof(ublMetaData_t, cntDnForced) - offsetof(ublMetaData_t, chkMD))

static const uint8 unlockedSecKey[32] =
{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/***                 The following are "Write" addresses for HalFlashWrite().                  ***/
#define UBL_META_DATA_ADDR_WR  \
  ((uint16)(UBL_META_DATA_PAGE * ((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))) + \
                                  (uint16)(UBL_META_DATA_IDX / HAL_FLASH_WORD_SIZE))

#define UBL_CNTDN_FORCED_ADDR (UBL_META_DATA_ADDR_WR + \
                              (offsetof(ublMetaData_t, cntDnForced) / HAL_FLASH_WORD_SIZE))
#define UBL_CNTDN_SECKEY_ADDR (UBL_META_DATA_ADDR_WR + \
                              (offsetof(ublMetaData_t, cntDnSecKey) / HAL_FLASH_WORD_SIZE))
/***                  The above are "Write" addresses for HalFlashWrite().                    ***/

// Allow test & development with final structures and memeory map without the burden of having to
// encrypt and sign every image to download until ready with the production build.
#if !defined UBL_SECURE
#define UBL_SECURE                   FALSE
#endif

#if UBL_SECURE
static const uint8 aesKey[KEY_BLENGTH] = {
  // This dummy key must be replaced by a randomly generated key that is kept secret.
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};
#endif

// Set UBL_SIGNER=TRUE in order to create a special boot loader that will accept
// an un-encrypted/un-authenticated image via download and then sign it and send it back
// with encryption on the read back.
#if !defined UBL_SIGNER
#define UBL_SIGNER                   FALSE
#endif

#if UBL_SIGNER
#if !UBL_SECURE
#error Mismatched definitions for UBL_SIGNER and UBL_SECURE.
#else
#warning You built a special "Signing" boot loader - do not release to market ... internal use only.
#endif
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define UBL_READ_ST(STCNT) st (             \
  do {  /* Get the sleep timer count; ST0 must be read first & re-read to verify. */\
    ((uint8 *) &(STCNT))[0] = ST0;          \
  } while (((uint8 *) &(STCNT))[0] != ST0); \
  ((uint8 *) &(STCNT))[1] = ST1;            \
  ((uint8 *) &(STCNT))[2] = ST2;            \
  ((uint8 *) &(STCNT))[3] = 0;              \
)

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

const uint8 UBL_RC_IMG_PG_BEG = UBL_PAGE_FIRST;
const uint8 UBL_RC_IMG_PG_END = UBL_PAGE_LAST;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init uint8 pgBuf[HAL_FLASH_PAGE_SIZE];  // RAM (XDATA) buffer for an Rx/Tx flash page.
__no_init ublMetaData_t ublMD;

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

#if defined UBL_GPIO_USE
static __no_init volatile uint8 *pForcePort;
static __no_init uint8 forcePin;
#endif
static __no_init uint32 stStart, stDelay;

#if UBL_SECURE
// Flag when built with UBL_SIGNER=TRUE to encrypt the read back.
static bool signMode;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

#if UBL_SECURE
static uint8 aesCheckCtrl(void *pBuf);
static void  aesLoadKey(void);
static void  aesInitSig(void);
#endif

static bool cntDnForcedDecr(void);
#if defined UBL_GPIO_USE
static bool gpioInit(void);
#endif
static void vddWait(void);

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
uint8 ublAesAuth(void)
{
#if UBL_SECURE
  uint8 pgCnt = 0;
  uint8 sigBuf[KEY_BLENGTH];

  aes_ctrl_blk_t ctrlBlk;
  UBL_NVM_GET(UBL_ADDR_CTRL, &ctrlBlk, sizeof(aes_ctrl_blk_t));

  aesInitSig();

  for (uint8 pgNum = UBL_PAGE_FIRST; pgNum <= UBL_PAGE_LAST; pgNum++)
  {
    if (!GET_BIT(ublMD.writeEn, pgNum))
    {
      continue;
    }

    pgCnt++;

    HalFlashRead(pgNum, 0, pgBuf, HAL_FLASH_PAGE_SIZE);

    for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; )
    {
      if ((pgCnt == 1) && (oset == 0))
      {
        oset += KEY_BLENGTH;  // Must not include the signature bytes in the signature calculation.
      }
      else if ((pgCnt == ctrlBlk.imageLen) && (oset == (HAL_FLASH_PAGE_SIZE - KEY_BLENGTH)))
      {
        break;  // Need to change mode to CBC-MAC for the last block.
      }

      ENCCS |= 0x01;
      for (uint8 cnt = 0; cnt < KEY_BLENGTH; cnt++)
      {
        ENCDI = pgBuf[oset++];
      }
      while ((ENCCS & BV(3)) == 0);
    }

    if (pgCnt >= ctrlBlk.imageLen)
    {
      break;
    }
  }

  ENCCS = CBC | AES_ENCRYPT | 0x01;  // Switch to CBC mode for the last block.

  // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
  // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
  ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

  for (uint16 oset = (HAL_FLASH_PAGE_SIZE - KEY_BLENGTH); oset < HAL_FLASH_PAGE_SIZE; oset++)
  {
    ENCDI = pgBuf[oset];
  }
  HAL_AES_DELAY();  // Delay required for non-DMA AES as RDY bit only goes hi after read out below.

  // CBC-MAC generates output on the last block.
  for (uint8 cnt = 0; cnt < KEY_BLENGTH; cnt++)
  {
    sigBuf[cnt] = ENCDO;
  }

  if (ctrlBlk.allowSignCmd != 0)  // If requested to sign this image.
  {
    ctrlBlk.allowSignCmd = 0;
    (void)memcpy(ctrlBlk.signature, sigBuf, KEY_BLENGTH);
    UBL_NVM_SET(UBL_ADDR_CTRL, &ctrlBlk, sizeof(aes_ctrl_blk_t));
    UBL_NVM_GET(UBL_ADDR_CTRL, &ctrlBlk, sizeof(aes_ctrl_blk_t));
  }

#if UBL_SIGNER
  signMode = TRUE;  // Now the Signer should encrypt the read back.
#else  // A signer must always return FALSE so that usb_msd.c does not save the file name to flash.
  if (memcmp(ctrlBlk.signature, sigBuf, KEY_BLENGTH))
#endif
  {
    return FALSE;
  }
#endif

  HalFlashRead(UBL_META_DATA_PAGE, UBL_META_DATA_IDX, (uint8 *)&ublMD.crcRC, 4);

  ublMD.crcShdw = ublMD.crcRC;
  ublMD.crcRC = UBL_CRC_ERASED;  // Do not write zero to the same bits more than twice.
  HalFlashWrite(UBL_META_DATA_ADDR_WR, (uint8 *)&ublMD.crcRC, 1);

  return TRUE;
}

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
uint8 ublAesCrypt(uint8 pgNum, uint8 *pgBuf)
{
#if UBL_SECURE
  if (!UBL_SIGNER || signMode)
  {
    // A0: L-encoding of L-1 = 2-1 = 1; starting 2-byte CTR at 1.
    uint8 ivNonce[KEY_BLENGTH] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
    uint8 *pBuf = pgBuf;

    ENCCS = CTR | AES_LOAD_IV | 0x01;

    // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
    // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
    ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

    for (uint8 idx = 0; idx < KEY_BLENGTH; idx++)
    {
      ENCDI = ivNonce[idx];
    }
    while ((ENCCS & BV(3)) == 0);

    for (uint8 cnt = 0; cnt < (HAL_FLASH_PAGE_SIZE / KEY_BLENGTH); cnt++)
    {
      ENCCS = CTR | AES_ENCRYPT | 0x01;

      // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
      // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
      ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

      for (uint8 blk = 0; blk < 4; blk++)
      {
        for (uint8 idx = 0; idx < 4; idx++)
        {
          ENCDI = pBuf[idx];
        }

        HAL_AES_DELAY();  // Delay required for non-DMA AES as RDY bit only goes hi after read out.

        for (uint8 idx = 0; idx < 4; idx++)
        {
          pBuf[idx] = ENCDO;
        }

        pBuf += 4;
      }
    }

    if ((pgNum == UBL_PAGE_FIRST) && !aesCheckCtrl(pgBuf))
    {
      return FALSE;
    }
  }
#else
  (void)pgNum;
  (void)pgBuf;
#endif

  return TRUE;
}

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
 * @return      TRUE if the cfg file is acceptable; FALSE otherwise.
 */
bool ublCfg(ublMetaData_t *pMD)
{
  // Thwart an attempt to break-in by an exhaustive Security Key trial-and-error.
  if (!UBL_UNLOCKED && (memcmp(&ublMD.secKey, &pMD->secKey, sizeof(ublMD.secKey)) != 0))
  {
    if (ublMD.cntDnSecKey == 0)
    {
      ublMassErase(TRUE);  // Force the erase of all pages outside of the UBL image.
      HAL_SYSTEM_RESET();
    }
    else
    {
      uint8 mask = 0x01;

      while ((mask & ublMD.cntDnSecKey) == 0)
      {
        mask <<= 1;
      }
      ublMD.cntDnSecKey ^= mask;

      // This will write zero to exactly one bit, so no bit is written to zero twice.
      uint8 secKeyAndPad[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
      secKeyAndPad[0] ^= mask;
      HalFlashWrite(UBL_CNTDN_SECKEY_ADDR, secKeyAndPad, 1);
    }
  }
  else
  {
    UBL_UNLOCK();
    (void)memcpy(&ublMD.chkMD, &pMD->chkMD, UBL_MD_CHKLEN);
    ublMD.crcShdw = UBL_CRC_ZEROED;  // Receiving a "cfg" file is a de facto force of UBL mode.
    return TRUE;
  }

  return FALSE;
}

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
void ublExec(void)
{
  while (1)
  {
    // No continual waits for VDD_MIN_NV - Vdd is assumed to be stable on a USB-powered device.
    //vddWait();

    if (usb_msd_poll() == TRUE)  // TRUE == "Eject".
    {
      break;
    }

    if (UBL_RC_VALID && (ublMD.dlyJmp != 0))
    {
#if defined UBL_GPIO_USE
      if (!GET_BIT(ublMD.cfgDiscs+0, gpioUseNot) &&
          (GET_BIT(ublMD.cfgDiscs+0, gpioPolarity) == ((*pForcePort & BV(forcePin)) != 0)))
      {
        ublMD.crcShdw = UBL_CRC_ZEROED;
      }
      else
#endif
      {
        uint32 stDelta;
        UBL_READ_ST(stDelta);  // Get the free-running count of 30.5 usec timer ticks.
        stDelta -= stStart;  // Calculate the elapsed ticks of the free-running timer.
        ((uint8 *)&stDelta)[3] = 0;  // Adjust for a carry on the 24-bit ST counter.

        if (stDelta > stDelay)
        {
          break;
        }
      }
    }
  }

  if (UBL_RC_VALID)
  {
    usb_msd_uninit();
    ublJump();
  }
}

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
void ublInit(void)
{
  HalFlashRead(UBL_META_DATA_PAGE, UBL_META_DATA_IDX, (uint8 *)&ublMD, sizeof(ublMetaData_t));

  // If the RC image area is erased or the Application doesn't care about security, unlock the UBL.
  if (memcmp(ublMD.secKey, unlockedSecKey, sizeof(unlockedSecKey)) == 0)
  {
    UBL_UNLOCK();
  }

  if (UBL_RC_VALID)
  {
    if ((ublMD.cntDnForced == 0) || (cntDnForcedDecr() == FALSE))
    {
#if defined UBL_GPIO_USE
      if (GET_BIT(ublMD.cfgDiscs+0, gpioUseNot) || (gpioInit() == FALSE))
#endif
      {
        if (ublMD.dlyJmp == 0)
        {
          ublJump();
        }
        else
        {
          stDelay = (ublMD.dlyJmp * 4096UL) / 125;
          UBL_READ_ST(stStart);
        }
      }
    }
  }

  vddWait();  // Stricter wait then in main, looking for safe Vdd for writing flash.

#if UBL_SECURE
  aesLoadKey();

  aes_ctrl_blk_t ctrlBlk;
  UBL_NVM_GET(UBL_ADDR_CTRL, &ctrlBlk, sizeof(aes_ctrl_blk_t));

  if ((ctrlBlk.allowSignCmd != 0) && (ctrlBlk.imageLen != 0xFF)) // If requested to sign this image.
  {
    // Signing after checking UBL_RC_VALID ensures run as MSD again so encrypted image can be read.
    (void)ublAesAuth();
  }

#if UBL_SIGNER  // A signer must allow image read back of what was written.
  for (uint8 idx = 0; idx < 16; idx++)
  {
    ublMD.readLock[idx] = ublMD.writeEn[idx] ^ 0xFF;
  }
#endif
#endif

  usb_msd_init();  // Initialize USB-MSD as late as possible for time limits after enabling D+ line.
}

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
void ublJump(void)
{
  asm("LJMP 0x820\n");  // Immediate jump to run-code.
  HAL_SYSTEM_RESET();
}

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
void ublMassErase(bool eraseAll)
{
  for (uint8 pg = UBL_RC_IMG_PG_BEG+1; pg <= UBL_PAGE_LAST; pg++)
  {
    if (eraseAll || GET_BIT(ublMD.eraseEn, pg))
    {
      FADDRH = pg * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE / 256);
      FCTL |= 0x01;
    }
  }

  // Now erase the page with the filename.
  FADDRH = UBL_RC_IMG_PG_BEG * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE / 256);
  FCTL |= 0x01;
}

#if UBL_SECURE
/**************************************************************************************************
 * @fn          aesCheckCtrl
 *
 * @brief       Check validity of a AES Control Block before writing it to flash.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the buffer containing the control block.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE for control block valid.
 */
static uint8 aesCheckCtrl(void *pBuf)
{
  uint8 sigBuf[KEY_BLENGTH] =
   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  aes_ctrl_blk_t *pCtl = (aes_ctrl_blk_t *)pBuf;

#if UBL_SIGNER
  if ( memcmp(pCtl->signature, sigBuf, sizeof(sigBuf))
          || (pCtl->imageLen == 0)
          || (pCtl->imageLen > UBL_PAGE_LAST)
          || (pCtl->allowSignCmd == 0))
#else
  if (!memcmp(pCtl->signature, sigBuf, sizeof(sigBuf))
          || (pCtl->imageLen == 0)
          || (pCtl->imageLen > UBL_PAGE_LAST)
          || (pCtl->allowSignCmd != 0))
#endif
  {
    return FALSE;
  }

  return TRUE;
}

/**************************************************************************************************
 * @fn          aesLoadKey
 *
 * @brief       Load the shared secret key into the AES for operations.
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
static void aesLoadKey(void)
{
  // Read the security key from flash 1 byte at a time to thwart an interrupt & read XDATA attack.
  uint8 *keyPtr = (uint8 *)aesKey;

  ENCCS = ECB | AES_LOAD_KEY | 0x01;

  // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
  // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
  ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

  for (uint8 cnt = 0; cnt < KEY_BLENGTH; cnt++)
  {
    ENCDI = *keyPtr++;
  }
}

/**************************************************************************************************
 * @fn          aesInitSig
 *
 * @brief       Initialize the AES for signature calculation.
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
static void aesInitSig(void)
{
  aes_ctrl_blk_t ctrlBlk;
  UBL_NVM_GET(UBL_ADDR_CTRL, &ctrlBlk, sizeof(aes_ctrl_blk_t));

  ENCCS = CBC_MAC | AES_LOAD_IV | 0x01;

  // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
  // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
  ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

  for (uint8 cnt = 0; cnt < KEY_BLENGTH; cnt++)
  {
    ENCDI = 0;
  }

  ENCCS = CBC_MAC | AES_ENCRYPT | 0x01;

  // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
  // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
  ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

  ENCDI = 0x3A;  // B0 Flag: Res=0, A_Data=0, (M-2)/2=7, (L-1)=2.

  ENCDI = LO_UINT16(ctrlBlk.spare[0]);
  ENCDI = HI_UINT16(ctrlBlk.spare[0]);
  for (uint8 idx = 0; idx < 10; idx++)
  {
    ENCDI = ctrlBlk.nonce10[idx];
  }

  // Image length in 3 bytes, MSB to LSB order - the Signature bytes are not to be included.
  uint32 imageLen = 2048UL * ctrlBlk.imageLen - KEY_BLENGTH;
  ENCDI = ((uint8 *)&imageLen)[2];
  ENCDI = ((uint8 *)&imageLen)[1];
  ENCDI = ((uint8 *)&imageLen)[0];

  while ((ENCCS & BV(3)) == 0);
}
#endif

/**************************************************************************************************
 * @fn          cntDnForcedDecr
 *
 * @brief       Decrement the cntDnForced.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the dlyJmp decrements to zero; FALSE otherwise.
 */
static bool cntDnForcedDecr(void)
{
  uint8 mask = 0x01;

  while ((mask & ublMD.cntDnForced) == 0)
  {
    mask <<= 1;
  }
  ublMD.cntDnForced ^= mask;

  if (ublMD.cntDnForced == 0)
  {
    // Don't write the zero to flash in case of another cold-boot before re-programming, just
    // force boot now by faking the CRC-shdw as zeroed.
    ublMD.crcShdw = UBL_CRC_ZEROED;  // No need to zero crcShdw in flash, this will always run.
    return TRUE;
  }
  else
  {
    // This will write zero to exactly one bit, so no bit is written to zero twice.
    uint8 forcedAndPad[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    forcedAndPad[0] ^= mask;
    HalFlashWrite(UBL_CNTDN_FORCED_ADDR, forcedAndPad, 1);
    return FALSE;
  }
}

#if defined UBL_GPIO_USE
/**************************************************************************************************
 * @fn          gpioInit
 *
 * @brief       Initialize the generic GPIO force configuration.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the GPIO is already forcing UBL mode; FALSE otherwise.
 */
static bool gpioInit(void)
{
  volatile uint8 *ptr;
  uint8 port = ublMD.gpioPort / 8;
  uint8 pin = ublMD.gpioPort % 8;

  ptr = ((port == 0) ? &X_P0SEL : ((port == 1) ? &X_P1SEL : &X_P2SEL));
  *ptr &= ~BV(pin);

  ptr = ((port == 0) ? &X_P0DIR : ((port == 1) ? &X_P1DIR : &X_P2DIR));
  *ptr &= ~BV(pin);

  ptr = ((port == 0) ? &X_P0INP : ((port == 1) ? &X_P1INP : &X_P2INP));
  if (GET_BIT(ublMD.cfgDiscs+0, gpioPullTri))     // If pulling.
  {
    if (GET_BIT(ublMD.cfgDiscs+0, gpioPullUpDn))  // If pulling up.
    {
      P2INP &= ~BV(port + 5);
    }
    else                                          // If pulling down.
    {
      P2INP |= BV(port + 5); \
    }
    *ptr &= ~BV(pin);
  }
  else                                            // Tri-state.
  {
    *ptr |=  BV(pin);
  }

  forcePin = pin;
  pForcePort = ((port == 0) ? &X_P0 : ((port == 1) ? &X_P1 : &X_P2));

  if (GET_BIT(ublMD.cfgDiscs+0, gpioPolarity) == ((*pForcePort & BV(forcePin)) != 0))
  {
    ublMD.crcShdw = UBL_CRC_ZEROED;
    return TRUE;
  }

  return FALSE;
}
#endif

/**************************************************************************************************
 * @fn          vddWait
 *
 * @brief       Loop waiting for 16 reads of the Vdd over the safe minimum to erase/write flash.
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
static void vddWait(void)
{
  uint8 cnt = 16;

  do {
    do {
      ADCCON3 = 0x0F;
      while (!(ADCCON1 & 0x80));
    } while (ADCH < VDD_MIN_NV);
  } while (--cnt);
}

/**************************************************************************************************
*/
