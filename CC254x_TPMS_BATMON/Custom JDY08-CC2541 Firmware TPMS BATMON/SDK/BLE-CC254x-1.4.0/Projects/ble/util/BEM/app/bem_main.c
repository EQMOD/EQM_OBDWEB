/**************************************************************************************************
  Filename:       bem_main.c
  Revised:        $Date: 2013-08-28 10:20:25 -0700 (Wed, 28 Aug 2013) $
  Revision:       $Revision: 35146 $

  Description:

  This module contains the definitions for the main functionality of an Image Boot Manager.


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

#include <stddef.h>
#include <string.h>

#include "hal_aes.h"
#include "hal_dma.h"
#include "hal_flash.h"
#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define BEM_IMG_A_PAGE        1
#define BEM_IMG_A_AREA        62

#define BEM_IMG_B_PAGE        8
#define BEM_IMG_B_AREA       (124 - BEM_IMG_A_AREA)

static const uint8 ImgPageBeg[2] = { BEM_IMG_A_PAGE, BEM_IMG_B_PAGE };
static const uint8 ImgPageLen[2] = { BEM_IMG_A_AREA, BEM_IMG_B_AREA };

#define BEM_AES_OSET          0x10

static const uint8 aesKey[KEY_BLENGTH] = {
  // This dummy key must be replaced by a randomly generated key that is kept secret.
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

#define BEM_PAGE_LEN         (uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)

// The OAD image is encrypted using the BootLoader-Encrypter PC tool with the EBL.
#define SBL_RW_BUF_LEN        64

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef struct {
  // Secure OAD uses the Signature for image validation instead of calculating a CRC, but the use
  // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
  uint16 crc0;       // CRC must not be 0x0000 or 0xFFFF.
  uint16 crc1;       // CRC-shadow must be 0xFFFF.
  // User-defined Image Version Number - default logic uses simple a '<' comparison to start an OAD.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} img_hdr_t;
static_assert((sizeof(img_hdr_t) == BEM_AES_OSET), "img_hdr_t size mismatch with BEM_AES_OSET");

// The AES Header must be encrypted and the Signature must include the Image Header.
typedef struct {
  uint8 signature[KEY_BLENGTH];  // The AES-128 CBC-MAC signature.
  uint8 nonce12[12];             // The 12-byte Nonce for calculating the signature.
  uint8 spare[4];
} aes_hdr_t;

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

// Convert a flash page number into the parameters for a call to HalFlashRead().
#define BEM_NVM_GET(PG, PBUF, BYTE_CNT)  HalFlashRead((PG), 0, (void *)(PBUF), (BYTE_CNT))

// Convert a BYTE_CNT into the HAL_FLASH_WORD_SIZE-length parameter for a call to HalFlashWrite().
#define BEM_NVM_SET(PG, PBUF, BYTE_CNT)  HalFlashWrite( \
   (PG) * (uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE), (void *)(PBUF), (BYTE_CNT) /\
                                                                             HAL_FLASH_WORD_SIZE)

// OAD requires whole flash pages, so no need to round up in this macro.
//efine BEM_IMG_HDR_LEN_2_PG_CNT(LEN)  (((LEN) + BEM_PAGE_LEN/2) / BEM_PAGE_LEN)
#define BEM_IMG_HDR_LEN_2_PG_CNT(LEN)  ((LEN) / BEM_PAGE_LEN)

#define BEM_IMG_R2R(IMG_HDR) \
  ((((IMG_HDR).crc0 != 0) && \
    ((IMG_HDR).crc0 != 0xFFFF) && \
    ((IMG_HDR).crc0 == (IMG_HDR).crc1)) ? 1 : 0)

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init halDMADesc_t dmaCh0;  // Locally setup for use by HalFlashWrite().

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init uint8 pageBuf[HAL_FLASH_PAGE_SIZE];

__no_init __data uint8 JumpToImageAorB @ 0x09;

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

#pragma location = "ALIGNED_CODE"
static void halSleepExec(void);

/**************************************************************************************************
 * @fn          halSleepExec
 *
 * @brief       This function puts the CC254x to sleep by writing to the PCON register.
 *              The instruction after writing to PCON must not be 4-byte aligned or excessive
 *              power consumption may result. Since the write to PCON is 3 instructions, this
 *              function is forced to be even-byte aligned. Thus, this function must not have any
 *              automatic variables and the write to PCON must be the first C statement.
 *              See the linker file ".xcl" for actual placement of this function.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
#pragma optimize=none
static void halSleepExec(void)
{
  PCON = 0x01;
  ASM_NOP;
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
 * @param       aesHdr - Pointer to the aes_hdr_t info to use for the signature calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void aesInitSig(aes_hdr_t *aesHdr)
{
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

  for (uint8 idx = 0; idx < 12; idx++)
  {
    ENCDI = aesHdr->nonce12[idx];
  }

  // Using 'spare' member for 3-byte image length.
  ENCDI = aesHdr->spare[2];
  ENCDI = aesHdr->spare[1];
  ENCDI = aesHdr->spare[0];

  while ((ENCCS & BV(3)) == 0);
}

/**************************************************************************************************
 * @fn          aesCrypt
 *
 * @brief       Run the AES CTR decryption/encryption over the parameter buffer.
 *
 * input parameters
 *
 * @param       skipCnt - Count of leading KEY_BLENGTH-byte blocks to skip.
 * @param       pBuf - Pointer to the SBL_RW_BUF_LEN-byte buffer to crypt in place.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the SBL_RW_BUF_LEN-byte buffer of crypted bytes.
 *
 * @return      None.
 */
static uint8 ivNonce[KEY_BLENGTH] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
static void aesCrypt(uint8 skipCnt, uint8 *pBuf)
{
  // A0: L-encoding of L-1 = 2-1 = 1; starting 2-byte CTR at 1.
  //uint8 ivNonce[KEY_BLENGTH] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  ENCCS = CTR | AES_LOAD_IV | 0x01;

  // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
  // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
  ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

  for (uint8 idx = 0; idx < KEY_BLENGTH; idx++)
  {
    ENCDI = ivNonce[idx];
  }
  while ((ENCCS & BV(3)) == 0);

  for (uint8 cnt = 0; cnt < (SBL_RW_BUF_LEN / KEY_BLENGTH); cnt++)
  {
    if (skipCnt == 0)
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
    else
    {
      skipCnt--;
      pBuf += KEY_BLENGTH;
    }
  }
}

/**************************************************************************************************
 * @fn          aesSignature
 *
 * @brief       Run the AES CRC-MAC calculation over the image specified according to the
 *              AES Control Block parameters and update the control block accordingly.
 *
 * input parameters
 *
 * @param       imgSel - Image select: 0 for Image-A and 1 for Image-B.
 * @param       aesHdr - Pointer to the aes_hdr_t info to use for the signature calculation.
 *
 * output parameters
 *
 * @param       aesHdr - Pointer to the aes_hdr_t info with the signature calculated.
 *
 * @return      TRUE or FALSE whether the AES signature calculated matches the one in the header.
 */
static uint8 aesSignature(uint8 imgSel, aes_hdr_t *aesHdr)
{
  uint8 sigBuf[KEY_BLENGTH];
  aesInitSig(aesHdr);

  uint8 pgEnd = ImgPageBeg[imgSel] + aesHdr->spare[3] - 1;

  if (imgSel == 0)
  {
    pgEnd += ImgPageLen[1];
  }

  for (uint8 pgNum = ImgPageBeg[imgSel]; pgNum <= pgEnd; )
  {
    BEM_NVM_GET(pgNum, pageBuf, HAL_FLASH_PAGE_SIZE);

    for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; )
    {
      if ((pgNum == ImgPageBeg[imgSel]) && (oset == sizeof(img_hdr_t)))
      {
        oset += KEY_BLENGTH;  // Must not include the signature bytes in the signature calculation.
      }
      else if ((pgNum == pgEnd) && (oset == (HAL_FLASH_PAGE_SIZE - KEY_BLENGTH)))
      {
        break;  // Need to change mode to CBC-MAC for the last block.
      }

      ENCCS |= 0x01;
      for (uint8 cnt = 0; cnt < KEY_BLENGTH; cnt++)
      {
        ENCDI = pageBuf[oset++];
      }
      while ((ENCCS & BV(3)) == 0);
    }

    pgNum++;

    if ((imgSel == 0) && (pgNum == ImgPageBeg[1]))
    {
      pgNum += ImgPageLen[1];
    }
  }

  ENCCS = CBC | AES_ENCRYPT | 0x01;  // Switch to CBC mode for the last block.

  // 'while ((ENCCS & BV(3)) == 0)' was seen to hang without #pragma optimize=none.
  // So proactively adding this wait after every 'ENCCS = ' which empirically seems to work.
  ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;

  for (uint16 oset = (HAL_FLASH_PAGE_SIZE - KEY_BLENGTH); oset < HAL_FLASH_PAGE_SIZE; oset++)
  {
    ENCDI = pageBuf[oset];
  }
  HAL_AES_DELAY();  // Delay required for non-DMA AES as RDY bit only goes hi after read out below.

  // CBC-MAC generates output on the last block.
  for (uint8 cnt = 0; cnt < KEY_BLENGTH; cnt++)
  {
    sigBuf[cnt] = ENCDO;
  }

  return ((memcmp(sigBuf, aesHdr->signature, KEY_BLENGTH) == 0) ? TRUE : FALSE);
}

/**************************************************************************************************
 * @fn          imgAuth
 *
 * @brief       Run the AES CTR decryption over the image area specified.
 * @brief       Run the AES CRC-MAC calculation over the image specified and set the image
 *              ready-to-run if the Signature is verified.
 *
 * input parameters
 *
 * @param       imgSel - Image select: 0 for Image-A and 1 for Image-B.
 * @param       imgHdr - Pointer to the Image Header corresponding to the image select.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the image Signature verifies; FALSE otherwise.
 */
static uint8 imgAuth(uint8 imgSel, img_hdr_t *imgHdr)
{
  if ((imgHdr->crc0 == 0) || (imgHdr->crc0 == 0xFFFF))
  {
    return FALSE;
  }

  if (imgHdr->crc0 != imgHdr->crc1)
  {
    aesLoadKey();

    aes_hdr_t aesHdr;
    HalFlashRead(ImgPageBeg[imgSel], BEM_AES_OSET, (uint8 *)&aesHdr, sizeof(aes_hdr_t));

    // Image length in 3 bytes, MSB to LSB order - the Signature bytes are not to be included.
    uint32 imageLen = (uint32)HAL_FLASH_PAGE_SIZE * imgHdr->res[0] - KEY_BLENGTH;

    aesHdr.spare[0] = ((uint8 *)&imageLen)[0];
    aesHdr.spare[1] = ((uint8 *)&imageLen)[1];
    aesHdr.spare[2] = ((uint8 *)&imageLen)[2];
    aesHdr.spare[3] = imgHdr->res[0];

    if (aesSignature(imgSel, &aesHdr))
    {
      uint16 crc[2] = { 0xFFFF, imgHdr->crc0 };

      BEM_NVM_SET(ImgPageBeg[imgSel], (uint8 *)crc, 4);
      BEM_NVM_GET(ImgPageBeg[imgSel], (uint8 *)&imgHdr, 4);
    }
  }

  return (imgHdr->crc0 == imgHdr->crc1);
}

/**************************************************************************************************
 * @fn          imgCrypt
 *
 * @brief       Run the AES CTR decryption over the image area specified.
 *
 * input parameters
 *
 * @param       imgSel - Image select: 0 for Image-A and 1 for Image-B.
 * @param       imgHdr - Pointer to the Image Header corresponding to the image select.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void imgCrypt(uint8 imgSel, img_hdr_t *imgHdr)
{
  aesLoadKey();

  uint8 pgEnd = imgHdr->res[0] + ImgPageBeg[imgSel];

  if (imgSel == 0)
  {
    pgEnd += ImgPageLen[1];
  }

  for (uint8 pgNum = ImgPageBeg[imgSel]; pgNum < pgEnd; )
  {
    BEM_NVM_GET(pgNum, pageBuf, HAL_FLASH_PAGE_SIZE);

    // EBL is used to encrypt the image in SBL_RW_BUF_LEN blocks, so it must be decrypted likewise.
    for (uint8 blk = 0; blk < (HAL_FLASH_PAGE_SIZE / SBL_RW_BUF_LEN); blk++)
    {
      if ((pgNum == ImgPageBeg[imgSel]) && (blk == 0))
      {
        aesCrypt(1, pageBuf + ((uint16)SBL_RW_BUF_LEN * blk));
      }
      else
      {
        aesCrypt(0, pageBuf + ((uint16)SBL_RW_BUF_LEN * blk));
      }
    }

    HalFlashErase(pgNum);
    BEM_NVM_SET(pgNum, pageBuf, HAL_FLASH_PAGE_SIZE);

    pgNum++;

    if ((imgSel == 0) && (pgNum == ImgPageBeg[1]))
    {
      pgNum += ImgPageLen[1];
    }
  }
}

/**************************************************************************************************
 * @fn          imgRun
 *
 * @brief       Setup for and jump to the image specified.
 *
 * input parameters
 *
 * @param       imgSelect - Select Image-A/B as 0/1.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
#pragma optimize=none  // The two LJMP's get optimized to just one to the same address.
static void imgRun(uint8 imgSelect)
{
  // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
  if ((JumpToImageAorB = imgSelect) == 0)
  {
    asm("LJMP 0x0830");
  }
  else
  {
    asm("LJMP 0x4030");
  }

  HAL_SYSTEM_RESET();  // Should not get here.
}

/**************************************************************************************************
 * @fn          imgSelect
 *
 * @brief       Select and setup for running Image-A/B by prefering to run Image-B over Image-A,
 *              except when Image-A is newly downloaded (i.e. ready to decrypt).
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
static void imgSelect(void)
{
  img_hdr_t imgHdr[2];

  for (uint8 idx = 0; idx < 2; idx++)
  {
    BEM_NVM_GET(ImgPageBeg[idx], (uint8 *)&imgHdr[idx], sizeof(img_hdr_t));

    // Local use of image header reserved byte for image length in whole flash pages.
    imgHdr[idx].res[0] = BEM_IMG_HDR_LEN_2_PG_CNT(imgHdr[idx].len);

    if (imgHdr[idx].res[0] > ImgPageLen[idx])
    {
      imgHdr[idx].crc0 = 0;  // Make image invalid.
    }
  }

  // If Image-A is ready to de-crypt.
  if ((imgHdr[0].crc0 != 0xFFFF) && (imgHdr[0].crc0 != 0x0000) && (imgHdr[0].crc1 == 0xFFFF))
  {
    imgCrypt(0, imgHdr+0);

    if (imgAuth(0, imgHdr+0))
    {
      // Invalidate Image-B to now always run newly downloaded and now validated Image-A.
      uint16 crc[2] = { 0x0000, 0xFFFF };
      BEM_NVM_SET(ImgPageBeg[1], (uint8 *)crc, sizeof(crc));

      imgRun(0);
    }
    else
    {
      // Invalidate Image-A to skip the above decrypt on every powerup.
      uint16 crc[2] = { 0x0000, 0xFFFF };
      BEM_NVM_SET(ImgPageBeg[0], (uint8 *)crc, sizeof(crc));
    }
  }

  if (BEM_IMG_R2R(imgHdr[1]))
  {
    imgRun(1);
  }

  // If Image-B is ready to de-crypt.
  if ((imgHdr[1].crc0 != 0xFFFF) && (imgHdr[1].crc0 != 0x0000) && (imgHdr[1].crc1 == 0xFFFF))
  {
    imgCrypt(1, imgHdr+1);

    if (imgAuth(1, imgHdr+1))
    {
      imgRun(1);
    }
    else
    {
      // Invalidate Image-B to skip the above decrypt on every powerup.
      uint16 crc[2] = { 0x0000, 0xFFFF };
      BEM_NVM_SET(ImgPageBeg[1], (uint8 *)crc, sizeof(crc));
    }
  }

  if (BEM_IMG_R2R(imgHdr[0]))
  {
    imgRun(0);
  }
}

/**************************************************************************************************
 * @fn          vddWait
 *
 * @brief       Loop waiting for 16 reads of the Vdd over the safe minimum to run.
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
    } while (ADCH < VDD_MIN_RUN);
  } while (--cnt);
}

/**************************************************************************************************
 * @fn          main
 *
 * @brief       C-code main function.
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
 **************************************************************************************************
 */
void main(void)
{
  vddWait();

  HAL_BOARD_INIT();

  /* This is in place of calling HalDmaInit() which would require init of the other 4 DMA
   * descriptors in addition to just Channel 0.
   */
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);

  // Map flash bank #7 into XDATA for access to "ROM mapped as data".
  MEMCTR = (MEMCTR & 0xF8) | 0x07;

  imgSelect();         // Attempt to select and run an image.

  SLEEPCMD |= 0x03;    // PM3, All clock oscillators off, voltage regulator off.
  halSleepExec();
  HAL_SYSTEM_RESET();  // Should not get here.
}

/**************************************************************************************************
*/
