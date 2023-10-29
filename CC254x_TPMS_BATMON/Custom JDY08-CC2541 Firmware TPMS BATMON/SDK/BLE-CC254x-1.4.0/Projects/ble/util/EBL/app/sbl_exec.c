/**************************************************************************************************
  Filename:       sbl_exec.c
  Revised:        $Date: 2013-08-28 10:57:16 -0700 (Wed, 28 Aug 2013) $
  Revision:       $Revision: 35148 $

  Description:  Serial Bootloader Executive.


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

#include <string.h>

#include "hal_adc.h"
#include "hal_aes.h"
#include "hal_board_cfg.h"
#include "hal_flash.h"
#include "hal_rpc.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "sbl_app.h"
#include "sbl_exec.h"

/* ------------------------------------------------------------------------------------------------
 *                                        Constants
 * ------------------------------------------------------------------------------------------------
 */

//#define AES_TEST_VECS

static const uint8 aesKey[KEY_BLENGTH] = {
#if defined AES_TEST_VECS
  0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF
#else
  // This dummy key must be replaced by a randomly generated key that is kept secret.
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
#endif
};

#if defined AES_TEST_VECS
static const uint8 testNonce[13] = {
  0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0x03, 0x02, 0x01, 0x00, 0x06
};
#endif

#define SBL_RW_BUF_LEN               64

// Commands to Bootloader
#define SBL_WRITE_CMD                0x01
#define SBL_READ_CMD                 0x02
#define SBL_ENABLE_CMD               0x03
#define SBL_HANDSHAKE_CMD            0x04

// Commands to Target Application
#define SBL_TGT_BOOTLOAD             0x10  // Erase the image valid signature & jump to bootloader.

#define SBL_SIGNATURE_CMD            0x20

// Responses from Bootloader - for historical consistency, SBL has OR'ed the MSBit of all commands
// when responding - this is probably not necessary for smooth functionality.
#define SBL_RSP_MASK                 0x80

// Status codes
#define SBL_SUCCESS                  0
#define SBL_FAILURE                  1
#define SBL_INVALID_FCS              2
#define SBL_INVALID_FILE             3
#define SBL_FILESYSTEM_ERROR         4
#define SBL_ALREADY_STARTED          5
#define SBL_NO_RESPOSNE              6
#define SBL_VALIDATE_FAILED          7
#define SBL_CANCELED                 8
#define SBL_IGNORED                  9

// Indices into the RPC data (RPC_POS_DAT0):
#define SBL_REQ_ADDR_LSB             RPC_POS_DAT0
#define SBL_REQ_ADDR_MSB            (SBL_REQ_ADDR_LSB+1)
#define SBL_REQ_DAT0                (SBL_REQ_ADDR_MSB+1)
#define SBL_RSP_STATUS               RPC_POS_DAT0
#define SBL_RSP_ADDR_LSB            (SBL_RSP_STATUS+1)
#define SBL_RSP_ADDR_MSB            (SBL_RSP_ADDR_LSB+1)
#define SBL_RSP_DAT0                (SBL_RSP_ADDR_MSB+1)
#define SBL_READ_HDR_LEN            (SBL_RSP_DAT0 - SBL_RSP_STATUS)

#if !defined SBL_UART_PORT
#define SBL_UART_PORT HAL_UART_PORT_0
#endif

// Allow test & development with final structures and memeory map without the burden of having to
// encrypt and sign every image to download until ready with the production build.
#if !defined SBL_SECURE
#define SBL_SECURE                   TRUE
#endif

// Set SBL_SECURE=FALSE & SBL_SIGNER=TRUE in order to create a special boot loader that will
// accept an un-encrypted/un-authenticated image via download and then sign it on command and
// send it back with encryption on the read back by the PC tool.
#if !defined SBL_SIGNER
#define SBL_SIGNER                   FALSE
#endif

// Buffer size - it has to be big enough for the largest RPC packet and overhead.
#define SBL_BUF_SIZE                 256
#define SBL_MAX_SIZE                (SBL_BUF_SIZE - RPC_FRAME_HDR_SZ - RPC_UART_FRAME_OVHD)

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum
{
  rpcSteSOF,
  rpcSteLen,
  rpcSteData,
  rpcSteFcs
} rpcSte_t;

/* ------------------------------------------------------------------------------------------------
 *                                     Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 rpcBuf[SBL_BUF_SIZE], sbFcs, sbIdx, sbLen;
static uint8 *const sbBuf = rpcBuf+1;
static rpcSte_t rpcSte;

// Local global that must be kept current with what exists in the corresponding flash area.
static __no_init img_hdr_t imgHdr;

static __no_init uint8 pageBuf[HAL_FLASH_PAGE_SIZE];

// Flag when built with SBL_SIGNER=TRUE & SBL_SECURE=FALSE to encrypt the read back.
static bool signMode;

#if defined AES_TEST_VECS
uint8 testSig[KEY_BLENGTH];
#endif

/* ------------------------------------------------------------------------------------------------
 *                                     Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void  sblProc(void);
static uint8 sblResp(void);

static void  aesLoadKey(void);
static void  aesInitSig(void);
static void aesCrypt(uint8 skipCnt, uint8 *pBuf);
static uint8 aesSignature(void *pBuf);

static uint8 imgHdrCheck(void *pBuf);
static uint8 checkRC(void);
static uint8 procSignatureCmd(void *pBuf);

/**************************************************************************************************
 * @fn          sblInit
 *
 * @brief       Boot Loader initialization.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if there is a valid RC image; FALSE otherwise.
 */
uint8 sblInit(void)
{
  aesLoadKey();

#if defined AES_TEST_VECS
  uint8 checkSig[KEY_BLENGTH] = {
    0xB9, 0xD7, 0x89, 0x67, 0x04, 0xBC, 0xFA, 0x20, 0xB2, 0x10, 0x36, 0x74, 0x45, 0xF9, 0x83, 0xD6
  };
  imgHdr.len = 1;

  while (1)
  {
    aesSignature(testSig);

    if (memcmp(testSig, checkSig, KEY_BLENGTH))  // If signature generated is not value expected.
    {
      ASM_NOP;
    }
  }
#else
  SBL_READ_IMG_HDR();

  return checkRC();
#endif
}

/**************************************************************************************************
 * @fn          sblPoll
 *
 * @brief       Serial Boot poll & parse according to the RPC protocol.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the downloaded code has been enabled; FALSE otherwise.
 */
uint8 sblPoll(void)
{
  uint8 ch;

  while (HalUARTRead(0, &ch, 1))
  {
    switch (rpcSte)
    {
    case rpcSteSOF:
      if (RPC_UART_SOF == ch)
      {
        rpcSte = rpcSteLen;
      }
      break;

    case rpcSteLen:
      if (ch > SBL_MAX_SIZE)
      {
        rpcSte = rpcSteSOF;
        break;
      }
      else
      {
        rpcSte = rpcSteData;
        sbFcs = sbIdx = 0;
        sbLen = ch + 3;  // Combine the parsing of Len, Cmd0 & Cmd1 with the data.
        // no break;
      }

    case rpcSteData:
      sbFcs ^= ch;
      sbBuf[sbIdx] = ch;

      if (++sbIdx == sbLen)
      {
        rpcSte = rpcSteFcs;
      }
      break;

    case rpcSteFcs:
      rpcSte = rpcSteSOF;

      if ((sbFcs == ch) && ((sbBuf[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) == RPC_SYS_BOOT))
      {
        sblProc();
        return sblResp();  // Send the SB response setup in the sbBuf passed to sblProc().
      }
      break;

    default:
      HAL_SYSTEM_RESET();
      break;
    }
  }

  return FALSE;
}

/**************************************************************************************************
 * @fn          sblProc
 *
 * @brief       Process the SB command and received buffer.
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
static void sblProc(void)
{
  uint16 t16 = BUILD_UINT16(sbBuf[SBL_REQ_ADDR_LSB], sbBuf[SBL_REQ_ADDR_MSB]) + SBL_ADDR_BEG;
  uint8 len = 1, rsp = SBL_SUCCESS;

  switch (sbBuf[RPC_POS_CMD1])
  {
  case SBL_WRITE_CMD:
    if ((t16 >= SBL_ADDR_BEG) && (t16 <= SBL_ADDR_END))
    {
      if ((t16 % SBL_PAGE_LEN) == 0)
      {
        HalFlashErase(t16 / SBL_PAGE_LEN);
      }

      if (SBL_SECURE)
      {
        if (t16 == SBL_ADDR_IMG_HDR)
        {
          if (!imgHdrCheck(sbBuf + SBL_REQ_DAT0))
          {
            rsp = SBL_FAILURE;
            break;
          }

          aesCrypt(1, sbBuf + SBL_REQ_DAT0);
        }
        else
        {
          aesCrypt(0, sbBuf + SBL_REQ_DAT0);
        }
      }

      SBL_NVM_SET(t16, (sbBuf + SBL_REQ_DAT0), SBL_RW_BUF_LEN);

      // Immediately read back what was written to keep the 'imgHdr' variable in sync with flash.
      if (t16 == SBL_ADDR_IMG_HDR)
      {
        SBL_READ_IMG_HDR();
      }
    }
    else
    {
      rsp = SBL_FAILURE;
    }
    break;

  case SBL_READ_CMD:
    if ((t16 >= SBL_ADDR_BEG) && (t16 <= SBL_ADDR_END))
    {
      len = SBL_RW_BUF_LEN + SBL_READ_HDR_LEN;
      sbBuf[SBL_RSP_ADDR_MSB] = sbBuf[SBL_REQ_ADDR_MSB];
      sbBuf[SBL_RSP_ADDR_LSB] = sbBuf[SBL_REQ_ADDR_LSB];

      SBL_NVM_GET(t16, (sbBuf + SBL_RSP_DAT0), SBL_RW_BUF_LEN);

      if (SBL_SECURE || (SBL_SIGNER && signMode))
      {
        if (t16 == SBL_ADDR_IMG_HDR)
        {
          aesCrypt(1, sbBuf + SBL_RSP_DAT0);
        }
        else
        {
          aesCrypt(0, sbBuf + SBL_RSP_DAT0);
        }
      }
    }
    else
    {
      rsp = SBL_FAILURE;
    }
    break;

  case SBL_ENABLE_CMD:
    if (SBL_SIGNER)  // A Signer must never enable the image for clean read back with crc[1]=0xFFFF.
    {
      signMode = FALSE;  // PC Tool read back must be un-encrypted after downloading a new image.
    }
    else if (!SBL_SECURE)
    {
      imgHdr.crc[1] = imgHdr.crc[0];
      imgHdr.crc[0] = 0xFFFF;
      SBL_NVM_SET(SBL_ADDR_CRC, imgHdr.crc, sizeof(imgHdr.crc));
      SBL_READ_IMG_HDR();
    }
    else if (!checkRC())
    {
      rsp = SBL_VALIDATE_FAILED;
    }
    break;

  case SBL_HANDSHAKE_CMD:
    break;

  case SBL_SIGNATURE_CMD:
    len = ((rsp = procSignatureCmd((sbBuf + RPC_POS_DAT0 + 1))) == SBL_SUCCESS) ? 17 : 1;
    break;

  default:
    rsp = SBL_FAILURE;
    break;
  }

  sbBuf[RPC_POS_LEN] = len;
  sbBuf[RPC_POS_CMD1] |= SBL_RSP_MASK;
  sbBuf[RPC_POS_DAT0] = rsp;
}

/**************************************************************************************************
 * @fn          sblResp
 *
 * @brief       Make the SB response.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the downloaded code has been enabled; FALSE otherwise.
 */
static uint8 sblResp(void)
{
  uint8 fcs = 0, len = sbBuf[RPC_POS_LEN] + RPC_FRAME_HDR_SZ;
  uint8 rtrn = FALSE;

  for (uint8 idx = RPC_POS_LEN; idx < len; idx++)
  {
    fcs ^= sbBuf[idx];
  }
  sbBuf[len] = fcs;

  if ((sbBuf[RPC_POS_CMD1] == (SBL_ENABLE_CMD | SBL_RSP_MASK)) &&
      (sbBuf[RPC_POS_DAT0] == SBL_SUCCESS))
  {
    len++;  // Send an extra garbage byte to flush the last good one before resetting.
    rtrn = TRUE;
  }

  rpcBuf[0] = RPC_UART_SOF;
  (void)HalUARTWrite(0, rpcBuf, len + RPC_UART_FRAME_OVHD);

  return rtrn;
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

#if defined AES_TEST_VECS
  ENCDI = 0x59;  // B0 Flag: Res=0, A_Data=1, (M-2)/2=3, (L-1)=1.

  for (uint8 idx = 0; idx < 13; idx++)
  {
    ENCDI = testNonce[idx];
  }
#else
  aes_hdr_t aesHdr;
  SBL_NVM_GET(SBL_ADDR_AES_HDR, &aesHdr, sizeof(aes_hdr_t));

  ENCDI = 0x3A;  // B0 Flag: Res=0, A_Data=0, (M-2)/2=7, (L-1)=2.

  for (uint8 idx = 0; idx < 12; idx++)
  {
    ENCDI = aesHdr.nonce12[idx];
  }
#endif

#if defined AES_TEST_VECS
  ENCDI = 0x00;
  ENCDI = 0x17;  // 23-byte test buffer preceded by the L(a) & 8-octet A_Data.
#else
  // Image length in 3 bytes, MSB to LSB order - the Signature bytes are not to be included.
  uint32 imageLen = (uint32)HAL_FLASH_WORD_SIZE * imgHdr.len - KEY_BLENGTH;
  ENCDI = ((uint8 *)&imageLen)[2];
  ENCDI = ((uint8 *)&imageLen)[1];
  ENCDI = ((uint8 *)&imageLen)[0];
#endif

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
static void aesCrypt(uint8 skipCnt, uint8 *pBuf)
{
  // A0: L-encoding of L-1 = 2-1 = 1; starting 2-byte CTR at 1.
  uint8 ivNonce[KEY_BLENGTH] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

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
 * @brief       Run the AES CRC-MAC calculation over the RC image according to the
 *              AES Control Block parameters and update the control block accordingly.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the KEY_BLENGTH-byte buffer to hold the signature calculated.
 *
 * @return      TRUE or FALSE whether the AES signature could be calculated.
 */
static uint8 aesSignature(void *pBuf)
{
  aesInitSig();

#if defined AES_TEST_VECS
  uint8 testInput[48] = {
    0x00, 0x08, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  memcpy(pageBuf+HAL_FLASH_PAGE_SIZE-48, testInput, 48);

  uint8 idx = 0;
  for (uint8 loop = 0; loop < 2; loop++)
  {
    ENCCS |= 0x01;
    for (uint8 cnt = 0; cnt < KEY_BLENGTH; cnt++)
    {
      ENCDI = testInput[idx++];
    }
    while ((ENCCS & BV(3)) == 0);
  }
#else
  const uint8 lastPg = imgHdr.len / SBL_PAGE_LEN + SBL_PAGE_BEG - 1;

  for (uint8 pg = SBL_PAGE_BEG; pg <= lastPg; pg++)
  {
    HalFlashRead(pg, 0, pageBuf, HAL_FLASH_PAGE_SIZE);

    for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; )
    {
      if ((pg == SBL_PAGE_BEG) && (oset == sizeof(img_hdr_t)))
      {
        oset += KEY_BLENGTH;  // Must not include the signature bytes in the signature calculation.
      }
      else if ((pg == lastPg) && (oset == (HAL_FLASH_PAGE_SIZE - KEY_BLENGTH)))
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
  }
#endif

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
  for (uint8 cnt = 0, *pSig = (uint8 *)pBuf; cnt < KEY_BLENGTH; cnt++)
  {
    *pSig++ = ENCDO;
  }

  return TRUE;
}

/**************************************************************************************************
 * @fn          imgHdrCheck
 *
 * @brief       Check validity of the Image and AES Headers before writing them to flash.
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
static const uint8 sigBuf[KEY_BLENGTH] =
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8 imgHdrCheck(void *pBuf)
{
  const img_hdr_t *pHdr = (const img_hdr_t *)pBuf;
  const uint8 *pSig = (const uint8 *)((uint8 *)pBuf + sizeof(img_hdr_t));

  if (!memcmp(pSig, sigBuf, sizeof(sigBuf))
          || (pHdr->crc[0] == 0)
          || (pHdr->crc[0] == 0xFFFF)
          || (pHdr->crc[1] != 0xFFFF)
          || (pHdr->len == 0)
          || (pHdr->len > SBL_FULL_LEN))
  {
    return FALSE;
  }

  return TRUE;
}

/**************************************************************************************************
 * @fn          checkRC
 *
 * @brief       Check validity of the run-code image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE for image valid.
 */
static uint8 checkRC(void)
{
  if ((imgHdr.crc[0] == 0) || (imgHdr.crc[0] == 0xFFFF))
  {
    return FALSE;
  }

  if (SBL_SECURE && (imgHdr.crc[0] != imgHdr.crc[1]) && (imgHdr.crc[1] == 0xFFFF))
  {
    uint8 pBuf[KEY_BLENGTH];

    if (aesSignature(pBuf))
    {
      uint8 sBuf[KEY_BLENGTH];
      SBL_NVM_GET(SBL_ADDR_AES_HDR, sBuf, KEY_BLENGTH);

      if (!memcmp(sBuf, pBuf, KEY_BLENGTH))
      {
        memset(pBuf, 0x00, KEY_BLENGTH);
        memset(sBuf, 0x00, KEY_BLENGTH);
        imgHdr.crc[1] = imgHdr.crc[0];
        imgHdr.crc[0] = 0xFFFF;
        while (!HalAdcCheckVdd(VDD_MIN_NV));
        SBL_NVM_SET(SBL_ADDR_CRC, imgHdr.crc, sizeof(imgHdr.crc));
        SBL_READ_IMG_HDR();
      }
      memset(sBuf, 0x00, KEY_BLENGTH);
    }
    memset(pBuf, 0x00, KEY_BLENGTH);
  }

  return (imgHdr.crc[0] == imgHdr.crc[1]);
}

/**************************************************************************************************
 * @fn          procSignatureCmd
 *
 * @brief       Run the AES CRC-MAC calculation over the RC image according to the
 *              AES Control Block parameters and update the control block accordingly.
 *              This is used as a tool for signing a binary image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the KEY_BLENGTH-byte buffer to hold the signature calculated.
 *
 * @return      SBL_SUCCESS or SBL_FAILURE to indicate whether or not the Signature was calculated.
 */
static uint8 procSignatureCmd(void *pBuf)
{
  uint8 sBuf[KEY_BLENGTH];

  if (!SBL_SIGNER || !aesSignature(pBuf))
  {
    return SBL_FAILURE;
  }

  SBL_NVM_GET(SBL_ADDR_AES_HDR, sBuf, KEY_BLENGTH);
  if (memcmp(sBuf, pBuf, KEY_BLENGTH))
  {
    SBL_NVM_SET(SBL_ADDR_AES_HDR, pBuf, KEY_BLENGTH);
    SBL_NVM_GET(SBL_ADDR_AES_HDR, sBuf, KEY_BLENGTH);
  }

  signMode = TRUE;  // Now the Signer must encrypt the PC Tool read back.

  return ((memcmp(sBuf, pBuf, KEY_BLENGTH) == 0) ? SBL_SUCCESS : SBL_FAILURE);
}

/**************************************************************************************************
*/
