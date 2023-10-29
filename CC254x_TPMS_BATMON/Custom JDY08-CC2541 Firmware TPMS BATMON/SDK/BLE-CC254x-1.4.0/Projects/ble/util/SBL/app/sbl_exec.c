/**************************************************************************************************
  Filename:       sbl_exec.c
  Revised:        $Date: 2012-09-07 14:26:14 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31498 $

  Description:  Serial Bootloader Executive.


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

#include "hal_adc.h"
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

#define SBL_RW_BUF_LEN               64

// Commands to Bootloader
#define SBL_WRITE_CMD                0x01
#define SBL_READ_CMD                 0x02
#define SBL_ENABLE_CMD               0x03
#define SBL_HANDSHAKE_CMD            0x04

// Commands to Target Application
#define SBL_TGT_BOOTLOAD             0x10  // Erase the image valid signature & jump to bootloader.

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

// Usually only the OAD bootloader should suffer the code size of calculating the CRC.
#if !defined SBL_CALC_CRC
#define SBL_CALC_CRC  FALSE
#endif

// Buffer size - it has to be big enough for the largest RPC packet and overhead.
#define SBL_BUF_SIZE                 256
#define SBL_MAX_SIZE                (SBL_BUF_SIZE - RPC_FRAME_HDR_SZ - RPC_UART_FRAME_OVHD)

// The SB page boundary since all SB addresses are "actual address / flash word size".
// Note for MSP - flash word size is 1, but 4 must be used for inter-compatibility w/ SoC.
#define SBL_PAGE_SIZE               (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)

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

/* ------------------------------------------------------------------------------------------------
 *                                     Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void   sblProc(void);
static uint8  sblResp(void);
static uint16 calcCRC(void);
static uint8  checkRC(void);
static uint16 crc16(uint16 crc, uint8 val);

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
  return checkRC();
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
  uint16 t16 = BUILD_UINT16(sbBuf[SBL_REQ_ADDR_LSB], sbBuf[SBL_REQ_ADDR_MSB]) + HAL_SBL_IMG_BEG;
  uint8 len = 1, rsp = SBL_SUCCESS;
  uint16 crc[2];

  switch (sbBuf[RPC_POS_CMD1])
  {
  case SBL_WRITE_CMD:
    if ((t16 % SBL_PAGE_SIZE) == 0)
    {
      HalFlashErase(t16 / SBL_PAGE_SIZE);
    }
    HalFlashWrite(t16, (sbBuf + SBL_REQ_DAT0), (SBL_RW_BUF_LEN / HAL_FLASH_WORD_SIZE));
    break;

  case SBL_READ_CMD:
    len = SBL_RW_BUF_LEN + SBL_READ_HDR_LEN;
    sbBuf[SBL_RSP_ADDR_MSB] = sbBuf[SBL_REQ_ADDR_MSB];
    sbBuf[SBL_RSP_ADDR_LSB] = sbBuf[SBL_REQ_ADDR_LSB];

    HalFlashRead(t16 / SBL_PAGE_SIZE,
                (t16 % SBL_PAGE_SIZE) << 2, (sbBuf + SBL_RSP_DAT0), SBL_RW_BUF_LEN);
    break;

  case SBL_ENABLE_CMD:
    HalFlashRead(HAL_SBL_IMG_CRC / SBL_PAGE_SIZE,
                (HAL_SBL_IMG_CRC % SBL_PAGE_SIZE) << 2, (uint8 *)crc, sizeof(crc));

    // Bootload master must have verified extra checks to be issuing the SBL_ENABLE_CMD.
    //if ((crc[0] != crc[1]) && (crc[0] != 0xFFFF) && (crc[0] != 0x0000))
    if (crc[1] != crc[0])
    {
      crc[1] = crc[0];
      crc[0] = 0xFFFF;
      HalFlashWrite(HAL_SBL_IMG_CRC, (uint8 *)crc, 1);
      HalFlashRead(HAL_SBL_IMG_CRC / SBL_PAGE_SIZE,
                  (HAL_SBL_IMG_CRC % SBL_PAGE_SIZE) << 2, (uint8 *)crc, sizeof(crc));
    }

    // Bootload master must have verified extra checks to be issuing the SBL_ENABLE_CMD.
    //if ((crc[0] == crc[1]) && (crc[0] != 0xFFFF) && (crc[0] != 0x0000))
    if (crc[0] != crc[1])
    {
      rsp = SBL_VALIDATE_FAILED;
    }
    break;

  case SBL_HANDSHAKE_CMD:
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
 * @fn          calcCRC
 *
 * @brief       Run the CRC16 Polynomial calculation over the RC image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 */
static uint16 calcCRC(void)
{
  uint16 addr, crc = 0;

  for (addr = HAL_SBL_IMG_BEG; addr < HAL_SBL_IMG_END; addr++)
  {
    if (addr != HAL_SBL_IMG_CRC)
    {
      uint8 buf[HAL_FLASH_WORD_SIZE];
      HalFlashRead(addr / SBL_PAGE_SIZE,
                  (addr % SBL_PAGE_SIZE) << 2, buf, HAL_FLASH_WORD_SIZE);
      for (uint8 idx = 0; idx < HAL_FLASH_WORD_SIZE; idx++)
      {
        crc = crc16(crc, buf[idx]);
      }
    }
  }

  // IAR note explains that poly must be run with value zero for each byte of crc.
  crc = crc16(crc, 0);
  crc = crc16(crc, 0);

  return crc;
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
  uint16 crc[2];
  HalFlashRead(HAL_SBL_IMG_CRC / SBL_PAGE_SIZE,
              (HAL_SBL_IMG_CRC % SBL_PAGE_SIZE) << 2, (uint8 *)crc, HAL_FLASH_WORD_SIZE);

  if ((crc[0] == 0) || (crc[0] == 0xFFFF))
  {
    return FALSE;
  }

  if (SBL_CALC_CRC && (crc[0] != crc[1])&& (crc[1] == 0xFFFF))
  {
    crc[1] = calcCRC();
    crc[0] = 0xFFFF;
    while (!HalAdcCheckVdd(VDD_MIN_NV));
    HalFlashWrite(HAL_SBL_IMG_CRC, (uint8 *)crc, 1);
    HalFlashRead(HAL_SBL_IMG_CRC / SBL_PAGE_SIZE,
                (HAL_SBL_IMG_CRC % SBL_PAGE_SIZE) << 2, (uint8 *)crc, HAL_FLASH_WORD_SIZE);
  }

  return (crc[0] == crc[1]);
}

/**************************************************************************************************
 * @fn          crc16
 *
 * @brief       Run the CRC16 Polynomial calculation over the byte parameter.
 *
 * input parameters
 *
 * @param       crc - Running CRC calculated so far.
 * @param       val - Value on which to run the CRC16.
 *
 * output parameters
 *
 * None.
 *
 * @return      crc - Updated for the run.
 **************************************************************************************************
 */
static uint16 crc16(uint16 crc, uint8 val)
{
  const uint16 poly = 0x1021;
  uint8 cnt;

  for (cnt = 0; cnt < 8; cnt++, val <<= 1)
  {
    uint8 msb = (crc & 0x8000) ? 1 : 0;

    crc <<= 1;
    if (val & 0x80)  crc |= 0x0001;
    if (msb)         crc ^= poly;
  }

  return crc;
}

/**************************************************************************************************
*/
