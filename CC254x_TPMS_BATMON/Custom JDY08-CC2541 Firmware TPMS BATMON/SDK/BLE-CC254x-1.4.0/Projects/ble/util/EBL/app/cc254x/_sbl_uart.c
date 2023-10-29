/**************************************************************************************************
  Filename:       _sbl_uart.c
  Revised:        $Date: 2012-09-07 14:46:45 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31500 $

  Description:

  This file contains the interface to the H/W transport driver by UART for the boot loader.


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

#include "hal_board.h"
#include "hal_mcu.h"
#include "hal_rpc.h"
#include "hal_types.h"
#include "hal_uart.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

// UxCSR - USART Control and Status Register.
#define CSR_MODE                   0x80
#define CSR_RE                     0x40
#define CSR_SLAVE                  0x20
#define CSR_FE                     0x10
#define CSR_ERR                    0x08
#define CSR_RX_BYTE                0x04
#define CSR_TX_BYTE                0x02
#define CSR_ACTIVE                 0x01

// UxUCR - USART UART Control Register.
#define UCR_FLUSH                  0x80
#define UCR_FLOW                   0x40
#define UCR_D9                     0x20
#define UCR_BIT9                   0x10
#define UCR_PARITY                 0x08
#define UCR_SPB                    0x04
#define UCR_STOP                   0x02
#define UCR_START                  0x01

#define P2DIR_PRIPO                0xC0

#undef  UxCSR
#undef  UxUCR
#undef  UxDBUF
#undef  UxBAUD
#undef  UxGCR
#if    (HAL_UART_SBL == 1)
#define UxCSR                      U0CSR
#define UxUCR                      U0UCR
#define UxDBUF                     U0DBUF
#define UxBAUD                     U0BAUD
#define UxGCR                      U0GCR
#elif  (HAL_UART_SBL == 2)
#define UxCSR                      U1CSR
#define UxUCR                      U1UCR
#define UxDBUF                     U1DBUF
#define UxBAUD                     U1BAUD
#define UxGCR                      U1GCR
#endif

#undef  PxSEL
#undef  HAL_UART_PERCFG_BIT
#undef  HAL_UART_PRIPO
#undef  HAL_UART_Px_SEL
#if    (HAL_UART_SBL == 1)
#define PxSEL                      P0SEL
#define HAL_UART_PERCFG_BIT        0x01         // USART0 on P0, Alt-1; so clear this bit.
#define HAL_UART_PRIPO             0x00         // USART0 priority over UART1.
#define HAL_UART_Px_SEL            0x0C         // Peripheral I/O Select for Rx/Tx.
#elif  (HAL_UART_SBL == 2)
#define PxSEL                      P1SEL
#define HAL_UART_PERCFG_BIT        0x02         // USART1 on P1, Alt-2; so set this bit.
#define HAL_UART_PRIPO             0x40         // USART1 priority over UART0.
#define HAL_UART_Px_SEL            0xC0         // Peripheral I/O Select for Rx/Tx.
#endif

#if defined POWER_SAVING
#undef  PxDIR
#if   (HAL_UART_SBL == 1)
#define PxDIR                      P0DIR
#define DMA_RDYIn                  P0_4
#define DMA_RDYOut                 P0_5
#define DMA_RDYOut_BIT             BV(5)        // Same as the I/O Select for manual RTS flow ctrl.
#elif  (HAL_UART_SBL == 2)
#define PxDIR                      P1DIR
#define DMA_RDYIn                  P1_4
#define DMA_RDYOut                 P1_5
#define DMA_RDYOut_BIT             BV(5)        // Same as the I/O Select for manual RTS flow ctrl.
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_UART_SBL_CLR_RDY_OUT()     (DMA_RDYOut = 1)
#define HAL_UART_SBL_SET_RDY_OUT()     (DMA_RDYOut = 0)

#define HAL_UART_SBL_RDY_IN()          (DMA_RDYIn == 0)
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 txIdx, txLen, *pTxBuf;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          uartInit
 *
 * @brief       Initialize the UART for SBL polling use.
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
static void uartInit(void)
{
#if defined POWER_SAVING
  /* Although the SBL does not implement PM, follow the protocol so that a Host App does not need
   * to drive the UART differently for communication with the NP versus the BL.
   */
  HAL_UART_SBL_SET_RDY_OUT();
  PxDIR |= DMA_RDYOut_BIT;
#endif

  UxCSR = CSR_MODE;                  // Mode is UART Mode.
  UxUCR = UCR_FLUSH;                 // Flush it.
#if (HAL_UART_SBL == 1)
  PERCFG &= ~HAL_UART_PERCFG_BIT;    // Set UART0 I/O to Alt. 1 location on P0.
#else
  PERCFG |= HAL_UART_PERCFG_BIT;     // Set UART1 I/O to Alt. 2 location on P1.
#endif
  PxSEL  |= HAL_UART_Px_SEL;         // Enable Peripheral control of Rx/Tx on Px.

  P2DIR &= ~P2DIR_PRIPO;
  P2DIR |= HAL_UART_PRIPO;

  // HAL_UART_BR_115200
  UxBAUD = 216;
  UxGCR = 11;

  UxUCR = UCR_STOP;
  UxCSR = (CSR_MODE | CSR_RE);
}

/**************************************************************************************************
 * @fn          sblRun
 *
 * @brief       Serial Boot run code for the UART transport.
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
static void sblRun(void)
{
  uint8 resetF = 0;
  uartInit();

  while (1)
  {
    if (txLen == 0)
    {
      if ((resetF != 0) && ((UxCSR & CSR_ACTIVE) == 0))  // If the last Tx byte has flushed out.
      {
        HAL_SYSTEM_RESET();
      }
    }
    else if (UxCSR & CSR_TX_BYTE)
    {
      UxCSR &= ~CSR_TX_BYTE;
      UxDBUF = pTxBuf[txIdx];

      if (++txIdx == txLen)
      {
        txLen = 0;
      }
    }
#if defined POWER_SAVING
    else if (!HAL_UART_SBL_RDY_IN())
    {
      // Master may have timed-out the SRDY asserted state & may need a new edge.
      HAL_UART_SBL_CLR_RDY_OUT();
      ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;
      HAL_UART_SBL_SET_RDY_OUT();
    }
#endif

    if (UxCSR & CSR_RX_BYTE)
    {
      resetF |= sblPoll();
    }
  }
}

/**************************************************************************************************
 * @fn          sblWait
 *
 * @brief       Serial Boot wait & poll for a Force SBL or Force Jump after a hard reset.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE to Force Boot Loader mode or not.
 */
static uint8 sblWait(void)
{
  uartInit();

  for (uint32 delay = 0; delay < 0x00100000; delay++)
  {
    if (UxCSR & CSR_RX_BYTE)
    {
      uint8 ch = UxDBUF;

      if (ch == SBL_FORCE_BOOT)
      {
        return TRUE;
      }
      else if (ch == SBL_FORCE_RUN)
      {
        return FALSE;
      }
    }
  }

  return FALSE;
}


uint16 HalUARTRead(uint8 port, uint8 *pBuffer, uint16 length)
{
  (void)port;
  (void)length;

  if (UxCSR & CSR_RX_BYTE)
  {
    *pBuffer = UxDBUF;
    return 1;
  }

  return 0;
}

uint16 HalUARTWrite(uint8 port, uint8 *pBuffer, uint16 length)
{
  (void)port;
  pTxBuf = pBuffer;
  txLen = length;
    txIdx = 1;
  UxCSR &= ~CSR_TX_BYTE;
  UxDBUF = *pBuffer;  // SBL does not generate messages asynchronously, so Tx will always be ready.

  return length;
}

/**************************************************************************************************
*/
