/**************************************************************************************************
  Filename:       sbl_main.c
  Revised:        $Date: 2012-09-07 14:46:45 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31500 $

  Description:

  This module contains the definitions for the main functionality of a Serial Boot Loader.


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
#include "hal_assert.h"
#include "hal_dma.h"
#include "hal_sleep.h"
#include "hal_types.h"
#include "sbl_exec.h"

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

halDMADesc_t dmaCh0;  // Locally setup for use by HalFlashWrite() instead of calling HalDMAInit().

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

#if HAL_UART_SPI
#include "_sbl_spi.c"
#elif HAL_UART_SBL
#include "_sbl_uart.c"
#else
#error No valid transport configured.
#endif

/**************************************************************************************************
 * @fn          main
 *
 * @brief       ISR for the reset vector.
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
  HAL_BOARD_INIT();

  /* This is in place of calling HalDmaInit() which would require init of the other 4 DMA
   * descriptors in addition to just Channel 0.
   */
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);

  if (sblInit() == TRUE)  // If the run code image is valid.
  {
    // Enable the following line to get a 2nd way to enter boot loader:
    // after a hard reset, SBL waits 30 seconds for 0xF8 or 0x07, then jumps.
    //if ((SLEEPSTA & 0x10) || !sblWait())
    {
      // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
      asm("LJMP 0x0800");
      HAL_SYSTEM_RESET();  // Should not get here.
    }
  }

  while (!HalAdcCheckVdd(VDD_MIN_NV));
  sblRun();
  HAL_SYSTEM_RESET();
}

/**************************************************************************************************
 * @fn          halSleepWait
 *
 * @brief       Perform a blocking wait for the specified number of microseconds.
 *              Use assumptions about number of clock cycles needed for the various instructions.
 *              This function assumes a 32 MHz clock.
 *              NB! This function is highly dependent on architecture and compiler!
 *
 * input parameters
 *
 * @param       duration - Duration of wait in microseconds.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
#pragma optimize=none
void halSleepWait(uint16 duration)
{
  duration >>= 1;

  while (duration-- > 0)
  {
    ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;
    ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;
    ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP; ASM_NOP;
  }
}

void halAssertHandler(void)
{
  HAL_SYSTEM_RESET();
}

/**************************************************************************************************
*/
