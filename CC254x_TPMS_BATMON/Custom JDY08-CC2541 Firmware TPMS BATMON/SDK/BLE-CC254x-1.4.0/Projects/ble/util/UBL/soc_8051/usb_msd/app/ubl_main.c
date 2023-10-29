/**************************************************************************************************
  Filename:       ubl_main.c
  Revised:        $Date: 2012-09-12 10:59:31 -0700 (Wed, 12 Sep 2012) $
  Revision:       $Revision: 31516 $

  Description:

  This module implements the main functionality of a Universal Boot Loader for an 8051-based SOC
  via the USB by MSD. The functionality is similar to ZStack OnBoard.c and hal_startup.c.


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

#include "hal_board_cfg.h"
#include "hal_dma.h"
#include "hal_flash.h"
#include "ubl_app.h"
#include "ubl_exec.h"
#include "usb_interrupt.h"

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init halDMADesc_t dmaCh0;  // Needed by the HAL flash write.

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void vddWait(void);

// If the code model is banked, low_level_init must be declared
// __near_func elsa a ?BRET is performed
//
#if (__CODE_MODEL__ == 2)
__near_func __root char
#else
__root char
#endif
__low_level_init(void);

/**************************************************************************************************
 * @fn          __low_level_init
 *
 * @brief       Abort boot loader as soon as possible after a Watchdog reset with a valid RC image.
 *              This function is called by the IAR start-up code before doing normal initialization
 *              of the data segments.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      0 - don't intialize data segments / 1 - do initialization.
 */
#if (__CODE_MODEL__ == 2)
__near_func __root char
#else
__root char
#endif
__low_level_init(void)
{
  // Map flash bank #7 into XDATA for access to "ROM mapped as data".
  MEMCTR = (MEMCTR & 0xF8) | 0x07;

  if (ResetWasWatchDog)
  {
    // Read from Flash the minimum subset of ublMetaData_t necessary for UBL_RC_VALID().
    HalFlashRead(UBL_META_DATA_PAGE, UBL_META_DATA_IDX, (uint8 *)&ublMD.crcRC, 4);

    if (UBL_RC_VALID)
    {
      ublJump();
    }
  }

  vddWait();
  HAL_BOARD_INIT();

  // Choose if segment initialization should be done or not: 0 to omit seg_init; 1 to run seg_init.
  return 1;
}

/**************************************************************************************************
 * @fn          main
 *
 * @brief       ISR for the reset vector, invoked by IAR after all segment initialization.
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
void main(void)
{
  /* This is in place of calling HalDmaInit() which would require init of the other 4 DMA
   * descriptors in addition to just Channel 0.
   */
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);

  ublInit();
  ublExec();

  HAL_SYSTEM_RESET();
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
*/
