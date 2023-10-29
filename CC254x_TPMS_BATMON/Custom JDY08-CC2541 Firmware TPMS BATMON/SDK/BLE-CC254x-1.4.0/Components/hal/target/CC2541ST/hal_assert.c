/**************************************************************************************************
  Filename:       hal_assert.c
  Revised:        $Date: 2012-08-08 13:29:09 -0700 (Wed, 08 Aug 2012) $
  Revision:       $Revision: 31145 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2012  Texas Instruments Incorporated. All rights reserved.

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
#include "hal_assert.h"
#include "hal_types.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_mcu.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants and macros
 * ------------------------------------------------------------------------------------------------
 */
#ifdef HAL_LED_BLINK_DELAY
#undef HAL_LED_BLINK_DELAY
#endif

#define HAL_LED_BLINK_DELAY()   st( { volatile uint32 i; for (i=0; i<0x1450; i++) { }; } ) // was: 0x5800

/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void halAssertHazardLights(void);


/**************************************************************************************************
 * @fn          halAssertHandler
 *
 * @brief       Logic to handle an assert.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void halAssertHandler(void)
{
  /* execute code that handles asserts */
#ifdef ASSERT_RESET
  HAL_SYSTEM_RESET();
#elif !defined ASSERT_WHILE
  halAssertHazardLights();
#else
  while(1);
#endif
}

#if !defined ASSERT_WHILE
/**************************************************************************************************
 * @fn          halAssertHazardLights
 *
 * @brief       Blink LEDs to indicate an error.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void halAssertHazardLights(void)
{

  /* disable all interrupts before anything else */
  HAL_DISABLE_INTERRUPTS();

  /*-------------------------------------------------------------------------------
   *  Master infinite loop.
   */
  for (;;)
  {
      HAL_LED_BLINK_DELAY();

      /* toggle LEDS */
      HAL_TOGGLE_LED1();
      HAL_TOGGLE_LED2();
  }
}
#endif

/* ------------------------------------------------------------------------------------------------
 *                                    Compile Time Assertions
 * ------------------------------------------------------------------------------------------------
 */

/* integrity check of type sizes */
HAL_ASSERT_SIZE(  int8, 1);
HAL_ASSERT_SIZE( uint8, 1);
HAL_ASSERT_SIZE( int16, 2);
HAL_ASSERT_SIZE(uint16, 2);
HAL_ASSERT_SIZE( int32, 4);
HAL_ASSERT_SIZE(uint32, 4);


/**************************************************************************************************
*/
