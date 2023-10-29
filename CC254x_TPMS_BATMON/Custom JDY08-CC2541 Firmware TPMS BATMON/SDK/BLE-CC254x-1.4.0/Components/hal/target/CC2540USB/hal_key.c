/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2012-06-22 10:04:42 -0700 (Fri, 22 Jun 2012) $
  Revision:       $Revision: 30773 $

  Description: This file contains the interface to the H/W Key driver.


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_drivers.h"
#include "hal_key.h"
#include "hal_types.h"
#include "osal.h"
#include "usb_interrupt.h"

#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_KEY_CLR_INT() \
st ( \
  /* PxIFG has to be cleared before PxIF. */\
  P1IFG = 0; \
  P1IF = 0; \
)

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

uint8 Hal_KeyIntEnable;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static halKeyCBack_t pHalKeyProcessFunction;
static volatile uint8 isrKeys;
static uint8 halKeys;

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalKeyInit
 *
 * @brief       This function is called by HalDriverInit to initialize the H/W keys.
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
void HalKeyInit(void)
{
}

/**************************************************************************************************
 * @fn          HalKeyConfig
 *
 * @brief       This function is called by HalDriverInit to initialize the H/W keys.
 *
 * input parameters
 *
 * @param       interruptEnable - TRUE/FALSE to enable the key interrupt.
 * @param       cback - The callback function for the key change event.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback)
{
  if ((Hal_KeyIntEnable = interruptEnable))
  {
    HAL_KEY_CLR_INT();             // Clear spurious ints.
    PICTL |= 0x01;                 // P1ICONL: Falling edge ints on pins 0-3.
    P1IEN |= PUSH1_BV | PUSH2_BV;  // Enable specific P1 bits for ints by bit mask.
    IEN2  |= 0x10;                 // Enable general P1 interrupts.
  }
  else
  {
    (void)osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
  }

  pHalKeyProcessFunction = cback;
}

/**************************************************************************************************
 * @fn          HalKeyPoll
 *
 * @brief       This function is called by Hal_ProcessEvent() on a HAL_KEY_EVENT.
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
void HalKeyPoll(void)
{
  uint8 newKeys;

  if (Hal_KeyIntEnable)
  {
    halIntState_t intState;
    HAL_ENTER_CRITICAL_SECTION(intState);
    newKeys = isrKeys;
    isrKeys = 0;
    HAL_EXIT_CRITICAL_SECTION(intState);
  }
  else
  {
    uint8 keys = HalKeyRead();
    newKeys = (halKeys ^ keys) & keys;
    halKeys = keys;
  }

  if (newKeys && pHalKeyProcessFunction)
  {
    (pHalKeyProcessFunction)(newKeys, HAL_KEY_STATE_NORMAL);
  }
}

/**************************************************************************************************
 * @fn          HalKeyRead
 *
 * @brief       This function is called anywhere.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The bit mask of all keys pressed.
 **************************************************************************************************
 */
uint8 HalKeyRead(void)
{
  uint8 keys = 0;

  if (HAL_PUSH_BUTTON1())
  {
    keys |= HAL_KEY_SW_1;
  }

  if (HAL_PUSH_BUTTON2())
  {
    keys |= HAL_KEY_SW_2;
  }

  return keys;
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalKeyRead () );
}

/**************************************************************************************************
 * @fn          usbKeyISR
 *
 * @brief       This function is the ISR for the Port2 USB/Key interrupt.
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
HAL_ISR_FUNCTION( usbKeyISR, P1INT_VECTOR )
{
  HAL_ENTER_ISR();

  if (P1IFG & PUSH1_BV)
  {
    isrKeys |= HAL_KEY_SW_1;
  }

  if (P1IFG & PUSH2_BV)
  {
    isrKeys |= HAL_KEY_SW_2;
  }

  HAL_KEY_CLR_INT();
  (void)osal_set_event(Hal_TaskID, HAL_KEY_EVENT);

  CLEAR_SLEEP_MODE();

  HAL_EXIT_ISR();

  return;
}

#else

void HalKeyInit(void){}
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback){}
uint8 HalKeyRead(void){ return 0;}
void HalKeyPoll(void){}

#endif /* HAL_KEY */

/**************************************************************************************************
*/
