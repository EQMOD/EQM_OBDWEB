/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2012-10-31 16:16:01 -0700 (Wed, 31 Oct 2012) $
  Revision:       $Revision: 32004 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

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
/*********************************************************************
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"
#include "osal_clock.h"
#include "hal_sleep.h"

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

#define HAL_KEY_BIT0            0x01
#define HAL_KEY_BIT1            0x02
#define HAL_KEY_BIT2            0x04
#define HAL_KEY_BIT3            0x08
#define HAL_KEY_BIT4            0x10
#define HAL_KEY_BIT5            0x20
#define HAL_KEY_BIT6            0x40
#define HAL_KEY_BIT7            0x80

#define HAL_KEY_RISING_EDGE     0
#define HAL_KEY_FALLING_EDGE    1

#define HAL_KEY_PDUP2           0x80
#define HAL_KEY_PDUP1           0x40
#define HAL_KEY_PDUP0           0x20

#define HAL_KEY_DEBOUNCE_VALUE  25  // TODO: adjust this value
#define HAL_KEY_POLLING_VALUE   100

#define HAL_KEY_CODE_NOKEY      0xFF

/* Define number of rows and columns in keypad matrix */
#define HAL_KEY_NUM_ROWS        3
#define HAL_KEY_NUM_COLUMNS     16

/* The following define which port pins are being used by keypad service */
#define HAL_KEY_P0_GPIO_PINS  ( HAL_KEY_BIT0 | HAL_KEY_BIT1 | HAL_KEY_BIT2 | HAL_KEY_BIT3 | HAL_KEY_BIT4)
#define HAL_KEY_P1_GPIO_PINS  ( HAL_KEY_BIT4 )

/* These defines indicate the direction of each pin */
#define HAL_KEY_P0_INPUT_PINS   ( HAL_KEY_BIT0 | HAL_KEY_BIT1 | HAL_KEY_BIT2 )
#define HAL_KEY_P0_OUTPUT_PINS  ( HAL_KEY_BIT3 | HAL_KEY_BIT4 )
#define HAL_KEY_P1_INPUT_PINS   0x00
#define HAL_KEY_P1_OUTPUT_PINS  ( HAL_KEY_BIT4 )

/* Which pins are used for key interrupts */
#define HAL_KEY_P0_INTERRUPT_PINS   ( HAL_KEY_BIT0 | HAL_KEY_BIT1 | HAL_KEY_BIT2 )

/* Defines for each output pin assignment */
#define HAL_KEY_SHIFT_REGISTER_CLOCK_PIN  P0_3
#define HAL_KEY_SHIFT_REGISTER_POWER_PIN  P0_4
#define HAL_KEY_SHIFT_REGISTER_DATA_PIN   P1_4
#define HAL_KEY_SHIFT_REGISTER_POWER_OFF 0
#define HAL_KEY_SHIFT_REGISTER_POWER_ON 1

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */
uint8 halSaveIntKey;              /* used by ISR to save state of interrupt-driven keys */

static uint8 HalKeyConfigured;
static uint8 halKeyTimerRunning;  // Set to true while polling timer is running in interrupt
                                  // enabled mode

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt (void);
void halClockShiftRegister (void);
void halPowerDownShiftRegister (void);
void halSetShiftRegisterData( uint8 data );
void halPowerUpShiftRegister( void );

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/
/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
#if (HAL_KEY == TRUE)
  /* Initialize previous key to 0 */
  halKeySavedKeys = HAL_KEY_CODE_NOKEY;

  /* The advanced remote doesn't have the same 8X8 row/column matrix as in other
   * products. Instead, a 3X16 row/column matrix is used, with the rows continuing
   * to be utilized by GPIOs, but the columns are generated via a 16 bit
   * shift register. Controls for the shift register are, however, utilized with
   * GPIOs.
   *
   * Another difference is that the GPIOs utilized for the rows are split between
   * P0 and P1.
   */

  /* Configure pin function as GPIO for pins related to keypad */
  P0SEL &= (uint8) ~HAL_KEY_P0_GPIO_PINS;
  P1SEL &= (uint8) ~HAL_KEY_P1_GPIO_PINS;

  /* Configure direction of pins related to keypad */
  P0DIR |= (uint8) HAL_KEY_P0_OUTPUT_PINS;
  P0DIR &= (uint8) ~HAL_KEY_P0_INPUT_PINS;

  P1DIR |= (uint8) HAL_KEY_P1_OUTPUT_PINS;
  P1DIR &= (uint8) ~HAL_KEY_P1_INPUT_PINS;

  P0INP = 0x00;

  /* Turn off LED pins */
  P1_0 = P1_1 = P1_4 = 0x00;

  /* Set up initial value on output pins */
  halPowerDownShiftRegister();

  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;

  halKeyTimerRunning = FALSE;
#endif /* HAL_KEY */
}

/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
#if (HAL_KEY == TRUE)
  /* Enable/Disable Interrupt */
  Hal_KeyIntEnable = interruptEnable;

  /* Register the callback fucntion */
  pHalKeyProcessFunction = cback;

  /* Determine if interrupt is enabled or not */
  if (Hal_KeyIntEnable)
  {
    /* Configure interrupt for falling edge */
    PICTL |= HAL_KEY_BIT0;

    /* Enable interrupts for individual port pins */
    P0IEN |= HAL_KEY_P0_INTERRUPT_PINS;

    /* Enable P0 interrupts */
    IEN1 |= HAL_KEY_BIT5;

    /* Do this only after the hal_key is configured - to work with sleep stuff */
    if (HalKeyConfigured == TRUE)
    {
      osal_stop_timerEx( Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
    }
  }
  else    /* Interrupts NOT enabled */
  {
    // disable interrupt
    P0IEN &= ~(HAL_KEY_P0_INTERRUPT_PINS);
    IEN1 &= ~(HAL_KEY_BIT5);

    osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_POLLING_VALUE);    /* Kick off polling */
  }

  /* Key now is configured */
  HalKeyConfigured = TRUE;
#endif /* HAL_KEY */
}

/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
  uint8 colkeys = 0, rowkeys = 0;
  uint8 keys, colcode = HAL_KEY_NUM_COLUMNS, rowcode = HAL_KEY_NUM_ROWS;

#if (HAL_KEY == TRUE)

  // Disable interrupt, as interrupt can be triggered without key press during
  // scanning process
  P0IEN &= ~(HAL_KEY_P0_INTERRUPT_PINS);

  // Detect row first while all columns are still asserted.
  keys = (P0 & HAL_KEY_P0_INPUT_PINS);
  for (uint8 row = 0; row < HAL_KEY_NUM_ROWS; row++)
  {
    if (((1 << row) & keys) == 0)
    {
      if (++rowkeys == 1)
      {
        // remember first row code
        rowcode = row;
      }
      else
      {
        // multiple key presses detected
        break;
      }
    }
  }

  /* Now check the columns. Since the pin supplying power to the shift register
   * can't power all the KPa outputs, we will change the KPb pins to have pull down
   * registers, which will result in all KPb pins reading 0. We will then shift 1
   * 1 through the shift register, and the column that is pressed will show up as
   * a 1 on the corresponding KPb pin.
   */
  P2INP |= HAL_KEY_BIT5;
  halPowerUpShiftRegister();

  // Wait some time for things to settle
  halSleepWait( 50 );

  /* Fix to address sending of false key release for power key and fav key */
  halSetShiftRegisterData( 0 );
  halClockShiftRegister();

  /* Check each column by shifting a 1 through the shift register */
  halSetShiftRegisterData( 1 );
  halClockShiftRegister();
  halSetShiftRegisterData( 0 );

  for (uint8 col = 0; col < HAL_KEY_NUM_COLUMNS; col++)
  {
    // read all rows
    keys = (P0 & HAL_KEY_P0_INPUT_PINS);

    // de-assert the column
    halClockShiftRegister();

    // check whether the specific row was on.
    if (keys & ((uint8)1 << rowcode))
    {
      if (++colkeys == 1)
      {
        // remember first col code
        colcode = col;
      }
      else
      {
        // multiple key presses detected
        break;
      }
    }
  }

  // assert all columns for interrupt and for lower current consumption
  halPowerDownShiftRegister();

  // change KPb pins back to pull up
  P2INP &= ~HAL_KEY_BIT5;

  if (Hal_KeyIntEnable)
  {
    // clear interrupt flag. It is necessary since key scanning sets
    // interrupt flag bits.
    P0IFG = (uint8) (~HAL_KEY_P0_INTERRUPT_PINS);
    P0IF = 0;

    // re-enable interrupts
    P0IEN |= HAL_KEY_P0_INTERRUPT_PINS;
  }

#endif /* HAL_KEY */

  // multiple key presses not supported
  if ((colcode == HAL_KEY_NUM_COLUMNS) ||
      (rowcode == HAL_KEY_NUM_ROWS)    ||
      (rowkeys > 1)                    ||
      (colkeys > 1))
  {
    keys = HAL_KEY_CODE_NOKEY; // no key pressed
  }
  else
  {
    keys = (rowcode << 4) | colcode;
  }

  return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll (void)
{
#if (HAL_KEY == TRUE)

  uint8 keys = 0;

  /*
  *  If interrupts are enabled, get the status of the interrupt-driven keys from 'halSaveIntKey'
  *  which is updated by the key ISR.  If Polling, read these keys directly.
  */
  keys = HalKeyRead();

  /* Exit if polling and no keys have changed */
  if (!Hal_KeyIntEnable)
  {
    if (keys == halKeySavedKeys)
    {
      return;
    }
    halKeySavedKeys = keys;     /* Store the current keys for comparation next time */
  }

  /* Invoke Callback if new keys were depressed */
  if ((keys != HAL_KEY_CODE_NOKEY || Hal_KeyIntEnable) &&
      (pHalKeyProcessFunction))
  {
    // When interrupt is enabled, send HAL_KEY_CODE_NOKEY as well so that
    // application would know the previous key is no longer depressed.
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
  }

  if (Hal_KeyIntEnable)
  {
    if (keys != HAL_KEY_CODE_NOKEY)
    {
      // In order to trigger callback again as far as the key is depressed,
      // timer is called here.
      osal_start_timerEx(Hal_TaskID, HAL_KEY_EVENT, 50);
    }
    else
    {
      halKeyTimerRunning = FALSE;
    }
  }
#endif /* HAL_KEY */

}

/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{

#if (HAL_KEY == TRUE)

  if (P0IFG & HAL_KEY_P0_INTERRUPT_PINS)
  {
    // Disable interrupt
    P0IEN &= (uint8) ~HAL_KEY_P0_INTERRUPT_PINS;

    // interrupt flag has been set
    P0IFG = (uint8) (~HAL_KEY_P0_INTERRUPT_PINS); // clear interrupt flag
    if (!halKeyTimerRunning)
    {
      halKeyTimerRunning = TRUE;
      osalTimeUpdate();
      osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
    }
    // Enable interrupt
    P0IEN |= HAL_KEY_P0_INTERRUPT_PINS;
  }
#endif /* HAL_KEY */
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
  /* Sleep!!!
   * Nothing to do.
   */
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
  /* Wakeup!!!
   * Nothing to do. In fact. HalKeyRead() may not be called here.
   * Calling HalKeyRead() will trigger key scanning and interrupt flag clearing in the end,
   * which is no longer compatible with hal_sleep.c module.
   */
  /* Wake up and read keys */
  return TRUE;
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINES
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  HAL_ENTER_ISR();

  halProcessKeyInterrupt();

#if HAL_KEY
  P0IFG = (uint8) (~HAL_KEY_P0_INTERRUPT_PINS);
  P0IF = 0;
#endif

  HAL_EXIT_ISR();
}

/**************************************************************************************************
 * @fn      halClockShiftRegister
 *
 * @brief   Simply provides a single clock pulse to the shift register.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halClockShiftRegister( void )
{
    HAL_KEY_SHIFT_REGISTER_CLOCK_PIN = 1;
    HAL_KEY_SHIFT_REGISTER_CLOCK_PIN = 0;
}

/**************************************************************************************************
 * @fn      halPowerDownShiftRegister
 *
 * @brief   Disables the shift register to save power.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halPowerDownShiftRegister( void )
{
  HAL_KEY_SHIFT_REGISTER_CLOCK_PIN = 0;
  HAL_KEY_SHIFT_REGISTER_POWER_PIN = HAL_KEY_SHIFT_REGISTER_POWER_OFF;
  HAL_KEY_SHIFT_REGISTER_DATA_PIN = 0;
}

/**************************************************************************************************
 * @fn      halPowerUpShiftRegister
 *
 * @brief   Supplies power to the shift register.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halPowerUpShiftRegister( void )
{
  HAL_KEY_SHIFT_REGISTER_POWER_PIN = HAL_KEY_SHIFT_REGISTER_POWER_ON;
}

/**************************************************************************************************
 * @fn      halSetShiftRegisterData
 *
 * @brief   Writes data to the input of the shift register.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halSetShiftRegisterData( uint8 data )
{
  /* Data input is 1 bit, so make sure we only use LSB */
  HAL_KEY_SHIFT_REGISTER_DATA_PIN = (data & 0x01);
}

/**************************************************************************************************
**************************************************************************************************/
