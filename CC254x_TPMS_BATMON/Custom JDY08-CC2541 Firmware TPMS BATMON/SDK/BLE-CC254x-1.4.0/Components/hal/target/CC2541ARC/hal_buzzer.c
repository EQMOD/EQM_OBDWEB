/**************************************************************************************************
  Filename:       hal_buzzer.c
  Revised:        $Date: 2012-05-29 16:28:12 -0700 (Tue, 29 May 2012) $
  Revision:       $Revision: 30649 $

  Description:    This file contains the interface to control the buzzer.


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

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
#include "hal_buzzer.h"
#include "hal_drivers.h"
#include "osal.h"

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* Defines for Timer 4 */
#define HAL_T4_CC0_VALUE                125   /* provides pulse width of 125 usec */
#define HAL_T4_TIMER_CTL_DIV32          0xA0  /* Clock pre-scaled by 32 */
#define HAL_T4_TIMER_CTL_START          0x10
#define HAL_T4_TIMER_CTL_CLEAR          0x04
#define HAL_T4_TIMER_CTL_OPMODE_MODULO  0x02  /* Modulo Mode, Count from 0 to CompareValue */
#define HAL_T4_TIMER_CCTL_MODE_COMPARE  0x04
#define HAL_T4_TIMER_CCTL_CMP_TOGGLE    0x10
#define HAL_T4_TIMER_CTL_DIV64          0xC0  /* Clock pre-scaled by 64 */

/* The following define which port pins are being used by the buzzer */
#define HAL_BUZZER_P1_GPIO_PINS  ( BV( 0 ) )

/* These defines indicate the direction of each pin */
#define HAL_BUZZER_P1_OUTPUT_PINS  ( BV( 0 ) )

/* Defines for each output pin assignment */
#define HAL_BUZZER_ENABLE_PIN  P1_0

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

#define HAL_BUZZER_DISABLE() ( HAL_BUZZER_ENABLE_PIN = 0 )

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
/* Function to call when ringing of buzzer is complete */
static halBuzzerCBack_t pHalBuzzerRingCompleteNotificationFunction;

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalBuzzerInit
 *
 * @brief   Initilize buzzer hardware
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalBuzzerInit( void )
{
  pHalBuzzerRingCompleteNotificationFunction = NULL;

  /* Initialize outputs */
  HAL_BUZZER_DISABLE();

  /* Configure direction of pins related to buzzer */
  P1DIR |= (uint8) HAL_BUZZER_P1_OUTPUT_PINS;
}

/**************************************************************************************************
 * @fn          HalBuzzerRing
 *
 * @brief       This function rings the buzzer once.
 *
 * input parameters
 *
 * @param       msec - Number of msec to ring the buzzer
 * @param       tone - Type of tone (low or high)
 * @param       buzzerCback - Callback function to call when ringing of buzzer is finished
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalBuzzerRing( uint16 msec,
                    uint8 tone,
                    halBuzzerCBack_t buzzerCback )
{
  /* Register the callback fucntion */
  pHalBuzzerRingCompleteNotificationFunction = buzzerCback;

  /* Configure output pin as peripheral since we're using T4 to generate */
  P1SEL |= (uint8) HAL_BUZZER_P1_GPIO_PINS;

  if ( tone == HAL_BUZZER_LOW_TONE )
  {
    /* Buzzer is "rung" by using T4, channel 0 to generate 2kHz square wave */
    T4CTL = HAL_T4_TIMER_CTL_DIV64 |
            HAL_T4_TIMER_CTL_CLEAR |
            HAL_T4_TIMER_CTL_OPMODE_MODULO;
  }
  else // tone == HAL_BUZZER_HIGH_TONE
  {
    /* Buzzer is "rung" by using T4, channel 0 to generate 4kHz square wave */
    T4CTL = HAL_T4_TIMER_CTL_DIV32 |
            HAL_T4_TIMER_CTL_CLEAR |
            HAL_T4_TIMER_CTL_OPMODE_MODULO;
  }

  T4CCTL0 = HAL_T4_TIMER_CCTL_MODE_COMPARE | HAL_T4_TIMER_CCTL_CMP_TOGGLE;
  T4CC0 = HAL_T4_CC0_VALUE;

  /* Start it */
  T4CTL |= HAL_T4_TIMER_CTL_START;

  /* Setup timer that will end the buzzing */
  osal_start_timerEx( Hal_TaskID,
                      HAL_BUZZER_EVENT,
                      msec );

}

/**************************************************************************************************
 * @fn      HalBuzzerStop
 *
 * @brief   Halts buzzer
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalBuzzerStop( void )
{
  /* Setting T4CTL to 0 disables it and masks the overflow interrupt */
  T4CTL = 0;

  /* Return output pin to GPIO */
  P1SEL &= (uint8) ~HAL_BUZZER_P1_GPIO_PINS;

  /* Inform application that buzzer is done */
  if (pHalBuzzerRingCompleteNotificationFunction != NULL)
  {
    pHalBuzzerRingCompleteNotificationFunction();
  }
}

/**************************************************************************************************
**************************************************************************************************/
