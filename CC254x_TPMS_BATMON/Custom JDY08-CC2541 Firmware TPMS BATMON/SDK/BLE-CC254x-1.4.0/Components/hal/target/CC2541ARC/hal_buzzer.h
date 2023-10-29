/**************************************************************************************************
  Filename:       hal_buzzer.h
  Revised:        $Date: 2012-05-29 16:28:12 -0700 (Tue, 29 May 2012) $
  Revision:       $Revision: 30649 $

  Description:

  This file contains the declaration to the HAL buzzer abstraction layer.


  Copyright 2010-2013 Texas Instruments Incorporated. All rights reserved.

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
#ifndef HAL_BUZZER_H
#define HAL_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "comdef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_BUZZER_LOW_TONE     0
#define HAL_BUZZER_HIGH_TONE    1

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
/* Callback function for when ringing of buzzer is complete */
typedef void (*halBuzzerCBack_t) (void);

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalBuzzerInit
 *
 * @brief       Initilize buzzer hardware
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
void HalBuzzerInit( void );

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
                    halBuzzerCBack_t buzzerCback );

/**************************************************************************************************
 * @fn      HalBuzzerStop
 *
 * @brief   Halts buzzer
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalBuzzerStop( void );

#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/
