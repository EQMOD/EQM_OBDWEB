/**************************************************************************************************
  Filename:       hal_interrupt.c
  Revised:        $Date: 2012-08-10 19:21:08 -0700 (Fri, 10 Aug 2012) $
  Revision:       $Revision: 31192 $

  Description:    Interrupt services routines for sensors


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
#include "hal_keys.h"
#include "hal_acc.h"
#include "hal_mag.h"
#include "hal_i2c.h"
/***************************************************************************************************
*                                    INTERRUPT SERVICE ROUTINE
***************************************************************************************************/

HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{

  HAL_ENTER_ISR();
  if (HAL_KEY_SW_1_PXIFG & HAL_KEY_SW_1_BIT)
  {
    halProcessKeyInterrupt();
  }

  /*
  Clear the CPU interrupt flag for Port_0
  PxIFG has to be cleared before PxIF
  */
  P0IFG = 0;
  P0IF = 0;
  HAL_EXIT_ISR();
}


/**************************************************************************************************
* @fn      halKeyPort1Isr
*
* @brief   Port1 ISR
**************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
    HAL_ENTER_ISR();
    if ((HAL_KEY_SW_2_PXIFG & HAL_KEY_SW_2_BIT) || (HAL_KEY_SW_3_PXIFG & HAL_KEY_SW_3_BIT))
    {
        halProcessKeyInterrupt();
    }
    /*
    Clear the CPU interrupt flag for Port_1
    PxIFG has to be cleared before PxIF
    */
    HAL_KEY_SW_2_PXIFG = 0;
    HAL_KEY_SW_3_PXIFG = 0;
    P1IF = 0;
    HAL_EXIT_ISR();
}


/**************************************************************************************************
* @fn      halKeyPort2Isr
*
* @brief   Port2 ISR
**************************************************************************************************/

HAL_ISR_FUNCTION( halI2CIsr, I2C_VECTOR )
{
  HAL_ENTER_ISR();

  /*
    Clear the CPU interrupt flag for Port_2
    PxIFG has to be cleared before PxIF
    Notes: P2_1 and P2_2 are debug lines.
  */
  P2IFG = 0;
  P2IF    = 0;

  HAL_EXIT_ISR();
}
