/**************************************************************************************************
  Filename:       hal_accel.h
  Revised:        $Date: 2012-05-29 16:28:12 -0700 (Tue, 29 May 2012) $
  Revision:       $Revision: 30649 $

  Description:

  This file contains the declaration to the HAL Accelerometer abstraction layer.


  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
#ifndef HAL_ACCEL_H
#define HAL_ACCEL_H

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
#define HAL_ACCEL_I2C_ADDRESS 0x0F
#define HAL_ACCEL_OUTPUT_DATA_ADDRESS 0x06 // corresponds to X_OUT register address

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
typedef void (*halAccelCBack_t) ( void );

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalAccelInit
 *
 * @brief       This function initializes the HAL Accelerometer abstraction layer.
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
void HalAccelInit( void );

/**************************************************************************************************
 * @fn          HalAccelEnable
 *
 * @brief       This function configures the accelerometer for operation.
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
void HalAccelEnable( void );

/**************************************************************************************************
 * @fn          HalAccelDisable
 *
 * @brief       Places the Accelerometer in low power mode.
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
void HalAccelDisable( void );

/**************************************************************************************************
 * @fn          HalAccelMotionDetect
 *
 * @brief       Places the Accelerometer in motion detection mode.
 *
 * input parameters
 *
 * @param       cback - callback function to invoke if motion is detected
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalAccelMotionDetect( halAccelCBack_t cback );

/**************************************************************************************************
 * @fn          HalAccelRead
 *
 * @brief       Reads Y and Z data out registers from Accelerometer.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @param       *y - Y register value.
 * @param       *z - Z register value.
 *
 * @return      None.
 */
void HalAccelRead( int8 *y, int8 *z );

/**************************************************************************************************
 * @fn          HalAccelProcessMotionDetectEvent
 *
 * @brief       Post interrupt processing when motion detect interrupt has been received.
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
void halAccelProcessMotionDetectEvent( void );

/**************************************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/
