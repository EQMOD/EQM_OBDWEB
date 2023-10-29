/**************************************************************************************************
  Filename:       hal_gyro.h
  Revised:        $Date: 2012-10-19 16:06:31 -0700 (Fri, 19 Oct 2012) $
  Revision:       $Revision: 31875 $

  Description:

  This file contains the declaration to the HAL Gyroscope abstraction layer.


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
#ifndef HAL_GYRO_H
#define HAL_GYRO_H

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
#define HAL_GYRO_OUTPUT_SETTLING_TIME 200 // time from power on until data valid, in ms
#define HAL_GYRO_CAL_SETTLING_TIME 10 // time from AZ asserted until outputs settled, in ms
#define HAL_GYRO_I2C_ADDRESS 0x68

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
typedef void (*halGyroEnableCback_t) ( void );

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalGyroInit
 *
 * @brief       This function initializes the HAL Gyroscope abstraction layer.
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
 **************************************************************************************************/
void HalGyroInit( void );

/**************************************************************************************************
 * @fn          HalGyroEnable
 *
 * @brief       This function configures the gyro for operation.
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
 **************************************************************************************************/
void HalGyroEnable( halGyroEnableCback_t cback );

/**************************************************************************************************
 * @fn          HalGyroHandleGyroRegisterAccessReadyEvent
 *
 * @brief       Callback to handle expiry of gyro powerup wait timer.
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************************/
void HalGyroHandleGyroRegisterAccessReadyEvent( void );

/**************************************************************************************************
 * @fn          HalGyroDisable
 *
 * @brief       This function powers off the gyro.
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
 **************************************************************************************************/
void HalGyroDisable( void );

/**************************************************************************************************
 * @fn      HalGyroRead
 *
 * @brief   Performs A/D reading on channels connected to X, Y, and Z output on Gyroscope.
 *
 * @param   None
 *
 * @return  X, Y, and Z channel readings
 **************************************************************************************************/
void HalGyroRead( int16 *x, int16 *y, int16 *z );

/**************************************************************************************************
 * @fn          HalGyroReadAccelData
 *
 * @brief       Reads X, Y and Z data out registers from Accelerometer.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @param       *x - X register value.
 * @param       *y - Y register value.
 * @param       *z - Z register value.
 *
 * @return      None.
 */
void HalGyroReadAccelData( int8 *x, int8 *y, int8 *z );

/**************************************************************************************************
 * @fn      HalGyroEnableI2CPassThru
 *
 * @brief   Enables I2C pass thru mode on the IMU-3000, which allows the MCU to talk to the
 *          accelerometer.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGyroEnableI2CPassThru( void );

/**************************************************************************************************
 * @fn      HalGyroDisableI2CPassThru
 *
 * @brief   Disables I2C pass thru mode on the IMU-3000, which means the gyro will master the I2C
 *          accesses to the accelerometer.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGyroDisableI2CPassThru( void );

/**************************************************************************************************
 * @fn      HalGyroStartGyroMeasurements
 *
 * @brief   Enables the gyro clock and also configures the gyro to control the aux I2C
 *          interface so it can read accelerometer data.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGyroStartGyroMeasurements( void );

#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/
