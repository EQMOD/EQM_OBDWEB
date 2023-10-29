/**************************************************************************************************
  Filename:       hal_motion.h
  Revised:        $Date: 2012-05-29 16:28:12 -0700 (Tue, 29 May 2012) $
  Revision:       $Revision: 30649 $

  Description:

  This file contains the declaration to the HAL Motion Sensor abstraction layer.


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
#ifndef HAL_MOTION_H
#define HAL_MOTION_H

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

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef void (*halMotionCBack_t) (int16 gyroMickeysX, int16 gyroMickeysY);
typedef void (*halMotionCalCBack_t) (void);

enum
{
  HAL_MOTION_RESOLUTION_DECREASE,
  HAL_MOTION_RESOLUTION_INCREASE
};
typedef uint8 halMotionMouseResolution_t;

enum
{
  HAL_MOTION_DEVICE_GYRO,
  HAL_MOTION_DEVICE_ACCELEROMETER
};
typedef uint8 halMotionDevice_t;

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalMotionInit
 *
 * @brief       This function initializes the HAL Motion Sensor abstraction layer.
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
void HalMotionInit(void);

/**************************************************************************************************
 * @fn          HalMotionConfig
 *
 * @brief       This function configures the HAL Motion Sensor abstraction layer.
 *
 * input parameters
 *
 * @param measCback - Pointer to the application layer callback function for processing motion data
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalMotionConfig(halMotionCBack_t measCback);

/**************************************************************************************************
 * @fn          HalMotionEnable
 *
 * @brief       This function powers on the motion sensor hardware and starts the associated H/W timer.
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
void HalMotionEnable(void);

/**************************************************************************************************
 * @fn      HalMotionCal
 *
 * @brief   Calibrate Motion Sensor hardware
 *
 * @param calCback - Pointer to the application layer callback function for notification of calibration complete
 * @param calDuration - duration, in milliseconds, of calibration procedure
 *
 * @return  TRUE if calibration started. FALSE, otherwise.
 **************************************************************************************************/
uint8 HalMotionCal( halMotionCalCBack_t calCback,
                    uint16 calDuration );

/**************************************************************************************************
 * @fn          HalMotionDisable
 *
 * @brief       This function powers off the motion sensor hardware and stops the associated H/W timer.
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
void HalMotionDisable(void);

/**************************************************************************************************
 * @fn      HalMotionStandby
 *
 * @brief   Places motion detection hardware in "standby" mode. This means the
 *          hardware is in a lower power state and will only be re-enabled if
 *          motion is detected.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalMotionStandby( void );

/**************************************************************************************************
 * @fn      HalMotionModifyAirMouseResolution
 *
 * @brief   Increase/decrease air mouse movement resolution
 *
 * @param   action - increase/decrease command
 *
 * @return  None
 **************************************************************************************************/
void HalMotionModifyAirMouseResolution( halMotionMouseResolution_t action );

/**************************************************************************************************
 * @fn      HalMotionHandleOSEvent
 *
 * @brief   Handles Motion Sensor OS Events
 *
 * @param   OS Events that triggered call
 *
 * @return  Event that was handled
 **************************************************************************************************/
uint16 HalMotionHandleOSEvent( uint16 events );

/**************************************************************************************************
 * @fn      HalMotionI2cRead
 *
 * @brief   Reads registers from devices connected to I2C bus
 *
 * @param   device - which device is being written
 *          addr - starting register address to read
 *          numBytes - Number of bytes to read
 *          pBuf - pointer to buffer to place read data
 *
 * @return  None
 **************************************************************************************************/
void HalMotionI2cRead( halMotionDevice_t device, uint8 addr, uint8 numBytes, uint8 *pBuf );

/**************************************************************************************************
 * @fn      HalMotionI2cWrite
 *
 * @brief   Writes registers on devices connected to I2C bus
 *
 * @param   device - which device is being written
 *          addr - starting register address to write
 *          pData - pointer to buffer containing data to be written
 *          numBytes - Number of bytes to read
 *
 * @return  None
 **************************************************************************************************/
void HalMotionI2cWrite( halMotionDevice_t device, uint8 addr, uint8 *pData, uint8 numBytes );

#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/
