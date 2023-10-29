/**************************************************************************************************
  Filename:       hal_sensor.c
  Revised:        $Date: 2012-09-21 06:30:38 -0700 (Fri, 21 Sep 2012) $
  Revision:       $Revision: 31581 $

  Description:    This file contains code that is common to all sensor drivers.


  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_led.h"
#include "hal_irtemp.h"
#include "hal_humi.h"
#include "hal_bar.h"
#include "hal_mag.h"
#include "hal_acc.h"
#include "hal_gyro.h"

/* ------------------------------------------------------------------------------------------------
*                                           Macros and constants
* ------------------------------------------------------------------------------------------------
*/
#define N_TEST_RUNS                           10

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 buffer[24];
static uint8 halSensorEnableMap;

/**************************************************************************************************
 * @fn          HalSensorReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor. The sensor must
 *              be selected before this routine is called.
 *
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - numbver of bytes to read
 *
 * @return      TRUE if the required number of bytes are reveived
 **************************************************************************************************/
bool HalSensorReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
  uint8 i = 0;

  /* Send address we're reading from */
  if (HalI2CWrite(1,&addr) == 1)
  {
    /* Now read data */
    i = HalI2CRead(nBytes,pBuf);
  }

  return i == nBytes;
}

/**************************************************************************************************
* @fn          HalSensorWriteReg
* @brief       This function implements the I2C protocol to write to a sensor. he sensor must
*              be selected before this routine is called.
*
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
bool HalSensorWriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
  uint8 i;
  uint8 *p = buffer;

  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;

  /* Send address and data */
  i = HalI2CWrite(nBytes, buffer);
  if ( i!= nBytes)
    HAL_TOGGLE_LED2();

  return (i == nBytes);
}

/*********************************************************************
 * @fn      HalSensorTest
 *
 * @brief   Run a self-test on all the sensors
 *
 * @param   none
 *
 * @return  bitmask of error flags
 */
uint16 HalSensorTest(void)
{
  uint16 i;
  uint8 selfTestResult;

  halSensorEnableMap = 0;
  selfTestResult = 0;
  HalGyroTurnOn();

  for  (i=0; i<N_TEST_RUNS; i++)
  {
    HalLedSet(HAL_LED_2,HAL_LED_MODE_TOGGLE);

    // 1. Temp sensor test
    if (HalIRTempTest())
      selfTestResult |= ST_IRTEMP;

    // 2. Humidity  sensor test
    if (HalHumiTest())
      selfTestResult |= ST_HUMID;

    // 3. Magnetometer test
    if (HalMagTest())
      selfTestResult |= ST_MAGN;

    // 4. Accelerometer test
    if (HalAccTest())
      selfTestResult |= ST_ACC;

    // 5. Barometer test
    if (HalBarTest())
      selfTestResult |= ST_PRESS;

    // 6. Gyro test
    if (HalGyroTest())
      selfTestResult |= ST_GYRO;
  }

  HalGyroTurnOff();

  return selfTestResult;
}

/*********************************************************************
 * @fn      HalDcDcControl
 *
 * @brief   Control the DCDC converter based on what sensors are enabled.
 *
 * @descr   This applies to IR Temp, Humidity and Gyroscope. If any of those
 *          are used, the DCDC-converter must be bypassed.
 *
 * @param   sensorID - bit indicating one sensor
 *
 * @param   powerOn - indicate whether the sensor is turned on or off
 *
 * @return  none
 */
void HalDcDcControl(uint8 sensorID, bool powerOn)
{
  if (powerOn)
  {
    halSensorEnableMap |= sensorID;
  }
  else
  {
    halSensorEnableMap &= ~sensorID;
  }

  if (halSensorEnableMap & HIGH_SUPPLY_SENSOR_MAP)
  {
    /* Bypass DCDC */
    DCDC_SBIT = 0;
  }
  else
  {
    /* Enable DCDC */
    DCDC_SBIT = 1;
  }
}

/*********************************************************************
*********************************************************************/
