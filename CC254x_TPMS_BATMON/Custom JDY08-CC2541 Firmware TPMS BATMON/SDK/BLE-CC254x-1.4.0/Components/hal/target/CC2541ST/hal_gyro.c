/**************************************************************************************************
  Filename:       hal_gyro.c
  Revised:        $Date: 2013-05-16 08:23:18 -0700 (Thu, 16 May 2013) $
  Revision:       $Revision: 34324 $

  Description:    Driver for the InvenSense IMU-3000 3-Axis Gyroscope


  Copyright 2012-2013  Texas Instruments Incorporated. All rights reserved.

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
#include "hal_gyro.h"
#include "hal_i2c.h"
#include "hal_sensor.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/

/* Slave address */
#define HAL_GYRO_I2C_ADDRESS                    0x68
#define HAL_GYRO_DATA_SIZE                      6

/* GYRO register addresses */

#define HAL_GYRO_REG_WHOAMI                     0x00 // R/W

// Offset configuration registers
#define HAL_GYRO_REG_XOFFS_USRH                 0x0C // R/W
#define HAL_GYRO_REG_XOFFS_USRL                 0x0D // R/W
#define HAL_GYRO_REG_YOFFS_USRH                 0x0E // R/W
#define HAL_GYRO_REG_YOFFS_USRL                 0x0F // R/W
#define HAL_GYRO_REG_ZOFFS_USRH                 0x10 // R/W
#define HAL_GYRO_REG_ZOFFS_USRL                 0x11 // R/W

// Configuration registers
#define HAL_GYRO_REG_FIFO_EN                    0x12 // R/W
#define HAL_GYRO_REG_AUX_VDDIO                  0x13 // R/W
#define HAL_GYRO_REG_AUX_SLV_ADDR               0x14 // R/W
#define HAL_GYRO_REG_SMPLRT_DIV                 0x15 // R/W
#define HAL_GYRO_REG_DLPF_FS                    0x16 // R/W
#define HAL_GYRO_REG_INT_CFG                    0x17 // R/W
#define HAL_GYRO_REG_AUX_BURST_ADDR             0x18 // R/W
#define HAL_GYRO_REG_INT_STATUS                 0x1A // R

// Sensor data registers
#define HAL_GYRO_REG_TEMP_OUT_H                 0x1B // R
#define HAL_GYRO_REG_TEMP_OUT_L                 0x1C // R
#define HAL_GYRO_REG_GYRO_XOUT_H                0x1D // R
#define HAL_GYRO_REG_GYRO_XOUT_L                0x1E // R
#define HAL_GYRO_REG_GYRO_YOUT_H                0x1F // R
#define HAL_GYRO_REG_GYRO_YOUT_L                0x20 // R
#define HAL_GYRO_REG_GYRO_ZOUT_H                0x21 // R
#define HAL_GYRO_REG_GYRO_ZOUT_L                0x22 // R
#define HAL_GYRO_REG_AUX_XOUT_H                 0x23 // R
#define HAL_GYRO_REG_AUX_XOUT_L                 0x24 // R
#define HAL_GYRO_REG_AUX_YOUT_H                 0x25 // R
#define HAL_GYRO_REG_AUX_YOUT_L                 0x26 // R
#define HAL_GYRO_REG_AUX_ZOUT_H                 0x27 // R
#define HAL_GYRO_REG_AUX_ZOUT_L                 0x28 // R

// FIFO registers
#define HAL_GYRO_REG_FIFO_COUNTH                0x3A // R
#define HAL_GYRO_REG_FIFO_COUNTL                0x3B // R
#define HAL_GYRO_REG_FIFO_R                     0x3C // R

// User control and Power Management registers
#define HAL_GYRO_REG_USER_CTRL                  0x3D // R/W
#define HAL_GYRO_REG_PWR_MGM                    0x3E // R/W

/* GYRO Register Bit masks */
#define HAL_GYRO_REG_FIFO_EN_TEMP_OUT           0x80
#define HAL_GYRO_REG_FIFO_EN_GYRO_XOUT          0x40
#define HAL_GYRO_REG_FIFO_EN_GYRO_YOUT          0x20
#define HAL_GYRO_REG_FIFO_EN_GYRO_ZOUT          0x10
#define HAL_GYRO_REG_FIFO_EN_AUX_XOUT           0x08
#define HAL_GYRO_REG_FIFO_EN_AUX_YOUT           0x04
#define HAL_GYRO_REG_FIFO_EN_AUX_ZOUT           0x02
#define HAL_GYRO_REG_FIFO_EN_FIFO_FOOTER        0x01

#define HAL_GYRO_USER_CTRL_DMP_EN               0x80
#define HAL_GYRO_USER_CTRL_FIFO_EN              0x40
#define HAL_GYRO_USER_CTRL_AUX_IF_EN            0x20
#define HAL_GYRO_USER_CTRL_AUX_IF_RST_EN        0x08
#define HAL_GYRO_USER_CTRL_DMP_RST              0x04
#define HAL_GYRO_USER_CTRL_FIFO_RST             0x02
#define HAL_GYRO_USER_CTRL_GYRO_RST             0x01

#define HAL_GYRO_PWR_MGM_H_RESET                0x80
#define HAL_GYRO_PWR_MGM_SLEEP                  0x40
#define HAL_GYRO_PWR_MGM_STBY_XG                0x20
#define HAL_GYRO_PWR_MGM_STBY_YG                0x10
#define HAL_GYRO_PWR_MGM_STBY_ZG                0x08
#define HAL_GYRO_PWR_MGM_STBY_ALL               0x38  // All axes

// Clock select
#define HAL_GYRO_PWR_MGM_CLOCK_INT_OSC          0x00
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_X            0x01
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_Y            0x02
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_Z            0x03
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_32768KHZ     0x04
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_19_2MHZ      0x05
#define HAL_GYRO_PWR_MGM_CLOCK_STOP             0x07

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
static void HalGyroSelect(void);

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 mStatus;
// Selected axes, as in HAL_GYRO_PWR_MGM register:
// Bit 0-2: 0
// Bit 3: NOT X enabled
// Bit 4: NOT Y enabled
// Bit 5: NOT Z enabled
// Bit 6-7: 0
static uint8 mDisabledAxes;
static uint8 cfgOn;

/* ------------------------------------------------------------------------------------------------
*                                           Public functions
* -------------------------------------------------------------------------------------------------
*/


/**************************************************************************************************
 * @fn          HalGyroInit
 *
 * @brief       Initialise the gyro sensor driver
 *
 * @return      none
 **************************************************************************************************/
void HalGyroInit(void)
{
  mStatus = HAL_GYRO_STOPPED;
  mDisabledAxes = HAL_GYRO_PWR_MGM_STBY_ALL;

  HalGyroTurnOff();
}


/**************************************************************************************************
 * @fn          HalGyroTurnOn
 *
 * @brief       Turn the sensor on
 *
 * @return      none
 **************************************************************************************************/
void HalGyroTurnOn(void)
{
  bool success;
  uint8 clk_select;

  HalDcDcControl(ST_GYRO,true);

  HalGyroSelect();

  // Default to internal oscillator if no axis is enabled (should not happen)
  clk_select = HAL_GYRO_PWR_MGM_CLOCK_INT_OSC;
  if(!(mDisabledAxes & HAL_GYRO_PWR_MGM_STBY_XG))
  {
    clk_select = HAL_GYRO_PWR_MGM_CLOCK_PLL_X;
  }
  else if(!(mDisabledAxes & HAL_GYRO_PWR_MGM_STBY_YG))
  {
    clk_select = HAL_GYRO_PWR_MGM_CLOCK_PLL_Y;
  }
  else if(!(mDisabledAxes & HAL_GYRO_PWR_MGM_STBY_ZG))
  {
    clk_select = HAL_GYRO_PWR_MGM_CLOCK_PLL_Z;
  }

  // Wake up from sleep, disable standby for all gyros, select reference PLL according to selected axes
  cfgOn = mDisabledAxes & (~HAL_GYRO_PWR_MGM_SLEEP | clk_select);
  success = HalSensorWriteReg(HAL_GYRO_REG_PWR_MGM,&cfgOn,1);
  if(success)
  {
    mStatus = HAL_GYRO_DATA_READY;
  }
  else
  {
    mStatus = HAL_GYRO_STATE_ERROR;
  }
}

/**************************************************************************************************
 * @fn          HalGyroTurnOff
 *
 * @brief       Turn the sensor off
 *
 * @return      none
 **************************************************************************************************/
void HalGyroTurnOff(void)
{
  bool success;
  uint8 val;

  HalGyroSelect();

  // Standby, 5 uA current consumption
  val = HAL_GYRO_PWR_MGM_SLEEP;
  success = HalSensorWriteReg(HAL_GYRO_REG_PWR_MGM,&val,1);

  if(success)
  {
    uint8 rv;

    success = HalSensorReadReg(HAL_GYRO_REG_PWR_MGM,&rv,1);
    if (success && rv == val)
    {
      mStatus = HAL_GYRO_STOPPED;
      mDisabledAxes = HAL_GYRO_PWR_MGM_STBY_ALL;
    }
  }

  if (!success)
  {
    mStatus = HAL_GYRO_STATE_ERROR;
  }

  HalDcDcControl(ST_GYRO,false);
}


/**************************************************************************************************
 * @fn          HalGyroRead
 *
 * @brief       Read gyro data
 *
 * @param       Voltage and temperature in raw format [X_LOW, X_HIGH, Y_LOW, Y_HIGH, Z_LOW, Z_HIGH]
 *
 * @return      TRUE if valid data
 **************************************************************************************************/
bool HalGyroRead(uint8 *pBuf)
{
  bool success;
  uint8 buf[HAL_GYRO_DATA_SIZE];
  uint8 val;

  HalGyroSelect();

  // Read sensor
  success = HalSensorReadReg(HAL_GYRO_REG_GYRO_XOUT_H,buf,HAL_GYRO_DATA_SIZE);

  if (success)
  {
    // Result in LE
    pBuf[0] = buf[1];
    pBuf[1] = buf[0];
    pBuf[2] = buf[3];
    pBuf[3] = buf[2];
    pBuf[4] = buf[5];
    pBuf[5] = buf[4];
  }

  // Put gyro back to sleep
  val = HAL_GYRO_PWR_MGM_SLEEP;
  success = HalSensorWriteReg(HAL_GYRO_REG_PWR_MGM,&val,1);
  mStatus = HAL_GYRO_SLEEP;

  return success;
}


/**************************************************************************************************
 * @fn          HalGyroWakeUp
 *
 * @brief       Wake up the sensor
 *
 * @return      Gyro status
 **************************************************************************************************/
bool HalGyroWakeUp(void)
{
  HalGyroSelect();

  mStatus = HAL_GYRO_DATA_READY;

  // Wake up GYRO
  return HalSensorWriteReg(HAL_GYRO_REG_PWR_MGM,&cfgOn,1);
}


/**************************************************************************************************
 * @fn          HalGyroStatus
 *
 * @brief       Read the state of the sensor
 *
 * @return      Gyro status
 **************************************************************************************************/
uint8 HalGyroStatus(void)
{
  return mStatus;
}

/**************************************************************************************************
 * @fn          HalGyroSelectAxes
 *
 * @brief       Select axes for gyroscope
 *
 * @param       Bitmask representing enabled axes (1 enable, 0 disable)
 *              Bit 0: X axis
 *              Bit 1: Y axis
 *              Bit 2: Z axis
 *              Bit 3-7: reserved
 * @return      none
 **************************************************************************************************/
void HalGyroSelectAxes(uint8 axes)
{
  mDisabledAxes = HAL_GYRO_PWR_MGM_STBY_ALL;

  if(axes & 0x01)
  {
    mDisabledAxes ^= HAL_GYRO_PWR_MGM_STBY_XG;
  }

  if(axes & 0x02)
  {
    mDisabledAxes ^= HAL_GYRO_PWR_MGM_STBY_YG;
  }

  if(axes & 0x04)
  {
    mDisabledAxes ^= HAL_GYRO_PWR_MGM_STBY_ZG;
  }

  if(HalGyroStatus() != HAL_GYRO_STOPPED)
  {
    HalGyroTurnOn();
  }
}

/**************************************************************************************************
 * @fn          HalGyroTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 **************************************************************************************************/
bool HalGyroTest(void)
{
  uint8 val;

  HalGyroSelect();

  // Check the WHO AM I register
  ST_ASSERT(HalSensorReadReg(HAL_GYRO_REG_WHOAMI, &val, 1));
  ST_ASSERT((val&HAL_GYRO_I2C_ADDRESS) == HAL_GYRO_I2C_ADDRESS);

  return TRUE;
}

/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
 * @fn          HalGyroSelect
 *
 * @brief       Select the Gyro slave and set the I2C bus speed
 *
 * @return      none
 **************************************************************************************************/
static void HalGyroSelect(void)
{
  // Select slave and set clock rate
  HalI2CInit(HAL_GYRO_I2C_ADDRESS,   i2cClock_533KHZ);
}

/*  Conversion algorithm for X, Y, Z
 *  ================================
 *
float calcGyro(int16 rawX)
{
    float v;

    //-- calculate rotation, unit deg/s, range -250, +250
    v = (rawX * 1.0) / (65536 / 500);

    return v;
}
*/

/*********************************************************************
*********************************************************************/
