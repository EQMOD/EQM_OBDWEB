/**************************************************************************************************
  Filename:       hal_mag.c
  Revised:        $Date: 2012-11-15 01:49:26 -0800 (Thu, 15 Nov 2012) $
  Revision:       $Revision: 32193 $

  Description:    Driver for the Freescale MAG3110 Magnetometer


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
#include "hal_mag.h"
#include "hal_sensor.h"
#include "hal_i2c.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/
#define MAG_REG_READ_ALL_LEN        6
#define MAG_REG_FAST_READ_LEN       3

// MAG3110 register addresses
#define MAG_REG_ADDR_DR_STATUS      0x00 // Read
#define MAG_REG_ADDR_X_MSB     		  0x01 // Read
#define MAG_REG_ADDR_X_LSB     		  0x02 // Read
#define MAG_REG_ADDR_Y_MSB     		  0x03 // Read
#define MAG_REG_ADDR_Y_LSB     		  0x04 // Read
#define MAG_REG_ADDR_Z_MSB     		  0x05 // Read
#define MAG_REG_ADDR_Z_LSB     		  0x06 // Read
#define MAG_REG_ADDR_WHO_AM_I       0x07 // Read
#define MAG_REG_ADDR_SYSMOD         0x08 // Read
#define MAG_REG_ADDR_OFF_X_MSB      0x09 // Read/Write
#define MAG_REG_ADDR_OFF_X_LSB     	0x0A // Read/Write
#define MAG_REG_ADDR_OFF_Y_MSB     	0x0B // Read/Write
#define MAG_REG_ADDR_OFF_Y_LSB     	0x0C // Read/Write
#define MAG_REG_ADDR_OFF_Z_MSB     	0x0D // Read/Write
#define MAG_REG_ADDR_OFF_Z_LSB     	0x0E // Read/Write
#define MAG_REG_ADDR_DIE_TEMP    	  0x0F // Read
#define MAG_REG_ADDR_CTRL_1     	  0x10 // Read/Write
#define MAG_REG_ADDR_CTRL_2      	  0x11 // Read/Write

#define MAG_REG_ADDR_READ_START	    MAG_REG_ADDR_X_MSB

// CTRL1 BIT MASKS
#define MAG_REG_CTRL_FR  		        0x04 // Fast Read Enable
#define MAG_REG_CTRL_TM   		      0x02 // Trigger Measurement Enable
#define MAG_REG_CTRL_EN   		      0x01 // Active Mode Enable

#define MAG_REG_CTRL_DR_10HZ_OSR_1  0x60
#define MAG_REG_CTRL_DR_10HZ_OSR_2  0x48
#define MAG_REG_CTRL_DR_10HZ_OSR_4  0x30
#define MAG_REG_CTRL_DR_10HZ_OSR_8  0x10

// Test values
#define WHO_AM_I_VALUE              0xC4
#define CTRL1_TEST_VALUE            0xF8

// Control register values
#define MAG_REG_CTRL_ON             MAG_REG_CTRL_DR_10HZ_OSR_1 | MAG_REG_CTRL_EN
#define MAG_REG_CTRL2_AMN           0x80
#define MAG_REG_CTRL_OFF            0x00

/* ------------------------------------------------------------------------------------------------
*                                           Typedefs
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Macros
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
static void HalMagSelect(void);

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static Magnetometer_States_t sensorState = MAG3110_OFF;
static uint8 halMagSensorConfig = MAG_REG_CTRL_ON;
static uint8 halMagSensorOff = MAG_REG_CTRL_OFF;
static uint8 halMagAutoMrstEn = MAG_REG_CTRL2_AMN;


/**************************************************************************************************
 * @fn          HalMagInit
 *
 * @brief       Initialise the magnetometer driver
 *
 * @return      none
 **************************************************************************************************/
void HalMagInit(void)
{
  sensorState = MAG3110_OFF;
}


/**************************************************************************************************
 * @fn          HalMagStatus
 *
 * @brief       Read the magnetometer status
 *
 * @return      none
 **************************************************************************************************/
Magnetometer_States_t HalMagStatus( void )
{
  if( sensorState == MAG3110_IDLE)
  {
    uint8 status = 0;

    HalMagSelect();
    HalSensorReadReg( MAG_REG_ADDR_DR_STATUS, &status, 1);
    if(status!=0)
    {
      sensorState = MAG3110_DATA_READY;
    }
  }

  return sensorState;
}

/**************************************************************************************************
* @fn          HalMagTurnOn
*
* @brief       Turn on the MAG3110.
*
* @return      none
*/
void HalMagTurnOn(void)
{	
  HalMagSelect();

  HalSensorWriteReg(MAG_REG_ADDR_CTRL_1, &halMagSensorConfig, sizeof(halMagSensorConfig));
  HalSensorWriteReg(MAG_REG_ADDR_CTRL_2, &halMagAutoMrstEn, sizeof(halMagAutoMrstEn));
  sensorState = MAG3110_IDLE;
}


/**************************************************************************************************
* @fn          HalMagTurnOff
*
* @brief       Turn off the MAG3110.
*
* @return      none
*/
void HalMagTurnOff(void)
{
  HalMagSelect();

  HalSensorWriteReg(MAG_REG_ADDR_CTRL_1, &halMagSensorOff, sizeof(halMagSensorOff));
  sensorState = MAG3110_OFF;
}


/**************************************************************************************************
* @fn          HalMagRead
*
* @brief       Read data from the magnetometer
*
* @param       pBuf - buffer to hold the data
*
* @return      TRUE if valid data
*/
bool HalMagRead(uint8 *pBuf)
{
  uint8 tmp[MAG_REG_READ_ALL_LEN];
  bool f;

  HalMagSelect();

  f = HalSensorReadReg(MAG_REG_ADDR_READ_START,tmp,MAG_REG_READ_ALL_LEN);
  if (f)
  {
    // Swap bytes in each value-pair
    pBuf[0] = tmp[1];
    pBuf[1] = tmp[0];
    pBuf[2] = tmp[3];
    pBuf[3] = tmp[2];
    pBuf[4] = tmp[5];
    pBuf[5] = tmp[4];
  }
  sensorState = MAG3110_IDLE;

  return f;
}


/**************************************************************************************************
 * @fn          HalMagTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 **************************************************************************************************/
bool HalMagTest(void)
{
  uint8 val;

  // Select this sensor on the I2C bus
  HalMagSelect();

  // Check who-am-i register
  ST_ASSERT(HalSensorReadReg(MAG_REG_ADDR_WHO_AM_I, &val, sizeof(val)));
  ST_ASSERT(val==WHO_AM_I_VALUE);

  // Check CTRL_REG1 in standby mode
  ST_ASSERT(HalSensorReadReg(MAG_REG_ADDR_CTRL_1, &val, sizeof(val)));
  ST_ASSERT(val==MAG_REG_CTRL_OFF);

  // Check that CTRL_REG1 can be written
  val = CTRL1_TEST_VALUE;
  ST_ASSERT(HalSensorWriteReg(MAG_REG_ADDR_CTRL_1, &val, sizeof(val)));
  ST_ASSERT(HalSensorReadReg(MAG_REG_ADDR_CTRL_1, &val, sizeof(val)));
  ST_ASSERT(val==CTRL1_TEST_VALUE);

  // Restore default value
  val = MAG_REG_CTRL_OFF;
  ST_ASSERT(HalSensorWriteReg(MAG_REG_ADDR_CTRL_1, &val, sizeof(val)));

  // Check that the sensor is in standby mode
  ST_ASSERT(HalSensorReadReg(MAG_REG_ADDR_SYSMOD, &val, sizeof(val)));
  ST_ASSERT(val==0);

  return TRUE;
}

/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
* @fn          HalMagSelect
*
* @brief       Select the magnetometer on the I2C-bus
*
* @return
*/
static void HalMagSelect(void)
{
  //Set up I2C that is used to communicate with MAG3110
  HalI2CInit(HAL_MAG3110_I2C_ADDRESS,i2cClock_267KHZ);
}


/*  Conversion algorithm for X, Y, Z
 *  ================================
 *
float calcMagn(int16 rawX)
{
    float v;

    //-- calculate magnetic force, unit uT, range -1000, +1000
    v = (rawX * 1.0) / (65536 / 2000);

    return v;
}
*/


/*********************************************************************
*********************************************************************/
