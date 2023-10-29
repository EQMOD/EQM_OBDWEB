/**************************************************************************************************'
  Filename:       hal_humi.c
  Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 33575 $

  Description:    Driver for the Sensirion SHT21 Humidity sensor


  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_humi.h"
#include "hal_sensor.h"
#include "hal_i2c.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/

// Sensor I2C address
#define HAL_SHT21_I2C_ADDRESS      0x40

#define S_REG_LEN                  2
#define DATA_LEN                   3

// Internal commands
#define SHT21_CMD_TEMP_T_H         0xE3 // command trig. temp meas. hold master
#define SHT21_CMD_HUMI_T_H         0xE5 // command trig. humidity meas. hold master
#define SHT21_CMD_TEMP_T_NH        0xF3 // command trig. temp meas. no hold master
#define SHT21_CMD_HUMI_T_NH        0xF5 // command trig. humidity meas. no hold master
#define SHT21_CMD_WRITE_U_R        0xE6 // command write user register
#define SHT21_CMD_READ_U_R         0xE7 // command read user register
#define SHT21_CMD_SOFT_RST         0xFE // command soft reset

#define HUMIDITY				           0x00
#define TEMPERATURE			           0x01

#define USR_REG_MASK               0x38  // Mask off reserved bits (3,4,5)
#define USR_REG_DEFAULT            0x02  // Disable OTP reload
#define USR_REG_RES_MASK           0x7E  // Only change bits 0 and 7 (meas. res.)
#define USR_REG_11BITRES           0x81  // 11-bit resolution

/* ------------------------------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
static void HalHumiSelect(void);
static bool HalHumiReadData(uint8 *pBuf,uint8 nBytes);
static bool HalHumiWriteCmd(uint8 cmd);

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 usr;                         // Keeps user register value
static uint8 buf[6];                      // Data buffer
static bool  success;

/**************************************************************************************************
* @fn          HalHumiInit
*
* @brief       Initialise the humidity sensor driver
*
* @return      none
**************************************************************************************************/
void HalHumiInit(void)
{
  HalHumiSelect();

  // Set 11 bit resolution
  HalSensorReadReg(SHT21_CMD_READ_U_R,&usr,1);
  usr &= USR_REG_RES_MASK;
  usr |= USR_REG_11BITRES;
  HalSensorWriteReg(SHT21_CMD_WRITE_U_R,&usr,1);
  success = FALSE;
}


/**************************************************************************************************
* @fn          HalHumiExecMeasurementStep
*
* @brief       Execute measurement step
*
* @return      none
*/
bool HalHumiExecMeasurementStep(uint8 state)
{
  HalHumiSelect();

  switch (state)
  {
    case 0:
      // Turn on DC-DC control
      HalDcDcControl(ST_HUMID,true);

      // Start temperature read
      success = HalHumiWriteCmd(SHT21_CMD_TEMP_T_NH);
      break;

    case 1:
      // Read and store temperature value
      if (success)
      {
        success = HalHumiReadData(buf, DATA_LEN);

        // Start for humidity read
        if (success)
        {
          success = HalHumiWriteCmd(SHT21_CMD_HUMI_T_NH);
        }
      }
      break;

    case 2:
      // Read and store humidity value
      if (success)
      {
        success = HalHumiReadData(buf+DATA_LEN, DATA_LEN);
      }

      // Turn of DC-DC control
      HalDcDcControl(ST_HUMID,false);
      break;
  }

  return success;
}


/**************************************************************************************************
* @fn          HalHumiReadMeasurement
*
* @brief       Get humidity sensor data
*
* @return      none
*/
bool HalHumiReadMeasurement(uint8 *pBuf)
{
  // Store temperature
  pBuf[0] = buf[1];
  pBuf[1] = buf[0];

  // Store humidity
  pBuf[2] = buf[4];
  pBuf[3] = buf[3];

  return success;
}



/**************************************************************************************************
* @fn          HalHumiTest
*
* @brief       Humidity sensor self test
*
* @return      none
**************************************************************************************************/
bool HalHumiTest(void)
{
  uint8 val;

  HalHumiSelect();

  // Verify default value
  ST_ASSERT(HalSensorReadReg(SHT21_CMD_READ_U_R,&val,1));
  ST_ASSERT(val==usr);

  return TRUE;
}


/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
* @fn          HalHumiSelect
*
* @brief       Select the humidity sensor on the I2C-bus
*
* @return
*/
static void HalHumiSelect(void)
{
  //Set up I2C that is used to communicate with SHT21
  HalI2CInit(HAL_SHT21_I2C_ADDRESS,i2cClock_267KHZ);
}


/**************************************************************************************************
* @fn          halHumiWriteCmd
*
* @brief       Write a command to the humidity sensor
*
* @param       cmd - command to write
*
* @return      TRUE if the command has been transmitted successfully
**************************************************************************************************/
static bool HalHumiWriteCmd(uint8 cmd)
{
  /* Send command */
  return HalI2CWrite(1,&cmd) == 1;
}


/**************************************************************************************************
* @fn          HalHumiReadData
*
* @brief       This function implements the I2C protocol to read from the SHT21.
*
* @param       pBuf - pointer to buffer to place data
*
* @param       nBytes - number of bytes to read
*
* @return      TRUE if the required number of bytes are received
**************************************************************************************************/
static bool HalHumiReadData(uint8 *pBuf, uint8 nBytes)
{
  /* Read data */
  return HalI2CRead(nBytes,pBuf ) == nBytes;
}

/*  Conversion algorithm, humidity
 *
double calcHumRel(uint16 rawH)
{
    double v;

    rawH &= ~0x0003; // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --
    v = -6.0 + 125.0/65536 * (double)rawH; // RH= -6 + 125 * SRH/2^16

    return v;
}

 *  Conversion algorithm, temperature
 *
double calcHumTmp(uint16 rawT)
{
    double v;

    //-- calculate temperature [°C] --
    v = -46.85 + 175.72/65536 *(double)(qint16)rawT;

    return v;
}
 */


/*********************************************************************
*********************************************************************/

