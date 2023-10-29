/**************************************************************************************************
  Filename:       hal_bar.c
  Revised:        $Date: 2012-08-17 08:49:24 -0700 (Fri, 17 Aug 2012) $
  Revision:       $Revision: 31282 $

  Description:    Driver for the TDK/EPCOS T5400/C953 pressure sensor


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
*************************************************************************************************/

/* -----------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/
#include "hal_bar.h"
#include "hal_sensor.h"
#include "hal_i2c.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/
// Sensor I2C address
#define HAL_C953_I2C_ADDRESS                0x77

// C953 data lengths
#define C953_DATA_LEN                       2
#define C953_CAL_LEN                        16

// C953 Register Addresses
#define C953_REG_ADDR_IFACE_SETTINGS        0x87 // Read/Write
#define C953_REG_ADDR_I2C_SLAVE             0x88 // Read
#define C953_REG_ADDR_RESET          		    0xF0 // Read/Write
#define C953_REG_ADDR_COMMAND          		  0xF1 // Read/Write
#define C953_REG_ADDR_TEMP_LSB          	  0xF3 // Read
#define C953_REG_ADDR_TEMP_MSB          	  0xF4 // Read
#define C953_REG_ADDR_PRESS_LSB       	    0xF5 // Read
#define C953_REG_ADDR_PRESS_MSB          	  0xF6 // Read

// C953 Register Addresses for calibration coefficients
#define C953_REG_CALIBRATION_1_LSB          0x8E
#define C953_REG_CALIBRATION_1_MSB          0x8F
#define C953_REG_CALIBRATION_2_LSB          0x90
#define C953_REG_CALIBRATION_2_MSB          0x91
#define C953_REG_CALIBRATION_3_LSB          0x92
#define C953_REG_CALIBRATION_3_MSB          0x93
#define C953_REG_CALIBRATION_4_LSB          0x94
#define C953_REG_CALIBRATION_4_MSB          0x95
#define C953_REG_CALIBRATION_5_LSB          0x96
#define C953_REG_CALIBRATION_5_MSB          0x97
#define C953_REG_CALIBRATION_6_LSB          0x98
#define C953_REG_CALIBRATION_6_MSB          0x99
#define C953_REG_CALIBRATION_7_LSB          0x9A
#define C953_REG_CALIBRATION_7_MSB          0x9B
#define C953_REG_CALIBRATION_8_LSB          0x9C
#define C953_REG_CALIBRATION_8_MSB          0x9D

// C953 Resolution Setting Bit Masks
#define C953_LOW_RES_LOW         		        0x00 // 2 ms Conversion Time
#define C953_LOW_RES_STANDARD       		    0x08 // 8 ms Conversion Time
#define C953_LOW_RES_HIGH         		      0x10 // 16 ms Conversion Time
#define C953_LOW_RES_ULTRA_HIGH    		      0x18 // 64 ms Conversion Time

// C953 Measurment Option Bit Masks
#define C953_PRESSURE_ONLY        		      0x00 // Execute Pressure Measurement Only
#define C953_TEMPERATURE_ONLY       		    0x02 // Execute Temperature Measurement Only

// C953 Commands
#define C953_TEMP_READ_COMMAND		          ( C953_TEMPERATURE_ONLY | C953_LOW_RES_LOW | 0x01 )
#define C953_PRESS_READ_COMMAND		          ( C953_PRESSURE_ONLY | C953_LOW_RES_LOW | 0x01 )
#define C953_OFF_COMMAND			              0x00
#define C953_RESET_COMMAND                  0x73
#define C953_TEST_COMMAND                   0x1E // For R/W test of register

/* ------------------------------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
static void HalBarSelect(void);

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 barData[4];
static uint8 barCmd = C953_TEMP_READ_COMMAND;
static bool  fCmdOk;

/**************************************************************************************************
 * @fn          HalBarInit
 *
 * @brief       Initalise the driver
 *
 * @return      none
 */
void HalBarInit(void)
{
  uint8 val = C953_OFF_COMMAND;

  HalBarSelect();
  HalSensorWriteReg(C953_REG_ADDR_COMMAND, &val, sizeof(val));
  barCmd = C953_TEMP_READ_COMMAND;
}


/**************************************************************************************************
 * @fn          HalBarStartMeasurement
 *
 * @brief       Start a conversion
 *
 * @return      none
 */
void HalBarStartMeasurement(void)
{
  HalBarSelect();
  fCmdOk = HalSensorWriteReg(C953_REG_ADDR_COMMAND, &barCmd, sizeof(barCmd));
}


/**************************************************************************************************
 * @fn          HalBarReadMeasurement
 *
 * @brief       Get temperature and pressure data (alternate reads)
 *
 * @param       pBuf - buffer for temperature and pressure (4 bytes)
 *
 * @return      TRUE if valid data
 */
bool HalBarReadMeasurement(uint8 *pBuf)
{
  bool success;
  uint8 dOffset = 0;

  if (!fCmdOk)
  {
    return FALSE;
  }

  HalBarSelect();

  if (barCmd==C953_PRESS_READ_COMMAND)
  {
    dOffset = 2;
  }

  success = HalSensorReadReg( C953_REG_ADDR_PRESS_LSB, &barData[dOffset], C953_DATA_LEN );
  if (success)
  {
    pBuf[0] = barData[0];
    pBuf[1] = barData[1];
    pBuf[2] = barData[2];
    pBuf[3] = barData[3];

    // Alternate
    if (barCmd==C953_PRESS_READ_COMMAND)
    {
      barCmd = C953_TEMP_READ_COMMAND;
    } else
    {
      barCmd = C953_PRESS_READ_COMMAND;
    }
  }

  return success;
}


/**************************************************************************************************
 * @fn          HalBarReadCalibration
 *
 * @brief       Read calibration data
 *
 * @param       pBuf - buffer for calibration data
 *
 * @return      none
 */
void HalBarReadCalibration(uint8 *pBuf)
{
  HalBarSelect();
  HalSensorReadReg(C953_REG_CALIBRATION_1_LSB, pBuf, C953_CAL_LEN);
}


/**************************************************************************************************
 * @fn          HalBarTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool HalBarTest(void)
{
  uint8 val;

  // Select this sensor on the I2C bus
  HalBarSelect();

  // Check the I2C slave address
  ST_ASSERT(HalSensorReadReg(C953_REG_ADDR_I2C_SLAVE, &val, sizeof(val)));
  ST_ASSERT(val==HAL_C953_I2C_ADDRESS);

  // Check that registers can be written
  val = C953_TEST_COMMAND;
  ST_ASSERT(HalSensorWriteReg(C953_REG_ADDR_COMMAND, &val, sizeof(val)));
  ST_ASSERT(HalSensorReadReg(C953_REG_ADDR_COMMAND, &val, sizeof(val)));
  ST_ASSERT(val==C953_TEST_COMMAND);

  // Restore values
  val = C953_OFF_COMMAND;
  ST_ASSERT(HalSensorWriteReg(C953_REG_ADDR_COMMAND, &val, sizeof(val)));

  return TRUE;
}

/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
 * @fn          HalBarSelect
 *
 * @brief       Select the barometer on the I2C-bus
 *
 * @return      none
 */
static void HalBarSelect(void)
{
  //Set up I2C that is used to communicate with the sensor
  HalI2CInit(HAL_C953_I2C_ADDRESS,i2cClock_267KHZ);
}

/*  Conversion algorithm for barometer temperature
 *  ==============================================
 *  Formula from application note, rev_X:
 *  Ta = ((c1 * Tr) / 2^24) + (c2 / 2^10)
 *
 *  c1 - c8: calibration coefficients the can be read from the sensor
 *  c1 - c4: unsigned 16-bit integers
 *  c5 - c8: signed 16-bit integers

double calcBarTmp(uint16 rawT)
{
  uint16 c1, c2;

  c1 = m_barCalib.c1;
  c2 = m_barCalib.c2;
  m_raw_temp = rawT;

  int64 temp, val;
	val = ((int64)(c1 * m_raw_temp) * 100);
	temp = (val >> 24);
	val = ((int64)c2 * 100);
	temp += (val >> 10);

	return ((double)temp) / 100;
}

 * Conversion algorithm for barometer pressure (hPa)
 * ==============================================
 * Formula from application note, rev_X:
 * Sensitivity = (c3 + ((c4 * Tr) / 2^17) + ((c5 * Tr^2) / 2^34))
 * Offset = (c6 * 2^14) + ((c7 * Tr) / 2^3) + ((c8 * Tr^2) / 2^19)
 * Pa = (Sensitivity * Pr + Offset) / 2^14

double TcalcBarPress(uint16 rawT)
{
	int64 s, o, pres, val;
  uint16 c3, c4;
  int16 c5, c6, c7, c8;
  uint16 Pr;
  int16 Tr;

  Pr = rawT;
  Tr = m_raw_temp;
  c3 = m_barCalib.c3;
  c4 = m_barCalib.c4;
  c5 = m_barCalib.c5;
  c6 = m_barCalib.c6;
  c7 = m_barCalib.c7;
  c8 = m_barCalib.c8;

  // Sensitivity
	s = (int64)c3;
	val = (int64)c4 * Tr;
	s += (val >> 17);
	val = (int64)c5 * Tr * Tr;
	s += (val >> 34);

  // Offset
	o = (int64)c6 << 14;
	val = (int64)c7 * Tr;
	o += (val >> 3);
	val = (int64)c8 * Tr * Tr;
	o += (val >> 19);

  // Pressure (Pa)
	pres = ((int64)(s * Pr) + o) >> 14;

  return (double)pres/100;
}

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

void storeCalibrationData(quint8 *pData)
{
    m_barCalib.c1 = BUILD_UINT16(pData[0],pData[1]);
    m_barCalib.c2 = BUILD_UINT16(pData[2],pData[3]);
    m_barCalib.c3 = BUILD_UINT16(pData[4],pData[5]);
    m_barCalib.c4 = BUILD_UINT16(pData[6],pData[7]);
    m_barCalib.c5 = BUILD_UINT16(pData[8],pData[9]);
    m_barCalib.c6 = BUILD_UINT16(pData[10],pData[11]);
    m_barCalib.c7 = BUILD_UINT16(pData[12],pData[13]);
    m_barCalib.c8 = BUILD_UINT16(pData[14],pData[15]);
}
*/
/*********************************************************************
*********************************************************************/

