/**************************************************************************************************
  Filename:       hal_irtemp.c
  Revised:        $Date: 2013-04-05 07:25:57 -0700 (Fri, 05 Apr 2013) $
  Revision:       $Revision: 33773 $

  Description:    Driver for the TI TMP06 infrared thermophile sensor.


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
#include "hal_irtemp.h"
#include "hal_i2c.h"
#include "hal_sensor.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/

/* Slave address */
#define TMP006_I2C_ADDRESS              0x44

/* TMP006 register addresses */
#define TMP006_REG_ADDR_VOLTAGE         0x00
#define TMP006_REG_ADDR_TEMPERATURE     0x01
#define TMP006_REG_ADDR_CONFIG          0x02
#define TMP006_REG_MANF_ID              0xFE
#define TMP006_REG_PROD_ID              0xFE

/* TMP006 register values */
#define TMP006_VAL_CONFIG_RESET         0x7400  // Sensor reset state
#define TMP006_VAL_CONFIG_ON            0x7000  // Sensor on state
#define TMP006_VAL_CONFIG_OFF           0x0000  // Sensor off state
#define TMP006_VAL_MANF_ID              0x5449  // Manufacturer ID
#define TMP006_VAL_PROD_ID              0x0067  // Product ID

/* Bit values */
#define DATA_RDY_BIT                    0x8000 // Data ready

/* Register length */
#define IRTEMP_REG_LEN                  2


/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
static void HalIRTempSelect(void);


/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static IRTemperature_States_t irtSensorState = TMP006_OFF;

static uint8 configSensorReset[2] = {0x80, 0x00};  // Sensor reset
static uint8 configSensorOff[2] = {0x00, 0x80};    // Sensor standby
static uint8 configSensorOn[2] =  {0x70, 0x00};    // Conversion time 0.25 sec

/* ------------------------------------------------------------------------------------------------
*                                           Public functions
* -------------------------------------------------------------------------------------------------
*/


/**************************************************************************************************
 * @fn          HALIRTempInit
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 **************************************************************************************************/
void HALIRTempInit(void)
{
  irtSensorState = TMP006_OFF;
  HalIRTempTurnOff();
}


/**************************************************************************************************
 * @fn          HalIRTempTurnOn
 *
 * @brief       Turn the sensor on
 *
 * @return      none
 **************************************************************************************************/
void HalIRTempTurnOn(void)
{
  HalDcDcControl(ST_IRTEMP,true);
  HalIRTempSelect();

  if (HalSensorWriteReg(TMP006_REG_ADDR_CONFIG, configSensorOn, IRTEMP_REG_LEN))
  {
    irtSensorState = TMP006_IDLE;
  }
}

/**************************************************************************************************
 * @fn          HalIRTempTurnOff
 *
 * @brief       Turn the sensor off
 *
 * @return      none
 **************************************************************************************************/
void HalIRTempTurnOff(void)
{
  HalIRTempSelect();

  if (HalSensorWriteReg(TMP006_REG_ADDR_CONFIG, configSensorOff, IRTEMP_REG_LEN))
  {
    irtSensorState = TMP006_OFF;
  }
  HalDcDcControl(ST_IRTEMP,false);
}

/**************************************************************************************************
 * @fn          HalIRTempRead
 *
 * @brief       Read the sensor voltage and sensor temperature registers
 *
 * @param       Voltage and temperature in raw format (2 + 2 bytes)
 *
 * @return      TRUE if valid data
 **************************************************************************************************/
bool HalIRTempRead(uint8 *pBuf)
{
  uint16 v;
  uint16 t;
  bool success;

  if (irtSensorState != TMP006_DATA_READY)
  {
    return FALSE;
  }

  HalIRTempSelect();

  // Read the sensor registers
  success = HalSensorReadReg(TMP006_REG_ADDR_VOLTAGE, (uint8 *)&v,IRTEMP_REG_LEN );
  if (success)
  {
    success = HalSensorReadReg(TMP006_REG_ADDR_TEMPERATURE, (uint8 *)&t,IRTEMP_REG_LEN );
  }

  if (success)
  {
    // Store values
    pBuf[0] = HI_UINT16( v );
    pBuf[1] = LO_UINT16( v );
    pBuf[2] = HI_UINT16( t );
    pBuf[3] = LO_UINT16( t );
  }

  // Turn off sensor
  if (HalSensorWriteReg(TMP006_REG_ADDR_CONFIG, configSensorOff, IRTEMP_REG_LEN))
  {
    irtSensorState = TMP006_OFF;
  }
  HalDcDcControl(ST_IRTEMP,false);

  return success;
}


/**************************************************************************************************
 * @fn          HalIRTempStatus
 *
 * @brief       Read the state of the sensor
 *
 * @return      none
 **************************************************************************************************/
IRTemperature_States_t HalIRTempStatus(void)
{
  if (irtSensorState != TMP006_OFF)
  {
    bool success;
    uint16 v;

    // Select this sensor on the I2C bus
    HalIRTempSelect();

    // Read the data ready bit
    success = HalSensorReadReg(TMP006_REG_ADDR_CONFIG, (uint8 *)&v,IRTEMP_REG_LEN );
    if ((v & DATA_RDY_BIT) && success)
    {
      irtSensorState = TMP006_DATA_READY;
    }
  }

  return irtSensorState;
}


/**************************************************************************************************
 * @fn          HalIRTempTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 **************************************************************************************************/
bool HalIRTempTest(void)
{
  uint16 val;

  // Select this sensor on the I2C bus
  HalIRTempSelect();

  // Check manufacturer ID
  ST_ASSERT(HalSensorReadReg(TMP006_REG_MANF_ID, (uint8 *)&val, IRTEMP_REG_LEN));
  val = (LO_UINT16(val) << 8) | HI_UINT16(val);
  ST_ASSERT(val == TMP006_VAL_MANF_ID);

  // Reset sensor
  ST_ASSERT(HalSensorWriteReg(TMP006_REG_ADDR_CONFIG, configSensorReset, IRTEMP_REG_LEN));

  // Check config register (reset)
  ST_ASSERT(HalSensorReadReg(TMP006_REG_ADDR_CONFIG, (uint8 *)&val, IRTEMP_REG_LEN));
  val = ((LO_UINT16(val) << 8) | HI_UINT16(val));
  ST_ASSERT(val == TMP006_VAL_CONFIG_RESET);

  // Turn sensor off
  ST_ASSERT(HalSensorWriteReg(TMP006_REG_ADDR_CONFIG, configSensorOff,IRTEMP_REG_LEN));

  // Check config register (off)
  ST_ASSERT(HalSensorReadReg(TMP006_REG_ADDR_CONFIG, (uint8 *)&val, IRTEMP_REG_LEN));
  val = ((LO_UINT16(val) << 8) | HI_UINT16(val));
  ST_ASSERT(val == TMP006_VAL_CONFIG_OFF);

  // Turn sensor on
  ST_ASSERT(HalSensorWriteReg(TMP006_REG_ADDR_CONFIG, configSensorOn, IRTEMP_REG_LEN));

  // Check config register (on)
  ST_ASSERT(HalSensorReadReg(TMP006_REG_ADDR_CONFIG, (uint8 *)&val, IRTEMP_REG_LEN));
  val = ((LO_UINT16(val) << 8) | HI_UINT16(val));
  ST_ASSERT(val == TMP006_VAL_CONFIG_ON);

  // Turn sensor off
  ST_ASSERT(HalSensorWriteReg(TMP006_REG_ADDR_CONFIG, configSensorOff, IRTEMP_REG_LEN));

  return TRUE;
}

/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
 * @fn          HalIRTempSelect
 *
 * @brief       Select the TMP006 slave and set the I2C bus speed
 *
 * @return      none
 **************************************************************************************************/
static void HalIRTempSelect(void)
{
  // Select slave and set clock rate
  HalI2CInit(TMP006_I2C_ADDRESS, i2cClock_533KHZ);
}

/*  Conversion algorithm for die temperature
 *  ================================================
 *
double calcTmpLocal(uint16 rawT)
{
    //-- calculate die temperature [°C] --
    m_tmpAmb = (double)((qint16)rawT)/128.0;

    return m_tmpAmb;
}

*
* Conversion algorithm for target temperature
*
double calcTmpTarget(uint16 rawT)
{
    //-- calculate target temperature [°C] -
    double Vobj2 = (double)(qint16)rawT;
    Vobj2 *= 0.00000015625;

    double Tdie2 = m_tmpAmb + 273.15;
    const double S0 = 6.4E-14;            // Calibration factor

    const double a1 = 1.75E-3;
    const double a2 = -1.678E-5;
    const double b0 = -2.94E-5;
    const double b1 = -5.7E-7;
    const double b2 = 4.63E-9;
    const double c2 = 13.4;
    const double Tref = 298.15;
    double S = S0*(1+a1*(Tdie2 - Tref)+a2*pow((Tdie2 - Tref),2));
    double Vos = b0 + b1*(Tdie2 - Tref) + b2*pow((Tdie2 - Tref),2);
    double fObj = (Vobj2 - Vos) + c2*pow((Vobj2 - Vos),2);
    double tObj = pow(pow(Tdie2,4) + (fObj/S),.25);
    tObj = (tObj - 273.15);

    return tObj;
}

*/

/*********************************************************************
*********************************************************************/

