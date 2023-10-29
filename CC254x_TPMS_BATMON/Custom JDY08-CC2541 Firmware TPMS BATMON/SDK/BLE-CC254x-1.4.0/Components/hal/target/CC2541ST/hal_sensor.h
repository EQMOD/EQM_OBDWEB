/**************************************************************************************************
  Filename:       hal_sensor.h
  Revised:        $Date: 2013-03-26 07:47:25 -0700 (Tue, 26 Mar 2013) $
  Revision:       $Revision: 33597 $

  Description:    Interface to sensor driver shared code.


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

#ifndef HAL_SENSOR_H
#define HAL_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"

/*********************************************************************
 * CONSTANTS and MACROS
 */

/* Sensor bit values for self-test and DCDC conbtrol */
#define ST_IRTEMP                             0x01
#define ST_HUMID                              0x02
#define ST_MAGN                               0x04
#define ST_ACC                                0x08
#define ST_PRESS                              0x10
#define ST_GYRO                               0x20

#define HIGH_SUPPLY_SENSOR_MAP                ( ST_IRTEMP | ST_HUMID | ST_GYRO)

/* Self test assertion; return FALSE (failed) if condition is not met */
#define ST_ASSERT(cond) st( if (!(cond)) return FALSE; )

/* Ative delay: 125 cycles ~1 msec */
#define ST_HAL_DELAY(n) st( { volatile uint32 i; for (i=0; i<(n); i++) { }; } )

/*********************************************************************
 * FUNCTIONS
 */
bool   HalSensorReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
bool   HalSensorWriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
uint16 HalSensorTest(void);
void   HalDcDcControl(uint8 sensorID, bool powerOn);

/*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HAL_SENSOR_H */
