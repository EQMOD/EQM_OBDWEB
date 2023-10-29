/**************************************************************************************************
  Filename:       hal_i2c.h
  Revised:        $Date: 2012-10-19 16:06:31 -0700 (Fri, 19 Oct 2012) $
  Revision:       $Revision: 31875 $

  Description:

  This module defines the HAL I2C API for the CC2533. Simplicity, speed and code size were the
  priorities over multi-master and/or multi-slave support which can easily be added as an
  enhancement to the detriment of the aforementioned.


  Copyright 2011-2012 Texas Instruments Incorporated. All rights reserved.

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
#ifndef HAL_I2C_H
#define HAL_I2C_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_types.h"
#if (defined HAL_I2C) && (HAL_I2C == TRUE)

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined HAL_I2C_MASTER
#define HAL_I2C_MASTER  FALSE
#endif

#if !defined HAL_I2C_SLAVE
#if HAL_I2C_MASTER
#define HAL_I2C_SLAVE   FALSE
#else
#define HAL_I2C_SLAVE   TRUE
#endif
#endif

#if HAL_I2C_MASTER
#define HAL_I2C_POLLED  FALSE  // Master does not use ISR and does not need periodic polling.
#else // if HAL_I2C_SLAVE
#if !defined HAL_I2C_POLLED
#define HAL_I2C_POLLED  FALSE  // Prefer the ISR as a Slave to speed reaction to Master READ req.
#endif
#endif

#if !defined HAL_I2C_BUF_MAX
#define HAL_I2C_BUF_MAX                  255
#endif

#define HAL_I2C_SLAVE_ADDR_DEF           0x41

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

#if (HAL_I2C_BUF_MAX < 256)
typedef uint8  i2cLen_t;
#else
typedef uint16 i2cLen_t;
#endif

#if HAL_I2C_MASTER
typedef enum {
  i2cClock_123KHZ = 0x00,
  i2cClock_144KHZ = 0x01,
  i2cClock_165KHZ = 0x02,
  i2cClock_197KHZ = 0x03,
  i2cClock_33KHZ  = 0x80,
  i2cClock_267KHZ = 0x81,
  i2cClock_533KHZ = 0x82
} i2cClock_t;
#else // if HAL_I2C_SLAVE
/**************************************************************************************************
 * @fn          i2cCallback_t
 *
 * @brief       Slave mode callback to Application to alert of a Master request to read or of
 *              received bytes from a Master write.
 *
 * input parameters
 *
 * @param       cnt - Zero indicates Master read request and slave Tx buffer empty. Non-zero
 *                    indicates the number of bytes received from a Master write,
 *                    this data is in the slave Rx buffer, ready to be read.
 *
 * output parameters
 *
 * None.
 *
 * @return      N/A when cnt is non-zero; otherwise TRUE if I2C should continue to clock stretch,
 *              FALSE to send a zero with last byte indication.
 */
typedef uint8 (*i2cCallback_t)(uint8 cnt);
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */

#if HAL_I2C_MASTER
void HalI2CInit(i2cClock_t clockRate);
i2cLen_t HalI2CRead(uint8 address, i2cLen_t len, uint8 *pBuf);
i2cLen_t HalI2CWrite(uint8 address, i2cLen_t len, uint8 *pBuf);
#else
void HalI2CInit(uint8 address, i2cCallback_t i2cCallback);
i2cLen_t HalI2CRead(i2cLen_t len, uint8 *pBuf);
i2cLen_t HalI2CWrite(i2cLen_t len, uint8 *pBuf);
#if HAL_I2C_POLLED
void HalI2CPoll(void);
#endif
#endif
uint8 HalI2CReady2Sleep(void);
void HalI2CEnterSleep(void);
void HalI2CExitSleep(void);

#endif
#endif
/**************************************************************************************************
 */
