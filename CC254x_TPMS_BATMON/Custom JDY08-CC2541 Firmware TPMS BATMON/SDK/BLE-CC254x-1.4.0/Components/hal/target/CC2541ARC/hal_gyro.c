/**************************************************************************************************
  Filename:       hal_gyro.c
  Revised:        $Date: 2012-10-19 16:06:31 -0700 (Fri, 19 Oct 2012) $
  Revision:       $Revision: 31875 $

  Description:    This file contains the hardware abstraction interface for the
                  Invensense IXZ-500 Gyroscope.


  Copyright 2006-2011 Texas Instruments Incorporated. All rights reserved.

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

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_accel.h"
#include "hal_mcu.h"
#include "hal_drivers.h"
#include "hal_gyro.h"
#include "hal_motion.h"
#include "osal.h"

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* The following define which port pins are being used by the gyro */
#define HAL_GYRO_P0_GPIO_PINS  ( BV( 7 ) | BV( 5 ) )
#define HAL_GYRO_P1_GPIO_PINS  ( BV( 3 ) )

/* These defines indicate the direction of each pin */
#define HAL_GYRO_P0_OUTPUT_PINS  ( BV( 7 ) | BV( 5 ) )
#define HAL_GYRO_P1_OUTPUT_PINS  ( BV( 3 ) )

/* Defines for each output pin assignment */
#define HAL_GYRO_POWER_PIN  P0_7
#define HAL_GYRO_CLKIN_PIN  P0_5
#define HAL_GYRO_INT_PIN    P1_3

/* IMU3000 register addresses */
#define IMU3000_REG_ADDR_WHO_AM_I       0x00
#define IMU3000_REG_ADDR_X_OFFS_USRH    0x0C
#define IMU3000_REG_ADDR_X_OFFS_USRL    0x0D
#define IMU3000_REG_ADDR_Y_OFFS_USRH    0x0E
#define IMU3000_REG_ADDR_Y_OFFS_USRL    0x0F
#define IMU3000_REG_ADDR_Z_OFFS_USRH    0x10
#define IMU3000_REG_ADDR_Z_OFFS_USRL    0x11
#define IMU3000_REG_ADDR_FIFO_EN        0x12
#define IMU3000_REG_ADDR_AUX_VDDIO      0x13
#define IMU3000_REG_ADDR_AUX_SLV_ADDR   0x14
#define IMU3000_REG_ADDR_SMPLRT_DIV     0x15
#define IMU3000_REG_ADDR_DLPF_FS        0x16
#define IMU3000_REG_ADDR_INT_CFG        0x17
#define IMU3000_REG_ADDR_AUX_BURST_ADDR 0x18
#define IMU3000_REG_ADDR_INT_STATUS     0x1A
#define IMU3000_REG_ADDR_TEMP_OUT_H     0x1B
#define IMU3000_REG_ADDR_TEMP_OUT_L     0x1C
#define IMU3000_REG_ADDR_GYRO_XOUT_H    0x1D
#define IMU3000_REG_ADDR_GYRO_XOUT_L    0x1E
#define IMU3000_REG_ADDR_GYRO_YOUT_H    0x1F
#define IMU3000_REG_ADDR_GYRO_YOUT_L    0x20
#define IMU3000_REG_ADDR_GYRO_ZOUT_H    0x21
#define IMU3000_REG_ADDR_GYRO_ZOUT_L    0x22
#define IMU3000_REG_ADDR_AUX_XOUT_H     0x23
#define IMU3000_REG_ADDR_AUX_XOUT_L     0x24
#define IMU3000_REG_ADDR_AUX_YOUT_H     0x25
#define IMU3000_REG_ADDR_AUX_YOUT_L     0x26
#define IMU3000_REG_ADDR_AUX_ZOUT_H     0x27
#define IMU3000_REG_ADDR_AUX_ZOUT_L     0x28
#define IMU3000_REG_ADDR_DMP_REG1       0x35
#define IMU3000_REG_ADDR_DMP_REG2       0x36
#define IMU3000_REG_ADDR_DMP_REG3       0x37
#define IMU3000_REG_ADDR_DMP_REG4       0x38
#define IMU3000_REG_ADDR_DMP_REG5       0x39
#define IMU3000_REG_ADDR_FIFO_COUNTH    0x3A
#define IMU3000_REG_ADDR_FIFO_COUNTL    0x3B
#define IMU3000_REG_ADDR_FIFO_R         0x3C
#define IMU3000_REG_ADDR_USER_CTRL      0x3D
#define IMU3000_REG_ADDR_PWR_MGM        0x3E

/* USER_CTL values */
#define AUX_IF_RST 0x08
#define AUX_IF_EN  0x20

/* PWR_MGM values */
#define CLK_SRC_PLL_X_GYRO_REF 0x01
#define PWR_MGM_SLEEP 0x40
#define PWR_MGM_STBY_XG 0x20
#define PWR_MGM_STBY_YG 0x10
#define PWR_MGM_STBY_ZG 0x08

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/
typedef struct
{
  uint8 addr;
  uint8 data;
} halGyroConfig_t;

/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static halGyroEnableCback_t pHalGyroEnableProcessFunction;
static halGyroConfig_t HalGyroConfigTable[] =
{
  { IMU3000_REG_ADDR_AUX_SLV_ADDR, HAL_ACCEL_I2C_ADDRESS },
  { IMU3000_REG_ADDR_AUX_BURST_ADDR, HAL_ACCEL_OUTPUT_DATA_ADDRESS },
  { IMU3000_REG_ADDR_SMPLRT_DIV, 4 }, // SMPLRT_DIV = 4 with DLPF_CFG = 1 gives 200 Hz rate
  { IMU3000_REG_ADDR_DLPF_FS, 0x19 }
};
#define HAL_GYRO_CONFIG_TABLE_SIZE (sizeof(HalGyroConfigTable) / sizeof(HalGyroConfigTable[0]))

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
static void gyroSleep( void );
static void gyroWake( void );

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalGyroInit
 *
 * @brief   Initilize Gyro hardware
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalGyroInit( void )
{
  /* Initialize outputs */
  HAL_GYRO_POWER_PIN = 1;
  HAL_GYRO_CLKIN_PIN = 0;
  HAL_GYRO_INT_PIN = 0;

  /* Configure pin function as GPIO for pins related to gyro */
  P0SEL &= (uint8) ~HAL_GYRO_P0_GPIO_PINS;
  P1SEL &= (uint8) ~HAL_GYRO_P1_GPIO_PINS;

  /* Configure direction of pins related to gyro */
  P0DIR |= (uint8) HAL_GYRO_P0_OUTPUT_PINS;
  P1DIR |= (uint8) HAL_GYRO_P1_OUTPUT_PINS;

  /* Indicate no callback */
  pHalGyroEnableProcessFunction = NULL;

  /* Start timer to wait for outputs to become stable */
  osal_start_timerEx( Hal_TaskID,
                      HAL_GYRO_REGISTER_ACCESS_READY_EVENT,
                      100 );
}

/**************************************************************************************************
 * @fn          HalGyroEnable
 *
 * @brief       This function configures the gyro for operation.
 *
 * @param   cback - function to call when gyro is configured
 *
 * @return      None.
 **************************************************************************************************/
void HalGyroEnable( halGyroEnableCback_t cback )
{
  /* Save callback */
  pHalGyroEnableProcessFunction = cback;

  /* Take gyro out of sleep mode */
  gyroWake();

  /* Start timer to wait for outputs to become stable */
  osal_start_timerEx( Hal_TaskID,
                      HAL_GYRO_REGISTER_ACCESS_READY_EVENT,
                      100 );
}

/**************************************************************************************************
 * @fn          HalGyroHandleGyroRegisterAccessReadyEvent
 *
 * @brief       Callback to handle expiry of gyro powerup wait timer.
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************************/
void HalGyroHandleGyroRegisterAccessReadyEvent( void )
{
  uint8 i;

  if (pHalGyroEnableProcessFunction != NULL)
  {
    /* Gyro is ready. Configure it. */
    for (i = 0; i < HAL_GYRO_CONFIG_TABLE_SIZE; i++)
    {
      HalMotionI2cWrite( HAL_MOTION_DEVICE_GYRO,
                         HalGyroConfigTable[i].addr,
                         &HalGyroConfigTable[i].data,
                         1 );
    }

    /* config is complete, so notify caller */
    pHalGyroEnableProcessFunction();
  }
  else
  {
    /* No callback, so just put gyro in sleep mode */
    gyroSleep();
  }
}

/**************************************************************************************************
 * @fn          HalGyroDisable
 *
 * @brief       This function powers off the gyro.
 *
 * @param    None.
 *
 * @return      None.
 **************************************************************************************************/
void HalGyroDisable( void )
{
  /* All we need to do is put gyro to sleep */
    gyroSleep();
}

/**************************************************************************************************
 * @fn      HalGyroRead
 *
 * @brief   Reads channels connected to X and Z output on Gyroscope.
 *
 * @param   None
 *
 * @return  X and Z channel readings
 **************************************************************************************************/
void HalGyroRead( int16 *x, int16 *y, int16 *z )
{
  uint8 temp[6];

  /* Get all 3 axes of gyro samples as the gyro seems to hang the I2C bus if we only read 2 of them. */
  HalMotionI2cRead( HAL_MOTION_DEVICE_GYRO,
                    IMU3000_REG_ADDR_GYRO_XOUT_H,
                    6,
                    (uint8 *)temp );

  /* Extract X and Z axis info, accounting for endian difference */
  *x = (temp[2] << 8) | temp[3];
  *y = (temp[0] << 8) | temp[1];
  *z = (temp[4] << 8) | temp[5];
}

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
void HalGyroReadAccelData( int8 *x, int8 *y, int8 *z )
{
  uint8 temp[6];

  /* Get all 3 axes of gyro samples as the gyro seems to hang the I2C bus if we only read 2 of them. */
  HalMotionI2cRead( HAL_MOTION_DEVICE_GYRO,
                    IMU3000_REG_ADDR_AUX_XOUT_H,
                    6,
                    (uint8 *)temp );

  /* Extract Y and Z axis info. Data is laid out as follows:
   *
   * ----------
   * | XOUT_L | B3  B2  B1  B0  x   x   x   x
   * |--------|
   * | XOUT_H | B11 B10 B9  B8  B7  B6  B5  B4
   * |--------|
   * | YOUT_L |
   * |--------|
   * | YOUT_H |
   * |--------|
   * | ZOUT_L |
   * |--------|
   * | ZOUT_H |
   * ----------
   *
   * Since the data we're interested in lies in the 8 most significant bits, all
   * we need to do is read the XOUT_H, YOUT_H, and ZOUT_H registers.
   */
  *x = temp[1];
  *y = temp[3];
  *z = temp[5];
}

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
void HalGyroEnableI2CPassThru( void )
{
  uint8 data = AUX_IF_RST;

  HalMotionI2cWrite( HAL_MOTION_DEVICE_GYRO,
                     IMU3000_REG_ADDR_USER_CTRL,
                     &data,
                     1 );
}

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
void HalGyroDisableI2CPassThru( void )
{
  uint8 data = AUX_IF_RST | AUX_IF_EN;

  HalMotionI2cWrite( HAL_MOTION_DEVICE_GYRO,
                     IMU3000_REG_ADDR_USER_CTRL,
                     &data,
                     1 );
}

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
void HalGyroStartGyroMeasurements( void )
{
  uint8 data = CLK_SRC_PLL_X_GYRO_REF;

  HalGyroDisableI2CPassThru();
  HalMotionI2cWrite( HAL_MOTION_DEVICE_GYRO,
                     IMU3000_REG_ADDR_PWR_MGM,
                     &data,
                     1 );
}

/**************************************************************************************************
 * @fn          gyroSleep
 *
 * @brief       This function puts the gyro in sleep mode.
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************************/
static void gyroSleep( void )
{
  /* All we need to do is put gyro to sleep */
  uint8 reg = PWR_MGM_SLEEP | PWR_MGM_STBY_XG | PWR_MGM_STBY_YG | PWR_MGM_STBY_ZG;
  HalMotionI2cWrite( HAL_MOTION_DEVICE_GYRO,
                     IMU3000_REG_ADDR_PWR_MGM,
                     &reg,
                     1 );
}

/**************************************************************************************************
 * @fn          gyroWake
 *
 * @brief       This function takes the gyro out of sleep mode.
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************************/
static void gyroWake( void )
{
  /* Clear SLEEP bit and place gyros in active mode */
    uint8 reg = 0;

    HalMotionI2cWrite( HAL_MOTION_DEVICE_GYRO,
                       IMU3000_REG_ADDR_PWR_MGM,
                       &reg,
                       1 );
}

/**************************************************************************************************
**************************************************************************************************/
