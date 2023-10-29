/**************************************************************************************************
  Filename:       hal_motion.c
  Revised:        $Date: 2012-10-19 16:06:31 -0700 (Fri, 19 Oct 2012) $
  Revision:       $Revision: 31875 $

  Description:    This file contains the interface to the HAL Motion Sensor Service.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_mcu.h"
#include "hal_board_cfg.h"
#include "hal_accel.h"
#include "hal_defs.h"
#include "hal_gyro.h"
#include "hal_i2c.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "hal_motion.h"
#include "osal.h"
#include "osal_snv.h"

/* Movea MotionSense Pointing Library */
#include "AIR_MOTION_Lib.h"

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

#define HAL_MOTION_TIME_BETWEEN_MEASUREMENTS 10 // in ms

/* Number of gyro samples to take for calibration */
#define HAL_MOTION_NUM_CAL_SAMPLES 8

/* Used to enable/disable roll (i.e. accelerometer) compensation */
#define HAL_MOTION_ENABLE_ROLL_COMPENSATION TRUE

/* SNV items */
#if !defined HAL_MOTION_NV_ID_BEG
#define HAL_MOTION_NV_ID_BEG 0xF0  // chose something toward the end of application IDs per OSAL API
#endif

#define HAL_MOTION_NV_ITEM_GYRO_OFFSETS_ID ( HAL_MOTION_NV_ID_BEG + 0 )

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/
enum
{
  HAL_MOTION_STATE_OFF,
  HAL_MOTION_STATE_INIT,
  HAL_MOTION_STATE_POWERING_ON,
  HAL_MOTION_STATE_CALIBRATING,
  HAL_MOTION_STATE_WAITING_FOR_NEXT_MEASUREMENT,
  HAL_MOTION_STATE_MEASURING,
  HAL_MOTION_STATE_STANDBY
};
typedef uint8 halMotionState_t;

typedef struct
{
  int8 gainX;
  int8 gainY;
} halMotionGainSettings_t;

/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static halMotionCBack_t pHalMotionProcessFunction;
static halMotionCalCBack_t pHalMotionCalCompleteNotificationFunction;
static halMotionState_t HalMotionState = HAL_MOTION_STATE_OFF;
static bool HalMotionCalibrateGyro;

static const halMotionGainSettings_t HalMotionMouseGainTable[] =
{
#if (HAL_MOTION_ENABLE_ROLL_COMPENSATION == TRUE)
  {  2,  2 },
  {  7,  7 },
  { 10, 10 },
  { 15, 15 }
#else
  {  2,  2 },
  {  7,  7 },
  { 10, 10 },
  { 15, 15 }
#endif
};
static uint8 HalMotionMouseGainTableIndex;

#define HAL_MOTION_MOUSE_GAIN_TABLE_SIZE (sizeof(HalMotionMouseGainTable) / sizeof(HalMotionMouseGainTable[0]))
#define HAL_MOTION_DEFAULT_GAIN_TABLE_INDEX 1;

static struct { int16 x,y,z; } HalMotionGyroOffsets;

static const uint8 HalMotionDeviceAddressTable[] =
{
  HAL_GYRO_I2C_ADDRESS,
  HAL_ACCEL_I2C_ADDRESS
};

static t_struct_AIR_MOTION_ProcessDeltaSamples samples;
static t_struct_AIR_MOTION_ProcessDeltaStatus motion_status;

// Parameters for AIR_MOTION_Init motion_init()
static t_struct_AIR_MOTION_Init motion_init = {
  15, 15,   // DeltaGain.X and DeltaGain.Y: The gain to apply along X and Y axis for computing deltas
  0, 0, 0,  // GyroOffsets.X, GyroOffsets.Y, GyroOffsets.Z: Initial offset values to use for gyros X is not relevant for dual axis gyros.
  8,        // GyroStaticMaxNoise: the max noise for detecting static position of gyroscope. When this value is exceed, the gyros will be supposed to move.
  400,      // StaticSamples: Number of samples for detecting if gyroscope is static
  400,      // SwipeDistMin: Minimum delta for detecting a swipe. Must be positive. Not relevant if swipes are not being used.
  512,      // SwipeNoiseMax: Maximum delta for detecting a swipe. Must be positive. Not relevant if swipes are not being used.
  32,       // NbSamplesGyroStartup: Number of mickeys to discard after starting up the gyroscope.
  40,       // ClickStillSamples
  AirMotionNormal, // ClickStillTolerance
  HAL_MOTION_ENABLE_ROLL_COMPENSATION,  // IsRollCompEnabled: If true, the roll compensation is enabled
  64,       // AccNbLsb1G: 3-axis L2 norm measured on the accelerometer when the device is still.
  262        // GyroSensitivity: Gyroscope sensitivity (16*LSB / °/s)
};

static int HalNbSamplesCalCnt = 0;

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
static void HalMotionHandleMeasurementStartEvent( void );
static void HalMotionHandleCalPowerupDoneEvent( void );
static void HalMotionHandleGyroActiveEvent( void );
static void HalMotionGyroReady( void );

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/
/**************************************************************************************************
 * @fn      HalMotionInit
 *
 * @brief   Initilize Motion Sensor Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalMotionInit( void )
{
  /* Initialize callback functions */
  pHalMotionProcessFunction = NULL;
  pHalMotionCalCompleteNotificationFunction = NULL;

  /* Set up the I2C interface */
  HalI2CInit( i2cClock_267KHZ ); // I2C clock rate

  /* Initialize accelerometer and gyro */
  HalAccelInit();
  HalGyroInit();

  /* Update state */
  HalMotionState = HAL_MOTION_STATE_INIT;
}

/**************************************************************************************************
 * @fn      HalMotionConfig
 *
 * @brief   Configure the Motion Sensor serivce
 *
 * @param   measCback - pointer to the motion sensor results data callback function
 *
 * @return  None
 **************************************************************************************************/
void HalMotionConfig( halMotionCBack_t measCback )
{
  uint8 snvStatus;

  /* Register the callback fucntion */
  pHalMotionProcessFunction = measCback;

  /* Set up data used by Movea's motion processing library */
  HalMotionMouseGainTableIndex = HAL_MOTION_DEFAULT_GAIN_TABLE_INDEX;
  snvStatus = osal_snv_read( HAL_MOTION_NV_ITEM_GYRO_OFFSETS_ID,
                             sizeof( HalMotionGyroOffsets ),
                             (uint8 *)&HalMotionGyroOffsets );
  if (snvStatus != SUCCESS)
  {
    /* read failed, so use settings that indicate no bias is present */
    HalMotionGyroOffsets.x = 0;
    HalMotionGyroOffsets.y = 0;
    HalMotionGyroOffsets.z = 0;
  }
}

/**************************************************************************************************
 * @fn      HalMotionEnable
 *
 * @brief   Enable Motion Sensor hardware
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalMotionEnable( void )
{
  /* Reset Movea's motion processing library */
  motion_init.DeltaGain.X = HalMotionMouseGainTable[HalMotionMouseGainTableIndex].gainX;
  motion_init.DeltaGain.Y = HalMotionMouseGainTable[HalMotionMouseGainTableIndex].gainY;
  motion_init.GyroOffsets.X = HalMotionGyroOffsets.x;
  motion_init.GyroOffsets.Y = HalMotionGyroOffsets.y;
  motion_init.GyroOffsets.Z = HalMotionGyroOffsets.z;

  AIR_MOTION_Init(&motion_init);

  /* Configure gyro first, since it needs to be powered up before accelerometer
   * can be configured (because we need to configure gyro's I2C as passthrough
   * in order to communicate with the accelerometer).
   */
  HalGyroEnable( HalMotionGyroReady );

  /* Indicate we're not going to calibrate gyro when it settles */
  HalMotionCalibrateGyro = FALSE;

  /* Update state */
  HalMotionState = HAL_MOTION_STATE_POWERING_ON;
}

/**************************************************************************************************
 * @fn      HalMotionCal
 *
 * @brief   Calibrate Motion Sensor hardware
 *
 * @param   calCback - pointer to the motion sensor calibration complete callback function
 * @param   calDuration - duration, in milliseconds, of calibration procedure
 *
 * @return  TRUE if calibration started. FALSE, otherwise.
 **************************************************************************************************/
uint8 HalMotionCal( halMotionCalCBack_t calCback,
                    uint16 calDuration )
{
  if (HalMotionState == HAL_MOTION_STATE_INIT)
  {
    /* Register the callback fucntion */
    pHalMotionCalCompleteNotificationFunction = calCback;

    /* Start timer to wait for outputs to become stable */
    osal_start_timerEx( Hal_TaskID,
                        HAL_MOTION_GYRO_POWERUP_DONE_EVENT,
                        calDuration );

    /* Reset the gyro offsets so library can calculate fresh ones */
    motion_init.GyroOffsets.X = 0;
    motion_init.GyroOffsets.Y = 0;
    motion_init.GyroOffsets.Z = 0;
    AIR_MOTION_Init(&motion_init);

    /* Update state */
    HalMotionState = HAL_MOTION_STATE_POWERING_ON;

    return TRUE;
  }

  return FALSE;
}

/**************************************************************************************************
 * @fn      HalMotionDisable
 *
 * @brief   Disable Motion Sensor hardware
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalMotionDisable( void )
{
  /* Power off gyro and accelerometer */
  HalGyroEnableI2CPassThru();
  HalAccelDisable();
  HalGyroDisable();

  /* Stop sample collection */
  osal_stop_timerEx( Hal_TaskID,
                     HAL_MOTION_MEASUREMENT_START_EVENT );

  /* Update state */
  HalMotionState = HAL_MOTION_STATE_INIT;
}

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
void HalMotionStandby( void )
{
  /* Stop sample collection */
  osal_stop_timerEx( Hal_TaskID,
                     HAL_MOTION_MEASUREMENT_START_EVENT );

  /* Put accelerometer in motion detection mode */
  HalGyroEnableI2CPassThru();
  HalAccelMotionDetect( HalMotionEnable );

  /* Power off gyro */
  HalGyroDisable();

  /* Update state */
  HalMotionState = HAL_MOTION_STATE_STANDBY;
}

/**************************************************************************************************
 * @fn      HalMotionModifyAirMouseResolution
 *
 * @brief   Increase/decrease air mouse movement resolution
 *
 * @param   action - increase/decrease command
 *
 * @return  None
 **************************************************************************************************/
void HalMotionModifyAirMouseResolution( halMotionMouseResolution_t action )
{
  if (action == HAL_MOTION_RESOLUTION_INCREASE)
  {
    if (HalMotionMouseGainTableIndex < HAL_MOTION_MOUSE_GAIN_TABLE_SIZE - 1)
    {
      HalMotionMouseGainTableIndex += 1;
    }
  }
  else
  {
    if (HalMotionMouseGainTableIndex > 0)
    {
      HalMotionMouseGainTableIndex -= 1;
    }
  }

  /* Re-initialize Movea's motion processing library */
  motion_init.DeltaGain.X = HalMotionMouseGainTable[HalMotionMouseGainTableIndex].gainX;
  motion_init.DeltaGain.Y = HalMotionMouseGainTable[HalMotionMouseGainTableIndex].gainY;
  motion_init.GyroOffsets.X = HalMotionGyroOffsets.x;
  motion_init.GyroOffsets.Y = HalMotionGyroOffsets.y;
  motion_init.GyroOffsets.Z = HalMotionGyroOffsets.z;

  AIR_MOTION_Init(&motion_init);
}

/**************************************************************************************************
 * @fn      HalMotionHandleOSEvent
 *
 * @brief   Handles Motion Sensor timer expiry, which means the gyro and
 *          accelerometer are ready to generate samples.
 *
 * @param   OS Events that triggered call
 *
 * @return  Event that was handled
 **************************************************************************************************/
uint16 HalMotionHandleOSEvent( uint16 events )
{
  uint16 rtnVal = 0;

  if (events & HAL_MOTION_MEASUREMENT_START_EVENT)
  {
    HalMotionHandleMeasurementStartEvent();
    rtnVal = HAL_MOTION_MEASUREMENT_START_EVENT;
  }
  else if (events & HAL_MOTION_GYRO_POWERUP_DONE_EVENT)
  {
    HalMotionHandleCalPowerupDoneEvent();
    rtnVal = HAL_MOTION_GYRO_POWERUP_DONE_EVENT;
  }
  else if (events & HAL_GYRO_ACTIVE_EVENT)
  {
    HalMotionHandleGyroActiveEvent();
    rtnVal = HAL_GYRO_ACTIVE_EVENT;
  }
  else if (events & HAL_MOTION_DETECTED_EVENT)
  {
    halAccelProcessMotionDetectEvent();
    rtnVal = HAL_MOTION_DETECTED_EVENT;
  }

  return rtnVal;
}

/**************************************************************************************************
 * @fn      HalMotionGyroReady
 *
 * @brief   Handles event indicating motion sensor hardware is powered up and
 *          ready to use.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
static void HalMotionGyroReady( void )
{
  if (HalMotionState == HAL_MOTION_STATE_POWERING_ON)
  {
    if (HalMotionCalibrateGyro == FALSE)
    {
      HalGyroEnableI2CPassThru();
      HalAccelEnable();
    }

    HalGyroStartGyroMeasurements();

    /* Start timer for taking measurements */
    osal_start_timerEx( Hal_TaskID,
                        HAL_GYRO_ACTIVE_EVENT,
                        50 );
  }
}

/**************************************************************************************************
 * @fn      HalMotionHandleGyroActiveEvent
 *
 * @brief   Handles event indicating gyro should be producing samples.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
static void HalMotionHandleGyroActiveEvent( void )
{
  if (HalMotionState == HAL_MOTION_STATE_POWERING_ON)
  {
    /* Start timer for taking measurements */
    osal_start_reload_timer( Hal_TaskID,
                             HAL_MOTION_MEASUREMENT_START_EVENT,
                             HAL_MOTION_TIME_BETWEEN_MEASUREMENTS );

    if (HalMotionCalibrateGyro == FALSE)
    {
      /* Update state */
      HalMotionState = HAL_MOTION_STATE_WAITING_FOR_NEXT_MEASUREMENT;
    }
    else
    {
      /* Initiating a cal process ==> Reset Movea's motion processing library */
        motion_init.DeltaGain.X = HalMotionMouseGainTable[HalMotionMouseGainTableIndex].gainX;
        motion_init.DeltaGain.Y = HalMotionMouseGainTable[HalMotionMouseGainTableIndex].gainY;
        motion_init.GyroOffsets.X = HalMotionGyroOffsets.x;
        motion_init.GyroOffsets.Y = HalMotionGyroOffsets.y;
        motion_init.GyroOffsets.Z = HalMotionGyroOffsets.z;
        AIR_MOTION_Init(&motion_init);
    }
  }
}

/**************************************************************************************************
 * @fn      HalMotionHandleCalPowerupDoneEvent
 *
 * @brief   Handles event indicating gyro should be in a no motion condition
 *          and thus we can power it up and take some measurements for calibration.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalMotionHandleCalPowerupDoneEvent( void )
{
  /* power up gyro */
  HalGyroEnable( HalMotionGyroReady );

  /* Indicate we're going to run calibration */
  HalMotionCalibrateGyro = TRUE;

  /* Load counter of samples used during the calibration process */
  HalNbSamplesCalCnt = motion_init.StaticSamples * 2;
}

/**************************************************************************************************
 * @fn      HalMotionHandleMeasurementStartEvent
 *
 * @brief   Handles event indicating a new 10 msec measurement cycle has begun,
 *          so we should start taking samples.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
static void HalMotionHandleMeasurementStartEvent( void )
{
  if (HalMotionState == HAL_MOTION_STATE_WAITING_FOR_NEXT_MEASUREMENT)
  {
    int16 x, y, z;
#if (HAL_MOTION_ENABLE_ROLL_COMPENSATION == TRUE)
    int8 ax, ay, az;
#endif

    /* Update state */
    HalMotionState = HAL_MOTION_STATE_MEASURING;

    HalGyroRead(&x, &y, &z );

    samples.GyroSamples.X = x;
    samples.GyroSamples.Y = -y;
    samples.GyroSamples.Z = z;

    /* Done with Gyro samples, now get Accelerometer samples */
#if (HAL_MOTION_ENABLE_ROLL_COMPENSATION == TRUE)
    HalGyroReadAccelData( &ax, &ay, &az );
    samples.AccSamples.X = ax;
    samples.AccSamples.Y = ay;
    samples.AccSamples.Z = -az;
#endif

    /* Done with samples, so call Movea library routine to process them */
    motion_status = AIR_MOTION_ProcessDelta(samples);

    /* Update state */
    HalMotionState = HAL_MOTION_STATE_WAITING_FOR_NEXT_MEASUREMENT;

    if (motion_status.Status.IsDeltaComputed)
    {
      /* Avalid delta was computed                 */
      /* Inform application that results are ready */
      if (pHalMotionProcessFunction != NULL)
      {
        pHalMotionProcessFunction( motion_status.Delta.X,
                                   motion_status.Delta.Y );
      }
    }
  }
  else if (HalMotionCalibrateGyro == TRUE && HalMotionState == HAL_MOTION_STATE_POWERING_ON)
  {
    // Currently calibrating
    int16 x, y, z;
#if (HAL_MOTION_ENABLE_ROLL_COMPENSATION == TRUE)
    int8 ax, ay, az;
#endif

    HalMotionState = HAL_MOTION_STATE_MEASURING;

    HalGyroRead( &x, &y, &z );

    samples.GyroSamples.X = x;
    samples.GyroSamples.Y = -y;
    samples.GyroSamples.Z = z;

    /* Done with Gyro samples, now get Accelerometer samples */
#if (HAL_MOTION_ENABLE_ROLL_COMPENSATION == TRUE)
    HalGyroReadAccelData( &ax, &ay, &az );
    samples.AccSamples.X = ax;
    samples.AccSamples.Y = ay;
    samples.AccSamples.Z = -az;
#endif

    /* Done with samples, so call Movea library routine to process them */
    motion_status = AIR_MOTION_ProcessDelta(samples);

    if (motion_status.Status.NewGyroOffset == true)
    {
      // Found a new valid gyro offset
      HalMotionGyroOffsets.x = motion_status.GyroOffsets.X;
      HalMotionGyroOffsets.y = motion_status.GyroOffsets.Y;
      HalMotionGyroOffsets.z = motion_status.GyroOffsets.Z;

      /* Store gyro offsets in NVM */
      (void) osal_snv_write( HAL_MOTION_NV_ITEM_GYRO_OFFSETS_ID,
                             sizeof( HalMotionGyroOffsets ),
                            (uint8 *)&HalMotionGyroOffsets );

      /* Power down motion hardware and go back to initial state */
      HalMotionDisable();

      /* Inform application that calibration is done */
      if (pHalMotionCalCompleteNotificationFunction != NULL)
      {
        pHalMotionCalCompleteNotificationFunction();
      }
    }
    else if (HalNbSamplesCalCnt == 0)
    {
      // Too many attempts ==> Reset the cal process:
      if (pHalMotionCalCompleteNotificationFunction != NULL)
      {
        /* Inform application that the current calibration is still valid */
        pHalMotionCalCompleteNotificationFunction();
      }
      HalMotionDisable();
    }
    else
    {
      HalNbSamplesCalCnt--;
      /* Update state */
      HalMotionState = HAL_MOTION_STATE_POWERING_ON;
    }
  }
}

/**************************************************************************************************
 * @fn          HalMotionI2cRead
 *
 * @brief       This function implements the I2C protocol to read from the IMU-3000 gyroscope.
 *
 * input parameters
 *
 * @param       device - which device is being written
 *              addr - which register to read
 *              numBytes - number of bytes to read
 *              pBuf - pointer to buffer to place data
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************/
void HalMotionI2cRead( halMotionDevice_t device, uint8 addr, uint8 numBytes, uint8 *pBuf )
{
  uint8 localAddr = addr;

  /* Send address we're reading from */
  HalI2CWrite( HalMotionDeviceAddressTable[device], 1, &localAddr );

  /* Now read data */
  HalI2CRead( HalMotionDeviceAddressTable[device], numBytes, pBuf );
}

/**************************************************************************************************
 * @fn          HalMotionI2cWrite
 *
 * @brief       This function implements the I2C protocol to write to the IMU-3000 gyroscope.
 *
 * input parameters
 *
 * @param       device - which device is being written
 *              addr - which register to write
 *              pData - pointer to buffer containing data to be written
 *              numBytes - number of bytes of data to be written
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************/
void HalMotionI2cWrite( halMotionDevice_t device, uint8 addr, uint8 *pData, uint8 numBytes )
{
  static uint8 HalMotionI2CBuf[20];
  uint8 i;
  uint8 *pBuf = HalMotionI2CBuf;

  /* Copy address and data to local buffer for burst write */
  *pBuf++ = addr;
  for (i = 0; i < numBytes; i++)
  {
    *pBuf++ = *pData++;
  }

  /* Send address and data */
  HalI2CWrite( HalMotionDeviceAddressTable[device], numBytes + 1, HalMotionI2CBuf );
}

/**************************************************************************************************
**************************************************************************************************/
