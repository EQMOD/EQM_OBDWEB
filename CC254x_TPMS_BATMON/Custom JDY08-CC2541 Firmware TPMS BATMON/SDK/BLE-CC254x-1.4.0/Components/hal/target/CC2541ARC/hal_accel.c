/**************************************************************************************************
  Filename:       hal_accel.c
  Revised:        $Date: 2012-10-31 15:44:56 -0700 (Wed, 31 Oct 2012) $
  Revision:       $Revision: 32001 $

  Description:

  This file contains the declaration to the HAL Accelerometer abstraction layer
  for the Kionix KXTF9 Accelerometer.


  Copyright 2010-2013 Texas Instruments Incorporated. All rights reserved.

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

#include "comdef.h"
#include "hal_accel.h"
#include "hal_board_cfg.h"
#include "hal_drivers.h"
#include "hal_motion.h"
#include "osal.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* KXTF9 register addresses */
#define ACC_REG_ADDR_XOUT_HPF_L            0x00
#define ACC_REG_ADDR_XOUT_HPF_H            0x01
#define ACC_REG_ADDR_YOUT_HPF_L            0x02
#define ACC_REG_ADDR_YOUT_HPF_H            0x03
#define ACC_REG_ADDR_ZOUT_HPF_L            0x04
#define ACC_REG_ADDR_ZOUT_HPF_H            0x05
#define ACC_REG_ADDR_XOUT_L                0x06
#define ACC_REG_ADDR_XOUT_H                0x07
#define ACC_REG_ADDR_YOUT_L                0x08
#define ACC_REG_ADDR_YOUT_H                0x09
#define ACC_REG_ADDR_ZOUT_L                0x0A
#define ACC_REG_ADDR_ZOUT_H                0x0B
#define ACC_REG_ADDR_DCST_RESP             0x0C
#define ACC_REG_ADDR_WHO_AM_I              0x0F
#define ACC_REG_ADDR_TILT_POS_CUR          0x10
#define ACC_REG_ADDR_TILT_POS_PRE          0x11
#define ACC_REG_ADDR_INT_SRC_REG1          0x15
#define ACC_REG_ADDR_INT_SRC_REG2          0x16
#define ACC_REG_ADDR_STATUS_REG            0x18
#define ACC_REG_ADDR_INT_REL               0x1A
#define ACC_REG_ADDR_CTRL_REG1             0x1B
#define ACC_REG_ADDR_CTRL_REG2             0x1C
#define ACC_REG_ADDR_CTRL_REG3             0x1D
#define ACC_REG_ADDR_INT_CTRL_REG1         0x1E
#define ACC_REG_ADDR_INT_CTRL_REG2         0x1F
#define ACC_REG_ADDR_INT_CTRL_REG3         0x20
#define ACC_REG_ADDR_DATA_CTRL_REG         0x21
#define ACC_REG_ADDR_TILT_TIMER            0x28
#define ACC_REG_ADDR_WUF_TIMER             0x29
#define ACC_REG_ADDR_TDT_TIMER             0x2B
#define ACC_REG_ADDR_TDT_H_THRESH          0x2C
#define ACC_REG_ADDR_TDT_L_THRESH          0x2D
#define ACC_REG_ADDR_TAP_TIMER             0x2E
#define ACC_REG_ADDR_TDT_TOTAL_TIMER       0x2F
#define ACC_REG_ADDR_TDT_LATENCY_TIMER     0x30
#define ACC_REG_ADDR_TDT_WINDOW_TIMER      0x31
#define ACC_REG_ADDR_SELF_TEST             0x3A
#define ACC_REG_ADDR_WUF_THRESH            0x5A
#define ACC_REG_ADDR_TILT_ANGLE            0x5C
#define ACC_REG_ADDR_HYST_SET              0x5F

/* KXTF9 register field values */
#define CTRL_REG1_PC1_OPERATING_MODE 0x80
#define CTRL_REG1_PC1_STANDBY_MODE   0x00
#define CTRL_REG1_RES_12BIT          0x40
#define CTRL_REG1_RES_8BIT           0x00
#define CTRL_REG1_DRDYE_INT_ENABLED  0x20
#define CTRL_REG1_DRDYE_INT_DISABLED 0x00
#define CTRL_REG1_GSEL_8G            0x10
#define CTRL_REG1_GSEL_4G            0x08
#define CTRL_REG1_GSEL_2G            0x00
#define CTRL_REG1_TDTE_ENABLED       0x04
#define CTRL_REG1_TDTE_DISABLED      0x00
#define CTRL_REG1_WUFE_ENABLED       0x02
#define CTRL_REG1_WUFE_DISABLED      0x00
#define CTRL_REG1_TPE_ENABLED        0x01
#define CTRL_REG1_TPE_DISABLED       0x00

#define CTRL_REG3_OWUF_ODR_100HZ     0x02

#define INT_CTRL_REG1_INT_ENABLED    0x20
#define INT_CTRL_REG1_POLARITY_HIGH  0x10

#define INT_CTRL_REG2_X_AXIS_ENABLED 0x80
#define INT_CTRL_REG2_Y_AXIS_ENABLED 0x40
#define INT_CTRL_REG2_Z_AXIS_ENABLED 0x20

#define DATA_CTRL_REG_LPF_ODR_50HZ   0x02
#define DATA_CTRL_REG_LPF_ODR_100HZ  0x03
#define DATA_CTRL_REG_LPF_ODR_200HZ  0x04
#define DATA_CTRL_REG_LPF_ODR_400HZ  0x05
#define DATA_CTRL_REG_LPF_ODR_800HZ  0x06

/* The following define which port pins are being used by the accelerometer */
#define HAL_ACCEL_P1_GPIO_PINS       ( BV(2) )

/* These defines indicate the direction of each pin */
#define HAL_ACCEL_P1_INPUT_PINS      ( BV(2) )

/* Which pins are used for key interrupts */
#define HAL_ACCEL_P1_INTERRUPT_PINS  ( BV(2) )

/* Defines for each output pin assignment */
#define HAL_ACCEL_POWER_PIN_BV       ( BV(6) )
#define HAL_ACCEL_POWER_PIN          P1_6
#define HAL_ACCEL_POWER_PxDIR        P1DIR
#define HAL_ACCEL_POWER_PxSEL        P1SEL

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint8 addr;
  uint8 data;
} halAccelConfig_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */
static halAccelConfig_t HalAccelEnableConfigTable[] =
{
  { ACC_REG_ADDR_CTRL_REG1, 0 }, // must first set PC1 to 0 before changing configuration
  { ACC_REG_ADDR_CTRL_REG2, 0 },
  { ACC_REG_ADDR_CTRL_REG3, 0 },
  { ACC_REG_ADDR_INT_CTRL_REG1, INT_CTRL_REG1_POLARITY_HIGH },
  { ACC_REG_ADDR_INT_CTRL_REG2, 0 },
  { ACC_REG_ADDR_INT_CTRL_REG3, 0 },
  { ACC_REG_ADDR_DATA_CTRL_REG, DATA_CTRL_REG_LPF_ODR_200HZ },
  { ACC_REG_ADDR_CTRL_REG1, CTRL_REG1_PC1_OPERATING_MODE | CTRL_REG1_GSEL_2G}
};
#define HAL_ACCEL_ENABLE_CONFIG_TABLE_SIZE (sizeof(HalAccelEnableConfigTable) / sizeof(HalAccelEnableConfigTable[0]))

static halAccelConfig_t HalAccelMotionDetectConfigTable[] =
{
  { ACC_REG_ADDR_CTRL_REG1, 0 }, // must first set PC1 to 0 before changing configuration
  { ACC_REG_ADDR_CTRL_REG3, CTRL_REG3_OWUF_ODR_100HZ },
  { ACC_REG_ADDR_INT_CTRL_REG1, INT_CTRL_REG1_INT_ENABLED | INT_CTRL_REG1_POLARITY_HIGH },
  { ACC_REG_ADDR_INT_CTRL_REG2, INT_CTRL_REG2_X_AXIS_ENABLED | INT_CTRL_REG2_Y_AXIS_ENABLED | INT_CTRL_REG2_Z_AXIS_ENABLED },
  { ACC_REG_ADDR_INT_CTRL_REG3, 0 },
  { ACC_REG_ADDR_DATA_CTRL_REG, DATA_CTRL_REG_LPF_ODR_200HZ },
  { ACC_REG_ADDR_WUF_TIMER, 25 }, // since ODR is 100 Hz, this will be 250ms
  { ACC_REG_ADDR_WUF_THRESH, 0x08 }, // this is factory setting of 0.5g, but is here in case we want to change it
  { ACC_REG_ADDR_CTRL_REG1, CTRL_REG1_PC1_OPERATING_MODE | CTRL_REG1_WUFE_ENABLED | CTRL_REG1_GSEL_2G }
};
#define HAL_ACCEL_MOTION_DETECT_CONFIG_TABLE_SIZE (sizeof(HalAccelMotionDetectConfigTable) / sizeof(HalAccelMotionDetectConfigTable[0]))

static halAccelConfig_t HalAccelLowPowerConfigTable[] =
{
  { ACC_REG_ADDR_CTRL_REG1, 0 } // should put part in standby mode
};
#define HAL_ACCEL_LOW_POWER_CONFIG_TABLE_SIZE (sizeof(HalAccelLowPowerConfigTable) / sizeof(HalAccelLowPowerConfigTable[0]))

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
static void halAccelProcessMotionDetectInterrupt( void );

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static halAccelCBack_t pHalAccelProcessFunction;

/**************************************************************************************************
 * @fn      halProcessMotionDetectInterrupt
 *
 * @brief   Processes motion detection interrupt by informing the entity that
 *          enabled motion detection that it has occurred. Also disables further
 *          motion detection interrupts.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
static void halAccelProcessMotionDetectInterrupt( void )
{
  /* Disable port interrupt */
  P1IEN &= (uint8) ~HAL_ACCEL_P1_INTERRUPT_PINS;

  /* Disable P1 interrupts */
  IEN2 &= ~(BV( 4 ));

  /* Set event to process motion detection not in interrupt context */
  osal_set_event( Hal_TaskID, HAL_MOTION_DETECTED_EVENT );
}

/**************************************************************************************************
 * @fn      halAccelProcessMotionDetectEvent
 *
 * @brief   Post interrupt motion detection event processing.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halAccelProcessMotionDetectEvent( void )
{
  /* Inform application that motion was detected */
  if (pHalAccelProcessFunction != NULL)
  {
    pHalAccelProcessFunction();
  }
}

/**************************************************************************************************
 * @fn          HalAccelInit
 *
 * @brief       This function initializes the HAL Accelerometer abstraction layer.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalAccelInit( void )
{
  /* Configure POWER PIN as output */
  HAL_ACCEL_POWER_PxDIR |= HAL_ACCEL_POWER_PIN_BV;

  /* Set POWER output high */
  HAL_ACCEL_POWER_PIN = 1;

  /* Configure pin function as GPIO for pins related to accelerometer */
  HAL_ACCEL_POWER_PxSEL &= ~HAL_ACCEL_POWER_PIN_BV;
  P1SEL &= (uint8) ~HAL_ACCEL_P1_GPIO_PINS;

  /* Configure direction of pins related to accelerometer */
  P1DIR &= (uint8) ~HAL_ACCEL_P1_INPUT_PINS;

  /* Configure port 1 inputs (i.e. accelerometer interrupt pin) as pulldown
   * to save current.
   */
  P2INP |= 0x40;

  /* Indicate no callback has been registered */
  pHalAccelProcessFunction = NULL;
}

/**************************************************************************************************
 * @fn          HalAccelEnable
 *
 * @brief       This function configures the accelerometer for operation.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalAccelEnable( void )
{
  uint8 i, dummy;

  /* Ensure accelerometer is in standby mode */
  HalMotionI2cWrite( HAL_MOTION_DEVICE_ACCELEROMETER,
                     ACC_REG_ADDR_CTRL_REG1,
                     &HalAccelEnableConfigTable[0].data,
                     1 );

  /* Read INT_REL register to clear any prior accelerometer interrupts */
  HalMotionI2cRead( HAL_MOTION_DEVICE_ACCELEROMETER,
                    ACC_REG_ADDR_INT_REL,
                    1,
                    &dummy );

  /* Configure accelerometer for measurement mode */
  for (i = 0; i < HAL_ACCEL_ENABLE_CONFIG_TABLE_SIZE; i++)
  {
    HalMotionI2cWrite( HAL_MOTION_DEVICE_ACCELEROMETER,
                       HalAccelEnableConfigTable[i].addr,
                       &HalAccelEnableConfigTable[i].data,
                       1 );
  }
}

/**************************************************************************************************
 * @fn          HalAccelDisable
 *
 * @brief       Places the Accelerometer in low power mode.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalAccelDisable( void )
{
  uint8 i;

  /* Configure accelerometer for low power mode */
  for (i = 0; i < HAL_ACCEL_LOW_POWER_CONFIG_TABLE_SIZE; i++)
  {
    HalMotionI2cWrite( HAL_MOTION_DEVICE_ACCELEROMETER,
                       HalAccelLowPowerConfigTable[i].addr,
                       &HalAccelLowPowerConfigTable[i].data,
                       1 );
  }
}

/**************************************************************************************************
 * @fn          HalAccelMotionDetect
 *
 * @brief       Places the Accelerometer in motion detection mode.
 *
 * input parameters
 *
 * @param       cback - callback function to invoke if motion is detected
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalAccelMotionDetect( halAccelCBack_t cback )
{
  uint8 i;

  /* Register the callback fucntion */
  pHalAccelProcessFunction = cback;

  /* Configure accelerometer for motion detection mode */
  for (i = 0; i < HAL_ACCEL_MOTION_DETECT_CONFIG_TABLE_SIZE; i++)
  {
    HalMotionI2cWrite( HAL_MOTION_DEVICE_ACCELEROMETER,
                       HalAccelMotionDetectConfigTable[i].addr,
                       &HalAccelMotionDetectConfigTable[i].data,
                       1 );
  }

  /* Clear any prior P1 interrupts */
  P1IFG = (uint8) (~HAL_ACCEL_P1_INTERRUPT_PINS);
  P1IF = 0;

  /* Enable interrupt when motion is detected */
  P1IEN |= HAL_ACCEL_P1_INTERRUPT_PINS;

  /* Enable P1 interrupts */
  IEN2 |= BV( 4 );
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINES
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halAccelPort1Isr
 *
 * @brief   Port 1 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halAccelPort1Isr, P1INT_VECTOR )
{
  HAL_ENTER_ISR();

  /* Make sure we're processing the desired interrupt */
  if ((P1IFG & HAL_ACCEL_P1_INTERRUPT_PINS) && (P1IEN & HAL_ACCEL_P1_INTERRUPT_PINS))
  {
    halAccelProcessMotionDetectInterrupt();
  }

  P1IFG = 0;
  P1IF = 0;

  HAL_EXIT_ISR();
}
/**************************************************************************************************
*/
