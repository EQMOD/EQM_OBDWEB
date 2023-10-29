/**************************************************************************************************
  Filename:       heartrateservice.h
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the Heart Rate service definitions and
                  prototypes.

  Copyright 2012 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef HEARTRATESERVICE_H
#define HEARTRATESERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Heart Rate Service Parameters
#define HEARTRATE_MEAS                      0
#define HEARTRATE_MEAS_CHAR_CFG             1
#define HEARTRATE_SENS_LOC                  2
#define HEARTRATE_COMMAND                   3

// Maximum length of heart rate measurement characteristic
#define HEARTRATE_MEAS_MAX                  (ATT_MTU_SIZE -5)

// Values for flags
#define HEARTRATE_FLAGS_FORMAT_UINT16       0x01
#define HEARTRATE_FLAGS_CONTACT_NOT_SUP     0x00
#define HEARTRATE_FLAGS_CONTACT_NOT_DET     0x04
#define HEARTRATE_FLAGS_CONTACT_DET         0x06
#define HEARTRATE_FLAGS_ENERGY_EXP          0x08
#define HEARTRATE_FLAGS_RR                  0x10

// Values for sensor location
#define HEARTRATE_SENS_LOC_OTHER            0x00
#define HEARTRATE_SENS_LOC_CHEST            0x01
#define HEARTRATE_SENS_LOC_WRIST            0x02
#define HEARTRATE_SENS_LOC_FINGER           0x03
#define HEARTRATE_SENS_LOC_HAND             0x04
#define HEARTRATE_SENS_LOC_EARLOBE          0x05
#define HEARTRATE_SENS_LOC_FOOT             0x06

// Value for command characteristic
#define HEARTRATE_COMMAND_ENERGY_EXP        0x01

// ATT Error code
// Control point value not supported
#define HEARTRATE_ERR_NOT_SUP               0x80

// Heart Rate Service bit fields
#define HEARTRATE_SERVICE                   0x00000001

// Callback events
#define HEARTRATE_MEAS_NOTI_ENABLED         1
#define HEARTRATE_MEAS_NOTI_DISABLED        2
#define HEARTRATE_COMMAND_SET               3

/*********************************************************************
 * TYPEDEFS
 */

// Heart Rate Service callback function
typedef void (*heartRateServiceCB_t)(uint8 event);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * HeartRate_AddService- Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t HeartRate_AddService( uint32 services );

/*
 * HeartRate_Register - Register a callback function with the
 *          Heart Rate Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void HeartRate_Register( heartRateServiceCB_t pfnServiceCB );

/*
 * HeartRate_SetParameter - Set a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t HeartRate_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * HeartRate_GetParameter - Get a Heart Rate parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t HeartRate_GetParameter( uint8 param, void *value );

/*********************************************************************
 * @fn          HeartRate_MeasNotify
 *
 * @brief       Send a notification containing a heart rate
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t HeartRate_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );

/*********************************************************************
 * @fn          HeartRate_HandleConnStatusCB
 *
 * @brief       Heart Rate Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
extern void HeartRate_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATESERVICE_H */
