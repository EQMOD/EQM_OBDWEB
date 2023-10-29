/**************************************************************************************************
  Filename:       cyclingservice.h
  Revised:        $Date: 2013-08-15 15:28:40 -0700 (Thu, 15 Aug 2013) $
  Revision:       $Revision: 34986 $

  Description:    This file contains the Cycling Speed and Cadence (CSC) service
                  definitions and prototypes.

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

#ifndef CYCLINGSERVICE_H
#define CYCLINGSERVICE_H

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

#define CYCLING_SERVICE             0x00000001

//ATT Error Codes
#define CSC_ERR_PROC_IN_PROGRESS    0x80
#define CSC_ERR_CCC_IMPROPER_CFG    0x81

#define CSC_SUCCESS                 1
#define CSC_OPCODE_NOT_SUPPORTED    2
#define CSC_INVALID_PARAMETER       3

//CSC Service Parameters
#define CSC_MEAS                    1
#define CSC_MEAS_CHAR_CFG           2
#define CSC_FEATURE                 3
#define CSC_SENS_LOC                4
#define CSC_COMMAND                 5
#define CSC_COMMAND_CHAR_CFG        6
#define CSC_AVAIL_SENS_LOCS         7

//CSC Fields
#define CSC_WHEEL_REV_PRESENT       0x01
#define CSC_CRANK_REV_PRESENT       0x02

//CSC SUPPORTED FEATURES
#define CSC_NO_SUPPORT              0x00
#define CSC_WHEEL_REV_SUPP          0x01
#define CSC_CRANK_REV_SUPP          0x02
#define CSC_MULTI_SENS_SUPP         0x04
#define CSC_FULL_SUPPORT            0x07

//CSC Censor Locations
#define CSC_SENSOR_LOC_OTHER        0
#define CSC_SENSOR_LOC_TOP_OF_SHOE  1
#define CSC_SENSOR_LOC_IN_SHOE      2
#define CSC_SENSOR_LOC_HIP          3
#define CSC_SENSOR_LOC_FRONT_WHEEL  4
#define CSC_SENSOR_LOC_LEFT_CRANK   5
#define CSC_SENSOR_LOC_RIGHT_CRANK  6
#define CSC_SENSOR_LOC_LEFT_PEDAL   7
#define CSC_SENSOR_LOC_RIGHT_PEDAL  8
#define CSC_SENSOR_LOC_FRONT_HUB    9
#define CSC_SENSOR_LOC_REAR_DROPOUT 10
#define CSC_SENSOR_LOC_CHAINSTAY    11
#define CSC_SENSOR_LOC_REAR_WHEEL   12
#define CSC_SENSOR_LOC_REAR_HUB     13

//Spec says there are 14 possible.
#define CSC_MAX_SENSOR_LOCS         14

//CSC Commands
#define CSC_SET_CUMM_VAL            1
#define CSC_START_SENS_CALIB        2
#define CSC_UPDATE_SENS_LOC         3
#define CSC_REQ_SUPP_SENS_LOC       4
#define CSC_COMMAND_RSP             16

// Values for flags
#define CSC_FLAGS_AT_REST           0x00
#define CSC_FLAGS_SPEED             0x01
#define CSC_FLAGS_CADENCE           0x02
#define CSC_FLAGS_SPEED_CADENCE     0x03


#define DEFAULT_NOTI_INTERVAL       1000  // in milliseconds

#define VALUE_ROLL_OVER             64000 // in milliseconds

// Callback events
#define CSC_CMD_SET_CUMM_VAL        1
#define CSC_CMD_START_SENS_CALIB    2
#define CSC_CMD_UPDATE_SENS_LOC     3
#define CSC_MEAS_NOTI_ENABLED       4
#define CSC_MEAS_NOTI_DISABLED      5
#define CSC_READ_ATTR               6
#define CSC_WRITE_ATTR              7

/*********************************************************************
 * TYPEDEFS
 */

// CSC service callback function
typedef void (*cyclingServiceCB_t)( uint8 event, uint32 *pNewCummVal );

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

/*********************************************************************
 * API FUNCTIONS
 */


/*********************************************************************
 * @fn      CyclingService_Init
 *
 * @brief   collect the OSAL task ID.
 *
 * @param   task_id - OSAL task ID.
 *
 * @return  none
 */
extern void CyclingService_Init( uint8 task_id );

/*********************************************************************
 * @fn      CyclingService_ProcessEvent
 *
 * @brief   process incoming event.
 *
 * @param   task_id - OSAL task id.
 *
 * @param   events - event bit(s) set for the task(s)
 *
 * @return  remaining event bits
 */
extern uint16 CyclingService_ProcessEvent( uint8 task_id, uint16 events );

/*
 * @fn      Cycling_AddService
 *
 * @brief   Initializes the CSC service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t Cycling_AddService( uint32 services );

/*
 * @fn      Cycling_Register
 *
 * @brief   Register a callback function with the
 *          CSC Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  none
 */

extern void Cycling_Register( cyclingServiceCB_t pfnServiceCB );

/*
 * @fn      Cycling_SetParameter
 *
 * @brief   Set a CSC parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Cycling_SetParameter( uint8 param, uint8 len, void *value );

/*********************************************************************
 * @fn      Cycling_GetParameter
 *
 * @brief   Get a CSC parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Cycling_GetParameter( uint8 param, void *value );

/*********************************************************************
 * @fn          Cycling_MeasNotify
 *
 * @brief       Send a notification containing a CSC
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Cycling_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );

/*********************************************************************
 * @fn          Cycling_HandleConnStatusCB
 *
 * @brief       CSC Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
extern void Cycling_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CYCLINGSERVICE_H */
