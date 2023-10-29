/**************************************************************************************************
  Filename:       bpService.h
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the BloodPressure service definitions and
                  prototypes.

 Copyright 2011 - 2013 Texas Instruments Incorporated. All rights reserved.

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

#ifndef BLOODPRESSURESERVICE_H
#define BLOODPRESSURESERVICE_H

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
  
// BloodPressure Service Parameters
#define BLOODPRESSURE_MEAS                      0
#define BLOODPRESSURE_MEAS_CHAR_CFG             1
#define BLOODPRESSURE_IMEAS_CHAR_CFG            2
#define BLOODPRESSURE_TIMESTAMP                 3
#define BLOODPRESSURE_PULSE                     4
#define BLOODPRESSURE_INTERVAL                  5
  
// Length of measurements  
#define BLOODPRESSURE_TIMESTAMP_LEN             7 //length of timestamp  
#define BLOODPRESSURE_TIME_LEN                  7 //length of timestamp 
#define BLOODPRESSURE_INTERVAL_LEN              1  

// Maximum length of blood pressure measurement characteristic
#define BLOODPRESSURE_MEAS_MAX                  (ATT_MTU_SIZE -5)

// Values for flags
#define BLOODPRESSURE_FLAGS_MMHG                0x00
#define BLOODPRESSURE_FLAGS_KPA                 0x01 
#define BLOODPRESSURE_FLAGS_TIMESTAMP           0x02
#define BLOODPRESSURE_FLAGS_PULSE               0x04 
#define BLOODPRESSURE_FLAGS_USER                0x08   
#define BLOODPRESSURE_FLAGS_STATUS              0x10
  
  
// Values for sensor location
#define BLOODPRESSURE_SITE_ARMPIT               0x01
#define BLOODPRESSURE_SITE_BODY                 0x02
#define BLOODPRESSURE_SITE_EAR                  0x03
#define BLOODPRESSURE_SITE_FINGER               0x04
#define BLOODPRESSURE_SITE_GASTRO               0x05
#define BLOODPRESSURE_SITE_MOUTH                0x06
#define BLOODPRESSURE_SITE_RECTUM               0x07
#define BLOODPRESSURE_SITE_TOE                  0x08
#define BLOODPRESSURE_SITE_TYMPNUM              0x09
   
// BloodPressure Service bit fields
#define BLOODPRESSURE_SERVICE                   0x00000001

// Callback events
#define BLOODPRESSURE_MEAS_NOTI_ENABLED         1
#define BLOODPRESSURE_MEAS_NOTI_DISABLED        2
#define BLOODPRESSURE_IMEAS_NOTI_ENABLED        3
#define BLOODPRESSURE_IMEAS_NOTI_DISABLED       4  
#define BLOODPRESSURE_TIME_SET                  5

/*********************************************************************
 * TYPEDEFS
 */

// BloodPressure Service callback function
typedef void (*bloodPressureServiceCB_t)(uint8 event);

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
 * BloodPressure_AddService- Initializes the BloodPressure service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t BloodPressure_AddService( uint32 services );

/*
 * BloodPressure_Register - Register a callback function with the
 *          BloodPressure Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void BloodPressure_Register( bloodPressureServiceCB_t pfnServiceCB );

/*
 * BloodPressure_SetParameter - Set a BloodPressure parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t BloodPressure_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * BloodPressure_GetParameter - Get a BloodPressure parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t BloodPressure_GetParameter( uint8 param, void *value );

/*********************************************************************
 * @fn          BloodPressure_MeasIndicate
 *
 * @brief       Send a notification containing a blood pressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t BloodPressure_MeasIndicate( uint16 connHandle, attHandleValueInd_t *pNoti, uint8 taskId );




/*********************************************************************
 * @fn          BloodPressure_IMeasNotify
 *
 * @brief       Send a notification containing a blood pressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t BloodPressure_IMeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BLOODPRESSURESERVICE_H */
