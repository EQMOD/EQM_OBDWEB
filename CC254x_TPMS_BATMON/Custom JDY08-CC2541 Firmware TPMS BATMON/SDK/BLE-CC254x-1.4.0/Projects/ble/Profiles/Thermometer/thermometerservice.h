/**************************************************************************************************
  Filename:       ThermomterService.h
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the Thermometer service definitions and
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

#ifndef THERMOMETERSERVICE_H
#define THERMOMETERSERVICE_H

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

// Thermometer Service Parameters
#define THERMOMETER_TEMP                      0
#define THERMOMETER_TEMP_CHAR_CFG             1
#define THERMOMETER_TYPE                      2
#define THERMOMETER_INTERVAL                  3
#define THERMOMETER_INTERVAL_CHAR_CFG         4 
#define THERMOMETER_IMEAS_CHAR_CFG            5  
#define THERMOMETER_IRANGE                    6   

// Position  in attribute array
#define THERMOMETER_TEMP_VALUE_POS            2
#define THERMOMETER_TEMP_CHAR_CONFIG_POS      3
#define THERMOMETER_IMEAS_VALUE_POS           7
#define THERMOMETER_IMEAS_CHAR_CONFIG_POS     8
#define THERMOMETER_INTERVAL_VALUE_POS       10
#define THERMOMETER_INTERVAL_CHAR_CONFIG_POS 11   
  
  // Length of bytes  
#define THERMOMETER_INTERVAL_LEN              1
#define THERMOMETER_TYPE_LEN                  1    
#define THERMOMETER_IRANGE_LEN                2  

// Maximum length of thermometer
// measurement characteristic
#define THERMOMETER_MEAS_MAX  (ATT_MTU_SIZE -5)

// Values for flags
#define THERMOMETER_FLAGS_CELCIUS          0x00
#define THERMOMETER_FLAGS_FARENHEIT        0x01 
#define THERMOMETER_FLAGS_TIMESTAMP        0x02
#define THERMOMETER_FLAGS_TYPE             0x04  

// Values for sensor location
#define THERMOMETER_TYPE_ARMPIT            0x01
#define THERMOMETER_TYPE_BODY              0x02
#define THERMOMETER_TYPE_EAR               0x03
#define THERMOMETER_TYPE_FINGER            0x04
#define THERMOMETER_TYPE_GASTRO            0x05
#define THERMOMETER_TYPE_MOUTH             0x06
#define THERMOMETER_TYPE_RECTUM            0x07
#define THERMOMETER_TYPE_TOE               0x08
#define THERMOMETER_TYPE_TYMPNUM           0x09
  
// Thermometer Service bit fields
#define THERMOMETER_SERVICE          0x00000001

// Callback events
#define THERMOMETER_TEMP_IND_ENABLED          1
#define THERMOMETER_TEMP_IND_DISABLED         2
#define THERMOMETER_IMEAS_NOTI_ENABLED        3
#define THERMOMETER_IMEAS_NOTI_DISABLED       4
#define THERMOMETER_INTERVAL_IND_ENABLED      5
#define THERMOMETER_INTERVAL_IND_DISABLED     6  
#define THERMOMETER_INTERVAL_SET              7
#define THERMOMETER_TESTCMD_C                 8
#define THERMOMETER_TESTCMD_F                 9  

/*********************************************************************
 * TYPEDEFS
 */

// Thermometer Service callback function
typedef void (*thermometerServiceCB_t)(uint8 event);

/**
 * Thermometer Interval Range 
 */
typedef struct
{
  uint8 low;         
  uint8 high; 
} thermometerIRange_t;
  
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
 * Thermometer_AddService- Initializes the Thermometer service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t Thermometer_AddService( uint32 services );

/*
 * Thermometer_Register - Register a callback function with the
 *          Thermometer Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void Thermometer_Register( thermometerServiceCB_t pfnServiceCB );

/*
 * Thermometer_SetParameter - Set a Thermometer parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Thermometer_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * Thermometer_GetParameter - Get a Thermometer parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Thermometer_GetParameter( uint8 param, void *value );

/*********************************************************************
 * @fn          Thermometer_TempIndicate
 *
 * @brief       Send a notification containing a thermometer
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Thermometer_TempIndicate( uint16 connHandle, attHandleValueInd_t *pNoti, uint8 taskId );

/*********************************************************************
 * @fn          Thermometer_IntervalIndicate
 *
 * @brief       Send a interval change indication
 *              
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Thermometer_IntervalIndicate( uint16 connHandle, attHandleValueInd_t *pNoti,uint8 taskId );

/*********************************************************************
 * @fn          Thermometer_IMeasNotify
 *
 * @brief       Send a intermediate temperature notification
 *              
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Thermometer_IMeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* THERMOMETERSERVICE_H */
