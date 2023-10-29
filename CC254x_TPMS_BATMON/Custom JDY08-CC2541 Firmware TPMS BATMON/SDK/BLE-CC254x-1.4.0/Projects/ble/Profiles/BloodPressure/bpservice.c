/**************************************************************************************************
  Filename:       bpservice.c
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the BloodPressure sample service 
                  for use with the BloodPressure   sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "bpservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of bloodPressure measurement value in attribute array
#define BLOODPRESSURE_MEAS_VALUE_POS       2
#define BLOODPRESSURE_MEAS_CONFIG_POS      3
#define BLOODPRESSURE_IMEAS_VALUE_POS      5 
#define BLOODPRESSURE_IMEAS_CONFIG_POS     6 
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// BloodPressure service
CONST uint8 bloodPressureServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLOODPRESSURE_SERV_UUID), HI_UINT16(BLOODPRESSURE_SERV_UUID)
};

// BloodPressure temperature characteristic
CONST uint8 bloodPressureTempUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLOODPRESSURE_MEAS_UUID), HI_UINT16(BLOODPRESSURE_MEAS_UUID)
};

// BloodPressure Intermediate Cuff Pressure
CONST uint8 bloodPressureImeasUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(IMEDIATE_CUFF_PRESSURE_UUID), HI_UINT16(IMEDIATE_CUFF_PRESSURE_UUID)
};

// BloodPressure Feature
CONST uint8 bpFeatureUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLOODPRESSURE_FEATURE_UUID), HI_UINT16(BLOODPRESSURE_FEATURE_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static bloodPressureServiceCB_t bloodPressureServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// BloodPressure Service attribute
static CONST gattAttrType_t bloodPressureService = { ATT_BT_UUID_SIZE, bloodPressureServUUID };

// BloodPressure Characteristic
static uint8 bloodPressureTempProps = GATT_PROP_INDICATE;
static gattCharCfg_t bloodPressureMeasConfig[GATT_MAX_NUM_CONN];
static uint8 bloodPressureTemp = 0;

// Intermediate Measurement
static uint8  bloodPressureImeasProps = GATT_PROP_NOTIFY;
static uint8  bloodPressureImeas=0;
static gattCharCfg_t bloodPressureIMeasConfig[GATT_MAX_NUM_CONN];

// BP Feature
    /*
    bit 0 Body Movement Detection Support bit
    bit 1 Cuff Fit Detection Support bit
    bit 2 Irregular Pulse Detection Support bit
    bit 3 Pulse Rate Range Detection Support bit
    bit 4 Measurement Position Detection Support bit
    bit 5 Multiple Bond Support bit
    bit 6. Reserved for Future Use
    */
static uint8  bpFeatureProps = GATT_PROP_READ;
static uint16 bpFeature = 0;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t bloodPressureAttrTbl[] = 
{
  // BloodPressure Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&bloodPressureService            /* pValue */
  },

    //////////////////////////////////////////////
    // BLOOD PRESSURE MEASUREMENT 
    //////////////////////////////////////////////
    
    // 1. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &bloodPressureTempProps 
    },

    // 2. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, bloodPressureTempUUID },
      0, //return READ_NOT_PERMITTED
      0, 
      &bloodPressureTemp 
    },

    // 3.Characteristic Configuration
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *)&bloodPressureMeasConfig
    }, 
 
    //////////////////////////////////////////////
    // INTERMEDIATE CUFF PRESSURE
    //////////////////////////////////////////////
    
    // 4.Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &bloodPressureImeasProps 
    },

    // 5.Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, bloodPressureImeasUUID },
      0, //return READ_NOT_PERMITTED
      0, 
      &bloodPressureImeas 
    },

    // 6.Characteristic Configuration
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *)&bloodPressureIMeasConfig
    },

    //////////////////////////////////////////////
    // FEATURE
    //////////////////////////////////////////////
    
    // 7.Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &bpFeatureProps 
    },

    // 8.Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, bpFeatureUUID },
      GATT_PERMIT_READ,
      0, 
      (uint8 *)&bpFeature 
    },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 bloodPressure_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t bloodPressure_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void bloodPressure_HandleConnStatusCB( uint16 connHandle, uint8 changeType );
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Blood Pressure Service Callbacks
CONST gattServiceCBs_t bloodPressureCBs =
{
  bloodPressure_ReadAttrCB,  // Read callback function pointer
  bloodPressure_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BloodPressure_AddService
 *
 * @brief   Initializes the BloodPressure   service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t BloodPressure_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, bloodPressureMeasConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, bloodPressureIMeasConfig );
  
  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( bloodPressure_HandleConnStatusCB ); 
  
  if ( services & BLOODPRESSURE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( bloodPressureAttrTbl, 
                                          GATT_NUM_ATTRS( bloodPressureAttrTbl ),
                                          &bloodPressureCBs );
     
  }
  return ( status );
}

/*********************************************************************
 * @fn      BloodPressure_Register
 *
 * @brief   Register a callback function with the BloodPressure Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void BloodPressure_Register( bloodPressureServiceCB_t pfnServiceCB )
{
  bloodPressureServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      BloodPressure_SetParameter
 *
 * @brief   Set a thermomter parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t BloodPressure_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
 
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      BloodPressure_GetParameter
 *
 * @brief   Get a BloodPressure   parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t BloodPressure_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          BloodPressure_MeasIndicate
 *
 * @brief       Send a indication containing a bloodPressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t BloodPressure_MeasIndicate( uint16 connHandle, attHandleValueInd_t *pNoti,uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, bloodPressureMeasConfig );

  // If indications enabled
  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    // Set the handle
    pNoti->handle = bloodPressureAttrTbl[BLOODPRESSURE_MEAS_VALUE_POS].handle;
  
    // Send the Indication
    return GATT_Indication( connHandle, pNoti, FALSE, taskId );
  }

  return bleIncorrectMode;
}


/*********************************************************************
 * @fn          BloodPressure_IMeasNotify
 *
 * @brief       Send a notification containing a bloodPressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t BloodPressure_IMeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, bloodPressureIMeasConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = bloodPressureAttrTbl[BLOODPRESSURE_IMEAS_VALUE_POS].handle;
  
    // Send the Indication
    return GATT_Notification( connHandle, pNoti, FALSE);
   
  }
    return bleIncorrectMode;
  
}

/*********************************************************************
 * @fn          bloodPressure_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 bloodPressure_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {

      case BLOODPRESSURE_FEATURE_UUID:
        {
          *pLen = 2;
          pValue[0] = 0;
          pValue[1] = 0;
          }
        break;         
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  return ( status );
}

/*********************************************************************
 * @fn      bloodPressure_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t bloodPressure_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
 
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch ( uuid )
  {
    case GATT_CLIENT_CHAR_CFG_UUID:
      if ( pAttr->handle == bloodPressureAttrTbl[BLOODPRESSURE_MEAS_CONFIG_POS].handle )
      {
        // BloodPressure Indications
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        if ( status == SUCCESS )
        {
          uint16 value = BUILD_UINT16( pValue[0], pValue[1] );

          (*bloodPressureServiceCB)( (value == GATT_CFG_NO_OPERATION) ? 
                                     BLOODPRESSURE_MEAS_NOTI_DISABLED :
                                     BLOODPRESSURE_MEAS_NOTI_ENABLED);
        }
      }
      else if ( pAttr->handle == bloodPressureAttrTbl[BLOODPRESSURE_IMEAS_CONFIG_POS].handle )
      {
        // BloodPressure Notifications
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS )
        {
          uint16 value = BUILD_UINT16( pValue[0], pValue[1] );

          (*bloodPressureServiceCB)( (value == GATT_CFG_NO_OPERATION)  ? 
                                     BLOODPRESSURE_IMEAS_NOTI_DISABLED :
                                     BLOODPRESSURE_IMEAS_NOTI_ENABLED );
        }
      }
      else
      {
        status = ATT_ERR_INVALID_HANDLE;
      }
      break;       
 
    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }
  
  return ( status );

}


/*********************************************************************
 * @fn          bloodPressure_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void bloodPressure_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, bloodPressureMeasConfig );
      GATTServApp_InitCharCfg( connHandle, bloodPressureIMeasConfig );
    }
  }
}


/*********************************************************************
*********************************************************************/
