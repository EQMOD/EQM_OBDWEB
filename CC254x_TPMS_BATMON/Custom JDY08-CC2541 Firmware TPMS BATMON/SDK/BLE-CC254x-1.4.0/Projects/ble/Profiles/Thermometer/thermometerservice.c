/**************************************************************************************************
  Filename:       thermometerService.c
  Revised:        $Date: 2013-08-20 14:13:11 -0700 (Tue, 20 Aug 2013) $
  Revision:       $Revision: 35041 $

  Description:    This file contains the Simple BLE Peripheral sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

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
#include "thermometerservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */




/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Thermometer service
CONST uint8 thermometerServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(THERMOMETER_SERV_UUID), HI_UINT16(THERMOMETER_SERV_UUID)
};

// Thermometer temperature characteristic
CONST uint8 thermometerTempUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(TEMP_MEAS_UUID), HI_UINT16(TEMP_MEAS_UUID)
};

// Thermometer Site
CONST uint8 thermometerTypeUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(TEMP_TYPE_UUID), HI_UINT16(TEMP_TYPE_UUID)
};

// Thermometer Immediate Measurement
CONST uint8 thermometerImeasUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(IMEDIATE_TEMP_UUID), HI_UINT16(IMEDIATE_TEMP_UUID)
};

// Thermometer Measurement Interval
CONST uint8 thermometerIntervalUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MEAS_INTERVAL_UUID), HI_UINT16(MEAS_INTERVAL_UUID)
};

// Thermometer Test Commands
CONST uint8 thermometerIRangeUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GATT_VALID_RANGE_UUID), HI_UINT16(GATT_VALID_RANGE_UUID)
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

static thermometerServiceCB_t thermometerServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Thermometer Service attribute
static CONST gattAttrType_t thermometerService = { ATT_BT_UUID_SIZE, thermometerServUUID };

// Client Characteristic configuration. Each client has its own instantiation
// of the Client Characteristic Configuration. Reads of the Client Characteristic
// Configuration only shows the configuration for that client and writes only
// affect the configuration of that client.

// Thermometer Temperature Characteristic
static uint8 thermometerTempProps = GATT_PROP_INDICATE;
static uint8 thermometerTemp = 0;
static gattCharCfg_t thermometerTempConfig[GATT_MAX_NUM_CONN];

// Site
static uint8 thermometerTypeProps = GATT_PROP_READ;
static uint8 thermometerType  = 0;

// Intermediate Measurement
static uint8  thermometerImeasProps = GATT_PROP_NOTIFY;
static uint8  thermometerImeas=0;
static gattCharCfg_t thermometerIMeasConfig[GATT_MAX_NUM_CONN];

// Measurement Interval
static uint8  thermometerIntervalProps = GATT_PROP_INDICATE|GATT_PROP_READ|GATT_PROP_WRITE;
static uint8  thermometerInterval=30;  //default
static gattCharCfg_t thermometerIntervalConfig[GATT_MAX_NUM_CONN];

// Measurement Interval Range
static thermometerIRange_t  thermometerIRange = {1,60};

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t thermometerAttrTbl[] = 
{
  // Thermometer Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&thermometerService              /* pValue */
  },

    // TEMPERATURE MEASUREMENT
    
    // 1. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &thermometerTempProps 
    },

    // 2. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, thermometerTempUUID },
      0, 
      0, 
      &thermometerTemp 
    },

    // 3. Characteristic Configuration 
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *)&thermometerTempConfig
    },
 
    // MEASUREMENT TYPE
   
    // 4. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &thermometerTypeProps 
    },

    // 5. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, thermometerTypeUUID },
      GATT_PERMIT_READ, 
      0, 
      &thermometerType 
    },

    // IMMEDIATE MEASUREMENT
    
    // 6. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &thermometerImeasProps 
    },

    // 7. Characteristic Value 
    { 
      { ATT_BT_UUID_SIZE, thermometerImeasUUID },
      0, 
      0, 
      &thermometerImeas 
    },

    // 8. Characteristic Configuration
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *)&thermometerIMeasConfig
    },

    // INTERVAL
    
    // 9. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &thermometerIntervalProps 
    },

    // 10. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, thermometerIntervalUUID },
      GATT_PERMIT_READ | GATT_PERMIT_AUTHEN_WRITE,
      0, 
      &thermometerInterval 
    },
    
    // 11. Characteristic Configuration
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *)&thermometerIntervalConfig
    }, 
    
    // 12. Interval Range
    { 
      { ATT_BT_UUID_SIZE, thermometerIRangeUUID },
      GATT_PERMIT_READ,
      0, 
      (uint8 *)&thermometerIRange 
    },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 thermometer_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t thermometer_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void thermometer_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Thermometer Service Callbacks
CONST gattServiceCBs_t thermometerCBs =
{
  thermometer_ReadAttrCB,  // Read callback function pointer
  thermometer_WriteAttrCB, // Write callback function pointer
  NULL                     // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Thermometer_AddService
 *
 * @brief   Initializes the Thermometer   service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Thermometer_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, thermometerTempConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, thermometerIMeasConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, thermometerIntervalConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( thermometer_HandleConnStatusCB );  
  
  if ( services & THERMOMETER_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( thermometerAttrTbl, 
                                          GATT_NUM_ATTRS( thermometerAttrTbl ),
                                          &thermometerCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      Thermometer_Register
 *
 * @brief   Register a callback function with the Thermometer Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Thermometer_Register( thermometerServiceCB_t pfnServiceCB )
{
  thermometerServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Thermometer_SetParameter
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
bStatus_t Thermometer_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
 
    case THERMOMETER_TYPE:
      thermometerType = *((uint8*)value);
      break;
      
    case THERMOMETER_INTERVAL:
      thermometerInterval = *((uint8*)value);
      break;      
 
    case THERMOMETER_TEMP_CHAR_CFG:      
      // Need connection handle
      //thermometerTempConfig.value = *((uint8*)value);
      break;
      
    case THERMOMETER_IMEAS_CHAR_CFG:      
      // Need connection handle
      //thermometerIMeasConfig.value = *((uint8*)value);
      break; 

    case THERMOMETER_INTERVAL_CHAR_CFG:  
      // Need connection handle    
      //thermometerIntervalConfig.value = *((uint8*)value);
      break;       
      
    case THERMOMETER_IRANGE:      
      thermometerIRange = *((thermometerIRange_t*)value);
      break;       
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      Thermometer_GetParameter
 *
 * @brief   Get a Thermometer   parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Thermometer_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case THERMOMETER_TYPE:
      *((uint8*)value) = thermometerType;
      break;

    case THERMOMETER_INTERVAL:
      *((uint8*)value) = thermometerInterval;
      break;
    
    case THERMOMETER_IRANGE:
      *((thermometerIRange_t*)value) = thermometerIRange;
      break;
      
  case THERMOMETER_TEMP_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = thermometerTempConfig.value;
      break;
        
  case THERMOMETER_IMEAS_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = thermometerIMeasConfig.value;
      break;        
        
  case THERMOMETER_INTERVAL_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = thermometerIntervalConfig.value;
      break;        
    
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          Thermometer_TempIndicate
 *
 * @brief       Send a indication containing a thermometer
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t Thermometer_TempIndicate( uint16 connHandle, attHandleValueInd_t *pNoti,uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, thermometerTempConfig );

  // If indications enabled
  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    // Set the handle (uses stored relative handle to lookup actual handle)
    pNoti->handle = thermometerAttrTbl[pNoti->handle].handle;
  
    // Send the Indication
    return GATT_Indication( connHandle, pNoti, FALSE, taskId );
  }

  return bleIncorrectMode;
}

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
bStatus_t Thermometer_IntervalIndicate( uint16 connHandle, attHandleValueInd_t *pNoti,uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, thermometerIntervalConfig );

  // If indications enabled
  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    // Set the handle (uses stored relative handle to lookup actual handle)
    pNoti->handle = thermometerAttrTbl[pNoti->handle].handle;
  
    // Send the Indication
    return GATT_Indication( connHandle, pNoti, FALSE, taskId );
  }

  return bleIncorrectMode;
}

/*********************************************************************
 * @fn          Thermometer_IMeasNotify
 *
 * @brief       Send a notification containing a thermometer
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t Thermometer_IMeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti)
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, thermometerIMeasConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = thermometerAttrTbl[THERMOMETER_IMEAS_VALUE_POS].handle;
  
    // Send the Notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}

/*********************************************************************
 * @fn          thermometer_ReadAttrCB
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
static uint8 thermometer_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
      case TEMP_TYPE_UUID:
        *pLen = THERMOMETER_TYPE_LEN;
        VOID osal_memcpy( pValue, &thermometerType, THERMOMETER_TYPE_LEN ) ;
        break;
        
      case MEAS_INTERVAL_UUID:
        *pLen = THERMOMETER_INTERVAL_LEN;
        VOID osal_memcpy( pValue, &thermometerInterval, THERMOMETER_INTERVAL_LEN ) ;
        break;

      case GATT_VALID_RANGE_UUID:
        *pLen = THERMOMETER_IRANGE_LEN;
         VOID osal_memcpy( pValue, &thermometerIRange, THERMOMETER_IRANGE_LEN ) ;
        break;        
        
      default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      thermometer_WriteAttrCB
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
static bStatus_t thermometer_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                          uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch ( uuid )
  {
    case  GATT_CLIENT_CHAR_CFG_UUID:
      // Validate/Write Temperature measurement setting
      if ( pAttr->handle == thermometerAttrTbl[THERMOMETER_TEMP_CHAR_CONFIG_POS].handle )
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        if ( status == SUCCESS )
        {
          uint16 value = BUILD_UINT16( pValue[0], pValue[1] );      
        
          (*thermometerServiceCB)( (value == GATT_CFG_NO_OPERATION) ? 
                                   THERMOMETER_TEMP_IND_DISABLED :
                                   THERMOMETER_TEMP_IND_ENABLED );
        }
      }
      // Validate/Write Intermediate measurement setting
      else if ( pAttr->handle == thermometerAttrTbl[THERMOMETER_IMEAS_CHAR_CONFIG_POS].handle )
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS )
        {
          uint16 value = BUILD_UINT16( pValue[0], pValue[1] );
          
          // Notify application 
          (*thermometerServiceCB)( (value == GATT_CFG_NO_OPERATION) ? 
                                   THERMOMETER_IMEAS_NOTI_DISABLED :
                                   THERMOMETER_IMEAS_NOTI_ENABLED);
        }
      }
      // Validate/Write Interval Client Char Config
      else if ( pAttr->handle == thermometerAttrTbl[THERMOMETER_INTERVAL_CHAR_CONFIG_POS].handle )
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        if ( status == SUCCESS )
        {
          uint16 value = BUILD_UINT16( pValue[0], pValue[1] );
           
          // Notify application 
          (*thermometerServiceCB)( (value == GATT_CFG_NO_OPERATION) ? 
                                   THERMOMETER_INTERVAL_IND_DISABLED :
                                   THERMOMETER_INTERVAL_IND_ENABLED);
        }         
        else
        {
          status = ATT_ERR_INVALID_HANDLE;
        }
      }  
      else
      {
          status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      break;
  
    case MEAS_INTERVAL_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != THERMOMETER_INTERVAL_LEN )
          status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      
      //validate range
      if ((*pValue >= thermometerIRange.high) | ((*pValue <= thermometerIRange.low) & (*pValue != 0)))
      {
        status = ATT_ERR_INVALID_VALUE;
      }
      
      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        // *pCurValue = *pValue; 
        VOID osal_memcpy( pCurValue, pValue, THERMOMETER_INTERVAL_LEN ) ;
        
        //notify application of write
        (*thermometerServiceCB)(THERMOMETER_INTERVAL_SET);
        
      }
      break;
    
    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }
  
  return ( status );
}


/*********************************************************************
 * @fn          thermometer_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void thermometer_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, thermometerTempConfig );
      GATTServApp_InitCharCfg( connHandle, thermometerIMeasConfig );
      GATTServApp_InitCharCfg( connHandle, thermometerIntervalConfig );
    }
  }
}


/*********************************************************************
*********************************************************************/
