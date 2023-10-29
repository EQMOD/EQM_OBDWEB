/**************************************************************************************************
  Filename:       accelerometer.c
  Revised:        $Date: 2011-11-21 15:26:10 -0800 (Mon, 21 Nov 2011) $
  Revision:       $Revision: 28439 $

  Description:    Accelerometer Profile


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

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
#include "gattservapp.h"

#include "accelerometer.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        19

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Accelerometer Service UUID
CONST uint8 accServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ACCEL_SERVICE_UUID), HI_UINT16(ACCEL_SERVICE_UUID)
};

// Accelerometer Enabler UUID
CONST uint8 accEnablerUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ACCEL_ENABLER_UUID), HI_UINT16(ACCEL_ENABLER_UUID)
};

// Accelerometer Range UUID
CONST uint8 rangeUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ACCEL_RANGE_UUID), HI_UINT16(ACCEL_RANGE_UUID)
};

// Accelerometer X-Axis Data UUID
CONST uint8 xUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ACCEL_X_UUID), HI_UINT16(ACCEL_X_UUID)
};

// Accelerometer Y-Axis Data UUID
CONST uint8 yUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ACCEL_Y_UUID), HI_UINT16(ACCEL_Y_UUID)
};

// Accelerometer Z-Axis Data UUID
CONST uint8 zUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ACCEL_Z_UUID), HI_UINT16(ACCEL_Z_UUID)
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
static accelCBs_t *accel_AppCBs = NULL;


/*********************************************************************
 * Profile Attributes - variables
 */

// Accelerometer Service attribute
static CONST gattAttrType_t accelService = { ATT_BT_UUID_SIZE, accServUUID };


// Enabler Characteristic Properties
static uint8 accelEnabledCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Enabler Characteristic Value
static uint8 accelEnabled = FALSE;

// Enabler Characteristic user description
static uint8 accelEnabledUserDesc[14] = "Accel Enable\0";


// Range Characteristic Properties
static uint8 accelRangeCharProps = GATT_PROP_READ;

// Range Characteristic Value
static uint16 accelRange = ACCEL_RANGE_2G;

// Range Characteristic user description
static uint8 accelRangeUserDesc[13] = "Accel Range\0";


// Accel Coordinate Characteristic Properties
static uint8 accelXCharProps = GATT_PROP_NOTIFY;
static uint8 accelYCharProps = GATT_PROP_NOTIFY;
static uint8 accelZCharProps = GATT_PROP_NOTIFY;

// Accel Coordinate Characteristics
static int8 accelXCoordinates = 0;
static int8 accelYCoordinates = 0;
static int8 accelZCoordinates = 0;

// Client Characteristic configuration. Each client has its own instantiation
// of the Client Characteristic Configuration. Reads of the Client Characteristic
// Configuration only shows the configuration for that client and writes only
// affect the configuration of that client.

// Accel Coordinate Characteristic Configs
static gattCharCfg_t accelXConfigCoordinates[GATT_MAX_NUM_CONN];
static gattCharCfg_t accelYConfigCoordinates[GATT_MAX_NUM_CONN];
static gattCharCfg_t accelZConfigCoordinates[GATT_MAX_NUM_CONN];

// Accel Coordinate Characteristic user descriptions
static uint8 accelXCharUserDesc[20] = "Accel X-Coordinate\0";
static uint8 accelYCharUserDesc[20] = "Accel Y-Coordinate\0";
static uint8 accelZCharUserDesc[20] = "Accel Z-Coordinate\0";


/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t accelAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Accelerometer Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                   /* permissions */
    0,                                  /* handle */
    (uint8 *)&accelService                /* pValue */
  },
  
    // Accel Enabler Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &accelEnabledCharProps 
    },

      // Accelerometer Enable Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, accEnablerUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &accelEnabled 
      },

      // Accelerometer Enable User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&accelEnabledUserDesc 
      },

    // Accel Range Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &accelRangeCharProps 
    },

      // Accelerometer Range Char Value
      { 
        { ATT_BT_UUID_SIZE, rangeUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&accelRange 
      },

      // Accelerometer Range User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        accelRangeUserDesc 
      },
      
    // X-Coordinate Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &accelXCharProps 
    },
  
      // X-Coordinate Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, xUUID },
        0, 
        0, 
        (uint8 *)&accelXCoordinates
      },
      
      // X-Coordinate Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)accelXConfigCoordinates 
      },

      // X-Coordinate Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        accelXCharUserDesc
      },  

   // Y-Coordinate Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &accelYCharProps 
    },
  
      // Y-Coordinate Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, yUUID },
        0, 
        0, 
        (uint8 *)&accelYCoordinates 
      },
      
      // Y-Coordinate Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)accelYConfigCoordinates
      },

      // Y-Coordinate Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        accelYCharUserDesc
      },

   // Z-Coordinate Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &accelZCharProps 
    },
  
      // Z-Coordinate Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, zUUID },
        0, 
        0, 
        (uint8 *)&accelZCoordinates
      },
      
      // Z-Coordinate Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)accelZConfigCoordinates 
      },

      // Z-Coordinate Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        accelZCharUserDesc
      },  

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 accel_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                               uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t accel_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset );

static void accel_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
//  Accelerometer Service Callbacks
CONST gattServiceCBs_t  accelCBs =
{
  accel_ReadAttrCB,  // Read callback function pointer
  accel_WriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Accel_AddService
 *
 * @brief   Initializes the Accelerometer service by
 *          registering GATT attributes with the GATT server. Only
 *          call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Accel_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, accelXConfigCoordinates );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, accelYConfigCoordinates );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, accelZConfigCoordinates );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( accel_HandleConnStatusCB );  

  if ( services & ACCEL_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( accelAttrTbl, 
                                          GATT_NUM_ATTRS( accelAttrTbl ),
                                          &accelCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      Accel_RegisterAppCBs
 *
 * @brief   Does the profile initialization.  Only call this function
 *          once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Accel_RegisterAppCBs( accelCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    accel_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      Accel_SetParameter
 *
 * @brief   Set an Accelerometer Profile parameter.
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
bStatus_t Accel_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case ACCEL_ENABLER:
      if ( len == sizeof ( uint8 ) ) 
      {
        accelEnabled = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case ACCEL_RANGE:
      if ( (len == sizeof ( uint16 )) && ((*((uint8*)value)) <= ACCEL_RANGE_8G) ) 
      {
        accelRange = *((uint16*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case ACCEL_X_ATTR:
      if ( len == sizeof ( int8 ) ) 
      {      
        accelXCoordinates = *((int8*)value);

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( accelXConfigCoordinates, (uint8 *)&accelXCoordinates,
                                    FALSE, accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case ACCEL_Y_ATTR:
      if ( len == sizeof ( int8 ) ) 
      {      
        accelYCoordinates = *((int8*)value);

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( accelYConfigCoordinates, (uint8 *)&accelYCoordinates,
                                    FALSE, accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case ACCEL_Z_ATTR:
      if ( len == sizeof ( int8 ) ) 
      {      
        accelZCoordinates = *((int8*)value);

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( accelZConfigCoordinates, (uint8 *)&accelZCoordinates,
                                    FALSE, accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      Accel_GetParameter
 *
 * @brief   Get an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Accel_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ACCEL_ENABLER:
      *((uint8*)value) = accelEnabled;
      break;
      
    case ACCEL_RANGE:
      *((uint16*)value) = accelRange;
      break;
      
    case ACCEL_X_ATTR:
      *((int8*)value) = accelXCoordinates;
      break;

    case ACCEL_Y_ATTR:
      *((int8*)value) = accelYCoordinates;
      break;

    case ACCEL_Z_ATTR:
      *((int8*)value) = accelZCoordinates;
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          accel_ReadAttr
 *
 * @brief       Read an attribute.
 *
 * @param       pAttr - pointer to attribute
 * @param       pLen - length of data to be read
 * @param       pValue - pointer to data to be read
 * @param       signature - whether to include Authentication Signature
 *
 * @return      Success or Failure
 */
static uint8 accel_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                               uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {    
    // 16-bit UUID
    uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads
      case ACCEL_RANGE_UUID:
        *pLen = 2;
        pValue[0] = LO_UINT16( *((uint16 *)pAttr->pValue) );
        pValue[1] = HI_UINT16( *((uint16 *)pAttr->pValue) );
        break;
  
      case ACCEL_ENABLER_UUID:
      case ACCEL_X_UUID:
      case ACCEL_Y_UUID:
      case ACCEL_Z_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
      
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }


  return ( status );
}

/*********************************************************************
 * @fn      accel_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle – connection message was received on
 * @param   pReq - pointer to request
 *
 * @return  Success or Failure
 */
static bStatus_t accel_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notify = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case ACCEL_ENABLER_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else if ( pValue[0] != FALSE && pValue[0] != TRUE )
            status = ATT_ERR_INVALID_VALUE;
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          
          *pCurValue = pValue[0];
          notify = ACCEL_ENABLER;        
        }
             
        break;
          
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;      
          
      default:
          // Should never get here!
          status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }  

  // If an attribute changed then callback function to notify application of change
  if ( (notify != 0xFF) && accel_AppCBs && accel_AppCBs->pfnAccelEnabler )
    accel_AppCBs->pfnAccelEnabler();  
  
  return ( status );
}

/*********************************************************************
 * @fn          accel_HandleConnStatusCB
 *
 * @brief       Accelerometer Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void accel_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, accelXConfigCoordinates );
      GATTServApp_InitCharCfg( connHandle, accelYConfigCoordinates );
      GATTServApp_InitCharCfg( connHandle, accelZConfigCoordinates );
    }
  }
}


/*********************************************************************
*********************************************************************/
