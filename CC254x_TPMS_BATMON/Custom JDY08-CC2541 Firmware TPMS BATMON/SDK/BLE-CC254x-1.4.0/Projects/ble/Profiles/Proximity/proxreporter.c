/**************************************************************************************************
  Filename:       proxreporter.c
  Revised:        $Date: 2013-08-15 15:28:40 -0700 (Thu, 15 Aug 2013) $
  Revision:       $Revision: 34986 $

  Description:    Proximity Profile - Reporter Role


  Copyright 2009 - 2013 Texas Instruments Incorporated. All rights reserved.

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

#include "proxreporter.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define PP_DEFAULT_TX_POWER               0
#define PP_DEFAULT_PATH_LOSS              0x7F

#define SERVAPP_NUM_ATTR_SUPPORTED        5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Link Loss Service UUID
CONST uint8 linkLossServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( LINK_LOSS_SERV_UUID ), HI_UINT16( LINK_LOSS_SERV_UUID )
};

// Immediate Alert Service UUID
CONST uint8 imAlertServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( IMMEDIATE_ALERT_SERV_UUID ), HI_UINT16( IMMEDIATE_ALERT_SERV_UUID )
};

// Tx Power Level Service UUID
CONST uint8 txPwrLevelServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( TX_PWR_LEVEL_SERV_UUID ), HI_UINT16( TX_PWR_LEVEL_SERV_UUID )
};

// Alert Level Attribute UUID
CONST uint8 alertLevelUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( ALERT_LEVEL_UUID ), HI_UINT16( ALERT_LEVEL_UUID )
};

// Tx Power Level Attribute UUID
CONST uint8 txPwrLevelUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16( TX_PWR_LEVEL_UUID ), HI_UINT16( TX_PWR_LEVEL_UUID )
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
static proxReporterCBs_t *pp_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Link Loss Service
static CONST gattAttrType_t linkLossService = { ATT_BT_UUID_SIZE, linkLossServUUID };

// Alert Level Characteristic Properties
static uint8 llAlertLevelCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Alert Level attribute
// This attribute enumerates the level of alert.
static uint8 llAlertLevel = PP_ALERT_LEVEL_NO;

// Immediate Alert Service
static CONST gattAttrType_t imAlertService = { ATT_BT_UUID_SIZE, imAlertServUUID };

// Alert Level Characteristic Properties
static uint8 imAlertLevelCharProps = GATT_PROP_WRITE_NO_RSP;

// Alert Level attribute
// This attribute enumerates the level of alert.
static uint8 imAlertLevel = PP_ALERT_LEVEL_NO;

// Tx Power Level Service
static CONST gattAttrType_t txPwrLevelService = { ATT_BT_UUID_SIZE, txPwrLevelServUUID };

// Tx Power Level Characteristic Properties
static uint8 txPwrLevelCharProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Tx Power Level attribute
// This attribute represents the range of transmit power levels in dBm with
// a range from -20 to +20 to a resolution of 1 dBm.
static int8 txPwrLevel = PP_DEFAULT_TX_POWER;

// Tx Power Level Characteristic Configs
static gattCharCfg_t txPwrLevelConfig[GATT_MAX_NUM_CONN];


/*********************************************************************
 * Profile Attributes - Table
 */
// Link Loss Service Atttribute Table
static gattAttribute_t linkLossAttrTbl[] = 
{
  // Link Loss service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&linkLossService                 /* pValue */
  },

    // Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &llAlertLevelCharProps 
    },

      // Alert Level attribute
      { 
        { ATT_BT_UUID_SIZE, alertLevelUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &llAlertLevel 
      },
};

// Immediate Alert Service Atttribute Table
static gattAttribute_t imAlertAttrTbl[] = 
{
  // Immediate Alert service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&imAlertService                  /* pValue */
  },

    // Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &imAlertLevelCharProps 
    },

      // Alert Level attribute
      { 
        { ATT_BT_UUID_SIZE, alertLevelUUID },
        GATT_PERMIT_WRITE, 
        0, 
        &imAlertLevel 
      },
};

// Tx Power Level Service Atttribute Table
static gattAttribute_t txPwrLevelAttrTbl[] = 
{
  // Tx Power Level service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&txPwrLevelService               /* pValue */
  },

    // Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &txPwrLevelCharProps 
    },

      // Tx Power Level attribute
      { 
        { ATT_BT_UUID_SIZE, txPwrLevelUUID },
        GATT_PERMIT_READ, 
        0, 
        (uint8 *)&txPwrLevel 
      },

      // Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)txPwrLevelConfig 
      },

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 proxReporter_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                    uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t proxReporter_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                         uint8 *pValue, uint8 len, uint16 offset );

static void proxReporter_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Prox Reporter Service Callbacks
CONST gattServiceCBs_t proxReporterCBs =
{
  proxReporter_ReadAttrCB,  // Read callback function pointer
  proxReporter_WriteAttrCB, // Write callback function pointer
  NULL                      // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      proxReporter_AddService
 *
 * @brief   Initializes the Proximity Reporter service by
 *          registering GATT attributes with the GATT server.
 *          Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return   Success or Failure
 */
bStatus_t ProxReporter_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  if ( services & PP_LINK_LOSS_SERVICE )
  {
    // Register Link Loss attribute list and CBs with GATT Server App  
    status = GATTServApp_RegisterService( linkLossAttrTbl, 
                                          GATT_NUM_ATTRS( linkLossAttrTbl ),
                                          &proxReporterCBs );
  }

  if ( ( status == SUCCESS ) && ( services & PP_IM_ALETR_SERVICE ) )
  {
    // Register Link Loss attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( imAlertAttrTbl, 
                                          GATT_NUM_ATTRS( imAlertAttrTbl ),
                                          &proxReporterCBs );
  }
  
  if ( ( status == SUCCESS )  && ( services & PP_TX_PWR_LEVEL_SERVICE ) )
  {
    
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, txPwrLevelConfig );

    // Register with Link DB to receive link status change callback
    VOID linkDB_Register( proxReporter_HandleConnStatusCB );  

    
    // Register Tx Power Level attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( txPwrLevelAttrTbl, 
                                          GATT_NUM_ATTRS( txPwrLevelAttrTbl ),
                                          &proxReporterCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      proxReporter_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t ProxReporter_RegisterAppCBs( proxReporterCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pp_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
   

/*********************************************************************
 * @fn      proxReporter_SetParameter
 *
 * @brief   Set a Proximity Reporter parameter.
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
bStatus_t ProxReporter_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {

    case PP_LINK_LOSS_ALERT_LEVEL:
      if ( (len == sizeof ( uint8 )) && ((*((uint8*)value) <= PP_ALERT_LEVEL_HIGH)) ) 
      {
        llAlertLevel = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case PP_IM_ALERT_LEVEL:
      if ( len == sizeof ( uint8 ) ) 
      {
        imAlertLevel = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case PP_TX_POWER_LEVEL:
      if ( len == sizeof ( int8 ) ) 
      {
        txPwrLevel = *((int8*)value);

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( txPwrLevelConfig, (uint8 *)&txPwrLevel, FALSE, 
                                    txPwrLevelAttrTbl, GATT_NUM_ATTRS( txPwrLevelAttrTbl ),
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
 * @fn      ProxReporter_GetParameter
 *
 * @brief   Get a Proximity Reporter parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ProxReporter_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case PP_LINK_LOSS_ALERT_LEVEL:
      *((uint8*)value) = llAlertLevel;
      break;
      
    case PP_IM_ALERT_LEVEL:
      *((uint8*)value) = imAlertLevel;
      break;
      
    case PP_TX_POWER_LEVEL:
      *((int8*)value) = txPwrLevel;
      break;
 
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          proxReporter_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       
 *
 * @return      Success or Failure
 */
static uint8 proxReporter_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
      case ALERT_LEVEL_UUID:
      case TX_PWR_LEVEL_UUID:
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
    //128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      proxReporter_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle – connection message was received on
 * @param   pReq - pointer to request
 *
 * @return  Success or Failure
 */
static bStatus_t proxReporter_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notify = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  { 
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    { 
      case ALERT_LEVEL_UUID:
        // Validate the value
        // Make sure it's not a blob operation
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else
          {
            if ( pValue[0] > PP_ALERT_LEVEL_HIGH )
              status = ATT_ERR_INVALID_VALUE;
          }
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
          if( pAttr->pValue == &llAlertLevel )
            notify = PP_LINK_LOSS_ALERT_LEVEL;        
          else
            notify = PP_IM_ALERT_LEVEL;                      
        }
        
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }    
  
  // If an attribute changed then callback function to notify application of change
  if ( (notify != 0xFF) && pp_AppCBs && pp_AppCBs->pfnAttrChange )
    pp_AppCBs->pfnAttrChange( notify );

  return ( status );
}

/*********************************************************************
 * @fn          proxReporter_HandleConnStatusCB
 *
 * @brief       Tx Power Level Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void proxReporter_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, txPwrLevelConfig );
    }
  }
}





/*********************************************************************
*********************************************************************/
