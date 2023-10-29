/**************************************************************************************************
  Filename:       glucservice.c
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the Glucose sample service
                  for use with the Glucose   sample application.

 Copyright 2011-2013 Texas Instruments Incorporated. All rights reserved.

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
#include "glucservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of glucose measurement value in attribute array
#define GLUCOSE_MEAS_VALUE_POS         2
#define GLUCOSE_MEAS_CONFIG_POS        3
#define GLUCOSE_CONTEXT_VALUE_POS      5
#define GLUCOSE_CONTEXT_CONFIG_POS     6
#define GLUCOSE_CTL_PNT_VALUE_POS      10
#define GLUCOSE_CTL_PNT_CONFIG_POS     11

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Glucose service
CONST uint8 glucoseServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_SERV_UUID), HI_UINT16(GLUCOSE_SERV_UUID)
};

// Glucose characteristic
CONST uint8 glucoseMeasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_MEAS_UUID), HI_UINT16(GLUCOSE_MEAS_UUID)
};

// Glucose Measurement Context
CONST uint8 glucoseContextUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_CONTEXT_UUID), HI_UINT16(GLUCOSE_CONTEXT_UUID)
};

// Glucose Feature
CONST uint8 glucoseFeatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_FEATURE_UUID), HI_UINT16(GLUCOSE_FEATURE_UUID)
};

// Record Control Point
CONST uint8 recordControlPointUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RECORD_CTRL_PT_UUID), HI_UINT16(RECORD_CTRL_PT_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/* TRUE if record transfer in progress */
extern bool glucoseSendAllRecords;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static glucoseServiceCB_t glucoseServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Glucose Service attribute
static CONST gattAttrType_t glucoseService = { ATT_BT_UUID_SIZE, glucoseServUUID };

// Glucose Characteristic
static uint8 glucoseProps = GATT_PROP_NOTIFY;
static gattCharCfg_t glucoseMeasConfig[GATT_MAX_NUM_CONN];
static uint8 glucoseMeas = 0;

// Measurement Context
static uint8  glucoseContextProps = GATT_PROP_NOTIFY;
static uint8  glucoseContext=0;
static gattCharCfg_t glucoseContextConfig[GATT_MAX_NUM_CONN];

// Glucose Feature
static uint8 glucoseFeatureProps = GATT_PROP_READ;
static uint16 glucoseFeature = GLUCOSE_FEAT_ALL;

// Glucose Control
static uint8  glucoseControlProps = GATT_PROP_INDICATE | GATT_PROP_WRITE;
static uint8  glucoseControl=0;
static gattCharCfg_t glucoseControlConfig[GATT_MAX_NUM_CONN];

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t glucoseAttrTbl[] =
{
  // Glucose Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&glucoseService                /* pValue */
  },

    // 1. Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseProps
    },

    // 2. Characteristic Value
    {
      { ATT_BT_UUID_SIZE, glucoseMeasUUID },
      0, //return READ_NOT_PERMITTED
      0,
      &glucoseMeas
    },

    // 3.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&glucoseMeasConfig
    },

    //////////////////////////////////////////////
    // MEASUREMENT CONTEXT
    //////////////////////////////////////////////

    // 4.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseContextProps
    },

    // 5.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, glucoseContextUUID },
      0, //return READ_NOT_PERMITTED
      0,
      &glucoseContext
    },

    // 6.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&glucoseContextConfig
    },

    //////////////////////////////////////////////
    // GLUCOSE FEATURE
    //////////////////////////////////////////////

    // 7.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseFeatureProps
    },

    // 8.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, glucoseFeatureUUID },
      GATT_PERMIT_ENCRYPT_READ,
      0,
      (uint8 *) &glucoseFeature
    },

    //////////////////////////////////////////////
    // GLUCOSE CONTROL POINT
    //////////////////////////////////////////////

    // 9.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &glucoseControlProps
    },

    // 10.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, recordControlPointUUID },
      GATT_PERMIT_AUTHEN_WRITE,
      0,
      &glucoseControl
    },

    // 11.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&glucoseControlConfig
    }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 glucose_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t glucose_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void glucose_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Service Callbacks
CONST gattServiceCBs_t  glucoseCBs =
{
  glucose_ReadAttrCB,   // Read callback function pointer
  glucose_WriteAttrCB,  // Write callback function pointer
  NULL                  // Authorization callback function pointer
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Glucose_AddService
 *
 * @brief   Initializes the Glucose   service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Glucose_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, glucoseMeasConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, glucoseContextConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, glucoseControlConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( glucose_HandleConnStatusCB );

  if ( services & GLUCOSE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( glucoseAttrTbl, GATT_NUM_ATTRS( glucoseAttrTbl ),
                                          &glucoseCBs );
  }
  return ( status );
}

/*********************************************************************
 * @fn      Glucose_Register
 *
 * @brief   Register a callback function with the Glucose Service.
 *
 * @param   pfnServiceCB - Callback function.
 *          pfnCtlPntCB - Callback for control point
 *
 * @return  None.
 */
extern void Glucose_Register( glucoseServiceCB_t pfnServiceCB)
{
  glucoseServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Glucose_SetParameter
 *
 * @brief   Set a glucose parameter.
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
bStatus_t Glucose_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case GLUCOSE_FEATURE_PARAM:
      glucoseFeature = *((uint16*)value);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      Glucose_GetParameter
 *
 * @brief   Get a Glucose parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Glucose_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case GLUCOSE_FEATURE_PARAM:
      *((uint16*)value) = glucoseFeature;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          Glucose_MeasSend
 *
 * @brief       Send a glucose measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t Glucose_MeasSend( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, glucoseMeasConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = glucoseAttrTbl[GLUCOSE_MEAS_VALUE_POS].handle;

    // Send the Indication
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleNotReady;
}


/*********************************************************************
 * @fn          Glucose_ContextSend
 *
 * @brief       Send a glucose measurement context.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t Glucose_ContextSend( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, glucoseContextConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = glucoseAttrTbl[GLUCOSE_CONTEXT_VALUE_POS].handle;

    // Send the Indication
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleNotReady;
}

/*********************************************************************
 * @fn          Glucose_CtlPntIndicate
 *
 * @brief       Send an indication containing a control point
 *              message.
 *
 * @param       connHandle - connection handle
 * @param       pInd - pointer to indication structure
 *
 * @return      Success or Failure
 */
bStatus_t Glucose_CtlPntIndicate( uint16 connHandle, attHandleValueInd_t *pInd, uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, glucoseControlConfig );

  // If indications enabled
  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    // Set the handle
    pInd->handle = glucoseAttrTbl[GLUCOSE_CTL_PNT_VALUE_POS].handle;

    // Send the Indication
    return GATT_Indication( connHandle, pInd, FALSE, taskId );
  }

  return bleNotReady;
}

/*********************************************************************
 * @fn          glucose_ReadAttrCB
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
static uint8 glucose_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

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
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads

      case GLUCOSE_FEATURE_UUID:
        *pLen = 2;
        pValue[0] = LO_UINT16( glucoseFeature );
        pValue[1] = HI_UINT16( glucoseFeature );
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
 * @fn      glucose_WriteAttrCB
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
static bStatus_t glucose_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch ( uuid )
  {

  case  GATT_CLIENT_CHAR_CFG_UUID:
      // Glucose Notifications
      if ((pAttr->handle == glucoseAttrTbl[GLUCOSE_MEAS_CONFIG_POS].handle ||
           pAttr->handle == glucoseAttrTbl[GLUCOSE_CONTEXT_CONFIG_POS].handle))
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS )
        {
          uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

          if(pAttr->handle == glucoseAttrTbl[GLUCOSE_MEAS_CONFIG_POS].handle)
          {
            (*glucoseServiceCB)((charCfg == 0) ? GLUCOSE_MEAS_NTF_DISABLED :
                                                 GLUCOSE_MEAS_NTF_ENABLED, NULL, NULL);
          }
          else
          {
            (*glucoseServiceCB)((charCfg == 0) ? GLUCOSE_CONTEXT_NTF_DISABLED :
                                                 GLUCOSE_CONTEXT_NTF_ENABLED, NULL, NULL);
          }
        }
      }
      // Glucose Indications
      else if ( pAttr->handle == glucoseAttrTbl[GLUCOSE_CTL_PNT_CONFIG_POS].handle )
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        if ( status == SUCCESS )
        {
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

            (*glucoseServiceCB)((charCfg == 0) ? GLUCOSE_CTL_PNT_IND_DISABLED :
                                                 GLUCOSE_CTL_PNT_IND_ENABLED, NULL, NULL);
        }
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    break;

    case  RECORD_CTRL_PT_UUID:
      if(len >= GLUCOSE_CTL_PNT_MIN_SIZE  && len <= GLUCOSE_CTL_PNT_MAX_SIZE)
      {
        uint8 opcode = pValue[0];

        // if transfer in progress
        if (opcode != CTL_PNT_OP_ABORT && glucoseSendAllRecords)
        {
          status = GLUCOSE_ERR_IN_PROGRESS;
        }
        // if CCC not configured for glucose measurement
        else if ( opcode == CTL_PNT_OP_REQ &&
                 !( GATTServApp_ReadCharCfg( connHandle, glucoseMeasConfig ) & GATT_CLIENT_CFG_NOTIFY ) )
        {
          status = GLUCOSE_ERR_CCC_CONFIG;
        }
        else
        {
          (*glucoseServiceCB)(GLUCOSE_CTL_PNT_CMD, pValue, len);
        }
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
    break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

/*********************************************************************
 * @fn          glucose_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void glucose_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, glucoseMeasConfig );
      GATTServApp_InitCharCfg( connHandle, glucoseContextConfig );
      GATTServApp_InitCharCfg( connHandle, glucoseControlConfig );
    }
  }
}

/*********************************************************************
*********************************************************************/
