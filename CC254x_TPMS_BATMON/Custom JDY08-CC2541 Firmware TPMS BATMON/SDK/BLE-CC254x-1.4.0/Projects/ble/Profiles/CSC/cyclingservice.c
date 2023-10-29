/**************************************************************************************************
  Filename:       cyclingservice.c
  Revised:        $Date: 2013-08-15 15:28:40 -0700 (Thu, 15 Aug 2013) $
  Revision:       $Revision: 34986 $

  Description:    This file contains the Cycling Speed and Cadence (CSC) service
                  for use with the CyclingApp sample application.

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
#include "gatt_profile_uuid.h"
#include "cyclingservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
// Cycling Service Task Events
#define CSC_CMD_IND_SEND_EVT   0x0001

#define CSC_MEAS_VALUE_POS     2
#define CSC_MEAS_CFG_POS       3
#define CSC_COMMAND_VALUE_POS  9
#define CSC_COMMAND_CFG_POS    10
#define COMMAND_IND_LENGTH     2

/*********************************************************************
 * GLOBAL VARIABLES
 */

// CSC service
CONST uint8 cyclingServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CSC_SERV_UUID), HI_UINT16(CSC_SERV_UUID)
};

// CSC measurement characteristic
CONST uint8 cyclingMeasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CSC_MEAS_UUID), HI_UINT16(CSC_MEAS_UUID)
};

// CSC feature characteristic
CONST uint8 cyclingFeatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CSC_FEATURE_UUID), HI_UINT16(CSC_FEATURE_UUID)
};

// CSC sensor location characteristic
CONST uint8 cyclingSensLocUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SENSOR_LOC_UUID), HI_UINT16(SENSOR_LOC_UUID)
};

// CSC command characteristic
CONST uint8 cyclingCommandUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SC_CTRL_PT_UUID), HI_UINT16(SC_CTRL_PT_UUID)
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

static cyclingServiceCB_t cyclingServiceCB = NULL;

static uint8 supportedSensors = 0;
static bool scOpInProgress = FALSE;

// Variables used in CSC command processing
static uint16 connectionHandle;
static attHandleValueInd_t cscCmdInd;

/*********************************************************************
 * Profile Attributes - variables
 */

// TaskID
uint8 cyclingService_TaskID = 0;

// CSC Service attribute
static CONST gattAttrType_t cyclingService = { ATT_BT_UUID_SIZE, cyclingServUUID };

// Available sensor locations
static uint8 supportedSensorLocations[CSC_MAX_SENSOR_LOCS];

// Cycling Measurement Characteristic
// Note: characteristic value is not stored here
static uint8 cyclingMeasProps = GATT_PROP_NOTIFY;
static uint8 cyclingMeas = 0;
static gattCharCfg_t cyclingMeasClientCharCfg[GATT_MAX_NUM_CONN];

// Feature Characteristic
static uint8 cyclingFeatureProps = GATT_PROP_READ;
static uint16 cyclingFeatures = CSC_NO_SUPPORT;


// Sensor Location Characteristic
static uint8 cyclingSensLocProps = GATT_PROP_READ;
static uint8 cyclingSensLoc = CSC_SENSOR_LOC_TOP_OF_SHOE;

// Command Characteristic
static uint8 cyclingCommandProps = GATT_PROP_WRITE | GATT_PROP_INDICATE;
static uint8 cyclingCommand = 0;
static gattCharCfg_t cyclingCommandClientCharCfg[GATT_MAX_NUM_CONN];

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t cyclingAttrTbl[] =
{
  // CSC Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&cyclingService                  /* pValue */
  },

    // CSC Measurement Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &cyclingMeasProps
    },

      // Measurement Value
      {
        { ATT_BT_UUID_SIZE, cyclingMeasUUID },
        0,
        0,
        &cyclingMeas
      },

      // Measurement Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &cyclingMeasClientCharCfg
      },

    // CSC Feature Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &cyclingFeatureProps
    },

      // Feature Value
      {
        { ATT_BT_UUID_SIZE, cyclingFeatureUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) &cyclingFeatures
      },

    // CSC Sensor Location Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &cyclingSensLocProps
    },

      // Sensor Location Value
      {
        { ATT_BT_UUID_SIZE, cyclingSensLocUUID },
        GATT_PERMIT_READ,
        0,
        &cyclingSensLoc
      },

    // CSC Command Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &cyclingCommandProps
    },

      // Command Value
      {
        { ATT_BT_UUID_SIZE, cyclingCommandUUID },
        GATT_PERMIT_WRITE,
        0,
        &cyclingCommand
      },

      // Command Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &cyclingCommandClientCharCfg
      }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 cycling_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t cycling_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );
static void cycling_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void cycling_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static bool cycling_SensorLocSupported( uint8 sensorLoc );
static void cycling_ProcessCSCCmd( uint16 attrHandle, uint8 *pValue, uint8 len );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// CSC Service Callbacks
CONST gattServiceCBs_t cyclingCBs =
{
  cycling_ReadAttrCB,  // Read callback function pointer
  cycling_WriteAttrCB, // Write callback function pointer
  NULL                 // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
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
void CyclingService_Init( uint8 task_id )
{
  // Only purpose is to obtain task ID
  cyclingService_TaskID = task_id;
}

/*********************************************************************
 * @fn      CyclingService_ProcessEvent
 *
 * @brief   process incoming event.
 *
 * @param   task_id - OSAL task id.
 *
 * @param   events - event bit(s) set for the task(s)
 *
 * @return  none
 */
uint16 CyclingService_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id;

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( cyclingService_TaskID )) != NULL )
    {
      cycling_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & CSC_CMD_IND_SEND_EVT )
  {
    GATT_Indication( connectionHandle, &cscCmdInd, FALSE, cyclingService_TaskID );

    // Set Control Point Cfg done
    scOpInProgress = FALSE;

    return ( events ^ CSC_CMD_IND_SEND_EVT );
  }

  return 0;
}

/*********************************************************************
 * @fn      cycling_ProcessOSALMsg
 *
 * @brief   process incoming OSAL msg.
 *
 * @param   pMsg- pointer to messag to be read.
 *
 * @return  none
 */
void cycling_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case GATT_MSG_EVENT:
      cycling_ProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;

    default: // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      cycling_ProcessGATTMsg
 *
 * @brief   process incoming GATT msg.
 *
 * @param   pMsg- pointer to messag to be read.
 *
 * @return  none
 */
void cycling_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( pMsg->method == ATT_HANDLE_VALUE_CFM )
  {
    // Indication receipt was confirmed by the client.
    // This is a placeholder for future.
  }
}

/*********************************************************************
 * @fn      cycling_SensorLocSupported
 *
 * @brief   check to see if sensor location is supported
 *
 * @param   sensorLoc - location to check for
 *
 * @return  TRUE if supported, FALSE otherwise
 */
static bool cycling_SensorLocSupported( uint8 sensorLoc )
{
  uint8 i;
  for (i = 0; i <= supportedSensors; i++)
  {
    if (supportedSensorLocations[i] == sensorLoc)
    {
      return TRUE;
    }
  }
  return FALSE;
}

/*********************************************************************
 * @fn      Cycling_AddService
 *
 * @brief   Initializes the CSC service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Cycling_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, cyclingMeasClientCharCfg );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, cyclingCommandClientCharCfg);

  if ( services & CYCLING_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( cyclingAttrTbl,
                                          GATT_NUM_ATTRS( cyclingAttrTbl ),
                                          &cyclingCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      Cycling_Register
 *
 * @brief   Register a callback function with the CSC Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
void Cycling_Register( cyclingServiceCB_t pfnServiceCB )
{
  cyclingServiceCB = pfnServiceCB;
}


/*********************************************************************
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
 *
 * @return  bStatus_t
 */
bStatus_t Cycling_SetParameter( uint8 param, uint8 len, void *pValue )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case CSC_SENS_LOC:
      cyclingSensLoc = *((uint8*)pValue);
    break;

    case CSC_FEATURE:
      cyclingFeatures = *((uint8*)pValue);
    break;

    case CSC_AVAIL_SENS_LOCS:
      if (supportedSensors  < CSC_MAX_SENSOR_LOCS)
      {
        supportedSensorLocations[supportedSensors++] = *((uint8*)pValue);
      }
    break;

    default:
      ret = INVALIDPARAMETER;
    break;
  }

  return ( ret );
}

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
bStatus_t Cycling_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case CSC_FEATURE:
      *((uint8*)value) = cyclingFeatures;

    case CSC_SENS_LOC:
      *((uint8*)value) = cyclingSensLoc;
      break;

    case CSC_COMMAND:
      *((uint8*)value) = cyclingCommand;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

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
bStatus_t Cycling_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, cyclingMeasClientCharCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = cyclingAttrTbl[CSC_MEAS_VALUE_POS].handle;

    // Send the notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}

 /*********************************************************************
 * @fn      cycling_ProcessCSCCmd
 *
 * @brief   process an incoming CSC command.
 *
 * @param   attrHandle - attribute handle
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 *
 * @return  none
 */
static void cycling_ProcessCSCCmd( uint16 attrHandle, uint8 *pValue, uint8 len )
{
  uint8 cscStatus = CSC_SUCCESS;

  // Set Control Point Cfg in progress
  scOpInProgress = TRUE;

  // Set indication info to be sent out
  cscCmdInd.handle = attrHandle;

  cscCmdInd.len = 3;
  cscCmdInd.value[0] = CSC_COMMAND_RSP;
  cscCmdInd.value[1] = pValue[0];

  switch ( pValue[0] )
  {
    case CSC_SET_CUMM_VAL:
      // If wheel revolutions is a feature
      if ( ( len <= 5 ) && ( cyclingFeatures & CSC_WHEEL_REV_SUPP ) )
      {
        uint32 cummWheelRevolutions;

        // full 32 bits were specified.
        if (( len - 1 ) == 4)
        {
          cummWheelRevolutions = BUILD_UINT32( pValue[1], pValue[2], pValue[3], pValue[4]);
        }
        else
        {
          cummWheelRevolutions = 0;

          // In case only lower bits were specified and upper bits remain zero.
          for( int i = 0; i < (len - 1); ++i )
          {
            cummWheelRevolutions += pValue[i + 1] << (i*8);
          }
        }

        // Notify app
        if ( cyclingServiceCB != NULL )
        {
          (*cyclingServiceCB)( CSC_CMD_SET_CUMM_VAL, &cummWheelRevolutions );
        }
      }
      else // characteristic not supported.
      {
        cscStatus = CSC_INVALID_PARAMETER;
      }
      break;

    case CSC_UPDATE_SENS_LOC:
      // If multiple sensor locations is supported and that this is a valid location.
      if ( ( len == 2 )                              &&
           ( cyclingFeatures & CSC_MULTI_SENS_SUPP ) &&
           ( cycling_SensorLocSupported( pValue[1] ) == TRUE ) )
      {
        // Update sensor location
        cyclingSensLoc = pValue[1];

        // Notify app
        if ( cyclingServiceCB != NULL )
        {
          (*cyclingServiceCB)( CSC_CMD_UPDATE_SENS_LOC, NULL );
        }
      }
      else // characteristic not supported.
      {
        cscStatus = CSC_INVALID_PARAMETER;
      }
      break;

    case CSC_REQ_SUPP_SENS_LOC:
      // If multiple sensor locations are supported and list requested
      if ( ( len == 1 ) && ( cyclingFeatures & CSC_MULTI_SENS_SUPP ) )
      {
        cscCmdInd.len += supportedSensors;
        osal_memcpy( &(cscCmdInd.value[3]), supportedSensorLocations, supportedSensors );
      }
      else // characteristic not supported.
      {
        // Send an indication with the list.
        cscStatus = CSC_INVALID_PARAMETER;
      }
      break;

     default:
      // Send an indication with opcode not suported response
      cscStatus = CSC_OPCODE_NOT_SUPPORTED;
      break;
  }

  // Send indication of operation result
  cscCmdInd.value[2] = cscStatus;

  // Ask our task to send out indication
  osal_set_event( cyclingService_TaskID, CSC_CMD_IND_SEND_EVT );
}

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
void Cycling_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, cyclingMeasClientCharCfg );
      GATTServApp_InitCharCfg( connHandle, cyclingCommandClientCharCfg );
    }
  }
}

/*********************************************************************
 * @fn          cycling_ReadAttrCB
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
static uint8 cycling_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                         uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch ( uuid )
  {
    case SENSOR_LOC_UUID:
    {
      // Read Sensor Location
      *pLen = 1;
      pValue[0] = pAttr->pValue[0];
    }
    break;

    case CSC_FEATURE_UUID:
    {
      //Read Cycling Feature
      *pLen = 2;
      pValue[0] = LO_UINT16(pAttr->pValue[0]);
      pValue[1] = HI_UINT16(pAttr->pValue[0]);
    }
    break;

    case GATT_CLIENT_CHAR_CFG_UUID:
    {
      // Read Measurement or Command Configuration
      if ( pAttr->pValue == (uint8*)cyclingMeasClientCharCfg )
      {
        *pLen = 1;
         pValue[0] = GATTServApp_ReadCharCfg(connHandle, cyclingMeasClientCharCfg );
      }
      else if ( pAttr->pValue == (uint8*)cyclingCommandClientCharCfg )
      {
        *pLen = 1;
         pValue[0] = GATTServApp_ReadCharCfg(connHandle, cyclingCommandClientCharCfg );
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_FOUND;
      }
    }
    break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
    break;
  }

  // Notify app
  if ( cyclingServiceCB != NULL )
  {
    (*cyclingServiceCB)( CSC_READ_ATTR, NULL );
  }

  return ( status );
}


/*********************************************************************
 * @fn      cycling_WriteAttrCB
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
static bStatus_t cycling_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                      uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if ( offset > 0 )
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  switch ( uuid )
  {
    case SC_CTRL_PT_UUID:
      // Make sure Control Point Cfg is not already in progress
      if ( scOpInProgress == TRUE )
      {
        status = CSC_ERR_PROC_IN_PROGRESS;
      }
      // Make sure Control Point Cfg is configured for Indications
      else if ( (cyclingCommandClientCharCfg[connHandle].value & GATT_CLIENT_CFG_INDICATE) == FALSE )
      {
        status = CSC_ERR_CCC_IMPROPER_CFG;
      }
      else
      {
        // Process CSC command
        cycling_ProcessCSCCmd( pAttr->handle, pValue, len );
        connectionHandle = connHandle;
      }
      break;

    // For Measure and Commands CCC
    case GATT_CLIENT_CHAR_CFG_UUID:
      if ( pAttr->handle == cyclingAttrTbl[CSC_COMMAND_CFG_POS].handle )
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        // Notify app
        if ( cyclingServiceCB != NULL )
        {
          (*cyclingServiceCB)( CSC_WRITE_ATTR, NULL );
        }
      }
      else if ( pAttr->handle == cyclingAttrTbl[CSC_MEAS_CFG_POS].handle )
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS )
        {
          // Notify app
          if ( cyclingServiceCB != NULL )
          {
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

            (*cyclingServiceCB)( ((charCfg == GATT_CFG_NO_OPERATION) ?
                                   CSC_MEAS_NOTI_DISABLED :
                                   CSC_MEAS_NOTI_ENABLED ), NULL );
          }
        }
      }
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/