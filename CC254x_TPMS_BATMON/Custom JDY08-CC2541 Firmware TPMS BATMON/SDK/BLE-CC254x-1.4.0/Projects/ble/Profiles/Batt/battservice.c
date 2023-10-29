/**************************************************************************************************
  Filename:       battservice.c
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the Battery service.

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
#include "hal_adc.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "hiddev.h"

#include "battservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// ADC voltage levels
#define BATT_ADC_LEVEL_3V           409
#define BATT_ADC_LEVEL_2V           273

#define BATT_LEVEL_VALUE_IDX        2 // Position of battery level in attribute array
#define BATT_LEVEL_VALUE_CCCD_IDX   3 // Position of battery level CCCD in attribute array

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Battery service
CONST uint8 battServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATT_SERV_UUID), HI_UINT16(BATT_SERV_UUID)
};

// Battery level characteristic
CONST uint8 battLevelUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATT_LEVEL_UUID), HI_UINT16(BATT_LEVEL_UUID)
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

// Application callback
static battServiceCB_t battServiceCB;

// Measurement setup callback
static battServiceSetupCB_t battServiceSetupCB = NULL;

// Measurement teardown callback
static battServiceTeardownCB_t battServiceTeardownCB = NULL;

// Measurement calculation callback
static battServiceCalcCB_t battServiceCalcCB = NULL;

static uint16 battMinLevel = BATT_ADC_LEVEL_2V; // For VDD/3 measurements
static uint16 battMaxLevel = BATT_ADC_LEVEL_3V; // For VDD/3 measurements

// Critical battery level setting
static uint8 battCriticalLevel;

// ADC channel to be used for reading
static uint8 battServiceAdcCh = HAL_ADC_CHANNEL_VDD;

/*********************************************************************
 * Profile Attributes - variables
 */

// Battery Service attribute
static CONST gattAttrType_t battService = { ATT_BT_UUID_SIZE, battServUUID };

// Battery level characteristic
static uint8 battLevelProps = GATT_PROP_READ | GATT_PROP_NOTIFY; 
static gattCharCfg_t battLevelClientCharCfg[GATT_MAX_NUM_CONN];

// HID Report Reference characteristic descriptor, battery level
static uint8 hidReportRefBattLevel[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_BATT_LEVEL_IN, HID_REPORT_TYPE_INPUT };

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t battAttrTbl[] =
{
  // Battery Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&battService                     /* pValue */
  },

    // Battery Level Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &battLevelProps
    },

      // Battery Level Value
      {
        { ATT_BT_UUID_SIZE, battLevelUUID },
        GATT_PERMIT_READ,
        0,
        &battLevel
      },

      // Battery Level Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &battLevelClientCharCfg
      },

      // HID Report Reference characteristic descriptor, batter level input
      {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefBattLevel
      }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 battReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t battWriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset );
static void battNotifyCB( linkDBItem_t *pLinkItem );
static uint8 battMeasure( void );
static void battNotifyLevel( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Battery Service Callbacks
CONST gattServiceCBs_t battCBs =
{
  battReadAttrCB,  // Read callback function pointer
  battWriteAttrCB, // Write callback function pointer
  NULL             // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Batt_AddService
 *
 * @brief   Initializes the Battery Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Batt_AddService( void )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, battLevelClientCharCfg );

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( battAttrTbl,
                                        GATT_NUM_ATTRS( battAttrTbl ),
                                        &battCBs );

  return ( status );
}

/*********************************************************************
 * @fn      Batt_Register
 *
 * @brief   Register a callback function with the Battery Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Batt_Register( battServiceCB_t pfnServiceCB )
{
  battServiceCB = pfnServiceCB;
}


/*********************************************************************
 * @fn      Batt_SetParameter
 *
 * @brief   Set a Battery Service parameter.
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
bStatus_t Batt_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case BATT_PARAM_CRITICAL_LEVEL:
      battCriticalLevel = *((uint8*)value);

      // If below the critical level and critical state not set, notify it
      if ( battLevel < battCriticalLevel )
      {
        battNotifyLevel();
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      Batt_GetParameter
 *
 * @brief   Get a Battery Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Batt_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BATT_PARAM_LEVEL:
      *((uint8*)value) = battLevel;
      break;

    case BATT_PARAM_CRITICAL_LEVEL:
      *((uint8*)value) = battCriticalLevel;
      break;

    case BATT_PARAM_SERVICE_HANDLE:
      *((uint16*)value) = GATT_SERVICE_HANDLE( battAttrTbl );
      break;

    case BATT_PARAM_BATT_LEVEL_IN_REPORT:
      {
        hidRptMap_t *pRpt = (hidRptMap_t *)value;

        pRpt->id = hidReportRefBattLevel[0];
        pRpt->type = hidReportRefBattLevel[1];
        pRpt->handle = battAttrTbl[BATT_LEVEL_VALUE_IDX].handle;
        pRpt->cccdHandle = battAttrTbl[BATT_LEVEL_VALUE_CCCD_IDX].handle;
        pRpt->mode = HID_PROTOCOL_MODE_REPORT;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          Batt_MeasLevel
 *
 * @brief       Measure the battery level and update the battery
 *              level value in the service characteristics.  If
 *              the battery level-state characteristic is configured
 *              for notification and the battery level has changed
 *              since the last measurement, then a notification
 *              will be sent.
 *
 * @return      Success
 */
bStatus_t Batt_MeasLevel( void )
{
  uint8 level;

  level = battMeasure();

  // If level has gone down
  if (level < battLevel)
  {
    // Update level
    battLevel = level;

    // Send a notification
    battNotifyLevel();
  }

  return SUCCESS;
}

/*********************************************************************
 * @fn      Batt_Setup
 *
 * @brief   Set up which ADC source is to be used. Defaults to VDD/3.
 *
 * @param   adc_ch - ADC Channel, e.g. HAL_ADC_CHN_AIN6
 * @param   minVal - max battery level
 * @param   maxVal - min battery level
 * @param   sCB - HW setup callback
 * @param   tCB - HW tear down callback
 * @param   cCB - percentage calculation callback
 *
 * @return  none.
 */
void Batt_Setup( uint8 adc_ch, uint16 minVal, uint16 maxVal,
                 battServiceSetupCB_t sCB, battServiceTeardownCB_t tCB,
                 battServiceCalcCB_t cCB )
{
  battServiceAdcCh = adc_ch;
  battMinLevel = minVal;
  battMaxLevel = maxVal;

  battServiceSetupCB = sCB;
  battServiceTeardownCB = tCB;
  battServiceCalcCB = cCB;
}

/*********************************************************************
 * @fn          battReadAttrCB
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
static uint8 battReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1] );

  // Measure battery level if reading level
  if ( uuid == BATT_LEVEL_UUID )
  {
    uint8 level;

    level = battMeasure();

    // If level has gone down
    if (level < battLevel)
    {
      // Update level
      battLevel = level;
    }

    *pLen = 1;
    pValue[0] = battLevel;
  }
  else if ( uuid == GATT_REPORT_REF_UUID )
  {
    *pLen = HID_REPORT_REF_LEN;
    osal_memcpy( pValue, pAttr->pValue, HID_REPORT_REF_LEN );
  }
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return ( status );
}

/*********************************************************************
 * @fn      battWriteAttrCB
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
static bStatus_t battWriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch ( uuid )
  {
    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );
      if ( status == SUCCESS )
      {
        uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

        if ( battServiceCB )
        {
          (*battServiceCB)( (charCfg == GATT_CFG_NO_OPERATION) ?
                            BATT_LEVEL_NOTI_DISABLED :
                            BATT_LEVEL_NOTI_ENABLED);
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
 * @fn          battNotifyCB
 *
 * @brief       Send a notification of the level state characteristic.
 *
 * @param       connHandle - linkDB item
 *
 * @return      None.
 */
static void battNotifyCB( linkDBItem_t *pLinkItem )
{
  if ( pLinkItem->stateFlags & LINK_CONNECTED )
  {
    uint16 value = GATTServApp_ReadCharCfg( pLinkItem->connectionHandle,
                                            battLevelClientCharCfg );
    if ( value & GATT_CLIENT_CFG_NOTIFY )
    {
      attHandleValueNoti_t noti;

      noti.handle = battAttrTbl[BATT_LEVEL_VALUE_IDX].handle;
      noti.len = 1;
      noti.value[0] = battLevel;

      GATT_Notification( pLinkItem->connectionHandle, &noti, FALSE );
    }
  }
}

/*********************************************************************
 * @fn      battMeasure
 *
 * @brief   Measure the battery level with the ADC and return
 *          it as a percentage 0-100%.
 *
 * @return  Battery level.
 */
 static uint8 battMeasure( void )
{
  uint16 adc;
  uint8 percent;

  /**
   * Battery level conversion from ADC to a percentage:
   *
   * The maximum ADC value for the battery voltage level is 511 for a
   * 10-bit conversion.  The ADC value is references vs. 1.25v and
   * this maximum value corresponds to a voltage of 3.75v.
   *
   * For a coin cell battery 3.0v = 100%.  The minimum operating
   * voltage of the CC2540 is 2.0v so 2.0v = 0%.
   *
   * To convert a voltage to an ADC value use:
   *
   *   (v/3)/1.25 * 511 = adc
   *
   * 3.0v = 409 ADC
   * 2.0v = 273 ADC
   *
   * We need to map ADC values from 409-273 to 100%-0%.
   *
   * Normalize the ADC values to zero:
   *
   * 409 - 273 = 136
   *
   * And convert ADC range to percentage range:
   *
   * percent/adc = 100/136 = 25/34
   *
   * Resulting in the final equation, with round:
   *
   * percent = ((adc - 273) * 25) + 33 / 34
   */

  // Call measurement setup callback
  if (battServiceSetupCB != NULL)
  {
    battServiceSetupCB();
  }

  // Configure ADC and perform a read
  HalAdcSetReference( HAL_ADC_REF_125V );
  adc = HalAdcRead( battServiceAdcCh, HAL_ADC_RESOLUTION_10 );

  // Call measurement teardown callback
  if (battServiceTeardownCB != NULL)
  {
    battServiceTeardownCB();
  }

  if (adc >= battMaxLevel)
  {
    percent = 100;
  }
  else if (adc <= battMinLevel)
  {
    percent = 0;
  }
  else
  {
    if (battServiceCalcCB != NULL)
    {
      percent = battServiceCalcCB(adc);
    }
    else
    {
      uint16 range =  battMaxLevel - battMinLevel + 1;

      // optional if you want to keep it even, otherwise just take floor of divide
      // range += (range & 1);
      range >>= 2; // divide by 4

      percent = (uint8) ((((adc - battMinLevel) * 25) + (range - 1)) / range);
    }
  }

  return percent;
}

/*********************************************************************
 * @fn      battNotifyLevelState
 *
 * @brief   Send a notification of the battery level state
 *          characteristic if a connection is established.
 *
 * @return  None.
 */
static void battNotifyLevel( void )
{
  // Execute linkDB callback to send notification
  linkDB_PerformFunc( battNotifyCB );
}

/*********************************************************************
 * @fn          Batt_HandleConnStatusCB
 *
 * @brief       Battery Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
void Batt_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, battLevelClientCharCfg );
    }
  }
}


/*********************************************************************
*********************************************************************/
