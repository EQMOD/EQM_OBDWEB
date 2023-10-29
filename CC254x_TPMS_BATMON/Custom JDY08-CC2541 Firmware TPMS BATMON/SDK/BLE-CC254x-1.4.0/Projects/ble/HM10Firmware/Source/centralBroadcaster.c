/**************************************************************************************************
  Filename:       ibeacon.c
  Description:    This file implements an device acting as multi role device:
                  - Broadcaster role: advertises ibeacon data
                  - Central role: discovers nearby ibeaons

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_uart.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "centralBroadcasterProfile.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"

#include "centralBroadcaster.h"
#include "uartManager.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  20

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 5000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          200

// Company Identifiers
#define TI_COMPANY_ID                         0x000D  //texas instruments
#define APPLE_COMPANY_ID                      0x004C  //apple

/*********************************************************************
 * TYPEDEFS
 */
// GAP - SCAN RSP data (max size = 31 bytes)
//   this data can be requested by a central role device
typedef struct ibeaconScanRspData
{
  uint8 len1;
  uint8 type1;
  uint8 powerLevel;
  
  uint8 len2;
  uint8 type2;
  uint8 name[NAME_LENGTH_MAX];
}ibeaconScanRspData_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 simpleBLETaskId; // Task ID for task/event processing

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "customBeacon";
static bool isStarted = FALSE;

static uint8 advertData[] =
{
  2,  //length
  GAP_ADTYPE_FLAGS,  //type
  GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | GAP_ADTYPE_FLAGS_GENERAL  //data
};

static struct ibeaconScanRspData scanRspData =
{
  .len1 = 2,
  .type1 = GAP_ADTYPE_POWER_LEVEL,
  .powerLevel = 0,       // 0dBm  
  .len2 = 10,
  .type2 = GAP_ADTYPE_LOCAL_NAME_COMPLETE, 
  .name = "<unnamed>"
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/********* Central ************/
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );

char *bdAddr2Str ( uint8 *pAddr );

/********* Broadcast ****************/
static void peripheralStateNotificationCB( gaprole_States_t newState );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Role Callbacks - Central and Peripheral
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,          // RSSI callback
  simpleBLECentralEventCB,         // Event callback
  peripheralStateNotificationCB   // Broadcast state callback    
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
   simpleBLETaskId = task_id;
  
  /***** Broadcaster ********/
  {
    uint8 initial_advertising_enable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
      
    //uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;   // use non-connectable advertisements
    uint8 advType = GAP_ADTYPE_ADV_SCAN_IND; // use scannable unidirected advertisements

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), &scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), &advertData );

    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
  }

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  /*********** Central *********************/
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );
  
  osal_set_event( simpleBLETaskId, CB_INIT_DEVICE_EVT );  //delayed startup
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  /*** Sys Event ***/
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      VOID osal_msg_deallocate( pMsg );      // Release the OSAL message
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  /*** Init Device ***/
  if ( events & CB_INIT_DEVICE_EVT )
  {
    HalLedBlink(HAL_LED_1, 3, 50, 1000);
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );      // start scanning
    return ( events ^ CB_INIT_DEVICE_EVT );
  }
  
  /*** Start Device ***/
  if ( events & CB_START_DEVICE_EVT )
  {
    if(!isStarted)
    {
      isStarted = TRUE;
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                         DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                         DEFAULT_DISCOVERY_WHITE_LIST );
      uint8 advertising_enable = TRUE;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
      HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
    }
    return ( events ^ CB_START_DEVICE_EVT );
  }

  /*** Stop Device ***/
  if ( events & CB_STOP_DEVICE_EVT )
  {
    if(isStarted)
    {
      isStarted = FALSE;
      GAPCentralRole_CancelDiscovery();
      uint8 advertising_enable = FALSE;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
      HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    }
    return ( events ^ CB_STOP_DEVICE_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case GATT_MSG_EVENT:
      break;
      
    case CB_MSG_SET_ADVERTISING_DATA:
      {
        setAdvertisingDataMsg_t* msg = (setAdvertisingDataMsg_t*) pMsg;
        GAP_UpdateAdvertisingData(simpleBLETaskId, TRUE, msg->length, msg->data); 
      }
      break;
    case CB_MSG_SET_NAME:
      {
        setNameMsg_t* msg = (setNameMsg_t*) pMsg;
        scanRspData.len2 = msg->length + 1; //name + 1byte type
        for(int i=0; i<msg->length; i++)
        {
          scanRspData.name[i] = msg->name[i];
        }
        GAP_UpdateAdvertisingData(simpleBLETaskId, FALSE, 5+msg->length, (uint8*) &scanRspData); 
      }
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:   //called when discovery is started
      break;

    case GAP_DEVICE_INFO_EVENT:       //called when device is found during discovery
      {
        // notify MCU
        if(pEvent->deviceInfo.eventType != GAP_ADRPT_SCAN_RSP )
        {
          deviceFoundMsg_t* msg = (deviceFoundMsg_t*) osal_msg_allocate(sizeof(deviceFoundMsg_t));
          msg->event = CB_MSG_DEVICE_FOUND;
            
          // copy parameters from centralRoleEvent
          osal_memcpy(&msg->address, &pEvent->deviceInfo.addr, B_ADDR_LEN );
          msg->rssi = -pEvent->deviceInfo.rssi;
          osal_memcpy(msg->advData, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen );
          msg->length = pEvent->deviceInfo.dataLen + B_ADDR_LEN + sizeof(uint8);
            
          // send message
          osal_msg_send(uartManagerTaskID, (uint8*) msg);
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:  //called when discovery is complete
      if(isStarted)
      {
          GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                         DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                         DEFAULT_DISCOVERY_WHITE_LIST );
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    // LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {    
        uint8 ownAddress[B_ADDR_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
      }
      break;
      
    case GAPROLE_ADVERTISING:
      break;

    case GAPROLE_WAITING:
      break;          

    case GAPROLE_ERROR:
      break;      
      
    default:
      break; 
  }
}

/*********************************************************************
*********************************************************************/
