/**************************************************************************************************
  Filename:       TPMSBroadcast_Tester.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Broadcaster sample application 
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

#include <stdio.h>
#include <stdlib.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"


#include "hci.h"
#include "gap.h"

#include "devinfoservice.h"
#include "broadcaster.h"
#include "TPMSBroadcast_Tester.h"


#define SBP_PERIODIC_EVT_PERIOD                   5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)

#define DEFAULT_ADVERTISING_INTERVAL          3200

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

uint8 hciExtApp_TaskID;   // Task ID for internal task/event processing


int sensorID = 0;
static uint8 TPMSBroadcast_Tester_TaskID;   // Task ID for internal task/event processing
 
// GAP - SCAN RSP data (max size = 31 bytes)
uint8 scanRspData1[] = {0x0d,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'T','P','M','S','1','_','1','0','0','6','B','5',0x02,GAP_ADTYPE_POWER_LEVEL,0};
uint8 scanRspData2[] = {0x0d,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'T','P','M','S','2','_','2','0','0','6','B','6',0x02,GAP_ADTYPE_POWER_LEVEL,0};
uint8 scanRspData3[] = {0x0d,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'T','P','M','S','3','_','3','0','0','6','B','7',0x02,GAP_ADTYPE_POWER_LEVEL,0};
uint8 scanRspData4[] = {0x0d,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'T','P','M','S','4','_','4','0','0','6','B','8',0x02,GAP_ADTYPE_POWER_LEVEL,0};
uint8 scanRspData5[] = {0x0d,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'T','P','M','S','5','_','5','0','0','6','B','9',0x02,GAP_ADTYPE_POWER_LEVEL,0};


uint8 baddr1[] = {0xb5,0x06,0x10,0xca,0xea,0x80};
uint8 baddr2[] = {0xb6,0x06,0x20,0xca,0xea,0x81};
uint8 baddr3[] = {0xb7,0x06,0x30,0xca,0xea,0x82};
uint8 baddr4[] = {0xb8,0x06,0x40,0xca,0xea,0x83};
uint8 baddr5[] = {0xb9,0x06,0x50,0xca,0xea,0x84};


uint8 aData1[] = {0x02,0x01,0x05,0x03,0x03,0xb0,0xfb,0x13,0xff,0x00,0x01,0x80,0xea,0xca,0x10,0x06,0xb5,0x2a,0x7a,0x02,0x00,0xee,0x09,0x00,0x00,0x38,0x00,0x00};
uint8 aData2[] = {0x02,0x01,0x05,0x03,0x03,0xb0,0xfb,0x13,0xff,0x00,0x01,0x81,0xea,0xca,0x20,0x06,0xb6,0xb0,0x71,0x02,0x00,0x71,0x0a,0x00,0x00,0x42,0x00,0x00};
uint8 aData3[] = {0x02,0x01,0x05,0x03,0x03,0xb0,0xfb,0x13,0xff,0x00,0x01,0x82,0xea,0xca,0x30,0x06,0xb7,0x05,0x9a,0x02,0x00,0x87,0x0a,0x00,0x00,0x35,0x00,0x00};
uint8 aData4[] = {0x02,0x01,0x05,0x03,0x03,0xb0,0xfb,0x13,0xff,0x00,0x01,0x83,0xea,0xca,0x40,0x06,0xb8,0x31,0x79,0x02,0x00,0x24,0x0a,0x00,0x00,0x2f,0x00,0x00};
uint8 aData5[] = {0x02,0x01,0x05,0x03,0x03,0xb0,0xfb,0x13,0xff,0x00,0x01,0x84,0xea,0xca,0x50,0x06,0xb9,0x05,0x80,0x02,0x00,0x50,0x0a,0x00,0x00,0x25,0x00,0x00};



/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void peripheralStateNotificationCB( gaprole_States_t newState );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t TPMSBroadcast_Tester_BroadcasterCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      TPMSBroadcast_Tester_Init
 *
 * @brief   Initialization function for the Simple BLE Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void TPMSBroadcast_Tester_Init( uint8 task_id )
{
  TPMSBroadcast_Tester_TaskID = task_id;
  uint8 new_adv_enabled_status;
  
  
  // SerialApp_Init(task_id);

  // Setup the GAP Broadcaster Role Profile
  {

      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;


    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 3;
    HCI_EXT_SetBDADDRCmd(baddr1);
	//GAP_ConfigDeviceAddr(1,baddr);
    //uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;   // use non-connectable advertisements
    uint8 advType = GAP_ADTYPE_ADV_SCAN_IND; // use scannable unidirected advertisements

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData1 ), scanRspData1 );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( aData1 ), aData1 );

    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
	
	new_adv_enabled_status = TRUE;
	
	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
  }

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  


//	char tbuf[30];
//	sprintf(tbuf,"Advert Started.\n");
//	Uart1SendString(tbuf,strlen(tbuf));
    
  

  osal_set_event( TPMSBroadcast_Tester_TaskID, SBP_START_DEVICE_EVT );
  

}


static void performPeriodicTask( void )
{
	
	sensorID++;
	if (sensorID >= 5) sensorID = 0;
	//sprintf(tbuf,"Change Advert.%d\n",sensorID & 0x3);
	//Uart1SendString(tbuf,strlen(tbuf));
	
	uint8 new_adv_enabled_status = FALSE;
	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
	
	switch(sensorID)
	{
	
		case 0:
				aData1[18] = rand() % 0xff;
				aData1[21] = rand() % 0xff;
				aData1[25] = 0x20 + (rand() % 0x30);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,TRUE,sizeof ( aData1 ),aData1);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,FALSE,sizeof ( scanRspData1 ),scanRspData1);
				HCI_EXT_SetBDADDRCmd(baddr1);
				break;
		case 1:
				aData2[18] = rand() % 0xff;
				aData2[21] = rand() % 0xff;
				aData2[25] = 0x20 + (rand() % 0x30);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,TRUE,sizeof ( aData2 ),aData2);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,FALSE,sizeof ( scanRspData2 ),scanRspData2);
				HCI_EXT_SetBDADDRCmd(baddr2);
				break;
		case 2:
				aData3[18] = rand() % 0xff;
				aData3[21] = rand() % 0xff;
				aData3[25] = 0x20 + (rand() % 0x30);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,TRUE,sizeof ( aData3 ),aData3);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,FALSE,sizeof ( scanRspData3 ),scanRspData3);
				HCI_EXT_SetBDADDRCmd(baddr3);
				break;
		case 3:		
				aData4[18] = rand() % 0xff;
				aData4[21] = rand() % 0xff;
				aData4[25] = 0x20 + (rand() % 0x30);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,TRUE,sizeof ( aData4 ),aData4);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,FALSE,sizeof ( scanRspData4 ),scanRspData4);
				HCI_EXT_SetBDADDRCmd(baddr4);
				break;
		default:
				aData5[18] = rand() % 0xff;
				aData5[21] = rand() % 0xff;
				aData5[25] = 0x20 + (rand() % 0x30);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,TRUE,sizeof ( aData5 ),aData5);
				GAP_UpdateAdvertisingData( TPMSBroadcast_Tester_TaskID,FALSE,sizeof ( scanRspData5 ),scanRspData5);
				HCI_EXT_SetBDADDRCmd(baddr5);
				break;

	}
	
	new_adv_enabled_status = TRUE;
	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );	
	
}



/*********************************************************************
 * @fn      TPMSBroadcast_Tester_ProcessEvent
 *
 * @brief   Simple BLE Broadcaster Application Task event processor. This
 *          function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 TPMSBroadcast_Tester_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
    return (events ^ SYS_EVENT_MSG);

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Set timer for first periodic event
    osal_start_timerEx( TPMSBroadcast_Tester_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    // Start the Device
    VOID GAPRole_StartDevice( &TPMSBroadcast_Tester_BroadcasterCBs );

    return ( events ^ SBP_START_DEVICE_EVT );
  }
  
  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
      osal_start_timerEx( TPMSBroadcast_Tester_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    // Perform periodic application task
    performPeriodicTask();
    return (events ^ SBP_PERIODIC_EVT);
  }  
  // Discard unknown events
  return 0;
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
	

}


/*********************************************************************
*********************************************************************/
