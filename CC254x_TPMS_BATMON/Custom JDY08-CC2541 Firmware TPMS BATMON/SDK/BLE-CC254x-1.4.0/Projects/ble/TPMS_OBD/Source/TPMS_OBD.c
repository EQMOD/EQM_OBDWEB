/**************************************************************************************************
  Filename:       TPMS_OBD.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $
  Description:    This file contains the Simple BLE Observer sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.
  Copyright 2011 Texas Instruments Incorporated. All rights reserved.
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
#include "stdarg.h"
#include "stdio.h"
#include "l2cap.h"
   
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
//#include "OnBoard.h"
#include "hal_uart.h"
#include "ll.h"
#include "hci.h"
#include "observer.h"
   
#include "gatt.h"
#include "gatt_uuid.h"
#include "osal_snv.h"
#if !defined ( GATT_DB_OFF_CHIP )
  #include "gattservapp.h"
  #include "gapgattserver.h"
  #if defined ( GATT_TEST ) || defined ( GATT_QUAL )
    #include "gatttest.h"
  #endif
#endif // GATT_DB_OFF_CHIP
#if defined ( GAP_BOND_MGR )
  #include "gapbondmgr.h"
#endif   
#include "TPMS_OBD.h"
#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif 


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
#define DEFAULT_SCAN_DURATION                 3000
// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL
// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE
// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE



#define TPMSOBD_UART_PORT                  HAL_UART_PORT_1
//#define TPMSOBD_UART_FC                    TRUE
#define TPMSOBD_UART_FC                    FALSE
#define TPMSOBD_UART_FC_THRESHOLD          48
#define TPMSOBD_UART_RX_BUF_SIZE           128
#define TPMSOBD_UART_TX_BUF_SIZE           128
#define TPMSOBD_UART_IDLE_TIMEOUT          6
#define TPMSOBD_UART_INT_ENABLE            TRUE
#define TPMSOBD_UART_BR                     HAL_UART_BR_115200 
   
   
static uint8 out_msg[40];   
/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
 * GLOBAL VARIABLES
 */
/*********************************************************************
 * EXTERNAL VARIABLES
 */
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
/*********************************************************************
 * LOCAL VARIABLES
 */
// Task ID for internal task/event processing
static uint8 simpleBLETaskId;
// GAP GATT Attributes

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;
// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];
// Scanning state
static uint8 simpleBLEScanning = FALSE;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void TPMS_OBDEventCB( gapObserverRoleEvent_t *pEvent );
static void TPMS_OBD_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType, uint8 len,uint8 *pEvtData );

 typedef unsigned char uchar;
 typedef unsigned int  uint;
 
 #define LED1 P1_2

 
 char temp;             
 uchar datanum = 0;    
 uchar RXflag = 1;
 uchar TXflag = 0;
 
 /****************************************************************
 *InitClock
 *****************************************************************/
 void InitClock(void)
 {
 CLKCONCMD &= ~0x40; // use crystal 32MHZ
 while(CLKCONSTA & 0x40); // wait for clock to stabilize 
 CLKCONCMD &= ~0x47; // set clock as 32MHZ
 }
 
 /******************************************************************************
 *?InitLED
 ******************************************************************************/
 void InitLED(uchar On_Off)
 {
   P1SEL &= ~0x04;      //P1.2 io mode
   P1DIR |=  0x04;      ///p1.2 led output
   LED1 = On_Off;       ///
 }
 
 /******************************************************************************
 *InitUART
 *UART0 115200 8N1
 ******************************************************************************/
 void InitUART(void)
 {
   PERCFG &= ~0x01;     //USART0 alt1
   P2DIR &= ~0xc0;      //IO priority USART0 >USART1 >timer1
   P0SEL |= 0x0c;       //P0_2,P0_3 set as UART
   U0CSR |= 0x80;       //set UART mode
   U0CSR |= 0x40;       //enable USART0 receive
   
   U0UCR = 0x02;        //8N1
   U0GCR |=11;          //32MHz BAUD_E:11, 115200
   U0BAUD |= 216;       //32MHz BAUD_M:216 11520
   UTX0IF = 0;          //UART0 TX interrupt reset
   IEN0 |= 0x04;        //USART0 RX interrupt enable
   EA = 1;              //enable interrupt
 }
 
 /****************************************************************************
 * UartSendString()
 ****************************************************************************/
 void UartSendString(char *Data, int len)
 {
   uint i;
   U0CSR &= ~0x40;  //disable receive
   
   for(i=0; i<len; i++)
   {
     U0DBUF = *Data++;
     while(UTX0IF == 0);
     UTX0IF = 0;
   }
   TXflag = 0;  
   U0CSR |= 0x40;       //enable receiving
 }
 
 /******************************************************************************
 *UART0_ISR
 ******************************************************************************/
 #pragma vector = URX0_VECTOR 
 __interrupt void UART0_ISR(void) 
 { 
   URX0IF = 0;           //UART0 RX interrupt reset
   temp = U0DBUF;        //U0DBUF
 }
 
 /******************************************************************************
 *InitUART1
 * 8N1, 115200 alt2 location
 ******************************************************************************/
 void InitUART1(void)
 {
   PERCFG |= 0x02;     //USART1 at alt location 2 TX P1.6, RXP1.7
   P2DIR &= ~0x80;      //priority USART1>USART0 >TIMER1
   P2DIR |=0x40;
   P1SEL |= 0xc0;       //P1.6, P1.7 as peripheral
   U1CSR |= 0x80;       //set UART mode
   U1CSR |= 0x40;       //enable USART1 receive
   
   U1UCR = 0x02;        //8N1
   U1GCR |=11;          //32MHz BAUD_E:11, 115200
   U1BAUD |= 216;       //32MHz BAUD_M:216 11520
   UTX1IF = 0;          //UART1 TX interrupt reset
   IEN0 |= 0x08;        //IEN0.URX1IE=1
   EA = 1;              //enable interrupt
 }
 
 /****************************************************************************
 * Uart1SendString()
 ****************************************************************************/
 void Uart1SendString(char *Data, int len)
 {
   uint i;
   U1CSR &= ~0x40;  
   
   for(i=0; i<len; i++)
   {
     U1DBUF = *Data++;
     while(UTX1IF == 0);
     UTX1IF = 0;
   }
   TXflag = 0;  
   U1CSR |= 0x40;       //enable
 }
 
 /******************************************************************************
 * UART1_ISR
 ******************************************************************************/
 #pragma vector = URX1_VECTOR 
 __interrupt void UART1_ISR(void) 
 { 
   URX1IF = 0;           //UART1 RX interrupt reset
   temp = U1DBUF;        //read from U1DBUF
 }


void SerialInitTransport(void)
{
    InitClock();
    InitUART1();  
  
}/*serialAppInitTransport*/



static uint8 sendMsgTo_TaskID;

void SerialApp_Init( uint8 taskID )
{
  SerialInitTransport(); 
  sendMsgTo_TaskID = taskID; //save task id, for spare using.
}/*SerialApp_Init*/


/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Role Callbacks
static const gapObserverRoleCB_t simpleBLERoleCB =
{
  NULL,                     // RSSI callback
  TPMS_OBDEventCB  // Event callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      TPMS_OBD_Init
 *
 * @brief   Initialization function for the Simple BLE Observer App Task.
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
void TPMS_OBD_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;
  SerialApp_Init(task_id);

  char tbuf[30];

  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter ( GAPOBSERVERROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
  
  //VOID GAPObserverRole_StartDevice( (gapObserverRoleCB_t *) &simpleBLERoleCB );

  sprintf(tbuf,"VC601 TPMS Scan Started.\n");
  Uart1SendString(tbuf,strlen(tbuf));

}

/*********************************************************************
 * @fn      TPMS_OBD_ProcessEvent
 *
 * @brief   Simple BLE Observer Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 TPMS_OBD_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      TPMS_OBD_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPObserverRole_StartDevice( (gapObserverRoleCB_t *) &simpleBLERoleCB );

    GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST ); 

    simpleBLEScanning = TRUE;
    simpleBLEScanRes = 0;									  

    return ( events ^ START_DEVICE_EVT );
  }
  
  // Discard unknown events
  return 0;
}



/*********************************************************************
 * @fn      TPMS_OBD_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void TPMS_OBD_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
     
  switch ( pMsg->event )
  {
    default:
      break; // ignore
  }

}

/*********************************************************************
 * @fn      TPMS_OBDEventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void TPMS_OBDEventCB( gapObserverRoleEvent_t *pEvent )
{
	
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  

      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType,  pEvent->deviceInfo.dataLen, pEvent->deviceInfo.pEvtData );
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        //simpleBLEScanning = FALSE;


        // Copy results
//        simpleBLEScanRes = pEvent->discCmpl.numDevs;
//        osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
//                     (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        
        // initialize scan index to last device	  
	  
		GAPObserverRole_CancelDiscovery();
		GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST ); 


		simpleBLEScanning = TRUE;


		simpleBLEScanRes = 0;
	
        simpleBLEScanIdx = simpleBLEScanRes;
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType, uint8 len,uint8 *pEvtData )
{
  uint8 i;
  
  char tbuf[30];
 
   // Send the TPMS Sensor Block Data the UART Port 1 (Pin 1.6-RX, 1.7-TX)
   // VC601 Sensors usually have 0xca at [3] and 0xea at [4] as the address. Change if necessary 
 
   if ((len > 14) && (pEvtData[8] == 0xff) && ((pAddr[3] == 0xca) ||(pAddr[3] == 0xcb))  && ((pAddr[4] == 0xea) ||(pAddr[4] == 0xeb)) )
    {
		tbuf[0] = 0xfe; // Block ID to send to MCU UART
		tbuf[1] = 15;	// Block Len of the Payload
		
		// Payload - Sensor Address 
		for(i=0;i<6;i++) 
			tbuf[i+2] = pAddr[i];
		
		// Payload - Sensor Data 
		for(i=0;i<9;i++) 
			tbuf[8+i] = pEvtData[i+17];
		
		// Send to Upper MCU
		Uart1SendString(tbuf,17);
    }
//	else
//		Uart1SendString("Other\n ",7);
 
 /*
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }

*/
}

/*********************************************************************
*********************************************************************/
