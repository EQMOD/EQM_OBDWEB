/**************************************************************************************************
  Filename:       BAT_MON.c
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
#include  "hal_adc.h"

#include "hci.h"
#include "gap.h"

#include "devinfoservice.h"
#include "broadcaster.h"
#include "BAT_MON.h"

#define HAL_ADC_EOC         0x80    /* End of Conversion bit */
#define HAL_ADC_START       0x40    /* Starts Conversion */

#define HAL_ADC_STSEL_EXT   0x00    /* External Trigger */
#define HAL_ADC_STSEL_FULL  0x10    /* Full Speed, No Trigger */
#define HAL_ADC_STSEL_T1C0  0x20    /* Timer1, Channel 0 Compare Event Trigger */
#define HAL_ADC_STSEL_ST    0x30    /* ADCCON1.ST =1 Trigger */

#define HAL_ADC_RAND_NORM   0x00    /* Normal Operation */
#define HAL_ADC_RAND_LFSR   0x04    /* Clock LFSR */
#define HAL_ADC_RAND_SEED   0x08    /* Seed Modulator */
#define HAL_ADC_RAND_STOP   0x0c    /* Stop Random Generator */
#define HAL_ADC_RAND_BITS   0x0c    /* Bits [3:2] */

#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_256     0x20    /* Decimate by 256 : 12-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_DEC_BITS    0x30    /* Bits [5:4] */

#define HAL_ADC_STSEL       HAL_ADC_STSEL_ST
#define HAL_ADC_RAND_GEN    HAL_ADC_RAND_STOP
#define HAL_ADC_REF_VOLT    HAL_ADC_REF_AVDD
#define HAL_ADC_DEC_RATE    HAL_ADC_DEC_064
#define HAL_ADC_SCHN        HAL_ADC_CHN_VDD3
#define HAL_ADC_ECHN        HAL_ADC_CHN_GND
#define SBP_PERIODIC_EVT_PERIOD                   1000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)

#define DEFAULT_ADVERTISING_INTERVAL          1600



// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


static uint8 adcRef;

int sensorID = 0;
uint16 bdcount = 0;

static uint8 BAT_MON_TaskID;   // Task ID for internal task/event processing
uint16 bdcount;
 
// GAP - SCAN RSP data (max size = 31 bytes)

uint8 hexval[]       = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',' '};
uint8 scanRspData1[] = {0x0d,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'B','A','T','M','O','N','1','0','0','6','B','5',0x02,GAP_ADTYPE_POWER_LEVEL,0};
uint8 baddr0[]       = {0xb5,0x06,0x10,0xcb,0xeb,0x80};
uint8 baddr1[]       = {0xb5,0x06,0x10,0xcb,0xeb,0x80};
uint8 aData1[]       = {0x02,0x01,0x05,0x03,0x03,0xb0,0xfb,0x13,0xff,0x00,0x01,0x80,0xeb,0xcb,0x10,0x06,0xb5,0x00,0x00,0x00,0x00,0x00,0x09,0x00,0x00,0x38,0x00,0x00};

uint8 LedBlink = 0;


// GAP Role Callbacks
static gapRolesCBs_t BAT_MON_BroadcasterCBs =
{
  NULL,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};


void WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

uint16 AdcRead (uint8 channel, uint8 resolution)
{
  int16  reading = 0;


  uint8   i, resbits;
  uint8  adcChannel = 1;

  int16 waitcount;
  
  if (channel <= HAL_ADC_CHANNEL_7)
  {
    for (i=0; i < channel; i++)
    {
      adcChannel <<= 1;
    }

    /* Enable channel */
    ADCCFG |= adcChannel;
  }

  /* Convert resolution to decimation rate */
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      resbits = HAL_ADC_DEC_064;
      break;
    case HAL_ADC_RESOLUTION_10:
      resbits = HAL_ADC_DEC_128;
      break;
    case HAL_ADC_RESOLUTION_12:
      resbits = HAL_ADC_DEC_256;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      resbits = HAL_ADC_DEC_512;
      break;
  }
  
  /* writing to this register starts the extra conversion */
  ADCCON3 = channel | resbits | adcRef;

  waitcount = 0;
  
  /* Wait for the conversion to be done */
  while (!(ADCCON1 & HAL_ADC_EOC))
  {
    WaitUs(1000);         // 1 Millsecond (?)
    waitcount++;
    if (waitcount > 500) /*Half a second wait */
      return ((uint16) 0xeffe);
  }

  /* Disable channel after done conversion */
  if (channel <= HAL_ADC_CHANNEL_7)
    ADCCFG &= (adcChannel ^ 0xFF);

  /* Read the result */
  reading = (int16) (ADCL);
  reading |= (int16) (ADCH << 8);

  /* Treat small negative as 0 */
  if (reading < 0)
    reading = 0;

  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      reading >>= 8;
      break;
    case HAL_ADC_RESOLUTION_10:
      reading >>= 6;
      break;
    case HAL_ADC_RESOLUTION_12:
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      reading >>= 2;
    break;
  }

  return ((uint16)reading);
}


void BAT_MON_Init( uint8 task_id )
{
  BAT_MON_TaskID = task_id;
  uint8 new_adv_enabled_status, randthird;
  uint16 adc;
  
  P0SEL=0x00;
  P0DIR=0x40;
  adcRef = 0x80;
  P0 = 0x00;
  
  bdcount = 0;
  
  adc = AdcRead(HAL_ADC_CHN_AIN7, HAL_ADC_RESOLUTION_14);

  srand(adc);
  randthird = rand() & 0xff;
  baddr1[0] = adc & 0xff;
  baddr1[1] = (adc >> 8) & 0xff;
  baddr1[2] = randthird;
  
  scanRspData1[13] = hexval[baddr1[0] & 0xf];
  scanRspData1[12] = hexval[(baddr1[0] >> 4) & 0xf];

  scanRspData1[11] = hexval[baddr1[1] & 0xf];
  scanRspData1[10] = hexval[(baddr1[1] >> 4) & 0xf];	

  scanRspData1[9] = hexval[baddr1[2] & 0xf];
  scanRspData1[8] = hexval[(baddr1[2] >> 4) & 0xf];
  
  aData1[16] = baddr1[0];
  aData1[15] = baddr1[1];
  aData1[14] = baddr1[2];
  
  HCI_EXT_SetBDADDRCmd(baddr1);
  // Setup the GAP Broadcaster Role Profile
  {
    // For other hardware platforms, device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 100;

    //GAP_ConfigDeviceAddr(1,baddr);
    uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;   // use non-connectable advertisements
    //uint8 advType = GAP_ADTYPE_ADV_SCAN_IND; // use scannable unidirected advertisements
    new_adv_enabled_status = TRUE;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData1 ), scanRspData1 );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( aData1 ), aData1 );
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
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
  
  osal_set_event( BAT_MON_TaskID, SBP_START_DEVICE_EVT );
  
}

static void performPeriodicTask( void )
{

        bdcount++;
  	LedBlink++;

      //  if (bdcount > 100)
      //     HAL_SYSTEM_RESET();

	
	if (LedBlink & 1) 
		P0 = 0x40;
	else
		P0 = 0x00;
  

	uint16 adc = AdcRead(HAL_ADC_CHN_AIN7, HAL_ADC_RESOLUTION_14); //P0.7 
	
	aData1[17] = adc & 0xff;
	aData1[18] = (adc >> 8) & 0xff;
        aData1[19] = bdcount & 0xff;
	aData1[20] = (bdcount >> 8) & 0xff;
        
        //aData1[19] = baddr0[0];
        //aData1[20] = baddr0[1];
        
	GAP_UpdateAdvertisingData( BAT_MON_TaskID,TRUE,sizeof ( aData1 ),aData1);
	
	//P0 = 0;
	
}



/*********************************************************************
 * @fn      BAT_MON_ProcessEvent
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
uint16 BAT_MON_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
    return (events ^ SYS_EVENT_MSG);

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Set timer for first periodic event
   //  VOID GAPRole_GetParameter(GAPROLE_BD_ADDR, baddr0);
    osal_start_timerEx( BAT_MON_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    // Start the Device
    VOID GAPRole_StartDevice( &BAT_MON_BroadcasterCBs );
   
    

    return ( events ^ SBP_START_DEVICE_EVT );
  }
  
  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
      osal_start_timerEx( BAT_MON_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    // Perform periodic application task
    performPeriodicTask();
    return (events ^ SBP_PERIODIC_EVT);
  }  
  // Discard unknown events
  return 0;
}




