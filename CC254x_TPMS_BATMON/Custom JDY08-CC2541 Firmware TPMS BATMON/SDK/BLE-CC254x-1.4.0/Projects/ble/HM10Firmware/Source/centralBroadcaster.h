/**************************************************************************************************
  Filename:       ibeacon.h
  Description:    
**************************************************************************************************/

#ifndef _CENTRALBROAD_H
#define _CENTRALBROAD_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * CONSTANTS
 */

//the name is part of the scan response which cannot exceed 31 bytes
#define NAME_LENGTH_MAX           20 
#define ADVERTISING_DATA_LEN_MAX  31
#define B_ADDR_LEN                6    //48 bit bluetooth address
  
//Events
#define CB_INIT_DEVICE_EVT             0x0001
#define CB_START_DEVICE_EVT            0x0002
#define CB_STOP_DEVICE_EVT             0x0004

//Messages
#define CB_MSG_SET_ADVERTISING_DATA 0xA1
#define CB_MSG_SET_NAME             0xA2
#define CB_MSG_DEVICE_FOUND         0xA3

/*********************************************************************
 * TYPES
 */
typedef struct setAdvertisingDataMsg
{
  uint8 event;  //will be IB_MSG_SET_ADVERTISING_DATA
  uint8 length;
  uint8 data[ADVERTISING_DATA_LEN_MAX];
}setAdvertisingDataMsg_t;

typedef struct setNameMsg
{
  uint8 event;  //will be IB_MSG_SET_NAME
  uint8 length;
  uint8 name[NAME_LENGTH_MAX];
}setNameMsg_t;

typedef struct deviceFoundMsg
{
  uint8 event;  //will be IB_MSG_DEVICE_FOUND
  uint8 length;
  uint8 address[B_ADDR_LEN];
  uint8 rssi;
  uint8 advData[ADVERTISING_DATA_LEN_MAX];
}deviceFoundMsg_t;

/*********************************************************************
 * MACROS
 */
 
/*********************************************************************
 * FUNCTIONS
 */
extern void SimpleBLECentral_Init( uint8 task_id );
extern uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events );

/*************************************************************
 * VARIABLES
 */
extern uint8 simpleBLETaskId;

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
