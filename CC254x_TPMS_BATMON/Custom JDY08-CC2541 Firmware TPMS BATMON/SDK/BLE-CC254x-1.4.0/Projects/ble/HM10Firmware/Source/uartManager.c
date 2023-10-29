/**************************************************************************************************
  Filename:       uartManager.c
  Description:    handles command send over the serial interface
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OSAL_centralBroadcaster.h"
#include "centralBroadcaster.h"

#include "hal_led.h"
#include "hal_uart.h"
#include "hal_flash.h"
#include "uartManager.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define BAUD_RATE                  HAL_UART_BR_9600
#define POLL_INTERVAL               50
#define READ_TRIES_MAX              10

// command opcodes and payloads
#define OPCODE_NULL                     0  //invalid opcode
#define OPCODE_TEST                     1  //just for testing - gets ACK back
#define OPCODE_SET_ADVERTISING_DATA     2  //changes advertising data
#define OPCODE_SET_NAME                 3  //changes ibeacon name
#define OPCODE_GET_FIRMWARE_INFO_STRING 4  //returns the firmware info string
#define OPCODE_START                    5  //starts scanning and advertising
#define OPCODE_STOP                     6  //stops
#define OPCODE_MAX                      7  //smallest invalid opcode number

#define PAYLOAD_LEN_UNKNOWN  0xFF
static uint8 payloadLengths[] = {
  0,                                              //NULL
  0,                                              //TEST
  PAYLOAD_LEN_UNKNOWN,                            //SET_ADVERTISING_DATA
  PAYLOAD_LEN_UNKNOWN,                            //SET_NAME
  0,                                              //GET_FIRMWARE_INFO_STRING
  2,                                              //START
  0,                                              //STOP
};

// responses
#define RESPONSE_DEVICE_FOUND          0x02  //used for sending results of device discovery
#define RESPONSE_ACK                   0x40
#define RESPONSE_NCK                   0x23

// events
#define UART_START_DEVICE_EVT 0x01
#define UART_PERIODIC_EVT     0x02

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 uartManagerTaskID;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void uartManager_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void HalUARTCback (uint8 port, uint8 event);
static void execute_command();
static void send_ack();
static void send_nck();

/*********************************************************************
 * LOCAL VARIABLES
 */
static halUARTCfg_t config =
{
  .configured = TRUE,
  .baudRate = BAUD_RATE,
  .flowControl = FALSE,
  .idleTimeout = 100,
  .rx = { .maxBufSize = 255 },
  .tx = { .maxBufSize = 255 },
  .intEnable = TRUE,
  .callBackFunc = HalUARTCback
};

//pahses
enum
{
  PHASE_OPCODE,  //waiting for opcode
  PHASE_LENGTH,  //waiting for length of playload
  PHASE_PAYLOAD  //waiting for playload data itself
};

static uint8 phase = PHASE_OPCODE;
static uint8 remainingPayloadLength, currentPayloadLength;
static uint8 opcode = OPCODE_NULL, payloadLength;
static uint8 remainingTries;
static uint8 payloadData[255];

static uint8 firmwareInfoString[] = "Firmware v1.1\nInfo: https://github.com/bjoerke/HM-10-Firmware";

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      uartManager_Init
 *
 * @brief   TODO
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void uartManager_Init( uint8 task_id )
{
  uartManagerTaskID = task_id;
  osal_set_event(uartManagerTaskID, UART_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      uartManager_ProcessEvent
 *
 * @brief   This function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 uartManager_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    if ( (pMsg = osal_msg_receive(uartManagerTaskID )) != NULL )
    {
      uartManager_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      VOID osal_msg_deallocate( pMsg ); //Release the OSAL message
    }
    return (events ^ SYS_EVENT_MSG);  //return unprocessed events
  }
  
  if ( events & UART_START_DEVICE_EVT )
  {
    HalUARTOpen(HAL_UART_PORT_1, &config);
    osal_start_reload_timer(uartManagerTaskID, UART_PERIODIC_EVT, POLL_INTERVAL);
    return ( events ^ UART_START_DEVICE_EVT );
  }
  
  if ( events & UART_PERIODIC_EVT)
  {
    if(phase != PHASE_OPCODE && --remainingTries == 0)
    {
      send_nck();  // took too long
      phase = PHASE_OPCODE;
    }
    if(Hal_UART_RxBufLen(HAL_UART_PORT_1) > 0)
    {
      // PHASE: OPCODE
      if(phase == PHASE_OPCODE)
      {
         HalUARTRead(HAL_UART_PORT_1, &opcode, 1);
         if(opcode == OPCODE_NULL || opcode >= OPCODE_MAX)
         {
           //invalid opcode
           send_nck();
         }
         else
         {
           uint8 length = payloadLengths[opcode];
           switch(length)
           {
            case 0:
              execute_command();
              break;
            case PAYLOAD_LEN_UNKNOWN:
              phase = PHASE_LENGTH;
              remainingTries = READ_TRIES_MAX;
              break;
            default:
              phase = PHASE_PAYLOAD;
              payloadLength = length;
              remainingPayloadLength = length;         
              currentPayloadLength = 0;
              remainingTries = READ_TRIES_MAX;
              break;
           }
         }
      }
      
      // PHASE: LENGTH
      if(phase == PHASE_LENGTH)
      {
        if(Hal_UART_RxBufLen(HAL_UART_PORT_1) >= 1)
        {       
          HalUARTRead(HAL_UART_PORT_1, &payloadLength, 1);
          remainingPayloadLength = payloadLength;         
          currentPayloadLength = 0;
          remainingTries = READ_TRIES_MAX;
          phase = PHASE_PAYLOAD;
        }
      }
      
      // PHASE: PAYLOAD
      if(phase == PHASE_PAYLOAD)
      {
        uint8 bytesRead = HalUARTRead(HAL_UART_PORT_1, &payloadData[currentPayloadLength], remainingPayloadLength);
        currentPayloadLength += bytesRead;
        remainingPayloadLength -= bytesRead;
        if(remainingPayloadLength == 0)
        {
          execute_command();
          phase = PHASE_OPCODE;
        }
      }
    }
    return ( events ^ UART_PERIODIC_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      uartManager_ProcessOSALMsg
 * @brief   Process an incoming task message.
 * @param   pMsg - message to process
 * @return  none
 */
static void uartManager_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case CB_MSG_DEVICE_FOUND:
    {
      deviceFoundMsg_t* msg = (deviceFoundMsg_t*) pMsg;
      msg->event = RESPONSE_DEVICE_FOUND;
      HalUARTWrite(HAL_UART_PORT_1, (uint8*) msg, msg->length + sizeof(uint8) + sizeof(uint8) );
      break;
    }
  }
}

/***************************
 * @fn       execute_command()
 * @breif    executes the command send over serial
 */
static void execute_command()
{
  switch(opcode)
  {
  case OPCODE_TEST:
    send_ack();
    break;
  case OPCODE_SET_ADVERTISING_DATA:
    {
      if(payloadLength > ADVERTISING_DATA_LEN_MAX)
      {
        send_nck();
        break;
      }
      setAdvertisingDataMsg_t* msg = (setAdvertisingDataMsg_t*) osal_msg_allocate(sizeof(setAdvertisingDataMsg_t));
      msg->event = CB_MSG_SET_ADVERTISING_DATA;
      msg->length = payloadLength;
      osal_memcpy(
                  &msg->data,
                  &payloadData,
                  payloadLength
                );
      osal_msg_send(simpleBLETaskId, (uint8*) msg);
      send_ack();
      break;
    }
  case OPCODE_SET_NAME:
    {
      if(payloadLength > NAME_LENGTH_MAX)
      {
        send_nck();
        break;
      }
      setNameMsg_t* msg = (setNameMsg_t*) osal_msg_allocate(sizeof(setNameMsg_t));
      msg->event = CB_MSG_SET_NAME;
      msg->length = payloadLength;
      osal_memcpy(&msg->name, &payloadData, payloadLength);
      osal_msg_send(simpleBLETaskId, (uint8*) msg);
      send_ack();
      break;
    }      
  case OPCODE_GET_FIRMWARE_INFO_STRING:
    {
       uint8 length = osal_strlen((char*) firmwareInfoString);
       HalUARTWrite(HAL_UART_PORT_1, &length, 1);
       HalUARTWrite(HAL_UART_PORT_1, firmwareInfoString, length);
    }
     break;
  case OPCODE_START:
    osal_set_event( simpleBLETaskId, CB_START_DEVICE_EVT );
    send_ack();
    break;
  case OPCODE_STOP:
    osal_set_event( simpleBLETaskId, CB_STOP_DEVICE_EVT );
    send_ack();
    break;
  }
}

/**
 * @brief      UART event callback
 */
static void HalUARTCback(uint8 port, uint8 event)
{
  switch(event)
  {
    case HAL_UART_RX_FULL:
    case HAL_UART_RX_ABOUT_FULL:
    case HAL_UART_RX_TIMEOUT:
    case HAL_UART_TX_FULL:
    case HAL_UART_TX_EMPTY:
      break;
  }
}

static void send_ack()
{
  uint8 ack = RESPONSE_ACK;
  HalUARTWrite(HAL_UART_PORT_1, &ack, 1);
}

static void send_nck()
{
  uint8 nck = RESPONSE_NCK;
  HalUARTWrite(HAL_UART_PORT_1, &nck, 1);
}

/** 
#define FLASH_PAGE_SIZE             2048
//This could be added in future versions
static void command_flash()
{
  while(Hal_UART_RxBufLen(HAL_UART_PORT_1) == 0); //wait for first data
  HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
  HalFlashErase(0);
  uint8 current_page = 0;
  uint32 current_byte = 0;
  uint8 buf[4];
  
  //start writing to flash
  while(TRUE)
  {
    while(Hal_UART_RxBufLen(HAL_UART_PORT_1) < 4) ;  //wait for at least 4 bytes
    HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE);
    if(current_byte == FLASH_PAGE_SIZE)
    {
      current_page++;
      HalFlashErase(current_page);
      current_byte=0;
    }
    HalUARTRead(HAL_UART_PORT_1, buf, 4);
    HalFlashWrite(current_byte / 4, buf, 1); 
    current_byte += 4;
  }
}
**/

/*********************************************************************
*********************************************************************/
