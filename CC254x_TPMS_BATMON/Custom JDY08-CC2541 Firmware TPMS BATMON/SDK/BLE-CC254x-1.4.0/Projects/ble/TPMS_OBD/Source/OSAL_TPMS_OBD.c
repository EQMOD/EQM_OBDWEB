/**************************************************************************************************
  Filename:       OSAL_TPMS_OBD.c
  Revised:        $Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
  Revision:       $Revision: 12 $

  Description:    OSAL task initalization for Simple BLE Observer app.


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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* HAL */
#include "hal_drivers.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbTimer.h"
#endif

/* L2CAP */

/* gap */
#include "gap.h"

/* GATT */

/* Profiles */
#include "observer.h"

/* Application */
#include "TPMS_OBD.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,
//  Hal_ProcessEvent,
  HCI_ProcessEvent,
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),
#endif
  //L2CAP_ProcessEvent,
  GAP_ProcessEvent,
  //GATT_ProcessEvent,
  //SM_ProcessEvent,
  GAPObserverRole_ProcessEvent,
  //GAPBondMgr_ProcessEvent,
  //GATTServApp_ProcessEvent,
  TPMS_OBD_ProcessEvent
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  uint8 taskID = 0;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  //char tbuf[30];
  //sprintf(tbuf,"LL INit ... \n");
 // Uart1SendString(tbuf,strlen(tbuf));

  /* LL Task */
  LL_Init( taskID++ );

 // sprintf(tbuf,"Hal Init ... \n");
 // Uart1SendString(tbuf,strlen(tbuf));

  /* Hal Task */
//  Hal_Init( taskID++ );

 // sprintf(tbuf,"HCI Init ... \n");
 // Uart1SendString(tbuf,strlen(tbuf));

  /* HCI Task */
  HCI_Init( taskID++ );

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

  /* L2CAP Task */
  //L2CAP_Init( taskID++ );

 // sprintf(tbuf,"Gap Init ... \n");
 // Uart1SendString(tbuf,strlen(tbuf));

  /* GAP Task */
  GAP_Init( taskID++ );

  /* GATT Task */
  //GATT_Init( taskID++ );

  /* SM Task */
  //SM_Init( taskID++ );

 // sprintf(tbuf,"Gap Observer Role Init ... \n");
//  Uart1SendString(tbuf,strlen(tbuf));
  /* Profiles */
  GAPObserverRole_Init( taskID++ );
  //GAPBondMgr_Init( taskID++ );

  //GATTServApp_Init( taskID++ );

  /* Application */
  TPMS_OBD_Init( taskID );
}

/*********************************************************************
*********************************************************************/
