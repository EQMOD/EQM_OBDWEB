/**************************************************************************************************
  Filename:       TPMS_OBD_Main.c
  Revised:        $Date: 2011-02-24 15:48:00 -0800 (Thu, 24 Feb 2011) $
  Revision:       $Revision: 11 $

  Description:    This file contains the main and callback functions for
                  the Simple BLE Observer sample application.


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
 *                                           Includes
 **************************************************************************************************/

/* Hal Drivers */

#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */

#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"


 /***************************************************************************************************/

void HalDriverInit2 (void)
{

  /* ADC */

#if (defined HAL_ADC) && (HAL_ADC == TRUE)
  //HalAdcInit();
#endif

  /* DMA */

//#if (defined HAL_DMA) && (HAL_DMA == TRUE)
  // Must be called before the init call to any module that uses DMA.
 // HalDmaInit();
//#endif

  /* AES */

//#if (defined HAL_AES) && (HAL_AES == TRUE)
//  HalAesInit();
//#endif


  /* UART */
//#if (defined HAL_UART) && (HAL_UART == TRUE)
 // HalUARTInit();
//#endif

  /* KEY */
//#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  //HalKeyInit();
//#endif

  /* SPI */

//#if (defined HAL_SPI) && (HAL_SPI == TRUE)
 // HalSpiInit();
//
//#endif

  /* HID */

//#if (defined HAL_HID) && (HAL_HID == TRUE)
 // usbHidInit();
//#endif

}

/**************************************************************************************************

 * FUNCTIONS

 **************************************************************************************************/



/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */

int main(void)
{

  /* Initialze the HAL driver */

  //HalDriverInit2();

  /* Initialize NV system */

  osal_snv_init();

  /* Initialize LL */

  /* Initialize the operating system */

  osal_init_system();

  /* Enable interrupts */

  //HAL_ENABLE_INTERRUPTS();

  // Final board initialization

  //InitBoard( OB_READY );

  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
  
  /* Start OSAL */
  osal_start_system(); // No Return from here

  return 0;
}



