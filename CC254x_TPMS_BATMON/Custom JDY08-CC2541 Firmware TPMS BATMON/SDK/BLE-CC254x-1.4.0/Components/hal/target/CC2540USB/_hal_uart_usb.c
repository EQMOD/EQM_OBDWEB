/**************************************************************************************************
  Filename:       _hal_uart_usb.c
  Revised:        $Date: 2012-08-17 16:36:33 -0700 (Fri, 17 Aug 2012) $
  Revision:       $Revision: 31296 $

  Description: This file contains the interface to the H/W UART driver by USB.


  Copyright 2009-2012 Texas Instruments Incorporated. All rights reserved.

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

#include "hal_uart.h"

#include "usb_board_cfg.h"
#include "usb_cdc.h"
#include "usb_cdc_hooks.h"
#include "usb_firmware_library_config.h"
#include "usb_firmware_library_headers.h"

/*********************************************************************
 * MACROS
 */

#if !defined HAL_UART_BAUD_RATE
#define HAL_UART_BAUD_RATE  115200
#endif

// The timeout tick is at 32-kHz, so multiply msecs by 33.
#define HAL_UART_MSECS_TO_TICKS    33

#if !defined HAL_UART_USB_HIGH
#define HAL_UART_USB_HIGH         (256 / 2 - 16)
#endif
#if !defined HAL_UART_USB_IDLE
#define HAL_UART_USB_IDLE         (0 * HAL_UART_MSECS_TO_TICKS)
#endif

// Max USB packet size, per specification; see also usb_cdc_descriptor.s51
#define HAL_UART_USB_TX_MAX        64

/***********************************************************************************
 * EXTERNAL VARIABLES
 */

/***********************************************************************************
 * GLOBAL VARIABLES
 */

/***********************************************************************************
 * LOCAL DATA
 */

__no_init static uint8 halUartRxQ[256];
__no_init static uint8 halUartTxQ[256];

static uint8 halUartRxH, halUartRxT;
static uint8 halUartTxH, halUartTxT;

#if !defined HAL_SB_BOOT_CODE
static uint8 rxTick;
static uint8 rxShdw;
static uint8 usbTxMT;
static halUARTCBack_t usbCB;
#endif

/***********************************************************************************
 * LOCAL FUNCTIONS
 */

static void HalUARTInitUSB(void);
static void HalUARTPollUSB(void);
static void halUartPollEvt(void);
static void halUartPollRx(void);
static void halUartPollTx(void);

/******************************************************************************
 * FUNCTIONS
 */

/***********************************************************************************
* @fn           usbUartInitUSB
*
* @brief        USB UART init function.
*               - Set initial line decoding to 8/NONE/1.
*               - Initialise the USB Firmware Library and the USB controller.
*
* @param        none
*
* @return       none
*/
void HalUARTInitUSB(void)
{
  // Set default line coding.
  currentLineCoding.dteRate = HAL_UART_BAUD_RATE;
  currentLineCoding.charFormat = CDC_CHAR_FORMAT_1_STOP_BIT;
  currentLineCoding.parityType = CDC_PARITY_TYPE_NONE;
  currentLineCoding.dataBits = 8;

  // Init USB library
  usbfwInit();

  // Initialize the USB interrupt handler with bit mask containing all processed USBIRQ events
  usbirqInit(0xFFFF);

#if defined HAL_SB_BOOT_CODE
  HAL_USB_PULLUP_ENABLE();  // Enable pullup on D+.
#endif
}

#if !defined HAL_SB_BOOT_CODE
/******************************************************************************
 * @fn      HalUARTOpenUSB
 *
 * @brief   Open a port according tp the configuration specified by parameter.
 *
 * @param   config - contains configuration information
 *
 * @return  none
 *****************************************************************************/
static void HalUARTOpenUSB(halUARTCfg_t *config)
{
  usbCB = config->callBackFunc;
  HAL_USB_PULLUP_ENABLE();  // Enable pullup on D+
}
#endif

/***********************************************************************************
* @fn           HalUartPollUSB
*
* @brief        The USB UART main task function. Should be called from the OSAL main loop.
*
* @param        none
*
* @return       none
*/
void HalUARTPollUSB(void)
{
#if defined HAL_SB_BOOT_CODE
  while (USBIF)  usbirqHandler();
#endif
  halUartPollEvt();
  halUartPollRx();
  halUartPollTx();
}

uint8 HalUARTRx(uint8 *buf, uint8 max);
uint8 HalUARTRx(uint8 *buf, uint8 max)
{
  uint8 cnt = 0;

  while ((halUartRxH != halUartRxT) && (cnt < max))
  {
    *buf++ = halUartRxQ[halUartRxH++];
    cnt++;
  }

  return cnt;
}

uint8 HalUARTTx(uint8 *buf, uint8 cnt);
uint8 HalUARTTx(uint8 *buf, uint8 cnt)
{
  while (cnt--)
  {
    halUartTxQ[halUartTxT++] = *buf++;
  }

#if !defined HAL_SB_BOOT_CODE
  usbTxMT = FALSE;
#endif
  return 1;
}

/**************************************************************************************************
 * @fn      HalUARTRxAvailUSB()
 *
 * @brief   Calculate Rx Buffer length - the number of bytes in the buffer.
 *
 * @param   none
 *
 * @return  length of current Rx Buffer
 **************************************************************************************************/
static uint16 HalUARTRxAvailUSB(void)
{
  return ((halUartRxT >= halUartRxH)?
          halUartRxT - halUartRxH : sizeof(halUartRxQ) - halUartRxH + halUartRxT);
}

/***********************************************************************************
* @fn           halUartPollEvt
*
* @brief        Poll for USB events which are not directly related to the UART.
*
* @param        none
*
* @return       none
*/
static void halUartPollEvt(void)
{
  // Handle reset signaling on the bus
  if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESET)
  {
    USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESET);
    usbfwResetHandler();
  }

  // Handle packets on EP0
  if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SETUP)
  {
    USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SETUP);
    usbfwSetupHandler();
  }

  // Handle USB suspend
  if (USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SUSPEND)
  {
    // Clear USB suspend interrupt
    USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SUSPEND);

#if HAL_UART_USB_SUSPEND
    // Take the chip into PM1 until a USB resume is deteceted.
    usbsuspEnter();
#endif

    // Running again; first clear RESUME interrupt
    USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESUME);
  }
}

/***********************************************************************************
* @fn           halUartPollRx
*
* @brief        Poll for data from USB.
*
* @param        none
*
* @return       none
*/
static void halUartPollRx(void)
{
  uint8 cnt;
  uint8 ep = USBFW_GET_SELECTED_ENDPOINT();
  USBFW_SELECT_ENDPOINT(4);

  // If the OUT endpoint has received a complete packet.
  if (USBFW_OUT_ENDPOINT_DISARMED())
  {
    halIntState_t intState;

    HAL_ENTER_CRITICAL_SECTION(intState);
    // Get length of USB packet, this operation must not be interrupted.
    cnt = USBFW_GET_OUT_ENDPOINT_COUNT_LOW();
    cnt += USBFW_GET_OUT_ENDPOINT_COUNT_HIGH() >> 8;
    HAL_EXIT_CRITICAL_SECTION(intState);

    while (cnt--)
    {
      halUartRxQ[halUartRxT++] = USBF4;
    }
    USBFW_ARM_OUT_ENDPOINT();

#if !defined HAL_SB_BOOT_CODE
    // If the USB has transferred in more Rx bytes, reset the Rx idle timer.

    // Re-sync the shadow on any 1st byte(s) received.
    if (rxTick == 0)
    {
      rxShdw = ST0;
    }
    rxTick = HAL_UART_USB_IDLE;
#endif
  }
#if !defined HAL_SB_BOOT_CODE
  else if (rxTick)
  {
    // Use the LSB of the sleep timer (ST0 must be read first anyway).
    uint8 decr = ST0 - rxShdw;

    if (rxTick > decr)
    {
      rxTick -= decr;
      rxShdw = ST0;
    }
    else
    {
      rxTick = 0;
    }
  }

  {
    uint8 evt = 0;
    cnt = halUartRxT - halUartRxH;

    if (cnt >= HAL_UART_USB_HIGH)
    {
      evt = HAL_UART_RX_ABOUT_FULL;
    }
    else if (cnt && !rxTick)
    {
      evt = HAL_UART_RX_TIMEOUT;
    }

    if (evt && (NULL != usbCB))
    {
      usbCB(0, evt);
    }
  }
#endif

  USBFW_SELECT_ENDPOINT(ep);
}

/***********************************************************************************
* @fn           halUartPollTx
*
* @brief        Poll for data to USB.
*
* @param        none
*
* @return       none
*/
static void halUartPollTx(void)
{
  uint8 ep = USBFW_GET_SELECTED_ENDPOINT();
  USBFW_SELECT_ENDPOINT(4);

  // If the IN endpoint is ready to accept data.
  if (USBFW_IN_ENDPOINT_DISARMED())
  {
    if (halUartTxT == halUartTxH)
    {
#if !defined HAL_SB_BOOT_CODE
      if (!usbTxMT && usbCB)
      {
        usbTxMT = TRUE;
        usbCB(0, HAL_UART_TX_EMPTY);
      }
#endif
    }
    else
    {
      uint8 max = HAL_UART_USB_TX_MAX;

      do
      {
        USBF4 = halUartTxQ[halUartTxH++];
      } while ((halUartTxH != halUartTxT) && (0 != --max));

      USBFW_ARM_IN_ENDPOINT();
    }
  }

  USBFW_SELECT_ENDPOINT(ep);
}

/******************************************************************************
******************************************************************************/
