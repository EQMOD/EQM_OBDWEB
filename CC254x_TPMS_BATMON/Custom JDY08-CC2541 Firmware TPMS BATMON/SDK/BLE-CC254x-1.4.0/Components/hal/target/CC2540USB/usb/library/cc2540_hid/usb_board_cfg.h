/**************************************************************************************************
    Filename:       usb_board_cfg.h
    Revised:        $Date: 2010-11-04 10:16:17 -0700 (Thu, 04 Nov 2010) $
    Revision:       $Revision: 24330 $

    Description:

    This file implements the Temperature/Voltage Sample Application.


    Copyright 2009-2013 Texas Instruments Incorporated. All rights reserved.

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
#ifndef USB_BOARD_CFG_H
#define USB_BOARD_CFG_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_board_cfg.h"
#include "hal_mcu.h"
#include "ioCC2540.h"
#include "usb_reg.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_BOARD_IO_USB_ENABLE_PORT   1   // USB pull-up enable
#define HAL_BOARD_IO_USB_ENABLE_PIN    0

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */
/* Check for lock of 32MHz XOSC and the source for 32K clock. */
#define HAL_BOARD_STABLE()  (CLKCONSTA == (CLKCONCMD_32MHZ | OSC_32KHZ))



#define HAL_USB_ENABLE()  st(  \
  while (!HAL_BOARD_STABLE()); \
  USBCTRL = USBCTRL_USB_EN | USBCTRL_PLL_EN; \
  while (!(USBCTRL & USBCTRL_PLL_LOCKED));   \
)

#define HAL_USB_RUN()  st(  \
  HAL_USB_PULLUP_ENABLE();  /* Enable pullup on D+. */\
  HAL_USB_INT_CLEAR();  \
  HAL_USB_INT_ENABLE(); \
)

#define HAL_USB_PLL_DISABLE()          st( USBCTRL &= ~USBCTRL_PLL_EN;  while (USBCTRL & USBCTRL_PLL_LOCKED); )
#define HAL_USB_DISABLE                st( USBCTRL = 0;                 while (USBCTRL & USBCTRL_PLL_LOCKED); )

#define HAL_USB_PULLUP_ENABLE() \
  MCU_IO_OUTPUT(HAL_BOARD_IO_USB_ENABLE_PORT, HAL_BOARD_IO_USB_ENABLE_PIN, 1)
#define HAL_USB_PULLUP_DISABLE() \
  MCU_IO_OUTPUT(HAL_BOARD_IO_USB_ENABLE_PORT, HAL_BOARD_IO_USB_ENABLE_PIN, 0)

#define HAL_USB_INT_ENABLE()           st( P2IEN|= 0x20; IEN2|= 0x02; )
#define HAL_USB_INT_DISABLE()          st( P2IEN&= ~0x20; )
#define HAL_USB_INT_CLEAR()            st( P2IFG= 0; P2IF= 0; )

#define HAL_USB_RESUME_INT_ENABLE()    st ( USBCIE |= 0x02; )
#define HAL_USB_RESUME_INT_DISABLE()   st ( USBCIE &= ~0x02; )

#define USBCTRL_PLL_LOCKED             0x80
#define USBCTRL_PLL_EN                 0x02
#define USBCTRL_USB_EN                 0x01

#define P2IFG_DPIF                     0x20

#define CC2540_IS_XOSC_STABLE()        (SLEEPSTA & XOSC_STB)

#define NOP()  asm("NOP")
#define MCU_IO_TRISTATE   1             // Used as "func" for the macros below
#define MCU_IO_PULLUP     2
#define MCU_IO_PULLDOWN   3

//-----------------------------------------------------------------------------
//  Macros for simple configuration of IO pins on TI LPW SoCs
//-----------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL(port, pin)   MCU_IO_PERIPHERAL_PREP(port, pin)
#define MCU_IO_INPUT(port, pin, func)  MCU_IO_INPUT_PREP(port, pin, func)
#define MCU_IO_OUTPUT(port, pin, val)  MCU_IO_OUTPUT_PREP(port, pin, val)
#define MCU_IO_SET(port, pin, val)     MCU_IO_SET_PREP(port, pin, val)
#define MCU_IO_SET_HIGH(port, pin)     MCU_IO_SET_HIGH_PREP(port, pin)
#define MCU_IO_SET_LOW(port, pin)      MCU_IO_SET_LOW_PREP(port, pin)
#define MCU_IO_TGL(port, pin)          MCU_IO_TGL_PREP(port, pin)
#define MCU_IO_GET(port, pin)          MCU_IO_GET_PREP(port, pin)

#define MCU_IO_DIR_INPUT(port, pin)    MCU_IO_DIR_INPUT_PREP(port, pin)
#define MCU_IO_DIR_OUTPUT(port, pin)   MCU_IO_DIR_OUTPUT_PREP(port, pin)

//----------------------------------------------------------------------------------
//  Macros for internal use (the macros above need a new round in the preprocessor)
//----------------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL_PREP(port, pin)   st( P##port##SEL |= BV(pin); )

#define MCU_IO_INPUT_PREP(port, pin, func)  st( P##port##SEL &= ~BV(pin); \
                                                P##port##DIR &= ~BV(pin); \
                                                switch (func) { \
                                                case MCU_IO_PULLUP: \
                                                    P##port##INP &= ~BV(pin); \
                                                    P2INP &= ~BV(port + 5); \
                                                    break; \
                                                case MCU_IO_PULLDOWN: \
                                                    P##port##INP &= ~BV(pin); \
                                                    P2INP |= BV(port + 5); \
                                                    break; \
                                                default: \
                                                    P##port##INP |= BV(pin); \
                                                    break; } )

#define MCU_IO_OUTPUT_PREP(port, pin, val)  st( P##port##SEL &= ~BV(pin); \
                                                P##port##_##pin## = val; \
                                                P##port##DIR |= BV(pin); )

#define MCU_IO_SET_HIGH_PREP(port, pin)     st( P##port##_##pin## = 1; )
#define MCU_IO_SET_LOW_PREP(port, pin)      st( P##port##_##pin## = 0; )

#define MCU_IO_SET_PREP(port, pin, val)     st( P##port##_##pin## = val; )
#define MCU_IO_TGL_PREP(port, pin)          st( P##port##_##pin## ^= 1; )
#define MCU_IO_GET_PREP(port, pin)          (P##port## & BV(pin))

#define MCU_IO_DIR_INPUT_PREP(port, pin)    st( P##port##DIR &= ~BV(pin); )
#define MCU_IO_DIR_OUTPUT_PREP(port, pin)   st( P##port##DIR |= BV(pin); )

#endif
/**************************************************************************************************
*/
