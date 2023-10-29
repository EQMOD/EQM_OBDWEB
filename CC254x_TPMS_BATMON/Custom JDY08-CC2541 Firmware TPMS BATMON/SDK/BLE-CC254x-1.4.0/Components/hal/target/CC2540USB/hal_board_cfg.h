/*******************************************************************************
  Filename:       hal_board_cfg.h
  Revised:        $Date: 2013-09-13 12:00:00 -0700 (Fri, 13 Sep 2013) $
  Revision:       $Revision: 35310 $

  Description:

  Abstract board-specific registers, addresses, & initialization for H/W based on the
  Texas Instruments CC2540USB (8051 core).


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
*******************************************************************************/
#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H

#ifdef __cplusplus
extern "C"
{
#endif


/*******************************************************************************
 * INCLUDES
 */

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "usb_board_cfg.h"

/*******************************************************************************
 * CONSTANTS
 */

/* Board Identifier */

#define HAL_BOARD_CC2540USB

/* Clock Speed */

#define HAL_CPU_CLOCK_MHZ              32

/* Sleep Clock */

#define EXTERNAL_CRYSTAL_OSC           0x00  // external 32kHz XOSC
#define INTERNAL_RC_OSC                0x80  // internal 32kHz RCOSC

// For USB, assume internal, unless an external crystal is explicitly indicated.
#if !defined (XOSC32K_INSTALLED) || (defined (XOSC32K_INSTALLED) && (XOSC32K_INSTALLED == FALSE))
#define OSC_32KHZ                      INTERNAL_RC_OSC
#else
#define OSC_32KHZ                      EXTERNAL_CRYSTAL_OSC
#endif

// Minimum Time for Stable External 32kHz Clock (in ms)
#define MIN_TIME_TO_STABLE_32KHZ_XOSC 400

  /* LCD Max Chars and Buffer */
#define HAL_LCD_MAX_CHARS   16
#define HAL_LCD_MAX_BUFF    25

/* LED Configuration */

#define HAL_NUM_LEDS                   2

#define HAL_LED_BLINK_DELAY()          st( { volatile uint32 i; for (i=0; i<0x5800; i++) { }; } )

/* 1 - Green */
#define LED1_BV                        BV(0)
#define LED1_SBIT                      P0_0
#define LED1_DDR                       P0DIR
#define LED1_POLARITY                  ACTIVE_LOW

/* 2 - Red */
#define LED2_BV                        BV(1)
#define LED2_SBIT                      P1_1
#define LED2_DDR                       P1DIR
#define LED2_POLARITY                  ACTIVE_HIGH

/* Push Button Configuration */

#define ACTIVE_LOW                     !
#define ACTIVE_HIGH                    !!    /* double negation forces result to be '1' */

/* S1 */
#define PUSH1_BV                       BV(2)
#define PUSH1_SBIT                     P1_2
#define PUSH1_POLARITY                 ACTIVE_LOW

/* S2 */
#define PUSH2_BV                       BV(3)
#define PUSH2_SBIT                     P1_3
#define PUSH2_POLARITY                 ACTIVE_LOW

/* OSAL NV implemented by internal flash pages. */

// Flash is partitioned into 8 banks of 32 KB or 16 pages.
#define HAL_FLASH_PAGE_PER_BANK        16

// Flash is constructed of 128 pages of 2 KB.
#define HAL_FLASH_PAGE_SIZE            2048
#define HAL_FLASH_WORD_SIZE            4

// CODE banks get mapped into the XDATA range 8000-FFFF.
#define HAL_FLASH_PAGE_MAP             0x8000

// The last 16 bytes of the last available page are reserved for flash lock bits.
#define HAL_FLASH_LOCK_BITS            16

// NV page definitions must coincide with segment declaration in project *.xcl file.
#if defined NON_BANKED
#define HAL_NV_PAGE_END                30
//#elif defined FEATURE_UBL_MSD
//#define HAL_NV_PAGE_END                121
#elif defined BLEMINI
#define HAL_NV_PAGE_END                121  
#else
#define HAL_NV_PAGE_END                126
#endif

// Re-defining Z_EXTADDR_LEN here so as not to include a Z-Stack .h file.
#define HAL_FLASH_IEEE_SIZE            8
#define HAL_FLASH_IEEE_PAGE            (HAL_NV_PAGE_END+1)
#define HAL_FLASH_IEEE_OSET            (HAL_FLASH_PAGE_SIZE - HAL_FLASH_LOCK_BITS - HAL_FLASH_IEEE_SIZE)
#define HAL_INFOP_IEEE_OSET            0xC

#define HAL_NV_PAGE_CNT                2
#define HAL_NV_PAGE_BEG                (HAL_NV_PAGE_END-HAL_NV_PAGE_CNT+1)

// Used by DMA macros to shift 1 to create a mask for DMA registers.
#define HAL_NV_DMA_CH                  0
#define HAL_DMA_CH_RX                  3
#define HAL_DMA_CH_TX                  4

#define HAL_NV_DMA_GET_DESC()          HAL_DMA_GET_DESC0()
#define HAL_NV_DMA_SET_ADDR(a)         HAL_DMA_SET_ADDR_DESC0((a))

/* Reserving 1st 3 pages and last page for USB-CDC boot loader. */

#define HAL_USB_IMG_ADDR               0x1800
#define HAL_USB_CRC_ADDR               0x1890
// Size of internal flash less 4 pages for boot loader and 6 pages for NV.
#define HAL_USB_IMG_SIZE              (0x40000 - 0x2000 - 0x3000)

/*******************************************************************************
 * MACROS
 */

/* Cache Prefetch Control */

#define PREFETCH_ENABLE()  st( FCTL = 0x08; )
#define PREFETCH_DISABLE() st( FCTL = 0x04; )

/* Setting Clocks */

// switch to the 16MHz HSOSC and wait until it is stable
#define SET_OSC_TO_HSOSC()                                                     \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_16MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_16MHZ );                            \
}

// switch to the 32MHz XOSC and wait until it is stable
#define SET_OSC_TO_XOSC()                                                      \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_32MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_32MHZ );                            \
}

// set 32kHz OSC and wait until it is stable
#define SET_32KHZ_OSC()                                                        \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & ~0x80) | OSC_32KHZ;                                 \
  while ( (CLKCONSTA & 0x80) != OSC_32KHZ );                                   \
}

#define START_HSOSC_XOSC()                                                     \
{                                                                              \
  SLEEPCMD &= ~OSC_PD;            /* start 16MHz RCOSC & 32MHz XOSC */         \
  while (!(SLEEPSTA & XOSC_STB)); /* wait for stable 32MHz XOSC */             \
}

#define STOP_HSOSC()                                                           \
{                                                                              \
  SLEEPCMD |= OSC_PD;             /* stop 16MHz RCOSC */                       \
}

/* Board Initialization */
#if defined HAL_USB_BOOT_CODE
#define HAL_BOARD_INIT()                                                       \
{                                                                              \
  /* Set to 16Mhz to set 32kHz OSC, then back to 32MHz */                      \
  START_HSOSC_XOSC();                                                          \
  SET_OSC_TO_HSOSC();                                                          \
  SET_32KHZ_OSC();                                                             \
  SET_OSC_TO_XOSC();                                                           \
  STOP_HSOSC();                                                                \
                                                                               \
  /* Disconnect D+ signal to host. */                                          \
  HAL_USB_PULLUP_DISABLE();                                                    \
                                                                               \
  /* Enable cache prefetch mode. */                                            \
  PREFETCH_ENABLE();                                                           \
}

#else
#define HAL_BOARD_INIT()                                                       \
{                                                                              \
  /* Set to 16Mhz to set 32kHz OSC, then back to 32MHz */                      \
  START_HSOSC_XOSC();                                                          \
  SET_OSC_TO_HSOSC();                                                          \
  SET_32KHZ_OSC();                                                             \
  SET_OSC_TO_XOSC();                                                           \
  STOP_HSOSC();                                                                \
                                                                               \
  /* Enable cache prefetch mode. */                                            \
  PREFETCH_ENABLE();                                                           \
}
#endif

/* Debounce */

#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }

/* Push Buttons */

#define HAL_PUSH_BUTTON1()        (PUSH1_POLARITY (PUSH1_SBIT))
#define HAL_PUSH_BUTTON2()        (PUSH2_POLARITY (PUSH2_SBIT))
#define HAL_PUSH_BUTTON3()        (0)
#define HAL_PUSH_BUTTON4()        (0)
#define HAL_PUSH_BUTTON5()        (0)
#define HAL_PUSH_BUTTON6()        (0)

/* LED's */

#define HAL_TURN_OFF_LED1()       st( LED1_SBIT = LED1_POLARITY (0); )
#define HAL_TURN_OFF_LED2()       st( LED2_SBIT = LED2_POLARITY (0); )
#define HAL_TURN_OFF_LED3()       HAL_TURN_OFF_LED1()
#define HAL_TURN_OFF_LED4()       HAL_TURN_OFF_LED2()

#define HAL_TURN_ON_LED1()        st( LED1_SBIT = LED1_POLARITY (1); )
#define HAL_TURN_ON_LED2()        st( LED2_SBIT = LED2_POLARITY (1); )
#define HAL_TURN_ON_LED3()        HAL_TURN_ON_LED1()
#define HAL_TURN_ON_LED4()        HAL_TURN_ON_LED2()

#define HAL_TOGGLE_LED1()         st( if (LED1_SBIT) { LED1_SBIT = 0; } else { LED1_SBIT = 1;} )
#define HAL_TOGGLE_LED2()         st( if (LED2_SBIT) { LED2_SBIT = 0; } else { LED2_SBIT = 1;} )
#define HAL_TOGGLE_LED3()         HAL_TOGGLE_LED1()
#define HAL_TOGGLE_LED4()         HAL_TOGGLE_LED2()

#define HAL_STATE_LED1()          (LED1_POLARITY (LED1_SBIT))
#define HAL_STATE_LED2()          (LED2_POLARITY (LED2_SBIT))
#define HAL_STATE_LED3()          HAL_STATE_LED1()
#define HAL_STATE_LED4()          HAL_STATE_LED2()

/* Driver Configuration */

/* Set to TRUE enable H/W TIMER usage, FALSE disable it */
#ifndef HAL_TIMER
#define HAL_TIMER FALSE
#endif

/* Set to TRUE enable ADC usage, FALSE disable it */
#ifndef HAL_ADC
#define HAL_ADC TRUE
#endif

/* Set to TRUE enable DMA usage, FALSE disable it */
#ifndef HAL_DMA
#define HAL_DMA TRUE
#endif

/* Set to TRUE enable Flash access, FALSE disable it */
#ifndef HAL_FLASH
#define HAL_FLASH TRUE
#endif

/* Set to TRUE enable AES usage, FALSE disable it */
#ifndef HAL_AES
#define HAL_AES TRUE
#endif

#ifndef HAL_AES_DMA
#define HAL_AES_DMA TRUE
#endif

/* Set to TRUE enable LCD usage, FALSE disable it */
#ifndef HAL_LCD
#define HAL_LCD FALSE
#endif

/* Set to TRUE enable LED usage, FALSE disable it */
#ifndef HAL_LED
#define HAL_LED TRUE
#endif
#if (!defined BLINK_LEDS) && (HAL_LED == TRUE)
#define BLINK_LEDS
#endif

/* Set to TRUE enable KEY usage, FALSE disable it */
#ifndef HAL_KEY
#define HAL_KEY TRUE
#endif

/* Set to TRUE enable UART usage, FALSE disable it */
#ifndef HAL_UART
#define HAL_UART TRUE
#define HAL_UART_DMA  0
#define HAL_UART_ISR  0
#define HAL_UART_USB  1
#endif

#ifndef HAL_UART_DMA
#define HAL_UART_DMA 0
#endif
#ifndef HAL_UART_ISR
#define HAL_UART_ISR 0
#endif

#ifndef HAL_UART_USB
# if HAL_UART
#  define HAL_UART_USB 1
# else
#  define HAL_UART_USB 0
# endif
#endif

/* Set to TRUE enable HID usage, FALSE disable it */
#ifndef HAL_HID
#define HAL_HID FALSE
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_BOARD_CFG_H */

/*******************************************************************************
*/
