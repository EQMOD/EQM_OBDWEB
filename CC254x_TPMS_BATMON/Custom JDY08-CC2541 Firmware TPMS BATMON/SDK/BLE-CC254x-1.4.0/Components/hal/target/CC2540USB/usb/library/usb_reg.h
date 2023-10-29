/***********************************************************************************

    Filename:     usb_reg.h

    Description:  Register bit defintions for CCxx11 and CC2531.

***********************************************************************************/

#ifndef USBREG_H
#define USBREG_H


// USBADDR
#define USBADDR_UPDATE              0x80
#define USBADDR_USBADDR             0x7F

// USBPOW
#define USBPOW_ISO_WAIT_SOF         0x80
#define USBPOW_RST                  0x08
#define USBPOW_RESUME               0x04
#define USBPOW_SUSPEND              0x02
#define USBPOW_SUSPEND_EN           0x01

// USBIIF
#define USBIIF_INEP5IF              0x20
#define USBIIF_INEP4IF              0x10
#define USBIIF_INEP3IF              0x08
#define USBIIF_INEP2IF              0x04
#define USBIIF_INEP1IF              0x02
#define USBIIF_EP0IF                0x01

// USBOIF
#define USBOIF_OUTEP5IF             0x20
#define USBOIF_OUTEP4IF             0x10
#define USBOIF_OUTEP3IF             0x08
#define USBOIF_OUTEP2IF             0x04
#define USBOIF_OUTEP1IF             0x02

// USBCIF
#define USBCIF_SOFIF                0x08
#define USBCIF_RSTIF                0x04
#define USBCIF_RESUMEIF             0x02
#define USBCIF_SUSPENDIF            0x01

// USBIIE
#define USBIIE_INEP5IE              0x20
#define USBIIE_INEP4IE              0x10
#define USBIIE_INEP3IE              0x08
#define USBIIE_INEP2IE              0x04
#define USBIIE_INEP1IE              0x02
#define USBIIE_EP0IE                0x01

// USBOIE
#define USBOIE_OUTEP5IE             0x20
#define USBOIE_OUTEP4IE             0x10
#define USBOIE_OUTEP3IE             0x08
#define USBOIE_OUTEP2IE             0x04
#define USBOIE_OUTEP1IE             0x02

// USBCIE
#define USBCIE_SOFIE                 0x08
#define USBCIE_RSTIE                 0x04
#define USBCIE_RESUMEIE              0x02
#define USBCIE_SUSPENDIE             0x01

// USBCS0
#define USBCS0_CLR_SETUP_END         0x80
#define USBCS0_CLR_OUTPKT_RDY        0x40
#define USBCS0_SEND_STALL            0x20
#define USBCS0_SETUP_END             0x10
#define USBCS0_DATA_END              0x08
#define USBCS0_SENT_STALL            0x04
#define USBCS0_INPKT_RDY             0x02
#define USBCS0_OUTPKT_RDY            0x01

// USBCSIL
#define USBCSIL_CLR_DATA_TOG         0x40
#define USBCSIL_SENT_STALL           0x20
#define USBCSIL_SEND_STALL           0x10
#define USBCSIL_FLUSH_PACKET         0x08
#define USBCSIL_UNDERRUN             0x04
#define USBCSIL_PKT_PRESENT          0x02
#define USBCSIL_INPKT_RDY            0x01

// USBCSIH
#define USBCSIH_AUTOSET              0x80
#define USBCSIH_ISO                  0x40
#define USBCSIH_FORCE_DATA_TOG       0x08
#define USBCSIH_IN_DBL_BUF           0x01

// USBCSOL
#define USBCSOL_CLR_DATA_TOG         0x80
#define USBCSOL_SENT_STALL           0x40
#define USBCSOL_SEND_STALL           0x20
#define USBCSOL_FLUSH_PACKET         0x10
#define USBCSOL_DATA_ERROR           0x08
#define USBCSOL_OVERRUN              0x04
#define USBCSOL_FIFO_FULL            0x02
#define USBCSOL_OUTPKT_RDY           0x01

// USBCSOH
#define USBCSOH_AUTOCLEAR            0x80
#define USBCSOH_ISO                  0x40
#define USBCSOH_OUT_DBL_BUF          0x01

#define SLEEP_USB_EN                 0x80

/*
+------------------------------------------------------------------------------
|  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.
|
|  IMPORTANT: Your use of this Software is limited to those specific rights
|  granted under the terms of a software license agreement between the user who
|  downloaded the software, his/her employer (which must be your employer) and
|  Texas Instruments Incorporated (the "License"). You may not use this Software
|  unless you agree to abide by the terms of the License. The License limits
|  your use, and you acknowledge, that the Software may not be modified, copied
|  or distributed unless embedded on a Texas Instruments microcontroller or used
|  solely and exclusively in conjunction with a Texas Instruments radio
|  frequency transceiver, which is integrated into your product. Other than for
|  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
|  works of, modify, distribute, perform, display or sell this Software and/or
|  its documentation for any purpose.
|
|  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
|  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
|  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
|  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
|  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
|  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
|  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING
|  BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
|  CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
|  SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
|  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
|
|  Should you have any questions regarding your right to use this Software,
|  contact Texas Instruments Incorporated at www.TI.com.
|
+------------------------------------------------------------------------------
*/
#endif
