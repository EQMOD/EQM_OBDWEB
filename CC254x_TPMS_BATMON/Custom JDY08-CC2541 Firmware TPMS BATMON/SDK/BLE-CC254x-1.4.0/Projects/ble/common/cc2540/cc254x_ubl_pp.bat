:: /**********************************************************************************************
::   Filename:       cc254x_ubl_pp.bat
::   Revised:        $Date:$
::   Revision:       $Revision:$
:: 
::   Description:
::
::   This is a launcher for the cc254x_ubl_pp.js script which does post-processing on IAR output
::   files; it is designed to be invoked from IAR project, Build Actions, Post-build command line.
::
::   There are 3 or 5 required arguments when invoking this batch file:
::
::   %1 must be the IAR project path: "$PROJ_DIR$"
::
::   %2 directs the action of the post-processing script as follows:
::
::   "ProdUBL" generates a binary application image from the IAR output "simple-code", .sim, file;
::   this binary is used for downloading via the UBL.
::
::   "ProdHex" generates a hex image that is the combination of the serial boot loader
::   with the application image; this combination hex file is for mass production programming
::   via a tool like SmartRF Programmer.
::
::   %3 must be the path and file name of the IAR output, like this:
::   "$PROJ_DIR$\CC2540F256_UBL\Exe\rnp_cc2540f256_ubl"
::   OR
::   "$PROJ_DIR$\CC2540F256_HEX\Exe\rnp_cc2540f256"
::
::   %4 (Required for "ProdHex") must be the path and file name of the IAR output UBL, like this:
::   "$PROJ_DIR$\..\..\SerialBoot\CC254x\CC2540F256_UART0_HEX\Exe\ubl_cc2540f256_uart0.hex"
::
::   %5 (Required for "ProdHex") the number of lines in the UBL hex file to pre-pend to the
::   application image (the remainder of the UBL hex file will be appended to the application).
::   Currently UBL is only using the first page to intercept the IVEC's, so 128 + 1 = 129 to
::   include the hex file header line (which is stripped from application image when appending it
::   to this root area of the UBL).
::
::   Note that the above serial boot loader corresponding to the RPC transport of the application
::   build target (e.g. CC2540F256, USART0) must have been built prior to this batch file execution.
::   This is accomplished with the 'Pre-build command line' option invoking the cc254x_ubl_pre.bat
::   with the corresponding arguments as specified in that batch file.
::
:: 
::   Copyright 2011 Texas Instruments Incorporated. All rights reserved.
:: 
::   IMPORTANT: Your use of this Software is limited to those specific rights
::   granted under the terms of a software license agreement between the user
::   who downloaded the software, his/her employer (which must be your employer)
::   and Texas Instruments Incorporated (the "License").  You may not use this
::   Software unless you agree to abide by the terms of the License. The License
::   limits your use, and you acknowledge, that the Software may not be modified,
::   copied or distributed unless embedded on a Texas Instruments microcontroller
::   or used solely and exclusively in conjunction with a Texas Instruments radio
::   frequency transceiver, which is integrated into your product.  Other than for
::   the foregoing purpose, you may not use, reproduce, copy, prepare derivative
::   works of, modify, distribute, perform, display or sell this Software and/or
::   its documentation for any purpose.
:: 
::   YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
::   PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
::   INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
::   NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
::   TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
::   NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
::   LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
::   INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
::   OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCU::
::   OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
::   (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
:: 
::   Should you have any questions regarding your right to use this Software,
::   contact Texas Instruments Incorporated at www.TI.com.
:: **********************************************************************************************/

@echo off
chdir %1\..\..\common\cc2540
start cc254x_ubl_pp.js %2 %3 %4 %5

