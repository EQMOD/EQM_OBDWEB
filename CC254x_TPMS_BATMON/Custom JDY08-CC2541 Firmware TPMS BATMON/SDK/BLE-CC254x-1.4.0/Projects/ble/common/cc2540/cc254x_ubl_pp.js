/**************************************************************************************************
  Filename:       cc254x_ubl_pp.js
  Revised:        $Date: 2010-08-23 12:24:40 -0700 (Mon, 23 Aug 2010) $
  Revision:       $Revision: 23475 $

  Description:

  This file is a JScript file that can be run by the Windows Script Host, which is installed by
  default on Windows XP SP2 and later. The script acts on the arguments to do post-processing for
  the UBL-enabled build targets.


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

var ForReading = 1;
var ForWriting = 2;

// void main(void)
{
  var ppType = WScript.Arguments(0);
  var fso = new ActiveXObject("Scripting.FileSystemObject");
  var ppPath = WScript.Arguments(1);
  var fin, fout;

  switch (ppType) {
  case "ProdUBL":
    fin = ppPath + ".sim"
    fout = ppPath + ".bin"
    fso.CopyFile(fin, "tmp.sim", true);
    var WshShell = new ActiveXObject("WScript.Shell");
    // Invoke and wait for the binary file conversion tool to finish.
    WshShell.Run("cc254x_sim2bin.exe tmp.sim tmp.bin", 8, true);
    fso.CopyFile("tmp.bin", fout, true);
    fso.DeleteFile("tmp.bin");
    fso.DeleteFile("tmp.sim");
    break;

  case "ProdHex":
    var ublFile = WScript.Arguments(2);
    var rootCnt = WScript.Arguments(3);
    var fubl = fso.OpenTextFile(ublFile, ForReading);
    fout = fso.CreateTextFile(ppPath + ".hex", true);

    // UBL is split between root (intercepting IVEC's) and the last or lock-bits page.
    while (rootCnt != 0)
    {
      var s = fubl.ReadLine();
      fout.WriteLine(s);
      rootCnt--;
    }

    fin = fso.OpenTextFile(ppPath + ".a51", ForReading)
    fin.ReadLine();  // Throw away the first line since appending to a valid .hex file.

    // Throw away the last two lines since a valid .hex file will be appended.
    var line = new Array(3);
    var rIdx = 2;
    var wIdx = 0;
    line[0] = fin.ReadLine();
    line[1] = fin.ReadLine();
    while (1)
    {
      fout.WriteLine(line[wIdx]);
      line[rIdx] = fin.ReadLine();
      if (fin.AtEndOfStream)
      {
        break;
      }
      rIdx = (rIdx+1) % 3;
      wIdx = (wIdx+1) % 3;
    }
    fin.Close();

    // Look to throw away the header line to the banked area of the UBL image.
    var s = fubl.ReadLine();
    if (s.substr(1,1) != "0")
    {
      fout.WriteLine(s);
    }

    while (!fubl.AtEndOfStream)
    {
      var s = fubl.ReadLine();
      fout.WriteLine(s);
    }

    fout.Close();
    fubl.Close();
    break;
  }
}

