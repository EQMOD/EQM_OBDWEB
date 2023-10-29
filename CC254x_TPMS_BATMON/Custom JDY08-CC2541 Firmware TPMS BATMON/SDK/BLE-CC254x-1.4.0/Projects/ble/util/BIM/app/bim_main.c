/**************************************************************************************************
  Filename:       bim_main.c
  Revised:        $Date: 2013-08-21 15:13:01 -0700 (Wed, 21 Aug 2013) $
  Revision:       $Revision: 35060 $

  Description:

  This module contains the definitions for the main functionality of an Boot Image Manager.


  Copyright 2012 - 2013 Texas Instruments Incorporated. All rights reserved.

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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_dma.h"
#include "hal_flash.h"
#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define BIM_IMG_A_PAGE        1
#define BIM_IMG_A_AREA        62

#define BIM_IMG_B_PAGE        8
#define BIM_IMG_B_AREA       (124 - BIM_IMG_A_AREA)

#define BIM_CRC_OSET          0x00
#define BIM_HDR_OSET          0x00

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef struct {
  // Secure OAD uses the Signature for image validation instead of calculating a CRC, but the use
  // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
  uint16 crc0;       // CRC must not be 0x0000 or 0xFFFF.
  uint16 crc1;       // CRC-shadow must be 0xFFFF.
  // User-defined Image Version Number - default logic uses simple a '<' comparison to start an OAD.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} img_hdr_t;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init halDMADesc_t dmaCh0;  // Locally setup for use by HalFlashWrite().

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init uint8 pgBuf[HAL_FLASH_PAGE_SIZE];

__no_init __data uint8 JumpToImageAorB @ 0x09;

#pragma location = "ALIGNED_CODE"
void halSleepExec(void);

void DMAExecCrc(uint8 page, uint16 offset, uint16 len);

/**************************************************************************************************
 * @fn          halSleepExec
 *
 * @brief       This function puts the CC254x to sleep by writing to the PCON register.
 *              The instruction after writing to PCON must not be 4-byte aligned or excessive
 *              power consumption may result. Since the write to PCON is 3 instructions, this
 *              function is forced to be even-byte aligned. Thus, this function must not have any
 *              automatic variables and the write to PCON must be the first C statement.
 *              See the linker file ".xcl" for actual placement of this function.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
#pragma optimize=none
void halSleepExec(void)
{
  PCON = 0x01;
  ASM_NOP;
}

/**************************************************************************************************
 * @fn          crcCalcDMA
 *
 * @brief       Run the CRC16 Polynomial calculation over the image specified,
 *              using DMA to read the flash memory into the CRC register
 *
 * input parameters
 *
 * @param       page - Flash page on which to beging the CRC calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 crcCalcDMA(uint8 page)
{
  uint16 crc;
  uint8 pageBeg;
  uint8 pageEnd;
  const img_hdr_t *pImgHdr;
  
  HalFlashRead(page, 0, pgBuf, HAL_FLASH_PAGE_SIZE);

  pImgHdr = (const img_hdr_t *)(pgBuf + BIM_HDR_OSET);

  pageBeg = page;
  pageEnd = pImgHdr->len / (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE);

  // One page is used for BIM, so we move this image's last page forward
  pageEnd += pageBeg;
  
  // If image A, set last page to be ImgA size + ImgB size
  if (pageBeg == BIM_IMG_A_PAGE)
  {
    pageEnd += BIM_IMG_B_AREA;
  }

  ADCCON1 &= 0xF3;  // CRC configuration of LRSR.

  // CRC seed of 0x0000.
  RNDL = 0x00;
  RNDL = 0x00;

  // Handle first page differently to skip CRC and CRC shadow when calculating
  DMAExecCrc(pageBeg, 4, HAL_FLASH_PAGE_SIZE-4);
  
  // Do remaining pages
  for (uint8 pg = pageBeg + 1; pg < pageEnd; pg++)
  {
    if (pg == BIM_IMG_B_PAGE)
    {
      pg += BIM_IMG_B_AREA;
    }
     
    DMAExecCrc(pg, 0, HAL_FLASH_PAGE_SIZE);
  }
  
  crc = RNDH;
  crc = (crc << 8) | RNDL;

  return crc;
}

#if 0
/**************************************************************************************************
 * @fn          crcCalc
 *
 * @brief       Run the CRC16 Polynomial calculation over the image specified.
 *
 * input parameters
 *
 * @param       page - Flash page on which to beging the CRC calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 crcCalc(uint8 page)
{
  HalFlashRead(page, 0, pgBuf, HAL_FLASH_PAGE_SIZE);

  const img_hdr_t *pImgHdr = (const img_hdr_t *)(pgBuf + BIM_HDR_OSET);

  uint8 pageBeg = page;
  uint8 pageEnd = pImgHdr->len / (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE);
  uint16 osetEnd = (pImgHdr->len - (pageEnd * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)))
                                                                   * HAL_FLASH_WORD_SIZE;
  pageEnd += pageBeg;
  if (pageBeg == BIM_IMG_A_PAGE)
  {
    pageEnd += BIM_IMG_B_AREA;
  }

  ADCCON1 &= 0xF3;  // CRC configuration of LRSR.

  // CRC seed of 0x0000.
  RNDL = 0x00;
  RNDL = 0x00;

  while(1)
  {
    for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; oset++)
    {
      if ((page == pageBeg) && (oset == BIM_CRC_OSET))
      {
        oset += 3;  // Skip the CRC and shadow.
      }
      else if ((page == pageEnd) && (oset == osetEnd))
      {
        uint16 crc = RNDH;
        crc = (crc << 8) | RNDL;

        return crc;
      }
      else
      {
        RNDH = pgBuf[oset];
      }
    }

    if (++page == BIM_IMG_B_PAGE)
    {
      page += BIM_IMG_B_AREA;
    }
    HalFlashRead(page, 0, pgBuf, HAL_FLASH_PAGE_SIZE);
  }
}
#endif

/**************************************************************************************************
 * @fn          crcCheck
 *
 * @brief       Calculate the image CRC and set it ready-to-run if it is good.
 *
 * input parameters
 *
 * @param       page - Flash page on which to beging the CRC calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      None, but no return from this function if the CRC check is good.
 **************************************************************************************************
 */
static void crcCheck(uint8 page, uint16 *crc)
{
  HAL_BOARD_INIT();

  /* This is in place of calling HalDmaInit() which would require init of the other 4 DMA
   * descriptors in addition to just Channel 0.
   */
  //P0DIR |= 1;
  //P0_0 = 0;
  //P0_0 = 1;
  //P0_0 = 0;
  //P0_0 = 1;
  //P0_0 = 0;
  //P0_0 = 1;
  if (crc[0] == crcCalcDMA(page))
  {
    //P0_0 = 0;
    uint16 addr = page * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE) +
                                 BIM_CRC_OSET / HAL_FLASH_WORD_SIZE;
    crc[1] = crc[0];
    crc[0] = 0xFFFF;

    HAL_DMA_SET_ADDR_DESC0(&dmaCh0);
    HalFlashWrite(addr, (uint8 *)crc, 1);
    HAL_SYSTEM_RESET();
  }
}

/**************************************************************************************************
 * @fn          main
 *
 * @brief       C-code main function.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void main(void)
{
  uint16 crc[2];

  // Prefer to run Image-B over Image-A so that Image-A does not have to invalidate itself.
  HalFlashRead(BIM_IMG_B_PAGE, BIM_CRC_OSET, (uint8 *)crc, 4);

  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1])
    {
      JumpToImageAorB = 1;
      // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
      asm("LJMP 0x4030");
      HAL_SYSTEM_RESET();  // Should not get here.
    }
    /* This check is disruptive when an OAD process to Image-A is interrupted - this check must
     * complete before the still good Image-A is run.
    else if (crc[1] == 0xFFFF)  // If first run of an image that was physically downloaded.*/
    {
      crcCheck(BIM_IMG_B_PAGE, crc);
    }
    /**/
  }

  HalFlashRead(BIM_IMG_A_PAGE, BIM_CRC_OSET, (uint8 *)crc, 4);

  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1])
    {
      JumpToImageAorB = 0;
      // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
      asm("LJMP 0x0830");
      HAL_SYSTEM_RESET();  // Should not get here.
    }
    else if (crc[1] == 0xFFFF)  // If first run of an image that was physically downloaded.
    {
      crcCheck(BIM_IMG_A_PAGE, crc);
    }
  }

  SLEEPCMD |= 0x03;  // PM3, All clock oscillators off, voltage regulator off.
  halSleepExec();
  HAL_SYSTEM_RESET();  // Should not get here.
}

/**************************************************************************************************
 * @fn          DMAExecCrc
 *
 * @brief       This function assumes CRC has been initialized and sets up and
 *              starts a dma tranfer from a flash page to the CRC HW module.
 *
 * @note        This function assumes DMA channel 0 is available for use.
 *
 * input parameters
 *
 * @param       page - A valid flash page number.
 * @param       offset - A valid offset into the page.
 * @param       len - A valid number of bytes to calculate crc of.
 *
 * @return      None.
 **************************************************************************************************
 */
void DMAExecCrc(uint8 page, uint16 offset, uint16 len) {

  uint8 memctr = MEMCTR;  // Save to restore.
  
  // Calculate the offset into the containing flash bank as it gets mapped into XDATA.
  uint16 address = (offset + HAL_FLASH_PAGE_MAP) +
                   ((page % HAL_FLASH_PAGE_PER_BANK) * HAL_FLASH_PAGE_SIZE);

  // Pointer to DMA config structure
  halDMADesc_t *dmaCh0_p = &dmaCh0;
  
#if !defined HAL_OAD_BOOT_CODE
  halIntState_t is;
#endif

  page /= HAL_FLASH_PAGE_PER_BANK;  // Calculate the flash bank from the flash page.

#if !defined HAL_OAD_BOOT_CODE
  HAL_ENTER_CRITICAL_SECTION(is);
#endif
  
  // Calculate and map the containing flash bank into XDATA.
  MEMCTR = (MEMCTR & 0xF8) | page;  // page is actually bank
  
  // Start address for CRC calculation in the XDATA mapped flash bank
  HAL_DMA_SET_SOURCE(dmaCh0_p, address);
  
  // Destination for data transfer, RNDH mapped to XDATA
  HAL_DMA_SET_DEST(dmaCh0_p, 0x70BD);
  
  // One whole page (or len) at a time
  HAL_DMA_SET_LEN(dmaCh0_p, len);
  
  // 8-bit, block, no trigger
  HAL_DMA_SET_WORD_SIZE(dmaCh0_p, HAL_DMA_WORDSIZE_BYTE);
  HAL_DMA_SET_TRIG_MODE(dmaCh0_p, HAL_DMA_TMODE_BLOCK);
  HAL_DMA_SET_TRIG_SRC(dmaCh0_p, HAL_DMA_TRIG_NONE);
  
  // SRC += 1, DST = constant, no IRQ, all 8 bits, high priority
  HAL_DMA_SET_SRC_INC(dmaCh0_p, HAL_DMA_SRCINC_1);
  HAL_DMA_SET_DST_INC(dmaCh0_p, HAL_DMA_DSTINC_0);
  HAL_DMA_SET_IRQ(dmaCh0_p, HAL_DMA_IRQMASK_DISABLE);
  HAL_DMA_SET_M8(dmaCh0_p, HAL_DMA_M8_USE_8_BITS);
  HAL_DMA_SET_PRIORITY(dmaCh0_p, HAL_DMA_PRI_HIGH);
  
  // Tell DMA Controller where above configuration can be found
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);
  
  // Arm the DMA channel (0)
  HAL_DMA_ARM_CH(0);
  
  // 9 cycles wait
  asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  
  // Start DMA tranfer
  HAL_DMA_MAN_TRIGGER(0);
  
  // Wait for dma to finish.
  while(DMAREQ & 0x1);
  
  // Restore bank mapping
  MEMCTR = memctr;

#if !defined HAL_OAD_BOOT_CODE
  HAL_EXIT_CRITICAL_SECTION(is);
#endif
}


/**************************************************************************************************
*/
