/**************************************************************************************************
  Filename:       oad_target.c
  Revised:        $Date: 2012-11-16 18:39:26 -0800 (Fri, 16 Nov 2012) $
  Revision:       $Revision: 32218 $

  Description:    This file contains OAD Target implementation.


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

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "hal_aes.h"
#include "hal_crc.h"
#include "hal_flash.h"
#include "hal_dma.h"
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
#include "hal_lcd.h"
#endif
#include "hal_types.h"
#include "linkdb.h"
#include "oad.h"
#include "oad_target.h"
#include "OSAL.h"

/*********************************************************************
 * CONSTANTS
 */

#define OAD_FLASH_PAGE_MULT  ((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))

#if defined (FEATURE_OAD_SECURE) && defined (HAL_IMAGE_A)
  // Enabled to ONLY build a BOOTSTRAP Encrypted Image-A (for programming over
  // BEM, not BIM). Comment line below to build a non-bootstrap Encrypted Image-A.
  #define BOOTP_E_IMAGE_A
#endif

#if !defined (OAD_IMAGE_VERSION)
  #define OAD_IMAGE_VERSION    0x0000
#endif

#if !defined (OAD_IMAGE_A_USER_ID)
  #define OAD_IMAGE_A_USER_ID  {'A', 'A', 'A', 'A'}
#endif

#if !defined (OAD_IMAGE_B_USER_ID)
  #define OAD_IMAGE_B_USER_ID  {'B', 'B', 'B', 'B'}
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// OAD Service UUID
static CONST uint8 oadServUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128( OAD_SERVICE_UUID )
};

static CONST uint8 oadCharUUID[OAD_CHAR_CNT][ATT_UUID_SIZE] =
{
 // OAD Image Identify UUID
 TI_BASE_UUID_128( OAD_IMG_IDENTIFY_UUID ),

 // OAD Image Block Request/Response UUID
 TI_BASE_UUID_128( OAD_IMG_BLOCK_UUID )
};

/*********************************************************************
 * Profile Attributes - variables
 */

// OAD Service attribute
static CONST gattAttrType_t oadService = { ATT_UUID_SIZE, oadServUUID };

// Place holders for the GATT Server App to be able to lookup handles.
static uint8 oadCharVals[OAD_CHAR_CNT];

// OAD Characteristic Properties
static uint8 oadCharProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// OAD Client Characteristic Configs
static gattCharCfg_t oadImgIdentifyConfig[GATT_MAX_NUM_CONN];
static gattCharCfg_t oadImgBlockConfig[GATT_MAX_NUM_CONN];

// OAD Characteristic user descriptions
static CONST uint8 oadImgIdentifyDesc[] = "Img Identify";
static CONST uint8 oadImgBlockDesc[] = "Img Block";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t oadAttrTbl[] =
{
  // OAD Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&oadService
  },

    // OAD Image Identify Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &oadCharProps
    },

      // OAD Image Identify Characteristic Value
      {
        { ATT_UUID_SIZE, oadCharUUID[0] },
        GATT_PERMIT_WRITE,
        0,
        oadCharVals+0
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)oadImgIdentifyConfig
      },

      // OAD Image Identify User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)oadImgIdentifyDesc
      },

    // OAD Image Block Request/Response Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &oadCharProps
    },

      // OAD Image Block Request/Response Characteristic Value
      {
        { ATT_UUID_SIZE, oadCharUUID[1] },
        GATT_PERMIT_WRITE,
        0,
        oadCharVals+1
      },

       // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)oadImgBlockConfig
      },

      // OAD Image Block Request/Response User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)oadImgBlockDesc
      }
};

#pragma location="IMAGE_HEADER"
const __code img_hdr_t _imgHdr = {
#if defined FEATURE_OAD_SECURE
  2012,                       // CRC must not be 0x0000 or 0xFFFF.
#endif
#if defined (BOOTP_E_IMAGE_A)
#warning "Forcing a CRC-shadow match with the BOOTP_E_IMAGE_A flag - is this bootstrap code?"
  2012,                       // CRC-shadow forced to match CRC for a bootstrap Encrypted Image-A
#else
  0xFFFF,                     // CRC-shadow must be 0xFFFF for everything else
#endif
  OAD_IMG_VER( OAD_IMAGE_VERSION ), // 15-bit Version #, left-shifted 1; OR with Image-B/Not-A bit.
  OAD_IMG_R_AREA * OAD_FLASH_PAGE_MULT,
#if defined HAL_IMAGE_A
  OAD_IMAGE_A_USER_ID,        // User-Id
#else
  OAD_IMAGE_B_USER_ID,        // User-Id
#endif
  { 0xFF, 0xFF, 0xFF, 0xFF }  // Reserved
};
#pragma required=_imgHdr

#pragma location="AES_HEADER"
static const __code aes_hdr_t _aesHdr = {
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
 { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B },  // Dummy Nonce
 { 0xFF, 0xFF, 0xFF, 0xFF }   // Spare
};
#pragma required=_aesHdr

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint16 oadBlkNum = 0, oadBlkTot = 0xFFFF;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 oadReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                           uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen);

static bStatus_t oadWriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint8 len, uint16 offset);

CONST gattServiceCBs_t oadCBs =
{
  oadReadAttrCB,  // Read callback function pointer.
  oadWriteAttrCB, // Write callback function pointer.
  NULL            // Authorization callback function pointer.
};

static void oadImgBlockReq(uint16 connHandle, uint16 blkNum);

static void oadImgIdentifyReq(uint16 connHandle, img_hdr_t *pImgHdr);

static bStatus_t oadImgIdentifyWrite( uint16 connHandle, uint8 *pValue );

static bStatus_t oadImgBlockWrite( uint16 connHandle, uint8 *pValue );

static void oadHandleConnStatusCB( uint16 connHandle, uint8 changeType );

#if !defined FEATURE_OAD_SECURE
static void DMAExecCrc(uint8 page, uint16 offset, uint16 len);
static uint8 checkDL(void);
#endif

/*********************************************************************
 * @fn      OADTarget_AddService
 *
 * @brief   Initializes the OAD Service by registering GATT attributes
 *          with the GATT server. Only call this function once.
 *
 * @return  The return value of GATTServApp_RegisterForMsg().
 */
bStatus_t OADTarget_AddService(void)
{
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, oadImgIdentifyConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, oadImgBlockConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( oadHandleConnStatusCB );

  return GATTServApp_RegisterService(oadAttrTbl, GATT_NUM_ATTRS(oadAttrTbl), &oadCBs);
}

/*********************************************************************
 * @fn      oadReadAttrCB
 *
 * @brief   Read an attribute.
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be read
 * @param   pLen - length of data to be read
 * @param   offset - offset of the first octet to be read
 * @param   maxLen - maximum length of data to be read
 *
 * @return  Success or Failure
 */
static uint8 oadReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                           uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen)
{
  bStatus_t status = SUCCESS;

  // TBD: is there any use for supporting reads
  *pLen = 0;
  status = ATT_ERR_INVALID_HANDLE;

  return status;
}

/*********************************************************************
 * @fn      oadWriteAttrCB
 *
 * @brief   Validate and Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t oadWriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint8 len, uint16 offset)
{
  bStatus_t status = SUCCESS;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    if ( uuid == GATT_CLIENT_CHAR_CFG_UUID)
    {
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND; // Should never get here!
    }
  }
  else
  {
    // 128-bit UUID
    if (osal_memcmp(pAttr->type.uuid, oadCharUUID[OAD_CHAR_IMG_IDENTIFY], ATT_UUID_SIZE))
    {
      status = oadImgIdentifyWrite( connHandle, pValue );
    }
    else if (osal_memcmp(pAttr->type.uuid, oadCharUUID[OAD_CHAR_IMG_BLOCK], ATT_UUID_SIZE))
    {
      status = oadImgBlockWrite( connHandle, pValue );
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND; // Should never get here!
    }
  }

  return status;
}

/*********************************************************************
 * @fn      oadImgIdentifyWrite
 *
 * @brief   Process the Image Identify Write.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to data to be written
 *
 * @return  status
 */
static bStatus_t oadImgIdentifyWrite( uint16 connHandle, uint8 *pValue )
{
  img_hdr_t rxHdr;
  img_hdr_t ImgHdr;

  rxHdr.ver = BUILD_UINT16( pValue[0], pValue[1] );
  rxHdr.len = BUILD_UINT16( pValue[2], pValue[3] );

  (void)osal_memcpy(rxHdr.uid, pValue+4, sizeof(rxHdr.uid));

  HalFlashRead(OAD_IMG_R_PAGE, OAD_IMG_HDR_OSET, (uint8 *)&ImgHdr, sizeof(img_hdr_t));

  oadBlkTot = rxHdr.len / (OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE);

  if ( (OAD_IMG_ID( ImgHdr.ver ) != OAD_IMG_ID( rxHdr.ver )) && // TBD: add customer criteria for initiating OAD here.
       (oadBlkTot <= OAD_BLOCK_MAX) &&
       (oadBlkTot != 0) )
  {
    oadBlkNum = 0;
    oadImgBlockReq(connHandle, 0);
  }
  else
  {
    oadImgIdentifyReq(connHandle, &ImgHdr);
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      oadImgBlockWrite
 *
 * @brief   Process the Image Block Write.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to data to be written
 *
 * @return  status
 */
static bStatus_t oadImgBlockWrite( uint16 connHandle, uint8 *pValue )
{
  uint16 blkNum = BUILD_UINT16( pValue[0], pValue[1] );

  // make sure this is the image we're expecting
  if ( blkNum == 0 )
  {
    img_hdr_t ImgHdr;
    uint16 ver = BUILD_UINT16( pValue[6], pValue[7] );
    uint16 blkTot = BUILD_UINT16( pValue[8], pValue[9] ) / (OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE);

    HalFlashRead(OAD_IMG_R_PAGE, OAD_IMG_HDR_OSET, (uint8 *)&ImgHdr, sizeof(img_hdr_t));

    if ( ( oadBlkNum != blkNum ) ||
         ( oadBlkTot != blkTot ) ||
         ( OAD_IMG_ID( ImgHdr.ver ) == OAD_IMG_ID( ver ) ) )
    {
      return ( ATT_ERR_WRITE_NOT_PERMITTED );
    }
  }

  if (oadBlkNum == blkNum)
  {
    uint16 addr = oadBlkNum * (OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE) +
                              (OAD_IMG_D_PAGE * OAD_FLASH_PAGE_MULT);
    oadBlkNum++;

#if defined FEATURE_OAD_SECURE
    if (blkNum == 0)
    {
      // Stop attack with crc0==crc1 by forcing crc1=0xffff.
      pValue[4] = 0xFF;
      pValue[5] = 0xFF;
    }
#endif

#if defined HAL_IMAGE_B
    // Skip the Image-B area which lies between the lower & upper Image-A parts.
    if (addr >= (OAD_IMG_B_PAGE * OAD_FLASH_PAGE_MULT))
    {
      addr += OAD_IMG_B_AREA * OAD_FLASH_PAGE_MULT;
    }
#endif
    if ((addr % OAD_FLASH_PAGE_MULT) == 0)
    {
      HalFlashErase(addr / OAD_FLASH_PAGE_MULT);
    }

    HalFlashWrite(addr, pValue+2, (OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE));
  }

  if (oadBlkNum == oadBlkTot)  // If the OAD Image is complete.
  {
#if defined FEATURE_OAD_SECURE
    HAL_SYSTEM_RESET();  // Only the secure OAD boot loader has the security key to decrypt.
#else
    if (checkDL())
    {
#if !defined HAL_IMAGE_A
      // The BIM always checks for a valid Image-B before Image-A,
      // so Image-A never has to invalidate itself.
      uint16 crc[2] = { 0x0000, 0xFFFF };
      uint16 addr = OAD_IMG_R_PAGE * OAD_FLASH_PAGE_MULT + OAD_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;
      HalFlashWrite(addr, (uint8 *)crc, 1);
#endif
      HAL_SYSTEM_RESET();
    }
#endif
  }
  else  // Request the next OAD Image block.
  {
    oadImgBlockReq(connHandle, oadBlkNum);
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      oadImgIdentifyReq
 *
 * @brief   Process the Image Identify Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pImgHdr - Pointer to the img_hdr_t data to send.
 *
 * @return  None
 */
static void oadImgBlockReq(uint16 connHandle, uint16 blkNum)
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, oadImgBlockConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    attHandleValueNoti_t noti;
    gattAttribute_t *pAttr = GATTServApp_FindAttr(oadAttrTbl, GATT_NUM_ATTRS(oadAttrTbl),
                                                  oadCharVals+OAD_CHAR_IMG_BLOCK);
    noti.handle = pAttr->handle;
    noti.len = 2;
    noti.value[0] = LO_UINT16(blkNum);
    noti.value[1] = HI_UINT16(blkNum);

    VOID GATT_Notification(connHandle, &noti, FALSE);
  }
}

/*********************************************************************
 * @fn      oadImgIdentifyReq
 *
 * @brief   Process the Image Identify Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pImgHdr - Pointer to the img_hdr_t data to send.
 *
 * @return  None
 */
static void oadImgIdentifyReq(uint16 connHandle, img_hdr_t *pImgHdr)
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, oadImgIdentifyConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    attHandleValueNoti_t noti;
    gattAttribute_t *pAttr = GATTServApp_FindAttr(oadAttrTbl, GATT_NUM_ATTRS(oadAttrTbl),
                                                  oadCharVals+OAD_CHAR_IMG_IDENTIFY);
    noti.handle = pAttr->handle;
    noti.len = OAD_IMG_HDR_SIZE;
    noti.value[0] = LO_UINT16(pImgHdr->ver);
    noti.value[1] = HI_UINT16(pImgHdr->ver);

    noti.value[2] = LO_UINT16(pImgHdr->len);
    noti.value[3] = HI_UINT16(pImgHdr->len);

    (void)osal_memcpy(noti.value+4, pImgHdr->uid, sizeof(pImgHdr->uid));

    VOID GATT_Notification(connHandle, &noti, FALSE);
  }
}

#if !defined FEATURE_OAD_SECURE

#if 0
/**************************************************************************************************
 * @fn          crcCalcDL
 *
 * @brief       Run the CRC16 Polynomial calculation over the DL image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 crcCalcDL(void)
{
  uint8 pageEnd = oadBlkTot / OAD_BLOCKS_PER_PAGE;
  uint16 osetEnd = (oadBlkTot - (pageEnd * OAD_BLOCKS_PER_PAGE)) * OAD_BLOCK_SIZE;

#if defined HAL_IMAGE_B
  pageEnd += OAD_IMG_D_PAGE + OAD_IMG_B_AREA;
#else
  pageEnd += OAD_IMG_D_PAGE;
#endif

  HalCRCInit(0x0000);  // Seed thd CRC calculation with zero.

  for (uint8 page = OAD_IMG_D_PAGE; ; page++)
  {
#if defined HAL_IMAGE_B
    // Skip the Image-B area which lies between the lower & upper Image-A parts.
    if (page == OAD_IMG_B_PAGE)
    {
      page += OAD_IMG_B_AREA;
    }
#endif

    for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; oset += HAL_FLASH_WORD_SIZE)
    {
      if ((page == OAD_IMG_D_PAGE) && (oset == OAD_IMG_CRC_OSET))
      {
        continue;  // Skip the CRC and shadow.
      }
      else if ((page == pageEnd) && (oset == osetEnd))
      {
        return HalCRCCalc();
      }
      else
      {
        uint8 buf[HAL_FLASH_WORD_SIZE];
        HalFlashRead(page, oset, buf, HAL_FLASH_WORD_SIZE);

        for (uint8 idx = 0; idx < HAL_FLASH_WORD_SIZE; idx++)
        {
          HalCRCExec(buf[idx]);
        }
      }
    }
  }
}
#endif

/**************************************************************************************************
 * @fn          crcCalcDLDMA
 *
 * @brief       Run the CRC16 Polynomial calculation over the DL image,
 *              using DMA to read the flash memory into the CRC register
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 crcCalcDLDMA(void)
{
  uint8 pageBeg = OAD_IMG_D_PAGE;
  uint8 pageEnd = oadBlkTot / OAD_BLOCKS_PER_PAGE;

#if defined HAL_IMAGE_B
  pageEnd += OAD_IMG_D_PAGE + OAD_IMG_B_AREA;
#else
  pageEnd += OAD_IMG_D_PAGE;
#endif

  HalCRCInit(0x0000);  // Seed thd CRC calculation with zero.

  // Handle first page differently to skip CRC and CRC shadow when calculating
  DMAExecCrc(pageBeg, 4, HAL_FLASH_PAGE_SIZE-4);

  // Do remaining pages
  for (uint8 pg = pageBeg + 1; pg < pageEnd; pg++)
  {
#if defined HAL_IMAGE_B  // Means we are receiving ImgA, so skip ImgB pages
    if (pg == OAD_IMG_B_PAGE)
    {
      pg += OAD_IMG_B_AREA;
    }
#endif
    
    DMAExecCrc(pg, 0, HAL_FLASH_PAGE_SIZE);
  }
  
  return HalCRCCalc();
}

/**************************************************************************************************
 * @fn          DMAExecCrc
 *
 * @brief       This function assumes CRC has been initialized and sets up and
 *              starts a dma tranfer from a flash page to the CRC HW module.
 *
 * @note        This function uses NV DMA Ch; Ch0
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
  halDMADesc_t *dmaCh0_p = HAL_DMA_GET_DESC0();
  
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
 * @fn          checkDL
 *
 * @brief       Check validity of the downloaded image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE for image valid.
 **************************************************************************************************
 */
static uint8 checkDL(void)
{
  uint16 crc[2];

  HalFlashRead(OAD_IMG_D_PAGE, OAD_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));

  if ((crc[0] == 0xFFFF) || (crc[0] == 0x0000))
  {
    return FALSE;
  }

  if (crc[1] == 0xFFFF)
  {
    //P0DIR |= 1;
    //P0_0 = 0;
    //P0_0 = 1;
    //P0_0 = 0;
    //P0_0 = 1;
    //P0_0 = 0;
    //P0_0 = 1;
    crc[1] = crcCalcDLDMA();
    //P0_0 = 0;

#if defined FEATURE_OAD_BIM  // If download image is made to run in-place, enable it here.
    uint16 addr = OAD_IMG_D_PAGE * OAD_FLASH_PAGE_MULT + OAD_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;
    crc[0] = 0xFFFF;
    HalFlashWrite(addr, (uint8 *)crc, 1);
    HalFlashRead(OAD_IMG_D_PAGE, OAD_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));
#endif
  }

  return (crc[0] == crc[1]);
}
#endif // !FEATURE_OAD_SECURE

/*********************************************************************
 * @fn          oadHandleConnStatusCB
 *
 * @brief       OAD Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void oadHandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, oadImgIdentifyConfig );
      GATTServApp_InitCharCfg( connHandle, oadImgBlockConfig );
    }
  }
}

/*********************************************************************
*********************************************************************/
