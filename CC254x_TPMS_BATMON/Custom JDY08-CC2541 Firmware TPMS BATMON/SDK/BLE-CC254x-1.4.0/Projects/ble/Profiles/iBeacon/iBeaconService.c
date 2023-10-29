/*

  Copyright (c) 2013 RedBearLab

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal 
  in the Software without restriction, including without limitation the rights 
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "iBeaconService.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define SERVAPP_NUM_ATTR_SUPPORTED        25

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// iBeacon Service UUID: 
CONST uint8 iBeaconServUUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_SERV_UUID), HI_UINT16(IBEACON_SERV_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 1 UUID: 
CONST uint8 iBeaconChar1UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR1_UUID), HI_UINT16(IBEACON_CHAR1_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 2 UUID: 
CONST uint8 iBeaconChar2UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR2_UUID), HI_UINT16(IBEACON_CHAR2_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 3 UUID:
CONST uint8 iBeaconChar3UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR3_UUID), HI_UINT16(IBEACON_CHAR3_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 4 UUID: 
CONST uint8 iBeaconChar4UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR4_UUID), HI_UINT16(IBEACON_CHAR4_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 5 UUID: 
CONST uint8 iBeaconChar5UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR5_UUID), HI_UINT16(IBEACON_CHAR5_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 6 UUID: 
CONST uint8 iBeaconChar6UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR6_UUID), HI_UINT16(IBEACON_CHAR6_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 7 UUID: 
CONST uint8 iBeaconChar7UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR7_UUID), HI_UINT16(IBEACON_CHAR7_UUID),
  UUID_BASE_HEAD,
};

// Characteristic 8 UUID: 
CONST uint8 iBeaconChar8UUID[ATT_UUID_SIZE] =
{ 
  UUID_BASE_TAIL,
  LO_UINT16(IBEACON_CHAR8_UUID), HI_UINT16(IBEACON_CHAR8_UUID),
  UUID_BASE_HEAD,
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static iBeaconCBs_t *iBeacon_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// iBeacon Service attribute
static CONST gattAttrType_t iBeaconService = { ATT_UUID_SIZE, iBeaconServUUID };

// Characteristic 1 Properties
static uint8 iBeaconChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 1 Value
static uint8 iBeaconChar1[] = {IBEACON_UUID};
// Characteristic 1 User Description
static uint8 iBeaconChar1UserDesp[17] = "Characteristic 1\0";

// Characteristic 2 Properties
static uint8 iBeaconChar2Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 2 Value
static uint8 iBeaconChar2[] = {0x00,0x00};
// Characteristic 2 User Description
static uint8 iBeaconChar2UserDesp[17] = "Characteristic 2\0";

// Characteristic 3 Properties
static uint8 iBeaconChar3Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 3 Value
static uint8 iBeaconChar3[] = {0x00,0x00};
// Characteristic 3 User Description
static uint8 iBeaconChar3UserDesp[17] = "Characteristic 3\0";

// Characteristic 4 Properties
static uint8 iBeaconChar4Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 4 Value
static uint8 iBeaconChar4[] = {0xC6};                                        
// Characteristic 4 User Description
static uint8 iBeaconChar4UserDesp[17] = "Characteristic 4\0";

// Characteristic 5 Properties
static uint8 iBeaconChar5Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 5 Value
static uint8 iBeaconChar5[] = {0x01};                                        
// Characteristic 5 User Description
static uint8 iBeaconChar5UserDesp[17] = "Characteristic 5\0";

// Characteristic 6 Properties
static uint8 iBeaconChar6Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 6 Value
static uint8 iBeaconChar6[] = {0x01, 0x90};                                       
// Characteristic 6 User Description
static uint8 iBeaconChar6UserDesp[17] = "Characteristic 6\0";

// Characteristic 7 Properties
static uint8 iBeaconChar7Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 7 Value
static uint8 iBeaconChar7[] = {0x02};                                       
// Characteristic 7 User Description
static uint8 iBeaconChar7UserDesp[17] = "Characteristic 7\0";

// Characteristic 8 Properties
static uint8 iBeaconChar8Props = GATT_PROP_READ;  
// Characteristic 8 Value
static uint8 iBeaconChar8[20] = {'M','i','n','i','B','e','a','c','o','n','_','2','0','1','4','0','1','2','1','\0'};                                         
// Characteristic 8 User Description
static uint8 iBeaconChar8UserDesp[17] = "Characteristic 8\0";

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t iBeaconAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // iBeacon Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&iBeaconService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_UUID_SIZE, iBeaconChar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar1 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar1UserDesp 
      },      

    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar2Props 
    },

      // Characteristic Value 2
      { 
        { ATT_UUID_SIZE, iBeaconChar2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar2 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar2UserDesp 
      },           
      
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar3Props 
    },

      // Characteristic Value 3
      { 
        { ATT_UUID_SIZE, iBeaconChar3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar3UserDesp 
      },

    // Characteristic 4 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar4Props 
    },

      // Characteristic Value 4
      { 
        { ATT_UUID_SIZE, iBeaconChar4UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar4 
      },
      
      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar4UserDesp 
      },

    // Characteristic 5 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar5Props 
    },

      // Characteristic Value 5
      { 
        { ATT_UUID_SIZE, iBeaconChar5UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar5 
      },
      
      // Characteristic 5 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar5UserDesp 
      },
      
    // Characteristic 6 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar6Props 
    },

      // Characteristic Value 6
      { 
        { ATT_UUID_SIZE, iBeaconChar6UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar6 
      },
      
      // Characteristic 6 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar6UserDesp 
      },
      
    // Characteristic 7 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar7Props 
    },

      // Characteristic Value 7
      { 
        { ATT_UUID_SIZE, iBeaconChar7UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar7 
      },
      
      // Characteristic 7 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar7UserDesp 
      },

    // Characteristic 8 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconChar8Props 
    },

      // Characteristic Value 8
      { 
        { ATT_UUID_SIZE, iBeaconChar8UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconChar8 
      },
      
      // Characteristic 8 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconChar8UserDesp 
      }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 iBeacon_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t iBeacon_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void iBeacon_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// iBeacon Service Callbacks
CONST gattServiceCBs_t iBeaconCBs =
{
  iBeacon_ReadAttrCB,  // Read callback function pointer
  iBeacon_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      iBeacon_AddService
 *
 * @brief   Initializes the iBeacon service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t iBeacon_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( iBeacon_HandleConnStatusCB );  
  
  if ( services & IBEACON_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( iBeaconAttrTbl, 
                                          GATT_NUM_ATTRS( iBeaconAttrTbl ),
                                          &iBeaconCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      iBeacon_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t iBeacon_RegisterAppCBs( iBeaconCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    iBeacon_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}  

/*********************************************************************
 * @fn      iBeacon_SetParameter
 *
 * @brief   Set a iBeacon parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t iBeacon_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case IBEACON_CHAR1:
      if ( len == IBEACON_CHAR1_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar1, value, IBEACON_CHAR1_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case IBEACON_CHAR2:
      if ( len == IBEACON_CHAR2_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar2, value, IBEACON_CHAR2_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case IBEACON_CHAR3:
      if ( len == IBEACON_CHAR3_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar3, value, IBEACON_CHAR3_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case IBEACON_CHAR4:
      if ( len == IBEACON_CHAR4_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar4, value, IBEACON_CHAR4_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case IBEACON_CHAR5:
      if ( len == IBEACON_CHAR5_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar5, value, IBEACON_CHAR5_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case IBEACON_CHAR6:
      if ( len == IBEACON_CHAR6_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar6, value, IBEACON_CHAR6_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case IBEACON_CHAR7:
      if ( len == IBEACON_CHAR7_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar7, value, IBEACON_CHAR7_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
     
    case IBEACON_CHAR8:
      if ( len == IBEACON_CHAR8_LEN ) 
      {
        VOID osal_memcpy( iBeaconChar8, value, IBEACON_CHAR8_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      iBeacon_GetParameter
 *
 * @brief   Get a iBeacon parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t iBeacon_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case IBEACON_CHAR1:
      VOID osal_memcpy( value, iBeaconChar1, IBEACON_CHAR1_LEN );
      break;

    case IBEACON_CHAR2:
      VOID osal_memcpy( value, iBeaconChar2, IBEACON_CHAR2_LEN );
      break;      

    case IBEACON_CHAR3:
      VOID osal_memcpy( value, iBeaconChar3, IBEACON_CHAR3_LEN );
      break;  

    case IBEACON_CHAR4:
      VOID osal_memcpy( value, iBeaconChar4, IBEACON_CHAR4_LEN );
      break;
      
    case IBEACON_CHAR5:
      VOID osal_memcpy( value, iBeaconChar5, IBEACON_CHAR5_LEN );
      break;
      
    case IBEACON_CHAR6:
      VOID osal_memcpy( value, iBeaconChar6, IBEACON_CHAR6_LEN );
      break;
      
    case IBEACON_CHAR7:
      VOID osal_memcpy( value, iBeaconChar7, IBEACON_CHAR7_LEN );
      break;
     
    case IBEACON_CHAR8:
      VOID osal_memcpy( value, iBeaconChar8, IBEACON_CHAR8_LEN );
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          iBeacon_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 iBeacon_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      case IBEACON_CHAR1_UUID:
        *pLen = IBEACON_CHAR1_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR1_LEN );
        break;
        
      case IBEACON_CHAR2_UUID:
        *pLen = IBEACON_CHAR2_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR2_LEN );
        break;
        
      case IBEACON_CHAR3_UUID:
        *pLen = IBEACON_CHAR3_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR3_LEN );
        break;
        
      case IBEACON_CHAR4_UUID:
        *pLen = IBEACON_CHAR4_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR4_LEN );
        break;
        
      case IBEACON_CHAR5_UUID:
        *pLen = IBEACON_CHAR5_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR5_LEN );
        break;
        
      case IBEACON_CHAR6_UUID:
        *pLen = IBEACON_CHAR6_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR6_LEN );
        break;
        
      case IBEACON_CHAR7_UUID:
        *pLen = IBEACON_CHAR7_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR7_LEN );
        break;
       
      case IBEACON_CHAR8_UUID:
        *pLen = IBEACON_CHAR8_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR8_LEN );
        break;
        
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else // 128-bit UUID
  {
    if ( osal_memcmp(pAttr->type.uuid, iBeaconChar1UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR1_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR1_LEN);
    }
    else if ( osal_memcmp(pAttr->type.uuid, iBeaconChar2UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR2_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR2_LEN);
    }
    else if ( osal_memcmp(pAttr->type.uuid, iBeaconChar3UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR3_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR3_LEN);
    }
    else if ( osal_memcmp(pAttr->type.uuid, iBeaconChar4UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR4_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR4_LEN);
    }
    else if ( osal_memcmp(pAttr->type.uuid, iBeaconChar5UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR5_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR5_LEN);
    }
    else if ( osal_memcmp(pAttr->type.uuid, iBeaconChar6UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR6_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR6_LEN);
    }
    else if ( osal_memcmp(pAttr->type.uuid, iBeaconChar7UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR7_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR7_LEN);
    }   
    else if ( osal_memcmp(pAttr->type.uuid, iBeaconChar8UUID, ATT_UUID_SIZE) )
    {
      *pLen = IBEACON_CHAR8_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_CHAR8_LEN);
    }
    else
    {
      *pLen = 0;
      status = ATT_ERR_INVALID_HANDLE;
    }
  }

  return ( status );
}

/*********************************************************************
 * @fn      iBeacon_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   complete - whether this is the last packet
 * @param   oper - whether to validate and/or write attribute value  
 *
 * @return  Success or Failure
 */
static bStatus_t iBeacon_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
 
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE ) // 16-bit UUID
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case IBEACON_CHAR1_UUID:
      case IBEACON_CHAR2_UUID:
      case IBEACON_CHAR3_UUID:
      case IBEACON_CHAR4_UUID:
      case IBEACON_CHAR5_UUID:
      case IBEACON_CHAR6_UUID:
      case IBEACON_CHAR7_UUID:  
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 1 )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          *pCurValue = pValue[0];

          if ( pAttr->pValue == iBeaconChar1 )
          {
            notifyApp = IBEACON_CHAR1;        
          }
          else if ( pAttr->pValue == iBeaconChar2 )
          {
            notifyApp = IBEACON_CHAR2;                    
          }
          else if ( pAttr->pValue == iBeaconChar3 )
          {
            notifyApp = IBEACON_CHAR3;                    
          }
          else if ( pAttr->pValue == iBeaconChar4 )
          {
            notifyApp = IBEACON_CHAR4;                    
          }
          else if ( pAttr->pValue == iBeaconChar5 )
          {
            notifyApp = IBEACON_CHAR5;                    
          }
          else if ( pAttr->pValue == iBeaconChar6 )
          {
            notifyApp = IBEACON_CHAR6;                    
          }
          else if ( pAttr->pValue == iBeaconChar7 )
          {
            notifyApp = IBEACON_CHAR7;                    
          }
        }
             
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else // 128-bit UUID
  {
    if (osal_memcmp(pAttr->type.uuid, iBeaconChar1UUID, ATT_UUID_SIZE) )
    {
      if (offset == 0)
      {
        if (len != IBEACON_CHAR1_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      
      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        osal_memcpy(pCurValue, pValue, IBEACON_CHAR1_LEN); 

        if ( pAttr->pValue == iBeaconChar1 )
          notifyApp = IBEACON_CHAR1;
      }
    }
    else if (osal_memcmp(pAttr->type.uuid, iBeaconChar2UUID, ATT_UUID_SIZE) )
    {
      if (offset == 0)
      {
        if (len != IBEACON_CHAR2_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        pCurValue[0] = pValue[0];
        pCurValue[1] = pValue[1];

        if ( pAttr->pValue == iBeaconChar2 )
          notifyApp = IBEACON_CHAR2;
      }
    }
    else if (osal_memcmp(pAttr->type.uuid, iBeaconChar3UUID, ATT_UUID_SIZE) )
    {
      if (offset == 0)
      {
        if (len != IBEACON_CHAR3_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        pCurValue[0] = pValue[0];
        pCurValue[1] = pValue[1];

        if ( pAttr->pValue == iBeaconChar3 )
          notifyApp = IBEACON_CHAR3;
      }
    }
    else if (osal_memcmp(pAttr->type.uuid, iBeaconChar4UUID, ATT_UUID_SIZE) )
    {
      if (offset == 0)
      {
        if (len != IBEACON_CHAR4_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      
      int8 newValue = (int8)pValue[0];
      if(newValue > 0.0 || newValue < -100.0)
      {
        status = ATT_ERR_UNSUPPORTED_GRP_TYPE;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        *pCurValue = pValue[0];

        if ( pAttr->pValue == iBeaconChar4 )
          notifyApp = IBEACON_CHAR4;
      }
    }
    else if (osal_memcmp(pAttr->type.uuid, iBeaconChar5UUID, ATT_UUID_SIZE) )
    {
      if (offset == 0)
      {
        if (len != IBEACON_CHAR5_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      
      if(pValue[0] != 0x00 && pValue[0] != 0x01)
      {
        status = ATT_ERR_UNSUPPORTED_GRP_TYPE;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        pCurValue[0] = pValue[0];

        if ( pAttr->pValue == iBeaconChar5 )
          notifyApp = IBEACON_CHAR5;
      }
    }
    else if (osal_memcmp(pAttr->type.uuid, iBeaconChar6UUID, ATT_UUID_SIZE) )
    {
      if (offset == 0)
      {
        if (len != IBEACON_CHAR6_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      
      uint16 adv = pValue[0] * 256 + pValue[1];
      if(adv > 10000 || adv < 100)
      {
        status = ATT_ERR_UNSUPPORTED_GRP_TYPE;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        pCurValue[0] = pValue[0];
        pCurValue[1] = pValue[1];

        if ( pAttr->pValue == iBeaconChar6 )  
          notifyApp = IBEACON_CHAR6;
      }
    }
    else if (osal_memcmp(pAttr->type.uuid, iBeaconChar7UUID, ATT_UUID_SIZE) )
    {
      if (offset == 0)
      {
        if (len != IBEACON_CHAR7_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      
      if(pValue[0] != 0x00 && pValue[0] != 0x01 && pValue[0] != 0x02 && pValue[0] != 0x03)
      {
        status = ATT_ERR_UNSUPPORTED_GRP_TYPE;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        pCurValue[0] = pValue[0];

        if ( pAttr->pValue == iBeaconChar7 )
          notifyApp = IBEACON_CHAR7;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_HANDLE;
    }
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && iBeacon_AppCBs && iBeacon_AppCBs->pfn_iBeaconChange )
  {
    iBeacon_AppCBs->pfn_iBeaconChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
 * @fn          iBeacon_HandleConnStatusCB
 *
 * @brief       iBeacon link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none  
 */
static void iBeacon_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 

    }
  }
}


/*********************************************************************
*********************************************************************/