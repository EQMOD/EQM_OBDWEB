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

#ifndef IBEACONSERVICE_H
#define IBEACONSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// Profile Parameters
#define IBEACON_CHAR1       0   // Characteristic 1 value, UUID 
#define IBEACON_CHAR2       1   // Characteristic 2 value, Marjor
#define IBEACON_CHAR3       2   // Characteristic 3 value, Minor
#define IBEACON_CHAR4       3   // Characteristic 4 value, Measure Power
#define IBEACON_CHAR5       4   // Characteristic 5 value, LED State
#define IBEACON_CHAR6       5   // Characteristic 6 value, Advertising Interval
#define IBEACON_CHAR7       6   // Characteristic 7 value, BLE TX Power
#define IBEACON_CHAR8       7   // Characteristic 8 value, Software Revision

// iBeacon Service UUID 
#define IBEACON_SERV_UUID             0x2880
  
#define UUID_BASE_HEAD                0x70,0xB0
#define UUID_BASE_TAIL                0xDE,0x12,0xA5,0x98,0x1A,0x03,0x34,0xF7,0xAB,0xA8,0x95,0xA2

#define IBEACON_UUID 0xAA,0xBB,0x11,0x22,0xDF,0xFB,0x48,0xD2,0xB0,0x60,0xD0,0xF5,0xA7,0x10,0x96,0xE0

// Char UUID
#define IBEACON_CHAR1_UUID            0x2881
#define IBEACON_CHAR2_UUID            0x2882
#define IBEACON_CHAR3_UUID            0x2883
#define IBEACON_CHAR4_UUID            0x2884
#define IBEACON_CHAR5_UUID            0x2885
#define IBEACON_CHAR6_UUID            0x2886
#define IBEACON_CHAR7_UUID            0x2887
#define IBEACON_CHAR8_UUID            0x2888
  
// iBeacon Services bit fields
#define IBEACON_SERVICE               0x00000001

// Length of Characteristics in bytes
#define IBEACON_CHAR1_LEN           16
#define IBEACON_CHAR2_LEN           2
#define IBEACON_CHAR3_LEN           2  
#define IBEACON_CHAR4_LEN           1 
#define IBEACON_CHAR5_LEN           1  
#define IBEACON_CHAR6_LEN           2 
#define IBEACON_CHAR7_LEN           1
#define IBEACON_CHAR8_LEN           20

/*********************************************************************
 * TYPEDEFS
 */
  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */
// Callback when a characteristic value has changed
typedef NULL_OK void (*iBeaconChange_t)( uint8 paramID );

typedef struct
{
  iBeaconChange_t        pfn_iBeaconChange;  // Called when characteristic value changes
} iBeaconCBs_t;   

/*********************************************************************
 * API FUNCTIONS 
 */
/*********************************************************************
 * iBeacon_AddService- Initializes the iBeacon service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t iBeacon_AddService( uint32 services );

/*********************************************************************
 * iBeacon_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t iBeacon_RegisterAppCBs( iBeaconCBs_t *appCallbacks );

/*********************************************************************
 * iBeacon_SetParameter - Set a iBeacon parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t iBeacon_SetParameter( uint8 param, uint8 len, void *value );
  
/*********************************************************************
 * iBeacon_GetParameter - Get a iBeacon parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t iBeacon_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* IBEACONSERVICE_H */