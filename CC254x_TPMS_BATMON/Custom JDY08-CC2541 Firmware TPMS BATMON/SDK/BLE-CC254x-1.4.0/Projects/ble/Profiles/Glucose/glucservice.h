/**************************************************************************************************
  Filename:       glucservice.h
  Revised:        $Date: 2011-12-16 15:46:52 -0800 (Fri, 16 Dec 2011) $
  Revision:       $Revision: 58 $

  Description:    This file contains the Glucose service definitions and
                  prototypes.

 Copyright 2011 - 2013 Texas Instruments Incorporated. All rights reserved.

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

#ifndef GLUCOSESERVICE_H
#define GLUCOSESERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "OSAL_clock.h"
  
/*********************************************************************
 * CONSTANTS
 */
  
// Glucose Service Parameters
#define GLUCOSE_FEATURE_PARAM             3

// Characteristic Value sizes
#define GLUCOSE_CTL_PNT_MIN_SIZE          2  
#define GLUCOSE_CTL_PNT_MAX_SIZE          20  

// Values for meas flags

#define GLUCOSE_MEAS_FLAG_TIME_OFFSET           0x01
#define GLUCOSE_MEAS_FLAG_CONCENTRATION         0x02
#define GLUCOSE_MEAS_FLAG_UNITS                 0x04
#define GLUCOSE_MEAS_FLAG_STATUS_ANNUNCIATION   0x08
#define GLUCOSE_MEAS_FLAG_CONTEXT_INFO          0x10

#define GLUCOSE_MEAS_FLAG_ALL  (GLUCOSE_MEAS_FLAG_TIME_OFFSET | GLUCOSE_MEAS_FLAG_CONCENTRATION | GLUCOSE_MEAS_FLAG_UNITS | GLUCOSE_MEAS_FLAG_STATUS_ANNUNCIATION | GLUCOSE_MEAS_FLAG_CONTEXT_INFO)
  
// Values for Type Nibble
#define GLUCOSE_TYPE_CAPILLARY_WHOLE            0x1
#define GLUCOSE_TYPE_CAPILLARY_PLASMA           0x2
#define GLUCOSE_TYPE_VENOUS_WHOLE               0x3
#define GLUCOSE_TYPE_VENOUS_PLASMA              0x4
#define GLUCOSE_TYPE_ARTERIAL_WHOLE             0x5
#define GLUCOSE_TYPE_ARTERIAL_PLASMA            0x6
#define GLUCOSE_TYPE_UNDETER_WHOLE              0x7
#define GLUCOSE_TYPE_UNDETER_PLASMA             0x8
#define GLUCOSE_TYPE_ISF                        0x9
#define GLUCOSE_TYPE_CONTROL                    0xA

// Values for Location Nibble
#define GLUCOSE_LOCATION_FINGER                 0x10
#define GLUCOSE_LOCATION_AST                    0x20
#define GLUCOSE_LOCATION_EARLOBE                0x30
#define GLUCOSE_LOCATION_CONTROL                0x40
#define GLUCOSE_LOCATION_NOT_AVAIL              0xF0

// Values for Annunciation
#define GLUCOSE_ANNUNCIATION_BATT_LOW           0x0001
#define GLUCOSE_ANNUNCIATION_SENS_BAD           0x0002
#define GLUCOSE_ANNUNCIATION_SAMP_SIZE          0x0004
#define GLUCOSE_ANNUNCIATION_INSERT             0x0008
#define GLUCOSE_ANNUNCIATION_STRIP              0x0010
#define GLUCOSE_ANNUNCIATION_TOO_HIGH           0x0020
#define GLUCOSE_ANNUNCIATION_TOO_LOW            0x0040
#define GLUCOSE_ANNUNCIATION_TEMP_HIGH          0x0080
#define GLUCOSE_ANNUNCIATION_TEMP_LOW           0x0100
#define GLUCOSE_ANNUNCIATION_INTERRUPTED        0x0200
#define GLUCOSE_ANNUNCIATION_GENERAL            0x0400
#define GLUCOSE_ANNUNCIATION_TIME               0x0800

// Values for context flags
#define GLUCOSE_CONTEXT_FLAG_CARBO              0x01
#define GLUCOSE_CONTEXT_FLAG_MEAL               0x02
#define GLUCOSE_CONTEXT_FLAG_TESTER_HEALTH      0x04
#define GLUCOSE_CONTEXT_FLAG_EXERCISE           0x08
#define GLUCOSE_CONTEXT_FLAG_MEDICATION         0x10
#define GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS   0x20
#define GLUCOSE_CONTEXT_FLAG_HbA1c              0x40
#define GLUCOSE_CONTEXT_FLAG_EXTENDED           0x80

//We leave the extended flags out of this all, since there aren't any in the current spec  
#define GLUCOSE_CONTEXT_FLAG_ALL  (GLUCOSE_CONTEXT_FLAG_CARBO | GLUCOSE_CONTEXT_FLAG_MEAL | GLUCOSE_CONTEXT_FLAG_TESTER_HEALTH | GLUCOSE_CONTEXT_FLAG_EXERCISE | GLUCOSE_CONTEXT_FLAG_MEDICATION | GLUCOSE_CONTEXT_FLAG_MEDICATION_UNITS | GLUCOSE_CONTEXT_FLAG_HbA1c)
  
// Values for Carbohydratea
#define GLUCOSE_CARBO_BREAKFAST                 0x01
#define GLUCOSE_CARBO_LUNCH                     0x02
#define GLUCOSE_CARBO_DINNER                    0x03
#define GLUCOSE_CARBO_SNACK                     0x04
#define GLUCOSE_CARBO_DRINK                     0x05
#define GLUCOSE_CARBO_SUPPER                    0x06
#define GLUCOSE_CARBO_BRUNCH                    0x07

// Values for Meal
#define GLUCOSE_MEAL_PREPRANDIAL                0x1
#define GLUCOSE_MEAL_POSTPRANDIAL               0x2
#define GLUCOSE_MEAL_FASTING                    0x3
#define GLUCOSE_MEAL_CASUAL                     0x4

// Values for Tester Nibble
#define GLUCOSE_TESTER_SELF                     0x1
#define GLUCOSE_TESTER_PROFESSIONAL             0x2
#define GLUCOSE_TESTER_LAB_TEST                 0x3
#define GLUCOSE_TESTER_UNAVAILABLE              0xF

// Values for Health Nibble
#define GLUCOSE_HEALTH_MINOR                    0x10
#define GLUCOSE_HEALTH_MAJOR                    0x20
#define GLUCOSE_HEALTH_MENSES                   0x30
#define GLUCOSE_HEALTH_STRESS                   0x40
#define GLUCOSE_HEALTH_NONE                     0x50
#define GLUCOSE_HEALTH_UNAVAILABLE              0xF0

// Values for Medication ID
#define GLUCOSE_MEDICATION_RAPID                0x1
#define GLUCOSE_MEDICATION_SHORT                0x2
#define GLUCOSE_MEDICATION_INTERMEDIATE         0x3
#define GLUCOSE_MEDICATION_LONG                 0x4
#define GLUCOSE_MEDICATION_PRE_MIXED            0x5

// Values for Glucose Features
#define GLUCOSE_FEAT_LOW_BAT_SUP                0x0001
#define GLUCOSE_FEAT_MALFUNC_SUP                0x0002
#define GLUCOSE_FEAT_SAMPLE_SIZE_SUP            0x0004
#define GLUCOSE_FEAT_INSERT_ERR_SUP             0x0008
#define GLUCOSE_FEAT_TYPE_ERR_SUP               0x0010
#define GLUCOSE_FEAT_RESULT_DET_SUP             0x0020
#define GLUCOSE_FEAT_TEMP_DET_SUP               0x0040
#define GLUCOSE_FEAT_READ_INT_SUP               0x0080
#define GLUCOSE_FEAT_GENERAL_FAULT_SUP          0x0100
#define GLUCOSE_FEAT_TIME_FAULT_SUP             0x0200
#define GLUCOSE_FEAT_MULTIPLE_BOND_SUP          0x0400

#define GLUCOSE_FEAT_ALL    (GLUCOSE_FEAT_LOW_BAT_SUP | GLUCOSE_FEAT_MALFUNC_SUP | GLUCOSE_FEAT_SAMPLE_SIZE_SUP | \
                             GLUCOSE_FEAT_INSERT_ERR_SUP | GLUCOSE_FEAT_TYPE_ERR_SUP | GLUCOSE_FEAT_RESULT_DET_SUP | \
                             GLUCOSE_FEAT_TEMP_DET_SUP | GLUCOSE_FEAT_READ_INT_SUP | GLUCOSE_FEAT_GENERAL_FAULT_SUP | \
                             GLUCOSE_FEAT_TIME_FAULT_SUP | GLUCOSE_FEAT_MULTIPLE_BOND_SUP)

// Glucose Service bit fields
#define GLUCOSE_SERVICE                      0x00000001

//Record Control Point values
#define CTL_PNT_OP_REQ                       0x01
#define CTL_PNT_OP_CLR                       0x02
#define CTL_PNT_OP_ABORT                     0x03
#define CTL_PNT_OP_GET_NUM                   0x04
#define CTL_PNT_OP_NUM_RSP                   0x05
#define CTL_PNT_OP_REQ_RSP                   0x06
 
//Record Control Point operator
#define CTL_PNT_OPER_NULL                    0x00
#define CTL_PNT_OPER_ALL                     0x01  
#define CTL_PNT_OPER_LESS_EQUAL              0x02
#define CTL_PNT_OPER_GREATER_EQUAL           0x03
#define CTL_PNT_OPER_RANGE                   0x04
#define CTL_PNT_OPER_FIRST                   0x05
#define CTL_PNT_OPER_LAST                    0x06  

//Record Control Point Response Codes
#define CTL_PNT_RSP_SUCCESS                  0x01
#define CTL_PNT_RSP_OPCODE_NOT_SUPPORTED     0x02
#define CTL_PNT_RSP_OPER_INVALID             0x03
#define CTL_PNT_RSP_OPER_NOT_SUPPORTED       0x04
#define CTL_PNT_RSP_OPERAND_INVALID          0x05
#define CTL_PNT_RSP_NO_RECORDS               0x06
#define CTL_PNT_RSP_ABORT_FAILED             0x07
#define CTL_PNT_RSP_PROC_NOT_CMPL            0x08
#define CTL_PNT_RSP_FILTER_NOT_SUPPORTED     0x09

//Record Control Point Filter Types
#define CTL_PNT_FILTER_SEQNUM                0x01
#define CTL_PNT_FILTER_TIME                  0x02

// Callback events
#define GLUCOSE_MEAS_NTF_ENABLED              1
#define GLUCOSE_MEAS_NTF_DISABLED             2
#define GLUCOSE_CONTEXT_NTF_ENABLED           3
#define GLUCOSE_CONTEXT_NTF_DISABLED          4    
#define GLUCOSE_CTL_PNT_IND_ENABLED           5
#define GLUCOSE_CTL_PNT_IND_DISABLED          6    
#define GLUCOSE_CTL_PNT_CMD                   7

// Procedure timeout in ms
#define GLUCOSE_PROCEDURE_TIMEOUT             30000

// ATT status values
#define GLUCOSE_ERR_IN_PROGRESS               0x80
#define GLUCOSE_ERR_CCC_CONFIG                0x81

/*********************************************************************
 * TYPEDEFS
 */

// Glucose Service callback function
typedef void (*glucoseServiceCB_t)(uint8 event, uint8* data, uint8 dataLen);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * Glucose_AddService- Initializes the Glucose service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t Glucose_AddService( uint32 services );

/*
 * Glucose_Register - Register a callback function with the
 *          Glucose Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void Glucose_Register( glucoseServiceCB_t pfnServiceCB);

/*
 * Glucose_SetParameter - Set a Glucose parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Glucose_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * Glucose_GetParameter - Get a Glucose parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Glucose_GetParameter( uint8 param, void *value );

/*********************************************************************
 * @fn          Glucose_MeasSend
 *
 * @brief       Send a glucose measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Glucose_MeasSend( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId );

/*********************************************************************
 * @fn          Glucose_ContextSend
 *
 * @brief       Send a glucose measurement context.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Glucose_ContextSend( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId );

/*********************************************************************
 * @fn          Glucose_CtlPntIndicate
 *
 * @brief       Send an indication containing a glucose
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pInd - pointer to indication structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Glucose_CtlPntIndicate( uint16 connHandle, attHandleValueInd_t *pInd, uint8 taskId );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GLUCOSESERVICE_H */
