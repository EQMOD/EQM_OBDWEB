/**************************************************************************************************
  Filename:       hal_i2c.c
  Revised:        $Date: 2012-10-19 16:06:31 -0700 (Fri, 19 Oct 2012) $
  Revision:       $Revision: 31875 $

  Description:    This module defines the HAL I2C API for the CC2533.


  Copyright 2011-2012 Texas Instruments Incorporated. All rights reserved.

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

#include "hal_types.h"
#include "hal_board_cfg.h"
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
#include "hal_assert.h"
#include "hal_i2c.h"
#include "osal.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define I2C_ENS1            BV(6)
#define I2C_STA             BV(5)
#define I2C_STO             BV(4)
#define I2C_SI              BV(3)
#define I2C_AA              BV(2)
#define I2C_MST_RD_BIT      BV(0)  // Master RD/WRn bit to be OR'ed with Slave address.

#define I2C_CLOCK_MASK      0x83

#define I2C_PXIFG           P2IFG
#define I2C_IF              P2IF
#define I2C_IE              BV(1)

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum {
  // HAL_I2C_MASTER mode statuses.
  mstStarted   = 0x08,
  mstRepStart  = 0x10,
  mstAddrAckW  = 0x18,
  mstAddrNackW = 0x20,
  mstDataAckW  = 0x28,
  mstDataNackW = 0x30,
  mstLostArb   = 0x38,
  mstAddrAckR  = 0x40,
  mstAddrNackR = 0x48,
  mstDataAckR  = 0x50,
  mstDataNackR = 0x58,

  // HAL_I2C_SLAVE mode statuses.
  slvAddrAckR  = 0x60,
  mstLostArbR  = 0x68,
  slvAddrAckG  = 0x70,
  mstLostArbG  = 0x78,
  slvDataAckR  = 0x80,
  slvDataNackR = 0x88,
  genDataAckR  = 0x90,
  genDataNackR = 0x98,
  slvStopped   = 0xA0,
  slvAddrAckW  = 0xA8,
  mstLostArbW  = 0xB0,
  slvDataAckW  = 0xB8,
  slvDataNackW = 0xC0,
  slvLastAckW  = 0xC8,

  // HAL_I2C_MASTER/SLAVE mode statuses.
  i2cIdle      = 0xF8
} i2cStatus_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define I2C_WRAPPER_DISABLE() st( I2CWC    =     0x0C;              )
#define I2C_CLOCK_RATE(x)     st( I2CCFG  &=    ~I2C_CLOCK_MASK;    \
                                  I2CCFG  |=     x;                 )

#define I2C_CLR_NACK()  st( I2CCFG |=  I2C_AA; I2CCFG &= ~I2C_SI; )
#define I2C_SET_NACK()  st( I2CCFG &= ~I2C_AA; )
#define I2C_SET_ACK()   st( I2CCFG |=  I2C_AA; )

// Master to Ack to every byte read from the Slave; Slave to Ack Address & Data from the Master.
#define I2C_ENABLE()  st( I2CCFG |= (I2C_ENS1 | I2C_AA); )

// Must clear SI before setting STA and then STA must be manually cleared.
#define I2C_STRT() st (             \
  I2CCFG &= ~I2C_SI;                \
  I2CCFG |= I2C_STA;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  I2CCFG &= ~I2C_STA;               \
)

// Must set STO before clearing SI.
#define I2C_STOP() st (             \
  I2CCFG |= I2C_STO;                \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_STO) != 0);  \
)

// Stop clock-stretching and then read when it arrives.
#define I2C_READ(_X_) st (          \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  (_X_) = I2CDATA;                  \
)

// First write new data and then stop clock-stretching.
#define I2C_WRITE(_X_) st (         \
  I2CDATA = (_X_);                  \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
)

// Stop clock-stretching and then read when it arrives.
#define SLV_READ(_X_) st (          \
  I2CCFG &= ~I2C_SI;                \
  while (((I2CCFG & I2C_SI) == 0)   \
    &&    (I2CSTAT != slvStopped)); \
  (_X_) = I2CDATA;                  \
)

// First write new data and then stop clock-stretching.
#define SLV_WRITE(_X_) st (         \
  I2CDATA = (_X_);                  \
  I2CCFG &= ~I2C_SI;                \
  while (((I2CCFG & I2C_SI) == 0)   \
    &&    (I2CSTAT != slvStopped)); \
)

#if HAL_I2C_POLLED
#define I2C_INT_ENABLE()
#else
#define I2C_INT_ENABLE()   st( IEN2 |=  I2C_IE; )
#endif
#define I2C_INT_DISABLE()  st( IEN2 &= ~I2C_IE; )
#define I2C_INT_ENABLED()    ( IEN2 &   I2C_IE  )

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 i2cAddrSave, i2cCfgSave;  // Save & restore variables for PM.

#if HAL_I2C_SLAVE
static i2cCallback_t i2cCB;
static volatile i2cLen_t i2cRxIdx, i2cTxIdx;
static uint8 i2cRxBuf[HAL_I2C_BUF_MAX+1], i2cTxBuf[HAL_I2C_BUF_MAX+1];
#endif

static volatile i2cLen_t i2cRxLen, i2cTxLen;

#if HAL_I2C_MASTER
/**************************************************************************************************
 * @fn          i2cMstStrt
 *
 * @brief       Attempt to send an I2C bus START and Slave Address as an I2C bus Master.
 *
 * input parameters
 *
 * @param       address - I2C address of the slave device we're attempting to talk to
 * @param       RD_WRn - The LSB of the Slave Address as Read/~Write.
 *
 * output parameters
 *
 * None.
 *
 * @return      The I2C status of the START request or of the Slave Address Ack.
 */
static uint8 i2cMstStrt(uint8 address, uint8 RD_WRn)
{
  I2C_STRT();

  if (I2CSTAT == mstStarted)
  {
    address <<= 1;
    I2C_WRITE(address | RD_WRn);
  }

  return I2CSTAT;
}

/**************************************************************************************************
 * @fn          HalI2CInit
 *
 * @brief       Initialize the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       clockRate - I2C clock rate.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalI2CInit(i2cClock_t clockRate)
{
  I2C_WRAPPER_DISABLE();
  I2CADDR = 0; // no multi master support at this time
  I2C_CLOCK_RATE(clockRate);
  I2C_ENABLE();
}

/**************************************************************************************************
 * @fn          HalI2CRead
 *
 * @brief       Read from the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       address - I2C address of the slave device we're attempting to read from
 * @param       len - Number of bytes to read.
 * @param       pBuf - Pointer to the data buffer to put read bytes.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes successfully read.
 */
i2cLen_t HalI2CRead(uint8 address, i2cLen_t len, uint8 *pBuf)
{
  uint8 cnt = 0;

  if (i2cMstStrt(address, I2C_MST_RD_BIT) != mstAddrAckR)
  {
    len = 0;
  }

  // All bytes are ACK'd except for the last one which is NACK'd. If only
  // 1 byte is being read, a single NACK will be sent. Thus, we only want
  // to enable ACK if more than 1 byte is going to be read.
  if (len > 1)
  {
    I2C_SET_ACK();
  }

  while (len > 0)
  {
    // slave devices require NACK to be sent after reading last byte
    if (len == 1)
    {
      I2C_SET_NACK();
    }

    // read a byte from the I2C interface
    I2C_READ(*pBuf++);
    cnt++;
    len--;

    if (I2CSTAT != mstDataAckR)
    {
      if (I2CSTAT != mstDataNackR)
      {
        // something went wrong, so don't count last byte
        cnt--;
      }
      break;
    }
  }

  I2C_STOP();
  return cnt;
}

/**************************************************************************************************
 * @fn          HalI2CWrite
 *
 * @brief       Write to the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       address - I2C address of the slave device we're attempting to write to
 * @param       len - Number of bytes to write.
 * @param       pBuf - Pointer to the data buffer to write.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes successfully written.
 */
i2cLen_t HalI2CWrite(uint8 address, i2cLen_t len, uint8 *pBuf)
{
  if (i2cMstStrt(address, 0) != mstAddrAckW)
  {
    len = 0;
  }

  for (i2cLen_t cnt = 0; cnt < len; cnt++)
  {
    I2C_WRITE(*pBuf++);

    if (I2CSTAT != mstDataAckW)
    {
      if (I2CSTAT == mstDataNackW)
      {
        len = cnt + 1;
      }
      else
      {
        len = cnt;
      }
      break;
    }
  }

  I2C_STOP();
  return len;
}

#else // if HAL_I2C_SLAVE

/**************************************************************************************************
 * @fn          i2cSlvRx
 *
 * @brief       Block, reading from the I2C bus as a Slave.
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
 */
static i2cLen_t i2cSlvRx(void)
{
  uint8 ch, idx = 0;

  do {
    SLV_READ(ch);

    if ((I2CSTAT == slvDataAckR) || (I2CSTAT == slvDataNackR))
    {
      i2cRxBuf[idx++] = ch;
    }

    if (idx == HAL_I2C_BUF_MAX)
    {
      I2C_SET_NACK();
    }
  } while (I2CSTAT == slvDataAckR);

  i2cRxLen = idx;
  i2cRxIdx = 0;
  I2C_CLR_NACK();

  return idx;
}

/**************************************************************************************************
 * @fn          i2cSlvTx
 *
 * @brief       Block, writing to the I2C bus as a Slave.
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
 */
static i2cLen_t i2cSlvTx(void)
{
  uint8 idx = i2cTxIdx, len = i2cTxLen;

  while (1)
  {
    if (len == 1)
    {
      I2C_SET_NACK();  // Setup to write last byte.
    }
    SLV_WRITE(i2cTxBuf[idx]);

    if ((I2CSTAT == slvDataAckW) || (I2CSTAT == slvLastAckW) || (I2CSTAT == slvDataNackW))
    {
      idx++;

      if ((--len == 0) || (I2CSTAT == slvDataNackW))
      {
        break;
      }
    }
    else
    {
      break;
    }
  }

  i2cTxIdx = idx;
  i2cTxLen = len;
  I2C_CLR_NACK();

  return len;
}

/**************************************************************************************************
 * @fn          HalI2CInit
 *
 * @brief       Initialize the I2C bus as a Slave.
 *
 * input parameters
 *
 * @param       address - I2C slave address.
 * @param       i2cCallback - I2C callback.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalI2CInit(uint8 address, i2cCallback_t i2cCallback)
{
  i2cCB = i2cCallback;
  HAL_ASSERT(i2cCB);

  I2C_WRAPPER_DISABLE();
  I2CADDR = address << 1;
  I2C_ENABLE();
  // Clear the CPU interrupt flag for Port_2 PxIFG has to be cleared before PxIF.
  I2C_PXIFG = 0;
  I2C_IF = 0;
  I2C_INT_ENABLE();
}

/**************************************************************************************************
 * @fn          HalI2CRead
 *
 * @brief       Read the data read from the I2C bus as a Slave.
 *
 * input parameters
 *
 * @param       len - Number of bytes to read.
 * @param       pBuf - Pointer to the data buffer to put read bytes.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes successfully read.
 */
i2cLen_t HalI2CRead(i2cLen_t len, uint8 *pBuf)
{
  uint8 rxLen = i2cRxLen;

  if (len >= rxLen)
  {
    len = rxLen;
    rxLen = 0;
  }
  else
  {
    rxLen -= len;
  }

  (void)osal_memcpy(pBuf, i2cRxBuf+i2cRxIdx, len);
  i2cRxIdx += len;
  i2cRxLen = rxLen;

  return len;
}

/**************************************************************************************************
 * @fn          HalI2CWrite
 *
 * @brief       Write the data to write to the I2C bus as a Slave.
 *
 * input parameters
 *
 * @param       len - Length of the data buffer to write.
 * @param       pBuf - Pointer to the data buffer to write.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes setup to write.
 */
i2cLen_t HalI2CWrite(i2cLen_t len, uint8 *pBuf)
{
#if ((HAL_I2C_BUF_MAX != 255) && (HAL_I2C_BUF_MAX != 65535))
  if (len > HAL_I2C_BUF_MAX)
  {
    len = HAL_I2C_BUF_MAX;
  }
#endif

  if (i2cTxLen != 0)  // Refuse to overwrite old msg with new data so no race with ISR.
  {
    return 0;
  }

  (void)osal_memcpy(i2cTxBuf, pBuf, len);
  i2cTxIdx = 0;
  i2cTxLen = len;

#if HAL_I2C_POLLED
  if (I2CSTAT == slvAddrAckW)
  {
    i2cSlvTx();
  }
#else
  // When the I2C slave is actively addressed for write and there is no Tx ready,
  // this driver gets a promise from the Application that a write will be made and effects
  // clock stretching, waiting for the Application to compose and now here to write the bytes
  // to be transmitted. Unfortunately, when the slave is clock stretching, the H/W continues to
  // fire the ISR and the Application cannot run (no throughput), so the I2C ISR has to be disabled
  // while purposely clock stretching.
  if (!I2C_INT_ENABLED())
  {
    I2C_INT_ENABLE();  // Constant ISR's being held off will trigger & invoke i2cSlvTx().
  }
#endif

  return len;
}

/**************************************************************************************************
 * @fn          HalI2CPoll
 *
 * @brief       Poll the I2C module as a Slave when not running by ISR.
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
 */
#if HAL_I2C_POLLED
void HalI2CPoll(void)
#else
HAL_ISR_FUNCTION(halI2CIsr, I2C_VECTOR)
#endif
{
  HAL_ENTER_ISR();

  switch (I2CSTAT)
  {
  case slvAddrAckR:
    if (i2cRxLen == 0)
    {
      i2cSlvRx();  // Block, receiving bytes from the Master.
    }
    else  // Rx overrun.
    {
      volatile uint8 ch;
      // Throw away these new bytes & leave old msg so no race with background HalI2CRead().
      I2C_SET_NACK();
      SLV_READ(ch);
      I2C_CLR_NACK();
    }

    (void)i2cCB(i2cRxLen);  // Alert Application that a Master Tx is ready to read.
    break;

  case slvAddrAckW:
    if (i2cTxLen != 0)
    {
      i2cSlvTx();  // Block, transmitting bytes to the Master.
    }
    else if (i2cCB(0) == FALSE)
    {
      I2C_SET_NACK();
      SLV_WRITE(0);
      I2C_CLR_NACK();
    }
    else
    {
      /* Slave will be clock-stretching, blocking master, until Application makes a HalI2CWrite().
       * While addressed as a slave, the I2C ISR keeps firing even though the I2CSTAT is not
       * changing and the flags are cleared below. So this special ISR disable here to be
       * re-enabled within the slave HalI2CWrite() function.
       */
      I2C_INT_DISABLE();
    }
    break;

  case i2cIdle:  // Not expected, but not really an error, so no need to execute a STOP.
    break;

  default:
    I2C_STOP();
    I2C_CLR_NACK();  // Setup to Ack the next time addressed.
    break;
  }

  // Clear the CPU interrupt flag for Port_2 PxIFG has to be cleared before PxIF.
  I2C_PXIFG = 0;
  I2C_IF = 0;

  HAL_EXIT_ISR();
}
#endif

/**************************************************************************************************
 * @fn          HalI2CReady2Sleep
 *
 * @brief       Determine whether the I2C is ready to sleep.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      1 if the I2C is ready to sleep; 0 otherwise.
 */
uint8 HalI2CReady2Sleep(void)
{
  return ((i2cRxLen == 0) && (i2cTxLen == 0) && (I2CSTAT == i2cIdle));
}

/**************************************************************************************************
 * @fn          HalI2CEnterSleep
 *
 * @brief       Save I2C state before entering a Power Mode.
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
 */
void HalI2CEnterSleep(void)
{
  i2cCfgSave = I2CCFG;
  i2cAddrSave = I2CADDR;
}

/**************************************************************************************************
 * @fn          HalI2CExitSleep
 *
 * @brief       Restore I2C state after exiting a Power Mode.
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
 */
void HalI2CExitSleep(void)
{
  I2CCFG = i2cCfgSave;
  I2CADDR = i2cAddrSave;
}

#endif
/**************************************************************************************************
*/
