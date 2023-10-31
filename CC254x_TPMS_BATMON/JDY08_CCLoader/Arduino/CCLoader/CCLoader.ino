
/*

Copyright (c) 2012-2014 RedBearLab

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

/******************************************************************************
* INCLUDES
*/

/******************************************************************************
* DEFINES
*/
// Start addresses on DUP (Increased buffer size improves performance)
#define ADDR_BUF0                   0x0000 // Buffer (512 bytes)
#define ADDR_DMA_DESC_0             0x0200 // DMA descriptors (8 bytes)
#define ADDR_DMA_DESC_1             (ADDR_DMA_DESC_0 + 8)

// DMA channels used on DUP
#define CH_DBG_TO_BUF0              0x01   // Channel 0
#define CH_BUF0_TO_FLASH            0x02   // Channel 1

// Debug commands
#define CMD_CHIP_ERASE              0x10
#define CMD_WR_CONFIG               0x19
#define CMD_RD_CONFIG               0x24
#define CMD_READ_STATUS             0x30
#define CMD_RESUME                  0x4C
#define CMD_DEBUG_INSTR_1B          (0x54|1)
#define CMD_DEBUG_INSTR_2B          (0x54|2)
#define CMD_DEBUG_INSTR_3B          (0x54|3)
#define CMD_BURST_WRITE             0x80
#define CMD_GET_CHIP_ID             0x68

// Debug status bitmasks
#define STATUS_CHIP_ERASE_BUSY_BM   0x80 // New debug interface
#define STATUS_PCON_IDLE_BM         0x40
#define STATUS_CPU_HALTED_BM        0x20
#define STATUS_PM_ACTIVE_BM         0x10
#define STATUS_HALT_STATUS_BM       0x08
#define STATUS_DEBUG_LOCKED_BM      0x04
#define STATUS_OSC_STABLE_BM        0x02
#define STATUS_STACK_OVERFLOW_BM    0x01

// DUP registers (XDATA space address)
#define DUP_DBGDATA                 0x6260  // Debug interface data buffer
#define DUP_FCTL                    0x6270  // Flash controller
#define DUP_FADDRL                  0x6271  // Flash controller addr
#define DUP_FADDRH                  0x6272  // Flash controller addr
#define DUP_FWDATA                  0x6273  // Clash controller data buffer
#define DUP_CLKCONSTA               0x709E  // Sys clock status
#define DUP_CLKCONCMD               0x70C6  // Sys clock configuration
#define DUP_MEMCTR                  0x70C7  // Flash bank xdata mapping
#define DUP_DMA1CFGL                0x70D2  // Low byte, DMA config ch. 1
#define DUP_DMA1CFGH                0x70D3  // Hi byte , DMA config ch. 1
#define DUP_DMA0CFGL                0x70D4  // Low byte, DMA config ch. 0
#define DUP_DMA0CFGH                0x70D5  // Low byte, DMA config ch. 0
#define DUP_DMAARM                  0x70D6  // DMA arming register

// Utility macros
//! Low nibble of 16bit variable
#define LOBYTE(w)           ((unsigned char)(w))
//! High nibble of 16bit variable
#define HIBYTE(w)           ((unsigned char)(((unsigned short)(w) >> 8) & 0xFF))
//! Convert XREG register declaration to an XDATA integer address
//#define XREG(addr)       ((unsigned char volatile __xdata *) 0)[addr]
//#define FCTL            XREG( 0x6270 )
//#define XREG_TO_INT(a)      ((unsigned short)(&(a)))

// Commands to Bootloader
#define SBEGIN                0x01
#define SDATA                 0x02
#define SRSP                  0x03
#define SEND                  0x04
#define ERRO                 0x05
#define WAITING               0x00
#define RECEIVING             0x01

// Debug control pins & the indicate LED
int DD = 6;
int DC = 5;
int RESET = 4;
int LED = 13;

/******************************************************************************
 VARIABLES*/
//! DUP DMA descriptor
const unsigned char dma_desc_0[8] =
{
    // Debug Interface -> Buffer
    HIBYTE(DUP_DBGDATA),            // src[15:8]
    LOBYTE(DUP_DBGDATA),            // src[7:0]
    HIBYTE(ADDR_BUF0),              // dest[15:8]
    LOBYTE(ADDR_BUF0),              // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    31,                             // trigger: DBG_BW
    0x11                            // increment destination
};
//! DUP DMA descriptor
const unsigned char dma_desc_1[8] =
{
    // Buffer -> Flash controller
    HIBYTE(ADDR_BUF0),              // src[15:8]
    LOBYTE(ADDR_BUF0),              // src[7:0]
    HIBYTE(DUP_FWDATA),             // dest[15:8]
    LOBYTE(DUP_FWDATA),             // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    18,                             // trigger: FLASH
    0x42,                           // increment source
};

/**************************************************************************//**
* @brief    Writes a byte on the debug interface. Requires DD to be
*           output when function is called.
* @param    data    Byte to write
* @return   None.
******************************************************************************/
#pragma inline
void write_debug_byte(unsigned char data)
{
    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        // Set clock high and put data on DD line
        digitalWrite(DC, HIGH);
        if(data & 0x80)
        {
          digitalWrite(DD, HIGH);
        }
        else
        {
          digitalWrite(DD, LOW);
        }
        data <<= 1;
        digitalWrite(DC, LOW); // set clock low (DUP capture flank)
    }
}

/**************************************************************************//**
* @brief    Reads a byte from the debug interface. Requires DD to be
*           input when function is called.
* @return   Returns the byte read.
******************************************************************************/
#pragma inline
unsigned char read_debug_byte(void)
{
    unsigned char i;
    unsigned char data = 0x00;
    for (i = 0; i < 8; i++)
    {
        digitalWrite(DC, HIGH);  // DC high
        data <<= 1;
        if(HIGH == digitalRead(DD))
        {
          data |= 0x01;
        }        
        digitalWrite(DC, LOW);     // DC low
    }
    return data;
}

/**************************************************************************//**
* @brief    Function waits for DUP to indicate that it is ready. The DUP will
*           pulls DD line low when it is ready. Requires DD to be input when
*           function is called.
* @return   Returns 0 if function timed out waiting for DD line to go low
* @return   Returns 1 when DUP has indicated it is ready.
******************************************************************************/
#pragma inline
unsigned char wait_dup_ready(void)
{
    // DUP pulls DD low when ready
    unsigned int count = 0;
    while ((HIGH == digitalRead(DD)) && count < 16)
    {
        // Clock out 8 bits before checking if DD is low again
        read_debug_byte();
        count++;
    }
    return (count == 16) ? 0 : 1;
}

/**************************************************************************//**
* @brief    Issues a command on the debug interface. Only commands that return
*           one output byte are supported.
* @param    cmd             Command byte
* @param    cmd_bytes       Pointer to the array of data bytes following the
*                           command byte [0-3]
* @param    num_cmd_bytes   The number of data bytes (input to DUP) [0-3]
* @return   Data returned by command
******************************************************************************/
unsigned char debug_command(unsigned char cmd, unsigned char *cmd_bytes,
                            unsigned short num_cmd_bytes)
{
    unsigned short i;
    unsigned char output = 0;
    // Make sure DD is output
    pinMode(DD, OUTPUT);
    // Send command
    write_debug_byte(cmd);
    // Send bytes
    for (i = 0; i < num_cmd_bytes; i++)
    {
        write_debug_byte(cmd_bytes[i]);
    }
    // Set DD as input
    pinMode(DD, INPUT);
    digitalWrite(DD, HIGH);
    // Wait for data to be ready
    wait_dup_ready();
    // Read returned byte
    output = read_debug_byte();
    // Set DD as output
    pinMode(DD, OUTPUT);

    return output;
}

/**************************************************************************//**
* @brief    Resets the DUP into debug mode. Function assumes that
*           the programmer I/O has already been configured using e.g.
*           ProgrammerInit().
* @return   None.
******************************************************************************/
void debug_init(void)
{
    volatile unsigned char i;

    // Send two flanks on DC while keeping RESET_N low
    // All low (incl. RESET_N)
    digitalWrite(DD, LOW);
    digitalWrite(DC, LOW);
    digitalWrite(RESET, LOW);
    delay(10);   // Wait
    digitalWrite(DC, HIGH);                    // DC high
    delay(10);   // Wait
    digitalWrite(DC, LOW);                     // DC low
    delay(10);   // Wait
    digitalWrite(DC, HIGH);                    // DC high
    delay(10);   // Wait
    digitalWrite(DC, LOW);                     // DC low
    delay(10);   // Wait
    digitalWrite(RESET, HIGH);              // Release RESET_N
    delay(10);   // Wait
}

/**************************************************************************//**
* @brief    Reads the chip ID over the debug interface using the
*           GET_CHIP_ID command.
* @return   Returns the chip id returned by the DUP
******************************************************************************/
unsigned char read_chip_id(void)
{
    unsigned char id = 0;

    // Make sure DD is output
    pinMode(DD, OUTPUT);
    delay(1);
    // Send command
    write_debug_byte(CMD_GET_CHIP_ID);
    // Set DD as input
    pinMode(DD, INPUT);
    digitalWrite(DD, HIGH);
    delay(1);
    // Wait for data to be ready
    if(wait_dup_ready() == 1)
    {
      // Read ID and revision
      id = read_debug_byte(); // ID
      read_debug_byte();      // Revision (discard)
    }
    // Set DD as output
    pinMode(DD, OUTPUT);

    return id;
}

/**************************************************************************//**
* @brief    Sends a block of data over the debug interface using the
*           BURST_WRITE command.
* @param    src         Pointer to the array of input bytes
* @param    num_bytes   The number of input bytes
* @return   None.
******************************************************************************/
void burst_write_block(unsigned char *src, unsigned short num_bytes)
{
    unsigned short i;

    // Make sure DD is output
    pinMode(DD, OUTPUT);

    write_debug_byte(CMD_BURST_WRITE | HIBYTE(num_bytes));
    write_debug_byte(LOBYTE(num_bytes));
    for (i = 0; i < num_bytes; i++)
    {
        write_debug_byte(src[i]);
    }

    // Set DD as input
    pinMode(DD, INPUT);
    digitalWrite(DD, HIGH);
    // Wait for DUP to be ready
    wait_dup_ready();
    read_debug_byte(); // ignore output
    // Set DD as output
    pinMode(DD, OUTPUT);
}

/**************************************************************************//**
* @brief    Issues a CHIP_ERASE command on the debug interface and waits for it
*           to complete.
* @return   None.
******************************************************************************/
void chip_erase(void)
{
    volatile unsigned char status;
    // Send command
    debug_command(CMD_CHIP_ERASE, 0, 0);

    // Wait for status bit 7 to go low
    do {
        status = debug_command(CMD_READ_STATUS, 0, 0);
    } while((status & STATUS_CHIP_ERASE_BUSY_BM));
}

/**************************************************************************//**
* @brief    Writes a block of data to the DUP's XDATA space.
* @param    address     XDATA start address
* @param    values      Pointer to the array of bytes to write
* @param    num_bytes   Number of bytes to write
* @return   None.
******************************************************************************/
void write_xdata_memory_block(unsigned short address,
                              const unsigned char *values,
                              unsigned short num_bytes)
{
    unsigned char instr[3];
    unsigned short i;

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    for (i = 0; i < num_bytes; i++)
    {
        // MOV A, values[i]
        instr[0] = 0x74;
        instr[1] = values[i];
        debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

        // MOV @DPTR, A
        instr[0] = 0xF0;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

        // INC DPTR
        instr[0] = 0xA3;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    }
}

/**************************************************************************//**
* @brief    Writes a byte to a specific address in the DUP's XDATA space.
* @param    address     XDATA address
* @param    value       Value to write
* @return   None.
******************************************************************************/
void write_xdata_memory(unsigned short address, unsigned char value)
{
    unsigned char instr[3];

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOV A, values[i]
    instr[0] = 0x74;
    instr[1] = value;
    debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

    // MOV @DPTR, A
    instr[0] = 0xF0;
    debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}

/**************************************************************************//**
* @brief    Read a byte from a specific address in the DUP's XDATA space.
* @param    address     XDATA address
* @return   Value read from XDATA
******************************************************************************/
unsigned char read_xdata_memory(unsigned short address)
{
    unsigned char instr[3];

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOVX A, @DPTR
    instr[0] = 0xE0;
    return debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}

/**************************************************************************//**
* @brief    Reads 1-32767 bytes from DUP's flash to a given buffer on the
*           programmer.
* @param    bank        Flash bank to read from [0-7]
* @param    address     Flash memory start address [0x0000 - 0x7FFF]
* @param    values      Pointer to destination buffer.
* @return   None.
******************************************************************************/
void read_flash_memory_block(unsigned char bank,unsigned short flash_addr,
                             unsigned short num_bytes, unsigned char *values)
{
    unsigned char instr[3];
    unsigned short i;
    unsigned short xdata_addr = (0x8000 + flash_addr);

    // 1. Map flash memory bank to XDATA address 0x8000-0xFFFF
    write_xdata_memory(DUP_MEMCTR, bank);

    // 2. Move data pointer to XDATA address (MOV DPTR, xdata_addr)
    instr[0] = 0x90;
    instr[1] = HIBYTE(xdata_addr);
    instr[2] = LOBYTE(xdata_addr);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    for (i = 0; i < num_bytes; i++)
    {
        // 3. Move value pointed to by DPTR to accumulator (MOVX A, @DPTR)
        instr[0] = 0xE0;
        values[i] = debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

        // 4. Increment data pointer (INC DPTR)
        instr[0] = 0xA3;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    }
}

/**************************************************************************//**
* @brief    Writes 4-2048 bytes to DUP's flash memory. Parameter \c num_bytes
*           must be a multiple of 4.
* @param    src         Pointer to programmer's source buffer (in XDATA space)
* @param    start_addr  FLASH memory start address [0x0000 - 0x7FFF]
* @param    num_bytes   Number of bytes to transfer [4-1024]
* @return   None.
******************************************************************************/
void write_flash_memory_block(unsigned char *src, unsigned long start_addr,
                              unsigned short num_bytes)
{
    // 1. Write the 2 DMA descriptors to RAM
    write_xdata_memory_block(ADDR_DMA_DESC_0, dma_desc_0, 8);
    write_xdata_memory_block(ADDR_DMA_DESC_1, dma_desc_1, 8);

    // 2. Update LEN value in DUP's DMA descriptors
    unsigned char len[2] = {HIBYTE(num_bytes), LOBYTE(num_bytes)};
    write_xdata_memory_block((ADDR_DMA_DESC_0+4), len, 2);  // LEN, DBG => ram
    write_xdata_memory_block((ADDR_DMA_DESC_1+4), len, 2);  // LEN, ram => flash

    // 3. Set DMA controller pointer to the DMA descriptors
    write_xdata_memory(DUP_DMA0CFGH, HIBYTE(ADDR_DMA_DESC_0));
    write_xdata_memory(DUP_DMA0CFGL, LOBYTE(ADDR_DMA_DESC_0));
    write_xdata_memory(DUP_DMA1CFGH, HIBYTE(ADDR_DMA_DESC_1));
    write_xdata_memory(DUP_DMA1CFGL, LOBYTE(ADDR_DMA_DESC_1));

    // 4. Set Flash controller start address (wants 16MSb of 18 bit address)
    write_xdata_memory(DUP_FADDRH, HIBYTE( (start_addr)));//>>2) ));
    write_xdata_memory(DUP_FADDRL, LOBYTE( (start_addr)));//>>2) ));

    // 5. Arm DBG=>buffer DMA channel and start burst write
    write_xdata_memory(DUP_DMAARM, CH_DBG_TO_BUF0);
    burst_write_block(src, num_bytes);

    // 6. Start programming: buffer to flash
    write_xdata_memory(DUP_DMAARM, CH_BUF0_TO_FLASH);
    write_xdata_memory(DUP_FCTL, 0x0A);//0x06

    // 7. Wait until flash controller is done
    while (read_xdata_memory(DUP_FCTL) & 0x80);
}

void RunDUP(void)
{
  volatile unsigned char i;

  // Send two flanks on DC while keeping RESET_N low
  // All low (incl. RESET_N)
  digitalWrite(DD, LOW);
  digitalWrite(DC, LOW);
  digitalWrite(RESET, LOW);
  delay(10);   // Wait

  digitalWrite(RESET, HIGH);
  delay(10);   // Wait
}

void ProgrammerInit(void)
{
  pinMode(DD, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(DD, LOW);
  digitalWrite(DC, LOW);
  digitalWrite(RESET, HIGH);
  digitalWrite(LED, LOW);
}

void setup() 
{  
  ProgrammerInit();  
  Serial.begin(115200);
  // If using Leonado as programmer, 
  //it should add below code,otherwise,comment it.
  while(!Serial);
}

void loop() 
{
  unsigned char chip_id = 0;
  unsigned char debug_config = 0;
  unsigned char Continue = 0;
  unsigned char Verify = 0;
  
  while(!Continue)     // Wait for starting
  {  
    
    if(Serial.available()==2)
    {      
      if(Serial.read() == SBEGIN)
      {
        Verify = Serial.read();
        Continue = 1;
      }
      else
      {
        Serial.read(); // Clear RX buffer
      }
    }
  }

  debug_init();
  chip_id = read_chip_id();
  if(chip_id == 0) 
  {
    Serial.write(ERRO);  
    return; // No chip detected, run loop again.
  }
  
  RunDUP();
  debug_init();
  
  chip_erase();
  RunDUP();
  debug_init();
  
  // Switch DUP to external crystal osc. (XOSC) and wait for it to be stable.
  // This is recommended if XOSC is available during programming. If
  // XOSC is not available, comment out these two lines.
  write_xdata_memory(DUP_CLKCONCMD, 0x80);
  while (read_xdata_memory(DUP_CLKCONSTA) != 0x80);//0x80)
  
  // Enable DMA (Disable DMA_PAUSE bit in debug configuration)
  debug_config = 0x22;
  debug_command(CMD_WR_CONFIG, &debug_config, 1);
  
  // Program data (start address must be word aligned [32 bit])
  Serial.write(SRSP);    // Request data blocks
  digitalWrite(LED, HIGH);  
  unsigned char Done = 0;
  unsigned char State = WAITING;
  unsigned char  rxBuf[514]; 
  unsigned int BufIndex = 0;
  unsigned int addr = 0x0000;
  while(!Done)
  {
    while(Serial.available())
    {
      unsigned char ch;    
      ch = Serial.read();        
      switch (State)
      {
        // Bootloader is waiting for a new block, each block begin with a flag byte
        case WAITING:
        {
          if(SDATA == ch)  // Incoming bytes are data
          {
            State = RECEIVING;
          }
          else if(SEND == ch)   // End receiving firmware
          {
            Done = 1;           // Exit while(1) in main function
          }
          break;
        }      
        // Bootloader is receiving block data  
        case RECEIVING:
        {          
          rxBuf[BufIndex] = ch;
          BufIndex++;            
          if (BufIndex == 514) // If received one block, write it to flash
          {
            BufIndex = 0;              
            uint16_t CheckSum = 0x0000;
            for(unsigned int i=0; i<512; i++)
            {
              CheckSum += rxBuf[i];
            }
            uint16_t CheckSum_t = rxBuf[512]<<8 | rxBuf[513];
            if(CheckSum_t != CheckSum)
            {
              State = WAITING;
              Serial.write(ERRO);                    
              chip_erase();
              return;
            } 
            write_flash_memory_block(rxBuf, addr, 512); // src, address, count                    
            if(Verify)
            {
              unsigned char bank = addr / (512 * 16);
              unsigned int  offset = (addr % (512 * 16)) * 4;
              unsigned char read_data[512];
              read_flash_memory_block(bank, offset, 512, read_data); // Bank, address, count, dest.            
              for(unsigned int i = 0; i < 512; i++) 
              {
                if(read_data[i] != rxBuf[i]) 
                {
                  // Fail
                  State = WAITING;
                  Serial.write(ERRO);                    
                  chip_erase();
                  return;
                }
              }
            }
            addr += (unsigned int)128;              
            State = WAITING;
            Serial.write(SRSP);
          }
          break;
        }      
        default:
          break;
      }
    }
  }
  
  digitalWrite(LED, LOW);
  RunDUP();
}


