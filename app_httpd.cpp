
//---------------------------------------------------------------------
// Copyright © 2020 Raymund Sarmiento
//
// Permission is hereby granted to use this Software for any purpose
// including combining with commercial products, creating derivative
// works, and redistribution of source or binary code, without
// limitation or consideration. Any redistributed copies of this
// Software must include the above Copyright Notice.
//
// THIS SOFTWARE IS PROVIDED "AS IS". THE AUTHOR OF THIS CODE MAKES NO
// WARRANTIES REGARDING THIS SOFTWARE, EXPRESS OR IMPLIED, AS TO ITS
// SUITABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
//
//  DISCLAIMER:

//  You can use the information on this site COMPLETELY AT YOUR OWN RISK.
//  The modification steps and other information on this site is provided
//  to you "AS IS" and WITHOUT WARRANTY OF ANY KIND, express, statutory,
//  implied or otherwise, including without limitation any warranty of
//  merchantability or fitness for any particular or intended purpose.
//  In no event the author will  be liable for any direct, indirect,
//  punitive, special, incidental or consequential damages or loss of any
//  kind whether or not the author  has been advised of the possibility
//  of such loss.
//---------------------------------------------------------------------


#ifndef COMPONENTS_PERFMON_INCLUDE_PERFMON_H_
#define COMPONENTS_PERFMON_INCLUDE_PERFMON_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

esp_err_t perfmon_start();
esp_err_t perfmon_stop();

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_PERFMON_INCLUDE_PERFMON_H_ */


#include "esp_http_server.h"



#include "SPIFFS.h"
#include "SdFat.h"


SdFs SD;

const uint8_t SD_CS_PIN = SS;

// Slow or Fast Cards

//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, 100000)

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, 25000000)


#define MAXFILESIZE 40000
#define MAXFILESIZE2 200000
#define FLASHSIZE 512

#define BLINK2  2
#define blinkDelay 50
#define LEDPIN0 GPIO_NUM_2

#define ETAGVAL "1667595595"
#define MAXAGEVAL  "max-age=7200, public"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "sdkconfig.h"

#include "esp_log.h"

static const char *TAG = "obd_httpd";

static uint64_t idle0Calls = 0;
static uint64_t idle1Calls = 0;
static int cpuload_offset = 0;
static int perfmonflag = 0;

httpd_handle_t obd_httpd = NULL;

#define WSCMDCNT 6  // Number of commands below

#define ToTPIDStack 500
#define conncount 20

#define FD_CMD_001 "speed_rpm200"
#define FD_CID_001 01
#define FD_DUR_001 200

#define FD_CMD_002 "all_value2000"
#define FD_CID_002 02
#define FD_DUR_002 2000

#define FD_CMD_003 "all_value3000"
#define FD_CID_003 03
#define FD_DUR_003 3000

#define FD_CMD_004 "tpms_2000"
#define FD_CID_004 04
#define FD_DUR_004 2000

#define FD_CMD_005 "bm2_1000"
#define FD_CID_005 05
#define FD_DUR_005 1000

#define FD_CMD_098 "cpu_stats"
#define FD_CID_098 98
#define FD_DUR_098 500

#define FD_CMD_099 "speed_rpm500"
#define FD_CID_099 99
#define FD_DUR_099 500

#define FD_CID_DIS -1
#define FD_DUR_DIS 0

static const size_t max_clients = 20;

#define GraphQueueCount 10
#define CPUGraphCount 20

int   wscnt[WSCMDCNT+1];

float p1[GraphQueueCount+1];
float p2[GraphQueueCount+1];
float c0[CPUGraphCount+1];
float c1[CPUGraphCount+1];

unsigned long perfmillis;


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


// Debug control pins & the indicate LED
int CC2541DD = GPIO_NUM_25; //GPIO14=D5 on NodeMCU/WeMos D1 Mini
int CC2541DC = GPIO_NUM_26; //GPIO4=D2 on NodeMCU/WeMos D1 Mini
int CC2541RESET = GPIO_NUM_27; //GPIO5=D1 on NodeMCU/WeMos D1 Mini

#define tireFL 0x1006b5
#define tireFR 0x2006b6
#define tireRL 0x3006b7
#define tireRR 0x4006b8
#define tireSP 0x5006b9

#define TPMS_COUNT  5

float tpms_pressure[TPMS_COUNT];
float tpms_temperature[TPMS_COUNT];
float tpms_voltage[TPMS_COUNT];


float bm2_voltage = 0;

// Handle BM2 Data

void store_BM2( float voltage)
{
    bm2_voltage = voltage;
}
// Handle TPMS Data

void store_TPMS(unsigned long tireID, float pressure, float temperature, float voltage)
{

      //Serial.printf("tireID: %06x  - Pressure: %5.2f psi, Temperature: %5.2f degC, Sensor Voltage: %4.2f V\n", tireID,pressure,temperature,voltage);

      switch(tireID)
      {
          case tireFL:
                       tpms_pressure[0] = pressure;
                       tpms_temperature[0] = temperature;
                       tpms_voltage[0] = voltage; 
                       break;
          case tireFR:
                       tpms_pressure[1] = pressure;
                       tpms_temperature[1] = temperature;
                       tpms_voltage[1] = voltage; 
                       break;
          case tireRL:
                       tpms_pressure[2] = pressure;
                       tpms_temperature[2] = temperature;
                       tpms_voltage[2] = voltage; 
                       break;
          case tireRR:
                       tpms_pressure[3] = pressure;
                       tpms_temperature[3] = temperature;
                       tpms_voltage[3] = voltage; 
                       break;
          case tireSP:
                       tpms_pressure[4] = pressure;
                       tpms_temperature[4] = temperature;
                       tpms_voltage[4] = voltage; 
                       break;
          default:
                       break;
      }


}


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
        digitalWrite( CC2541DC, HIGH);
        if(data & 0x80)
        {
          digitalWrite(CC2541DD, HIGH);
        }
        else
        {
          digitalWrite(CC2541DD, LOW);
        }
        data <<= 1;
        digitalWrite(CC2541DC, LOW); // set clock low (DUP capture flank)
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
        digitalWrite(CC2541DC, HIGH);  // DC high
        data <<= 1;
        if(HIGH == digitalRead(CC2541DD))
        {
          data |= 0x01;
        }        
        digitalWrite(CC2541DC, LOW);     // DC low
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
    while ((HIGH == digitalRead(CC2541DD)) && count < 16)
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
    pinMode(CC2541DD, OUTPUT);
    // Send command
    write_debug_byte(cmd);
    // Send bytes
    for (i = 0; i < num_cmd_bytes; i++)
    {
        write_debug_byte(cmd_bytes[i]);
    }
    // Set DD as input
    pinMode(CC2541DD, INPUT);
    digitalWrite(CC2541DD, HIGH);
    // Wait for data to be ready
    wait_dup_ready();
    // Read returned byte
    output = read_debug_byte();
    // Set DD as output
    pinMode(CC2541DD, OUTPUT);

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
    digitalWrite(CC2541DD, LOW);
    digitalWrite(CC2541DC, LOW);
    digitalWrite(CC2541RESET, LOW);
    delay(10);   // Wait
    digitalWrite(CC2541DC, HIGH);                    // DC high
    delay(10);   // Wait
    digitalWrite(CC2541DC, LOW);                     // DC low
    delay(10);   // Wait
    digitalWrite(CC2541DC, HIGH);                    // DC high
    delay(10);   // Wait
    digitalWrite(CC2541DC, LOW);                     // DC low
    delay(10);   // Wait
    digitalWrite(CC2541RESET, HIGH);              // Release RESET_N
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
    pinMode(CC2541DD, OUTPUT);
    delay(1);
    // Send command
    write_debug_byte(CMD_GET_CHIP_ID);
    // Set DD as input
    pinMode(CC2541DD, INPUT);
    digitalWrite(CC2541DD, HIGH);
    delay(1);
    // Wait for data to be ready
    if(wait_dup_ready() == 1)
    {
      // Read ID and revision
      id = read_debug_byte(); // ID
      read_debug_byte();      // Revision (discard)
    }
    // Set DD as output
    pinMode(CC2541DD, OUTPUT);

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
    pinMode(CC2541DD, OUTPUT);

    write_debug_byte(CMD_BURST_WRITE | HIBYTE(num_bytes));
    write_debug_byte(LOBYTE(num_bytes));
    for (i = 0; i < num_bytes; i++)
    {
        write_debug_byte(src[i]);
    }

    // Set DD as input
    pinMode(CC2541DD, INPUT);
    digitalWrite(CC2541DD, HIGH);
    // Wait for DUP to be ready
    wait_dup_ready();
    read_debug_byte(); // ignore output
    // Set DD as output
    pinMode(CC2541DD, OUTPUT);
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
  digitalWrite(CC2541DD, LOW);
  digitalWrite(CC2541DC, LOW);
  digitalWrite(CC2541RESET, LOW);
  delay(10);   // Wait

  digitalWrite(CC2541RESET, HIGH);
  delay(10);   // Wait
}

void ProgrammerInit(void)
{
  pinMode(CC2541DD, OUTPUT);
  pinMode(CC2541DC, OUTPUT);
  pinMode(CC2541RESET, OUTPUT);
  pinMode(LEDPIN0, OUTPUT);
  digitalWrite(CC2541DD, LOW);
  digitalWrite(CC2541DC, LOW);
  digitalWrite(CC2541RESET, HIGH);
  digitalWrite(LEDPIN0, LOW);
}

int Flash_CC2541(const char * path) 
{
  unsigned char chip_id = 0;
  unsigned char debug_config = 0;
  unsigned char Continue = 0;
  unsigned char Verify = 0;

  long fsize = 0;
  int BlkTot = 0;
  int Remain = 0;
  int BlkNum = 0;


  FsFile mfile=SD.open(path);
  if(!mfile)
        return 0;

  Serial.printf("Found %s \n",path);

  fsize = mfile.size();
  if (fsize < 20)
  {
      mfile.close();
      return 2; // No chip detected, run loop again.   
  }

	Remain = fsize % 512;	
	if(Remain != 0)
	{
		BlkTot = fsize / 512 + 1;
		Serial.printf("!!WARNING: File's size isn't the integer multiples of 512 bytes. \n");
		Serial.printf("           the last block will be filled in up to 512 bytes with 0xFF! \n");
	}
	else
	{
		BlkTot = fsize / 512;
	}
	Serial.printf("Block total: %d\n\n", BlkTot);
  BlkNum = 0;

  ProgrammerInit();  
  debug_init();
  chip_id = read_chip_id();
  if(chip_id == 0) 
        {
           Serial.printf("No CC2541 chip detected \n");  
          mfile.close();
           return 2; // No chip detected, run loop again.
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
  digitalWrite(LEDPIN0, HIGH);  
  unsigned char rxBuf[512]; 
  unsigned char read_data[512];
  unsigned int addr = 0x0000;

  int i;

  while(mfile.available()){

      mfile.read(&rxBuf,512);   


      Serial.printf("Wrting CC2541 Flash data at : %06x \n",addr);       
      //delay(100);
      write_flash_memory_block(rxBuf, addr, 512); // src, address, count                    
      if(Verify)
            {
              unsigned char bank = addr / (512 * 16);
              unsigned int  offset = (addr % (512 * 16)) * 4;
              read_flash_memory_block(bank, offset, 512, read_data); // Bank, address, count, dest.            
              for(unsigned int i = 0; i < 512; i++) 
              {
                if(read_data[i] != rxBuf[i]) 
                {
                  // Fail
                  mfile.close();
                  Serial.printf("Failed Verification at Bank %06x Offset %06x \n", bank, offset);
                  chip_erase();
                  return 2;
                }
              }
            }
      addr += (unsigned int)128;              
  
      }
  

  mfile.close();
  
  digitalWrite(LEDPIN0, LOW);
  RunDUP();

  pinMode(CC2541RESET, OUTPUT);
  digitalWrite(CC2541RESET, HIGH);
  delay(200);
  digitalWrite(CC2541RESET, LOW);
  delay(200);
  digitalWrite(CC2541RESET, HIGH);

   return 1;
}





static bool idle_task_0()
{
	idle0Calls += 1;
	return false;
}
static bool idle_task_1()
{

	idle1Calls += 1;
	return false;
}

static void perfmon_task(void *args)
{
	while (perfmonflag)
	{
     int i;

		 uint64_t idle0 = idle0Calls;
		 uint64_t idle1 = idle1Calls;
		idle0Calls = 0;
		idle1Calls = 0;

    // 5 second sampling
		//float cpu0 = 100.f - (((float)idle0 / (float)1702840) * 100.f);
		//float cpu1 = 100.f - (((float)idle1 / (float)6501072) * 100.f);

    // 1 second sampling
		//float cpu0 = 100.f - (((float)idle0 / (float)340568) * 100.f);
		//float cpu1 = 100.f - (((float)idle1 / (float)1300214) * 100.f);

    // .5 second sampling
		float cpu0 = 100.f - (((float)idle0 / (float)170284) * 100.f);
    //float cpu0 = 100.f - (((float)idle0 / (float)650107) * 100.f);
		float cpu1 = 100.f - (((float)idle1 / (float)650107) * 100.f);


    if ( cpu0 < 0) cpu0 = 0;
    if ( cpu1 < 0) cpu1 = 0;

    c0[CPUGraphCount]=cpu0;
    c1[CPUGraphCount]=cpu1;
    
    for(i=0;i<CPUGraphCount;i++)
    {
        c0[i] = c0[i+1]; 
        c1[i] = c1[i+1];
    }

   vTaskDelay(500 / portTICK_PERIOD_MS); 

	}
	
  esp_deregister_freertos_idle_hook_for_cpu(idle_task_0, 0);
	esp_deregister_freertos_idle_hook_for_cpu(idle_task_1, 1);

	vTaskDelete(NULL);
}

esp_err_t perfmon_start()
{
  int i;

  if (perfmonflag)
      return ESP_OK;

  idle0Calls = 170284;
	idle1Calls = 650107;

  for(i=0;i<=CPUGraphCount;i++)
    {
        c0[i]=0;
        c1[i]=0;
    }

	ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_task_0, 0));
	ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_task_1, 1));
	// TODO calculate optimal stack size
  perfmonflag = 1;
	xTaskCreate(perfmon_task, "perfmon", 2048, NULL, 1, NULL);
 
  Serial.println("Performance Monitor Started \n");
	return ESP_OK;
}

esp_err_t perfmon_stop()
{

  if (!perfmonflag)
      return ESP_OK;

  perfmonflag = 0;

  Serial.println("Performance Monitor Stopped/Killed \n");
	return ESP_OK;
}

void blinkX_ok(int num)
{
    int i;

    digitalWrite(LEDPIN0, LOW);
    delay(blinkDelay);
    for(i=0;i<num;i++)
    {
        delay(blinkDelay);
        digitalWrite(LEDPIN0, HIGH);
        delay(blinkDelay);
        digitalWrite(LEDPIN0, LOW);
    }
}


struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

int pidstack[ToTPIDStack];
float pidstackval[ToTPIDStack];
float tsens_out;
   
int fd_cmd[conncount];
unsigned long fd_millis[conncount],fd_curmillis[conncount];



void SDwriteFile(const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    FsFile mfile = SD.open(path, (O_RDWR | O_CREAT | O_AT_END));
    if(!mfile){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(mfile.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    mfile.close();
}


int SDreadFile(const char * path, uint8_t  ptr[]){

    int i,len,totlen;
    Serial.printf("Copying SD to SPIFFS file: %s\n", path);
    FsFile mfile=SD.open(path);
    if(!mfile){
        Serial.println("Failed to open file for reading");
        return -1;
    }
   totlen = 0;
   i = 0;

  while(mfile.available()){
    len = mfile.read(&ptr[i],1024);
    i=i+len;
    totlen=totlen+len;
  //  vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  ptr[totlen+1]=0;

  mfile.close();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  return totlen;

}

void SD_LoadSPIFF(const char * dirname, uint8_t levels){

  long  len,dsize;
  int flag;

  FsFile ddir;
  FsFile dfile;

  char fullpath[100];
  char filename[100];

  Serial.printf("Listing directory: %s\n", dirname);

  if (!ddir.open(dirname)){
     Serial.printf("Dirname error \n");
  }

  flag  = 0;
  while (dfile.openNext(&ddir, O_RDONLY)) {

        if(dfile.isDir())
        {
            dfile.getName(filename,50);
            dfile.close();

            fullpath[0] = 0;
            strcat(fullpath,dirname);
            if (strlen(dirname)>2)
                        strcat(fullpath,"/");
            strcat(fullpath,filename);
            if(levels)
                SD_LoadSPIFF(fullpath, levels -1);
        } else {
            dfile.getName(filename,50);
            dsize =  dfile.size();
            dfile.close();

            if ((dsize < MAXFILESIZE2) && (strlen(dirname) <100))
            {
                fullpath[0] = 0;
                strcat(fullpath,dirname);
                if (strlen(dirname)>2)
                        strcat(fullpath,"/");
                strcat(fullpath,filename);
            
                char  *sd_data = (char* ) ps_malloc(MAXFILESIZE2 * sizeof(char));

                len = SDreadFile(fullpath,  (uint8_t *) sd_data  );

                Serial.printf("Writing file to SPIIF: %d size \n", len);

                if (len > 0)
                {
                    blinkX_ok(2);
                    File file2 = SPIFFS.open(fullpath, FILE_WRITE);
                    if (file2)
                    {
                        file2.write((uint8_t *) sd_data, len);
                        flag = 1;
                    }
                    file2.close();
                }
                free(sd_data);
            }
        }
    }

    ddir.close();

    // Create a Lock file to prevent the next sd load

   if ((strlen(dirname)<2) && (flag == 1))
    SDwriteFile("/lock.file", "Hello There");

}

int readFile( const char * path, uint8_t ptr[]){

    int i,len,totlen;

    i = 0;
    totlen = 0;
  
    Serial.printf("Reading file: %s ", path);

    File file = SPIFFS.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return -1;
    }

  while(file.available()){
    len = file.read(&ptr[i],1024);
    i=i+len;
    totlen=totlen+len;
  }
  ptr[totlen]=0;
  file.close();
  Serial.printf("....Done at size %d\n", totlen);
  return totlen;
}

static void fd_cmdresp_001(async_resp_arg *arg)
{

    static char index_response[1024];
    char * data = index_response;

    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;

    digitalWrite(LEDPIN0, HIGH);
    data+=sprintf(data,"%.2f#%.2f#%.2f#%d",pidstackval[2],pidstackval[1],pidstackval[3]/0.39,fd);
    *data++ = 0;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*) index_response;
    ws_pkt.len = strlen(index_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);

    digitalWrite(LEDPIN0, LOW);
}

void update_p1()
{
  int i;

    for(i=0;i<GraphQueueCount;i++)
    {
        p1[i] = p1[i+1];
        //p2[i] = p2[i+1];
    }
    p1[GraphQueueCount] = pidstackval[7];
   // p2[GraphQueueCount] = pidstackval[3];
}




static void fd_cmdresp_002(async_resp_arg *arg)
{
    static char index_response[1024];
    char * data = index_response;
   
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;

    digitalWrite(LEDPIN0, HIGH);
    data+=sprintf(data,"%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d#%d",pidstackval[4],pidstackval[5],pidstackval[6],pidstackval[7],pidstackval[8],pidstackval[10],(int)p1[0],(int)p1[1],(int)p1[2],(int)p1[3],(int)p1[4],(int)p1[5],(int)p1[6],(int)p1[7],(int)p1[8],(int)p1[9],fd);
    *data++ = 0;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*) index_response;
    ws_pkt.len = strlen(index_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
    digitalWrite(LEDPIN0, LOW);

}


static void fd_cmdresp_004(async_resp_arg *arg)
{
    static char index_response[1024];
    char * data = index_response;
   
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;

    digitalWrite(LEDPIN0, HIGH);
    data+=sprintf(data,"%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%.2f#%d#",tpms_pressure[0],tpms_temperature[0],tpms_pressure[1],tpms_temperature[1],tpms_pressure[2],tpms_temperature[2],tpms_pressure[3],tpms_temperature[3],tpms_pressure[4],tpms_temperature[4],tpms_voltage[0],tpms_voltage[1],tpms_voltage[2],tpms_voltage[3],tpms_voltage[4],fd);
    *data++ = 0;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*) index_response;
    ws_pkt.len = strlen(index_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
    digitalWrite(LEDPIN0, LOW);

}

static void fd_cmdresp_005(async_resp_arg *arg)
{
    static char index_response[1024];
    char * data = index_response;
   
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;

    digitalWrite(LEDPIN0, HIGH);
    data+=sprintf(data,"%.2f#%d#",bm2_voltage,fd);
    *data++ = 0;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*) index_response;
    ws_pkt.len = strlen(index_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
    digitalWrite(LEDPIN0, LOW);

}



static void fd_cmdresp_098(async_resp_arg *arg)
{

    static char index_response[1024];
    char * data = index_response;
    int j;

    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;

    digitalWrite(LEDPIN0, HIGH);

    data+=sprintf(data,"[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]#",c0[0],c0[1],c0[2],c0[3],c0[4],c0[5],c0[6],c0[7],c0[8],c0[9],c0[10],c0[11],c0[12],c0[13],c0[14],c0[15],c0[16],c0[17],c0[18],c0[19]);
    data+=sprintf(data,"[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]" ,c1[0],c1[1],c1[2],c1[3],c1[4],c1[5],c1[6],c1[7],c1[8],c1[9],c1[10],c1[11],c1[12],c1[13],c1[14],c1[15],c1[16],c1[17],c1[18],c1[19]);
    data+=sprintf(data,"#%.2f#%.2f#",c0[20],c1[20]);
    data+=sprintf(data,"%d#",WSCMDCNT);
    for(j=0;j<=WSCMDCNT;j++)
        data+=sprintf(data,"%d#",wscnt[j]);
    *data++ = 0;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*) index_response;
    ws_pkt.len = strlen(index_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    Serial.write(0x27);
    Serial.write(0x27);

    Serial.print(index_response);

    Serial.write(0x0d);
    Serial.write(0x0a);
    Serial.write(0x00);

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
    digitalWrite(LEDPIN0, LOW);
}


static void fd_cmdresp_099(async_resp_arg *arg)
{

    static char index_response[1024];
    char * data = index_response;

    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;

    digitalWrite(LEDPIN0, HIGH);

    data+=sprintf(data,"%.2f#%.2f#%.2f#%d",pidstackval[2],pidstackval[1],pidstackval[3]/0.39,fd);
    *data++ = 0;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*) index_response;
    ws_pkt.len = strlen(index_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
    digitalWrite(LEDPIN0, LOW);
}

static esp_err_t page_handler(httpd_req_t *req){
    int len;
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[60] = {0,};
    esp_err_t resp;

  
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int res = 0;

    Serial.printf("Page: %s %s \n", variable,value);

    char  *sd_data = (char* ) ps_malloc(MAXFILESIZE2 * sizeof(char));

    if(strcmp(variable, "file") == 0) 
    {
      Serial.printf("reading file %s \n",value);
      len = readFile(value, (uint8_t *) sd_data  );
    }
    else 
        res = -1;

    if((res) && (len == 0)){
        free(sd_data);
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (strstr(value,"html") != NULL)
        httpd_resp_set_type(req, "text/html");
    else 
    {
      if (strstr(value,"css") != NULL)
           httpd_resp_set_type(req, "text/css");
     else
       if (strstr(value,"js") != NULL)
           httpd_resp_set_type(req, "text/javascript");
      else
        if (strstr(value,"jpg") != NULL)
            httpd_resp_set_type(req, "image/jpg");
         else
         if (strstr(value,"png") != NULL)
                httpd_resp_set_type(req, "image/png");        
         else
           httpd_resp_set_type(req, "text/html");

    }

    if (strstr(value,"gz") != NULL)
      httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    httpd_resp_set_hdr(req, "cache-control", MAXAGEVAL);
    httpd_resp_set_hdr(req, "etag", ETAGVAL);
    
    httpd_resp_set_hdr(req, "Connection", "close");

    resp = httpd_resp_send(req, sd_data,len);
    free(sd_data);

    return resp;
}


static esp_err_t getobd_handler(httpd_req_t *req){
    static char json_response[1024];

    int i;

    char * p = json_response;
    *p++ = '{';

    for(i=1;i<=pidstack[0];i++)
      if (pidstack[i] != 0)
          p+=sprintf(p, "\"PID%02d\":%.3f,",pidstack[i],pidstackval[i]);

    p+=sprintf(p, "\"PIDXX\":%.3f",xPortGetCoreID());     
    *p++ = '}';
    *p++ = 0;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}


static esp_err_t index_handler(httpd_req_t *req){

  int len;
  esp_err_t resp;

  //Serial.printf("index request\n");
  Serial.print("INDEX HANDLER running on core "); Serial.println(xPortGetCoreID());

  char  *sd_data = (char* ) ps_malloc(MAXFILESIZE * sizeof(char));
  len = readFile( "/index.html", (uint8_t *) sd_data  );
  httpd_resp_set_type(req, "text/html");
 // httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "cache-control", MAXAGEVAL);
    httpd_resp_set_hdr(req, "etag", ETAGVAL);
  httpd_resp_set_hdr(req, "Connection", "close");
  resp = httpd_resp_send(req, sd_data, len);
  free(sd_data);
  return resp;
}

static esp_err_t script_handler(httpd_req_t *req){

  int len;
  esp_err_t resp;

  char  *sd_data = (char* ) ps_malloc(MAXFILESIZE * sizeof(char));
  httpd_resp_set_type(req, "text/javascript");
  //httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "cache-control", MAXAGEVAL);
    httpd_resp_set_hdr(req, "etag", ETAGVAL);
  httpd_resp_set_hdr(req, "Connection", "close");
  len = readFile("/index.js", (uint8_t *) sd_data  );
  resp = httpd_resp_send(req, sd_data, len);
  free(sd_data);
  return resp;
}

static esp_err_t css_handler(httpd_req_t *req){

  int len;
  esp_err_t resp;

  char  *sd_data = (char* ) ps_malloc(MAXFILESIZE * sizeof(char));
  httpd_resp_set_type(req, "text/css");
  //httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "cache-control", MAXAGEVAL);
    httpd_resp_set_hdr(req, "etag", ETAGVAL);
  httpd_resp_set_hdr(req, "Connection", "close");
  len = readFile("/index.css", (uint8_t *) sd_data  );
  resp = httpd_resp_send(req, sd_data, len);
  free(sd_data);
  return resp;
}

static esp_err_t jquery_handler(httpd_req_t *req){

   int len;
   esp_err_t resp;
 
  httpd_resp_set_type(req, "text/javascript");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "cache-control", MAXAGEVAL);
    httpd_resp_set_hdr(req, "etag", ETAGVAL);
  httpd_resp_set_hdr(req, "Connection", "close");

  char  *sd_data = (char* ) ps_malloc(MAXFILESIZE * sizeof(char));
   len = readFile( "/jquery-min.js.gz", (uint8_t *) sd_data  );
   resp = httpd_resp_send(req, sd_data, len);
   free(sd_data);
   return resp;
}

static void  wss_server_send_messages(httpd_handle_t* server)
{
    bool send_messages = true;
    unsigned long curmillis;
    int fdofst;
    int tmp_wscnt[WSCMDCNT+1];
    int j,perfmessure;

    if (!*server) { // httpd might not have been created by now
            return;
        }
    size_t clients = max_clients;
    int    client_fds[max_clients];
    if (httpd_get_client_list(*server, &clients, client_fds) == ESP_OK) {

            perfmessure = 0;
            if ((millis()- perfmillis) > FD_DUR_098)
            {
                    perfmessure = perfmonflag;
                    perfmillis = millis();
            }

            if (perfmessure)
              for(j=0;j<=WSCMDCNT;j++)
                tmp_wscnt[j]=0;

            for (size_t i=0; i < clients; ++i) {
                int sock = client_fds[i];
                
                if (httpd_ws_get_fd_info(*server, sock) == HTTPD_WS_CLIENT_WEBSOCKET) {

                  // Add your commands to both of the CASE statements below;
                  // This one is for monitoring minus the influence of the millis, 2nd case statement with millis;

                  fdofst = 0x0f & sock;

                  if ((fd_cmd[fdofst] != -1) && perfmessure)
                  {
                   switch(fd_cmd[fdofst])
                    {
                      case 01 : 
                                tmp_wscnt[0]++;break;
                      case 02 :
                                tmp_wscnt[1]++;break;
                      case 03 :
                                tmp_wscnt[2]++;break;
                      case 98 :
                                tmp_wscnt[3]++;break;
                      case 99 :
                                tmp_wscnt[4]++;break;
                      default :
                                tmp_wscnt[5]++;break;
                                break;
                   }
                  }
                   curmillis = millis();

                   if ((fd_cmd[fdofst] != -1) && ((curmillis-fd_curmillis[fdofst]) >= fd_millis[fdofst]))
                   {
                          struct async_resp_arg *resp_arg = ( async_resp_arg *) malloc(sizeof(struct async_resp_arg));
                          resp_arg->hd = *server;
                          resp_arg->fd = sock;

                          switch(fd_cmd[fdofst])
                          {
                            case 01 :
                                        if (httpd_queue_work(resp_arg->hd,  (httpd_work_fn_t)  fd_cmdresp_001, resp_arg) != ESP_OK)
                                          send_messages = false;
                                        break;
                            case 02 :
                                        if (httpd_queue_work(resp_arg->hd,  (httpd_work_fn_t)  fd_cmdresp_002, resp_arg) != ESP_OK)
                                          send_messages = false;
                                        break;
                            case 03 :
                                        if (httpd_queue_work(resp_arg->hd,  (httpd_work_fn_t)  fd_cmdresp_002, resp_arg) != ESP_OK)
                                          send_messages = false;                                         
                                        break;
                            case 04 :
                                        if (httpd_queue_work(resp_arg->hd,  (httpd_work_fn_t)  fd_cmdresp_004, resp_arg) != ESP_OK)
                                          send_messages = false;                                         
                                        break;
                            case 05 :
                                        if (httpd_queue_work(resp_arg->hd,  (httpd_work_fn_t)  fd_cmdresp_005, resp_arg) != ESP_OK)
                                          send_messages = false;                                         
                                        break;

                            case 98 :
                                        if (httpd_queue_work(resp_arg->hd,  (httpd_work_fn_t)  fd_cmdresp_098, resp_arg) != ESP_OK) 
                                          send_messages = false;
                                        break;
                            case 99 :
                                        if (httpd_queue_work(resp_arg->hd,  (httpd_work_fn_t)  fd_cmdresp_099, resp_arg) != ESP_OK)
                                          send_messages = false;
                                        break;
                            default:
                                        break;
                          }
                          fd_curmillis[fdofst] = curmillis;                          
                   }


                }
            }

            if (perfmessure)
            {
             for(j=0;j<=WSCMDCNT;j++)
                    wscnt[j] = tmp_wscnt[j];

             if (tmp_wscnt[3] == 0)
                    perfmon_stop();                
            }
            perfmessure = 0;

        } else {
            ESP_LOGE(TAG, "httpd_get_client_list failed!");
            return;
        }
   
}


void ws_timer()
{
   wss_server_send_messages(&obd_httpd);
}

static esp_err_t echo_handler(httpd_req_t *req)
{
int fdofst;

    if (req->method == HTTP_GET) 
        return ESP_OK;

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
        return ret;

    if (ws_pkt.len) {

        buf = (uint8_t *) calloc(1, ws_pkt.len + 1);
        if (buf == NULL) 
            return ESP_ERR_NO_MEM;

        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(buf);
            return ret;
        }
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,FD_CMD_001) == 0) {
        Serial.printf("%s \n",FD_CMD_001);  
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_001;
        fd_millis[fdofst]=FD_DUR_001;
        free(buf);            
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,FD_CMD_002) == 0) {
        Serial.printf("%s \n",FD_CMD_002);  
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_002;
        fd_millis[fdofst]=FD_DUR_002;
        free(buf);            
        return ESP_OK;
    }
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,FD_CMD_003) == 0) {
        Serial.printf("%s \n",FD_CMD_003);  
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_003;
        fd_millis[fdofst]=FD_DUR_003;
        free(buf);            
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,FD_CMD_004) == 0) {
        Serial.printf("%s \n",FD_CMD_004);  
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_004;
        fd_millis[fdofst]=FD_DUR_004;
        free(buf);            
        return ESP_OK;
    }


    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,FD_CMD_005) == 0) {
        Serial.printf("%s \n",FD_CMD_005);  
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_005;
        fd_millis[fdofst]=FD_DUR_005;
        free(buf);            
        return ESP_OK;
    }
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,FD_CMD_098) == 0) {
        Serial.printf("%s \n",FD_CMD_098);  
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_098;
        fd_millis[fdofst]=FD_DUR_098;
        free(buf); 
        perfmon_start(); // Start the Perfomance Monitor Tasks           
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,FD_CMD_099) == 0) {
        Serial.printf("%s \n",FD_CMD_099);  
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_099;
        fd_millis[fdofst]=FD_DUR_099;
        free(buf);            
        return ESP_OK;
    }

  if (ws_pkt.type == HTTPD_WS_TYPE_TEXT ) {
        fdofst = httpd_req_to_sockfd(req) & 0x0f;
        fd_cmd[fdofst]=FD_CID_DIS;
        fd_millis[fdofst]=FD_DUR_DIS;
  }

    ret = httpd_ws_send_frame(req, &ws_pkt);
    free(buf);
    return ret;
}




void storePID(int pid, int ofst, int count, float val)
{
    pidstack[0] = count;
    pidstack[ofst+1] = pid;
    pidstackval[ofst+1]=val;

    if ((ofst+1) == 8)
      update_p1();
}

float GetPIDVal(int ofst)
{
    return(pidstackval[ofst+1]);
}
   
void startOBDServer(){

int i;
csd_t csd;
int sdflag;

  sdflag = 0;

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  if(!SD.cardBegin(SD_CONFIG)){
        Serial.println("Card Mount Failed");
        blinkX_ok(20);
    }
  else
    {
     if (!SD.volumeBegin()) 
        Serial.println("Volume Begin Error");
      else
        sdflag = 1;
    }

  if (sdflag)
  {
    SD.card()->readCSD(&csd);

    Serial.printf("Card type: "); 

    switch (SD.card()->type()) {
      case SD_CARD_TYPE_SD1:
        Serial.printf("SD1\n");
        break;

      case SD_CARD_TYPE_SD2:
        Serial.printf("SD2\n");
        break;

      case SD_CARD_TYPE_SDHC:
       if (csd.capacity() < 70000000) {
          Serial.printf("SDHC\n");
        } else {
          Serial.printf("SDXC\n");
       }
         break;

      default:
        Serial.printf("Unknown\n");
    }

    if (SD.fatType() <= 32) {
      Serial.printf("Volume is FAT%d\n",SD.fatType());
        } else {
      Serial.printf("Volume is exFAT\n");
       }
  
    float ccapacity = (float)csd.capacity() * (float)(0.000512);
    Serial.printf("cardSize:          %ld MBytes\n", (long) ccapacity);
    Serial.printf("sectorsPerCluster: %d\n", SD.sectorsPerCluster());
    Serial.printf("fatStartSector:    %d\n", SD.fatStartSector());
    Serial.printf("dataStartSector:   %d\n", SD.dataStartSector());
    Serial.printf("clusterCount:      %d\n", SD.clusterCount());  
    Serial.printf("Free Clusters:     %d\n",SD.freeClusterCount());
  }
  if(psramInit())
        Serial.println("\nPSRAM is correctly initialized");
  else
    {
        Serial.println("PSRAM not available");
        blinkX_ok(20);
    }

  if (sdflag) 
  {

    // Verify if cc2540.bin is present for CC2540 Flashing

   //Flash_CC2541("/cc2541.bin");
    if (Flash_CC2541("/cc2541.bin"))
          SD.remove("/cc2541.bin");

      char  *sd_data = (char* ) ps_malloc(MAXFILESIZE * sizeof(char));
      // LOAD SD card to SPIFFS only if lock.file does not exist
      if (SDreadFile("/lock.file",  (uint8_t *) sd_data  ) < 5)
     {
          Serial.println("Formatting SPIFFS file system");
          SPIFFS.format();
          Serial.println("Format Done.");
          Serial.println("Copying Files from SD card to SPIFFS");
          SD_LoadSPIFF("/", 5);
          Serial.println("Copy Done."); 
          Serial.printf("Flash Size : %d bytes, SPIFFS Total: %d bytes, SPIFFS Used: %d bytes\n",ESP.getFlashChipSize(), SPIFFS.totalBytes(),SPIFFS.usedBytes());
      }
      free(sd_data);
  }
  for(i=0;i<ToTPIDStack;i++)
    {
      pidstack[i] = 0;
      pidstackval[i] = 0;
    }
  for(i=0;i<conncount;i++)
    {
      fd_cmd[i]=FD_CID_DIS;
      fd_millis[i]=FD_DUR_DIS;
      fd_curmillis[i]=FD_DUR_DIS;
    }
  for(i=0;i<=GraphQueueCount;i++)
    {
        p1[i]=0;
        p2[i]=0;
    }
  for(i=0;i<=CPUGraphCount;i++)
    {
        c0[i]=0;
        c1[i]=0;
    }
  for(i=0;i<=WSCMDCNT;i++)
        wscnt[i]=0;

  for(i=0;i<=TPMS_COUNT;i++)
    {
        tpms_pressure[i]=0;
        tpms_temperature[i]=0;
        tpms_voltage[i]=0;
    }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_open_sockets = 13;
  config.core_id = 1;

  httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

  httpd_uri_t getobd_uri = {
        .uri       = "/getobd",
        .method    = HTTP_GET,
        .handler   = getobd_handler,
        .user_ctx  = NULL
    };

  httpd_uri_t page_uri = {
        .uri       = "/page",
        .method    = HTTP_GET,
        .handler   = page_handler,
        .user_ctx  = NULL
    };


  httpd_uri_t cmd_uri = {
        .uri       = "/index.js",
        .method    = HTTP_GET,
        .handler   = script_handler,
        .user_ctx  = NULL
    };

  httpd_uri_t jquery_uri = {
        .uri       = "/jquery-min.js.gz",
        .method    = HTTP_GET,
        .handler   = jquery_handler,
        .user_ctx  = NULL
    };   

  httpd_uri_t css_uri = {
        .uri       = "/index.css",
        .method    = HTTP_GET,
        .handler   = css_handler,
        .user_ctx  = NULL
    };    
 

  httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = echo_handler,
        .user_ctx   = NULL,
        .is_websocket = true
    };

    Serial.printf("Starting web server on port: '%d'\n", config.server_port);

    if (httpd_start(&obd_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(obd_httpd, &cmd_uri);	
        httpd_register_uri_handler(obd_httpd, &jquery_uri);		
        httpd_register_uri_handler(obd_httpd, &css_uri);	
        httpd_register_uri_handler(obd_httpd, &page_uri);	
        httpd_register_uri_handler(obd_httpd, &index_uri);
		    httpd_register_uri_handler(obd_httpd, &getobd_uri);
		    httpd_register_uri_handler(obd_httpd, &ws); 
    }
   
   
}
