
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


#include <WiFi.h>

#include "CANmod.h"
#include "OBD2_mod.h"

#define blinkDelay 50
#define numBlink 6 

#define LEDPIN GPIO_NUM_2
#define CANRSPIN GPIO_NUM_4
#define CANRXPIN GPIO_NUM_36
#define CANTXPIN GPIO_NUM_22

#define CANACTIVITYCOUNT 2
#define CANNOPACKETCOUNT 10

#define RXD2 GPIO_NUM_13          // ESP32 RX gpio for CC2541 TX pin     
#define TXD2 GPIO_NUM_14          // ESP32 TX gpio for CC2541 RX pin (unused)
#define CC2541RST1 GPIO_NUM_27    // ESP32 gpio pin for CC2541 reset 
#define CC2541Timeout 10          // Readtimeout per 100ms

#define OBDCMDSIZ     10
#define OBDREPSIZ     50

#define MILDUR_001 150
#define MILDUR_002 1000

const char* ssid = "RCSM_DB_OBD_DEBUG";
const char* password = "testpass";
WiFiServer wifiServer(35000);
WiFiClient client;

int clientflag = 0;

unsigned long prevmillis_001,prevmillis_002;

void store_BM2( float voltage);
void store_TPMS(unsigned long tireID, float pressure, float temperature, float voltage);
void startOBDServer();
void storePID(int pid, int ofst, int count, float val);
float GetPIDVal(int ofst);
void ws_timer();

// array of PID's to print values of
const int PIDS[] = {
  ENGINE_RPM,
  VEHICLE_SPEED,
  THROTTLE_POSITION,
  ENGINE_COOLANT_TEMPERATURE,
  AIR_INTAKE_TEMPERATURE,
  MAF_AIR_FLOW_RATE,
  CALCULATED_ENGINE_LOAD,
  CONTROL_MODULE_VOLTAGE,
  RELATIVE_THROTTLE_POSITION,
  CATALYST_TEMPERATURE_BANK_1_SENSOR_1 
};

const int NUM_PIDS = sizeof(PIDS) / sizeof(PIDS[0]);

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5       /* Time ESP32 will go to sleep (in seconds) */

const String ATE = "ATE";	// Echi
const String  ATI = "ATI"; 	// Vid
const String ATZ = "ATZ"; 	// Reset
const String ATS = "ATS";	// Set proto
const String ATH0 = "ATH0"; // Headers OFF
const String ATH1 = "ATH1"; // Headers ON
const String ATL = "ATL"; // Linefeeds off/on
const String ATM = "ATM"; // Memory off/on
const String ATDPN = "ATDPN"; //Describe the Protocol by Number
const String ATDESC = "AT@1";
const String ATAT = "ATAT";
const String ATR = "ATR";
const String ATTP = "ATTP";


const char* LF = "\n";
const char* VERSION = "ELM327 v2.1";
const char* VERSION_DESC = "ESP32 ELM327 Interpreter";
const char* ATOK = "OK";
const char* PROMPT = ">";
const char* ISO15765_4 = "A6"; // ISO ISO 15765-4
const char* ATRVREP = "12.0"; // ISO ISO 15765-4

// Add a "Q" character in front to disable buffering (more realtime but with adapter readout delay vs cached)

const String REQ0100 = "Q0100";
const String REQ0120 = "Q0120";

const String REQ0105 = "0105";
const String REQ010C = "010C";
const String REQ010D = "010D";

const String REQ0111 = "0111";
const String REQ0142 = "0142";
const String REQ0143 = "0143";

String ELMClientStream = "";
String ELMClientStream_BLE = "";


QueueHandle_t queue_T0s;
QueueHandle_t queue_T0r;
QueueHandle_t queue_T1s;
QueueHandle_t queue_T1r;

uint8_t OBD_headerflag = 0;


void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void TimerTask(void * parameter){
  for(;;){ // infinite loop
    ws_timer();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void blink_ok()
{
    int i;

    digitalWrite(LEDPIN, LOW);
    delay(blinkDelay);
    for(i=0;i<numBlink;i++)
    {
        delay(blinkDelay);
        digitalWrite(LEDPIN, HIGH);
        delay(blinkDelay);
        digitalWrite(LEDPIN, LOW);
    }
}


void ELM_AT(String command, int procID,  char ptr[]) {

int i,j, numline;
  
uint32_t pidData;
uint8_t b1,b2,b3,b4;
char  reply0[OBDREPSIZ];
char  reply1[OBDREPSIZ];
  
   ptr[0]=0;  

   if (command.equals(ATZ)) { ptr+=sprintf(ptr,VERSION);ptr+=sprintf(ptr,LF);ptr+=sprintf(ptr,ATOK);} 
   else if (command.startsWith(ATE)) { ptr+=sprintf(ptr,ATOK);} 
   else if (command.startsWith(ATI)) { ptr+=sprintf(ptr,VERSION);ptr+=sprintf(ptr,LF);ptr+=sprintf(ptr,ATOK);} 
   else if (command.startsWith(ATDESC)) { ptr+=sprintf(ptr,VERSION);ptr+=sprintf(ptr,LF);ptr+=sprintf(ptr,ATOK);} 
   else if (command.startsWith(ATL)) { ptr+=sprintf(ptr,ATOK);} 
   else if (command.startsWith(ATAT)) { ptr+=sprintf(ptr,ATOK);}
   else if (command.startsWith(ATTP)) { ptr+=sprintf(ptr,ATOK);}
   else if (command.startsWith(ATH0)) {
       OBD_headerflag = 0;
       ptr+=sprintf(ptr,ATOK);
   } 
   else if (command.startsWith(ATH1)) {
       OBD_headerflag = 1;
       ptr+=sprintf(ptr,ATOK);
   } 
   else if (command.startsWith(ATM)) { ptr+=sprintf(ptr,ATOK); } 
   else if (command.startsWith(ATR)) { ptr+=sprintf(ptr,ATRVREP); } 
   else if (command.startsWith(ATS)) { ptr+=sprintf(ptr,ATOK); } 
   else if (command.startsWith(ATDPN)) { ptr+=sprintf(ptr,ISO15765_4);} 
   else if (command.startsWith(REQ0143)) {
      if (OBD_headerflag)
           ptr+=sprintf(ptr,"%03x 04 ",0x7e8);
      ptr+=sprintf(ptr,"41 43 00 %02x",(int)GetPIDVal(6) &0xff); 
   } 
   else if (command.startsWith(REQ010C)) {
      if (OBD_headerflag)
           ptr+=sprintf(ptr,"%03x 04 ",0x7e8);
      ptr+=sprintf(ptr,"41 0C %02x %02x",((int)(GetPIDVal(0)*4) >> 8) & 0xff,((int)GetPIDVal(0)*4) &0xff);  
   } 
   else if (command.startsWith(REQ010D)) {
      if (OBD_headerflag)
           ptr+=sprintf(ptr,"%03x 03 ",0x7e8);
      ptr+=sprintf(ptr,"41 0D %02x",(int)GetPIDVal(1) &0xff);  
   } 
   else if (command.startsWith(REQ0105)) {
      if (OBD_headerflag)
           ptr+=sprintf(ptr,"%03x 03 ",0x7e8);
      ptr+=sprintf(ptr,"41 05 %02x",(int)(GetPIDVal(3)+40) &0xff)     ;  
   } 
   else if (command.startsWith(REQ0142)) {
      if (OBD_headerflag)
           ptr+=sprintf(ptr,"%03x 04 ",0x7e8);
     ptr+=sprintf(ptr,"41 42 %02x %02x",((int)(GetPIDVal(7)*1000) >> 8) & 0xff,((int)GetPIDVal(7)*4) &0xff)     ;  
  } 
  else if (command.startsWith(REQ0111)) {
      if (OBD_headerflag)
          ptr+=sprintf(ptr,"%03x 03 ",0x7e8);
      ptr+=sprintf(ptr,"41 11 %02x",(int)(GetPIDVal(2)*2.55) &0xff)     ;  
  } 
  else if (command.startsWith("AT")) { ptr+=sprintf(ptr,ATOK); } // Reply with the rest of AT Commands to OK
  else if (command.indexOf("AT") == -1) {

   if ((command.length() > 2) && (command[0] >= 0x30))  
   {
     if (procID)
      {
            xQueueSend(queue_T1s,&command,(TickType_t )0);
            if ( xQueueReceive(queue_T1r,&reply1,(TickType_t )(3000/portTICK_PERIOD_MS)))
            {
                  if (reply1[0] !=0 )
                  {
                    if (OBD_headerflag)
                      ptr+=sprintf(ptr,"%03x 04 ",0x7e8);
                    ptr+=sprintf(ptr,"41 %c%c ", command[2],command[3]);
                  
                    for(i=0;i<reply1[0];i++)    
                    {
                      if((i+1)<reply1[0])
                          ptr+=sprintf(ptr,"%02x ", reply1[i+1]);
                      else
                         ptr+=sprintf(ptr,"%02x", reply1[i+1]);
                    }
                  }
                  else
                  {
                      if (OBD_headerflag)
                         ptr+=sprintf(ptr,"%03x 03 ",0x7e8);
                      ptr+=sprintf(ptr,"41 %c%c 00",command[2],command[3]);
                  }
            }
            else // If not response after 1000 ms, make a default reply
            {
              if (OBD_headerflag)
                 ptr+=sprintf(ptr,"%03x 03 ",0x7e8);
              ptr+=sprintf(ptr,"41 %c%c 00",command[2],command[3]);
            }
      }
      else
      {
            xQueueSend(queue_T0s,&command,(TickType_t )0);
            if ( xQueueReceive(queue_T0r,&reply0,(TickType_t )(3000/portTICK_PERIOD_MS)))
            {
                  if (reply0[0] !=0 )
                  {
                    if (OBD_headerflag)
                      ptr+=sprintf(ptr,"%03x 04 ",0x7e8);
                    ptr+=sprintf(ptr,"41 %c%c ", command[2],command[3]);
                  
                    for(i=0;i<reply0[0];i++)    
                    {
                      if((i+1)<reply0[0])
                        ptr+=sprintf(ptr,"%02x ", reply0[i+1]);
                      else
                        ptr+=sprintf(ptr,"%02x", reply0[i+1]);
                    }
                  }
                  else
                  {
                      if (OBD_headerflag)
                         ptr+=sprintf(ptr,"%03x 03 ",0x7e8);
                      ptr+=sprintf(ptr,"41 %c%c 00",command[2],command[3]);
                  }
             }
            else // If not response after 1000 ms, make a default reply
            {
              if (OBD_headerflag)
                 ptr+=sprintf(ptr,"%03x 03 ",0x7e8);
              ptr+=sprintf(ptr,"41 %c%c 00",command[2],command[3]);
            }
      }
    }
   }
   ptr+=sprintf(ptr,LF);
   ptr+=sprintf(ptr,PROMPT);
   *ptr++ = 0;
}


void GetPIDs(int ofst) {

   int pid;
   pid = PIDS[ofst];

//     Serial.print("_");
   float pidValue = (float) OBD2mod.pidRead(pid);

    //   Serial.print(".");

   if (pid == 12) 
    {
       if (isnan(pidValue))
       {
            delay(1000);
            pidValue = (float) OBD2mod.pidRead(pid);
            if (isnan(pidValue))
            {
                  CANmod.end();
                  ESP.restart();    // Force boot

            }
       } 
    }

  if (!isnan(pidValue))
    storePID(pid, ofst, NUM_PIDS, pidValue);
   vTaskDelay(2 / portTICK_PERIOD_MS);
}

void obd_task(void * parameter){

unsigned long curmillis;
char cmd[OBDCMDSIZ];
char reply[OBDREPSIZ];
char * rrdata = reply;
char obdbuf[OBDREPSIZ];
int i,len;

char  strval[5];

for(;;){ // infinite loop

    curmillis = millis();
    if ((curmillis - prevmillis_001)> MILDUR_001)
    {
      GetPIDs(0);
      GetPIDs(1);
      GetPIDs(2);
      prevmillis_001 = curmillis;
    }
    if ((curmillis - prevmillis_002)> MILDUR_002)
    {    
      digitalWrite(LEDPIN, HIGH);
      for (int i = 3; i < NUM_PIDS; i++)
          GetPIDs(i);
      prevmillis_002 = curmillis;
      digitalWrite(LEDPIN, LOW);
    }

    if (xQueueReceive(queue_T0s,&cmd,(TickType_t )(5/portTICK_PERIOD_MS)))
    {
        sprintf(strval,"%c%c",cmd[0],cmd[1]);
        uint8_t pidmode = strtol(strval, NULL, 16);
        sprintf(strval,"%c%c",cmd[2],cmd[3]);
        uint8_t pidval = strtol(strval, NULL, 16);
      //  curmillis = millis();
        len = OBD2mod.pidReadPUB(pidmode, pidval, &obdbuf, 4);
       // Serial.printf("CAN Read:[%05d] Len: %02d Cmd: %s Reply: %02x %02x %02x %02x \n",millis()-curmillis,len,cmd, obdbuf[0],obdbuf[1],obdbuf[2],obdbuf[3]);
        reply[0] = len;
        for(i=0;i<len;i++)
           reply[i+1] = obdbuf[i];
        reply[i] = 0;
        xQueueSend(queue_T0r,&reply,(TickType_t )0);
    }
    if (xQueueReceive(queue_T1s,&cmd,(TickType_t )(5/portTICK_PERIOD_MS)))
    {
        sprintf(strval,"%c%c",cmd[0],cmd[1]);
        uint8_t pidmode = strtol(strval, NULL, 16);
        sprintf(strval,"%c%c",cmd[2],cmd[3]);
        uint8_t pidval = strtol(strval, NULL, 16);
      //  curmillis = millis();
        len = OBD2mod.pidReadPUB(pidmode, pidval, &obdbuf, 4);
      //  Serial.printf("CAN Read:[%05d] Len: %02d Cmd: %s\n",millis()-curmillis,len,cmd);
        reply[0] = len;
        for(i=0;i<len;i++)
           reply[i+1] = obdbuf[i];
        reply[i] = 0;
        xQueueSend(queue_T1r,&reply,(TickType_t )0);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }  
}

void elm_BLEtask(void * parameter){

static char buf[1024];
char c;
unsigned long curmillis;

for(;;){ // infinite loop

   while (Serial2.available()) {
    c = Serial2.read();
    if ((c == '\n' || c == '\r') && ELMClientStream_BLE.length() > 0) {
            ELMClientStream_BLE.toUpperCase();
          //  curmillis = millis();
            ELM_AT(ELMClientStream_BLE, 0,(char *) buf  );
          //   Serial.printf("Duration:[%05d] CMD: %s Reply: %s",millis()-curmillis,ELMClientStream_BLE,buf);
            Serial2.print(buf);            
            ELMClientStream_BLE = "";
     } else if (c != ' ' && c != '\n' && c !='\r') {
        ELMClientStream_BLE += c; 
     }

     vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);

 }
}



void Vc601_BLEtask(void * parameter){


char tbuf[30];
char inp;
int i,len;
int timout;

long tmpval,tmpval2,tmpval3,event;
double pressure;
double temperature;
double voltage;
double prevoltage = 0;
double prevoltage2 = 0;
int bm2count = 0;

for(;;){ // infinite loop

 while (Serial2.available()) {
    inp = Serial2.read();

    if (inp == 0xfe)
    {
        len = Serial2.read();
        if (len == 15)
        {
          for(i = 0;i<len;i++)
          {
              timout = 0;
              while ((!Serial2.available()) && (timout < CC2541Timeout))
              {
                  timout++;
                  vTaskDelay(100 / portTICK_PERIOD_MS);
              }
              if (timout >= CC2541Timeout)
              {
                  Serial.println("CC2541 Timeout Error");
                  goto skip;  // Abort Loop - This will happen only if we did not get the 15 byte data
              }
              tbuf[i] = Serial2.read();
              vTaskDelay(1 / portTICK_PERIOD_MS);
          }

          if ((tbuf[3] == 0xca) && (tbuf[4] == 0xEA))
          {

            tmpval = 0;
            tmpval2 = 0;
            tmpval3 = 0;
		
            memcpy(&tmpval,  tbuf+6, 4);
            memcpy(&tmpval2, tbuf+10,4);
            memcpy(&tmpval3, tbuf+14,1);
            pressure =    (double) ((tmpval / 100000.00) * 14.5);
            temperature = (double) (tmpval2 / 100.00);

            if ((tmpval3 >= 0) && (tmpval3 <= 4))
              voltage  = (((double) (((((((tmpval3 + 0) << 16) / 4) * 224) >> 16) + 1136) / 2)) / 1023.0) * 3.6;
            if ((tmpval3 >= 5) && (tmpval3 <= 28))
              voltage  = (((double) (((((((tmpval3 - 4) << 16) / 24) * 224) >> 16) + 1360) / 2)) / 1023.0) * 3.6;
            if ((tmpval3 >= 29) && (tmpval3 <= 100))
              voltage  = (((double) (((((((tmpval3 - 28) << 16) / 72) * 121) >> 16) + 1584) / 2)) / 1023.0) * 3.6;
                
    //        Serial.printf("TPMS Broadcast - Sensor[%x]: ",tbuf[05] & 0xf);        
    //        for(i=0;i<6;i++)
    //          Serial.printf("%02x ",tbuf[i]);        
    //        Serial.printf(" - Pressure: %5.2f psi, Temperature: %5.2f degC, Sensor Voltage: %4.2f V\n",pressure,temperature,voltage);

          store_TPMS( (tbuf[2] << 16)+(tbuf[1] << 8)+tbuf[0], pressure, temperature, voltage);

          }
          else
          {
            bm2count++;

            voltage = (double)(( (double)(tbuf[6]+(tbuf[7] << 8)) / 304.16) -7.1282 + 0.06);

            if (bm2count > 2)
            store_BM2(voltage);

      //      Serial.printf("BM2 Broadcast  -          : ");        
      //      for(i=0;i<6;i++)
      //        Serial.printf("%02x ",tbuf[i]);        
      //      Serial.printf(" - %02x%02x == %02f Volts\n",tbuf[7],tbuf[6],(voltage+prevoltage+prevoltage2)/3);
  
            prevoltage2 = prevoltage;
            prevoltage = voltage;

          }

        }
    }
    else
      Serial.print(inp);

  skip: 
  
  vTaskDelay(10 / portTICK_PERIOD_MS);

  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
 }
}
/*
void elm_WIFItask(void * parameter){

static char buf[1024];
char c;
unsigned long curmillis;

for(;;){ // infinite loop


 client = wifiServer.available();
 
 if (client) {
 
  while (client.connected()) {
      clientflag = 1;
 
   while (client.available()) {
    c = client.read();
    if ((c == '\n' || c == '\r') && ELMClientStream.length() > 0) {
            ELMClientStream.toUpperCase();
         //   curmillis = millis();
            ELM_AT(ELMClientStream, 1,(char *) buf  );
         //   Serial.printf("Duration:[%05d] CMD: %s Reply: %s",millis()-curmillis,ELMClientStream,buf);
            client.print(buf);            
            ELMClientStream = "";
     } else if (c != ' ' && c != '\n' && c !='\r') {
        ELMClientStream += c; 
     }

    vTaskDelay(1 / portTICK_PERIOD_MS);
    }
 
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    //else
    if (clientflag)
    {
      client.stop();
      Serial.println("Client disconnected");
      clientflag = 0;
    }
   }
  vTaskDelay(1 / portTICK_PERIOD_MS);
 }
}

*/



void CAN_Sniff()
{

   int actv,noactv;

  CANmod.begin(500E3);
  CANmod.observe();
  actv = 0;
  noactv = 0;
  do {

     if (CANmod.parsePacket() > 0)
     { 
        //Serial.println(CANmod.packetId(), HEX);
        while (CANmod.available()) {
           CANmod.read();
      }
        actv++;
        noactv = 0;
     }
    else
    {
       noactv++;
        if (noactv > CANNOPACKETCOUNT)
        {
            CANmod.end();
            Serial.printf("No OBD Activity -- ");
            esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // ESP32 wakes up every 5 seconds
            Serial.println("Sleeping for 5 seconds in POWER SAVE MODE ...");
            Serial.flush(); 
            esp_light_sleep_start();
            
            // Not sure really if this part is needed.
            // This is just to ensure that least power is consumed during the CAN Bus Polling
            
            pinMode(CANRSPIN,OUTPUT);
            digitalWrite(CANRSPIN, HIGH);
            
            pinMode(CC2541RST1, OUTPUT);
            digitalWrite(CC2541RST1, LOW);

            CANmod.begin(500E3);
            CANmod.observe();
            actv = 0;
            noactv = 0;

            //ESP.restart();
        }
    }
      vTaskDelay(200 / portTICK_PERIOD_MS);
  }
  while(actv < CANACTIVITYCOUNT);
  CANmod.end();
  Serial.println("CAN BUS activity detected ... Starting OBD2 in 5secs ...");
  delay(5000);
  pinMode(CANRSPIN,INPUT);
  digitalWrite(CC2541RST1, HIGH);

}


void setup() {


 int actv,noactv;

  // Put the CAN Transceiver in LISTEN MODE to avoid CAR ECU errors
  pinMode(CANRSPIN,OUTPUT);
  digitalWrite(CANRSPIN, HIGH);

  pinMode(CC2541RST1, OUTPUT);
  digitalWrite(CC2541RST1, LOW);
  
  Serial.begin(115200);
  //Serial.println("AT+BAUD8");

  while (!Serial);
  Serial.setDebugOutput(true);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  CANmod.setPins(CANRXPIN, CANTXPIN);

 // blink_ok();

  CAN_Sniff();

  while (true) {
    Serial.print(F("Attempting to connect to OBD2 CAN bus ... "));

    if (!OBD2mod.begin()) {
          Serial.println(F("failed!"));
         // print_wakeup_reason();
          esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // ESP32 wakes up every 5 seconds
          Serial.println("Sleeping for 5 seconds in POWER SAVE MODE ...");
          Serial.flush(); 
          esp_light_sleep_start();
          //esp_deep_sleep_start();
          print_wakeup_reason(); //Print the wakeup reason for ESP32


    } else
    {
          Serial.println(F("success"));
          break;
    }
  }


  Serial2.begin(115200,SERIAL_8N1,RXD2,TXD2);


  //Serial2.print("AT+BAUD9");
 // Serial2.print("AT+NAMERCS_BLE");
  WiFi.softAPConfig(IPAddress(192, 168, 0, 10), IPAddress(192, 168, 0, 10), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  wifiServer.begin();

  Serial.println();
  startOBDServer();

  blink_ok();

  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());

  Serial.print("OBDII Ready! Use 'http://");
  Serial.print(IP);
  Serial.println("' to connect");

  // Create the the OBD Queues

  queue_T0s = xQueueCreate(2,OBDCMDSIZ);
  queue_T0r = xQueueCreate(2,OBDREPSIZ);
  queue_T1s = xQueueCreate(2,OBDCMDSIZ);
  queue_T1r = xQueueCreate(2,OBDREPSIZ);

  // Create the tasks
 
  xTaskCreatePinnedToCore(
    TimerTask,    // Function that should be called
    "WsTimerTask",   // Name of the task (for debugging)
    5000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1 | portPRIVILEGE_BIT,               // Task priority
    NULL,             // Task handle
    1
  );


  prevmillis_001 = 0;
  prevmillis_002 = 0;

  xTaskCreatePinnedToCore(
    obd_task,    // Function that should be called
    "OBDTimerTask",   // Name of the task (for debugging)
    7000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1 | portPRIVILEGE_BIT,               // Task priority
    NULL,             // Task handle
    1
  );

// Uncomment Below to Activate ELm327 on BLE JDY08 with HM10 or JDY08 original firmware
// Need to comment Vc601_BLEtask below if ele_BLEtask is uncommented


/*
  xTaskCreatePinnedToCore(
    elm_BLEtask,    // Function that should be called
    "OBDBLETask",   // Name of the task (for debugging)
    7000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1 | portPRIVILEGE_BIT,               // Task priority
    NULL,             // Task handle
    1
  );
*/

  xTaskCreatePinnedToCore(
    Vc601_BLEtask,    // Function that should be called
    "OBDBLETask",   // Name of the task (for debugging)
    7000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1 | portPRIVILEGE_BIT,               // Task priority
    NULL,             // Task handle
    1
  );

/*
  xTaskCreatePinnedToCore(
    elm_WIFItask,    // Function that should be called
    "OBDWIFITask",   // Name of the task (for debugging)
    7000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1 | portPRIVILEGE_BIT,               // Task priority
    NULL,             // Task handle
    1
  );

*/

  // Reset the CC2541 at Serial initialization to allow the ESP32 to get the init message

  delay(500);
  digitalWrite(CC2541RST1, LOW);
  delay(500);
  digitalWrite(CC2541RST1, HIGH); 


}

void loop() {

static char buf[1024];
char c;
unsigned long curmillis;

 client = wifiServer.available();
 
 if (client) {
 
  while (client.connected()) {
      clientflag = 1;
 
   while (client.available()) {
    c = client.read();
    if ((c == '\n' || c == '\r') && ELMClientStream.length() > 0) {
            ELMClientStream.toUpperCase();
         //   curmillis = millis();
            ELM_AT(ELMClientStream, 1,(char *) buf  );
         //   Serial.printf("Duration:[%05d] CMD: %s Reply: %s",millis()-curmillis,ELMClientStream,buf);
            client.print(buf);            
            ELMClientStream = "";
        } else if (c != ' ' && c != '\n' && c !='\r') {
            ELMClientStream += c; 
          }

  }
      vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  if (clientflag)
    {
      client.stop();
      Serial.println("Client disconnected");
      clientflag = 0;
    }
   }

  vTaskDelay(1 / portTICK_PERIOD_MS);
 
}
