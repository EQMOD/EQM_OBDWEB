
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




/*
 * 
  THis was modified from the base codes below using the modfied CAN Libraries of this repository

  
  ECU emulator v0.01
   works as a  500kHz basic CAN bus OBD2 ECU
   Author: coniferconifer
   May 19 , 2019
   Copyright 2017 coniferconifer
    
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.


   
   OBD2 ECU emulator based on CAN and OBD2 library by sandeepmistry
   // https://github.com/sandeepmistry/arduino-CAN
   // https://github.com/sandeepmistry/arduino-OBD2

   This program is used to test OBD2 monitoring program at  
   https://github.com/sandeepmistry/arduino-OBD2/blob/master/examples/OBD2_03_DataPrinter/OBD2_03_DataPrinter.ino

   ESP32(ECU emulator)---TJA1050-----CAN bus connector----TJA1050 ---ESP32(OBD2_02_DataPrinter.ino)
   before I connect ESP32(OBD2_02_DataPrinter.ino) ---TJA1050---CAN bus  system to
   my car.

   
   TJA1050 CAN tranceiver board is connected to ESP32 devkitC board via
   Vcc <- 5V  ESP32
   GND <- GND ESP32
   CTX <--- GPIO_5 ESP32 devkitC board
   CRX ---> 5V/3.3V level converter ---> GPIO_4
   I have used 1N5819 shotkey diode and 1kOhm pulled up to 3.3V
   to connect CRX to ESP32. CTX is directly connected to ESP32 GPIO_5
   +3.3V ----1kOhm----+
                      |
       ESP32 GPIO4 <--+----anode IN1589 cathode>|---<-CRX

*/
#include "CANmod.h"
#include "OBD2_mod.h"

//#define LEDPIN0 GPIO_NUM_12
//#define LEDPIN0 GPIO_NUM_4 // ESP32 CAM BRIGHT LED
#define LEDPIN0 GPIO_NUM_33

#define ESP32
#ifdef ESP32
TaskHandle_t Task1;
#endif

#define ECU_ID 0x7e8 //ECU id + 8
#define MIL_ON 0x80
#define DTC_CNT 0x01
uint8_t DTC[] = { MIL_ON | DTC_CNT , 0x00, 0x00} ; //A,B,C,D
uint16_t freezeDTC = 0x1234;
uint16_t fuelSystemStatus = 0x0200;
float engineEfficiency = 22.75; //%

float shortTermFuelTrimBank1 = -5.47;//%

float longTermFuelTrimBank1 = 7.2;//%

float shortTermFuelTrimBank2 = -5.47;//%

float longTermFuelTrimBank2 = 7.2;//%


uint8_t fuelPressure = 765;
uint8_t intakeManifoldAbsolutePressure = 255;
uint16_t engineRPM = 3925;
uint8_t vehicleSpeed = 120;
uint8_t timingAdvance = 8;
uint8_t airIntakeTemperature = 42 ;
float mafAirFlowRate = 6.50 ;
float throttlePosition = 5.5 ;//%
uint8_t commandedSecondaryAirStatus = 1;
uint8_t oxygenSensorsPresentIn2Banks = 3;

float oxygenSensor1 = 1.23 ; //Volt
float shortTermFuelTrim = 99.2; //%

uint8_t obdStandardThisVehicleConformsTo = 0x0a;
uint8_t oxygenSensorsPresentIn4Banks = 0x01;
uint8_t auxiliaryInputStatus = 0x01;
uint16_t distanceTraveledWithMilOn = 1000; // km DISTANCE_TRAVELED_WITH_MIL_ON
float fuelRailPressure = 5177.265; //kPa
float fuelRailGaugePressure = 655350.0; //kPa
uint8_t engineCoolantTemperature = 82;
uint8_t fuelLevel = 80; //%

float oxygenSensorFuelAir = 1.0; //ratio
float oxygenSensorFuelAirVoltage = 1.23; //volt max 8V

float controlModuleVoltage = 14.24;//Volt
float commandedEGR = 1.8; //%
float EGRError = 0.0 ; //%
float commandedEvaporativePurge = 10.0; //% COMMANDED_EVAPORATIVE_PURGE
uint8_t warmUpsSinceCodesCleared = 255; // counts WARM_UPS_SINCE_CODES_CLEARED
uint16_t distanceTraveledSinceCodesCleared = 30613; //km DISTANCE_TRAVELED_SINCE_CODES_CLEARED
float evapSystemVaporPressure = 21.34 ; // EVAP_SYSTEM_VAPOR_PRESSURE
uint8_t absoluteBarometricPressure = 215; //kPa ABSOLULTE_BAROMETRIC_PRESSURE

float catalystTemperatureBank1Sensor1 = 48.0; //dgree CATALYST_TEMPERATURE_BANK_1_SENSOR_1
float absoluteLoadValue = 19.61; //% ABSOLUTE_LOAD_VALUE
uint8_t ambientAirTemperature = 31; //degree
uint8_t fuelType = 1; //FUEL_TYPE
int i = 0;


uint8_t pidList1[4] = {0xff  , 0xff, 0xff, 0xff} ;
uint8_t pidList2[4] = {0xff  , 0xff, 0xff, 0xff} ;
uint8_t pidList3[4] = {0xff  , 0xff, 0xff, 0xff} ;
uint8_t pidList4[4] = {0xff  , 0xff, 0xff, 0xff} ;
uint8_t pidList5[4] = {0xff  , 0xff, 0xff, 0xff} ;
uint8_t pidList6[4] = {0xff  , 0xff, 0xff, 0xff} ;
uint8_t pidList7[4] = {0xff  , 0xff, 0xff, 0xff} ;
void setPidList1_20(uint8_t pid)
{

  Serial.printf("PidList1_20 %d %02x %02x %02x %02x\r\n", pid, pidList1[0], pidList1[1], pidList1[2], pidList1[3]);
  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(pid);
  CANmod.write(pidList1[0]); CANmod.write(pidList1[1]); CANmod.write(pidList1[2]); CANmod.write(pidList1[3]);
  CANmod.endPacket();
}
void setPidList21_40(uint8_t pid)
{

  Serial.printf("PidList1_20 %d %02x %02x %02x %02x\r\n", pid, pidList2[0], pidList2[1], pidList2[2], pidList2[3]);
  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(pid);
  CANmod.write(pidList2[0]); CANmod.write(pidList2[1]); CANmod.write(pidList2[2]); CANmod.write(pidList2[3]);
  CANmod.endPacket();
}
void setPidList41_60(uint8_t pid)
{

  Serial.printf("PidList41-60 %d %02x %02x %02x %02x\r\n", pid, pidList3[0], pidList3[1], pidList3[2], pidList3[3]);

  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(pid);
  CANmod.write(pidList3[0]); CANmod.write(pidList3[1]); CANmod.write(pidList3[2]); CANmod.write(pidList3[3]);
  CANmod.endPacket();
}
void setPidList61_80(uint8_t pid)
{

  Serial.printf("PidList61-80 %d %02x %02x %02x %02x\r\n", pid, pidList4[0], pidList4[1], pidList4[2], pidList4[3]);

  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(pid);
  CANmod.write(pidList4[0]); CANmod.write(pidList4[1]); CANmod.write(pidList4[2]); CANmod.write(pidList4[3]);
  CANmod.endPacket();
}
void setPidList81_a0(uint8_t pid)
{

  Serial.printf("PidList81-a0 %d %02x %02x %02x %02x\r\n", pid, pidList5[0], pidList5[1], pidList5[2], pidList5[3]);

  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(pid);
  CANmod.write(pidList5[0]); CANmod.write(pidList5[1]); CANmod.write(pidList5[2]); CANmod.write(pidList5[3]);
  CANmod.endPacket();
}
void setPidLista1_c0(uint8_t pid)
{

  Serial.printf("PidListA1-C0 %d %02x %02x %02x %02x\r\n", pid, pidList6[0], pidList6[1], pidList6[2], pidList6[3]);

  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(pid);
  CANmod.write(pidList6[0]); CANmod.write(pidList6[1]); CANmod.write(pidList6[2]); CANmod.write(pidList6[3]);
  CANmod.endPacket();
}
void setPidListc1_e0(uint8_t pid)
{


  Serial.printf("PidListC1-E0 %d %02x %02x %02x %02x\r\n", pid, pidList7[0], pidList7[1], pidList7[2], pidList7[3]);

  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(pid);
  CANmod.write(pidList7[0]); CANmod.write(pidList7[1]); CANmod.write(pidList7[2]); CANmod.write(pidList7[3]);
  CANmod.endPacket();
}

/*
   Monitor status since DTCs cleared.
   (Includes malfunction indicator lamp (MIL) status and number of DTCs.)
*/
void setDTC(uint8_t *DTC)
{
  uint8_t *dtc = DTC;
  Serial.println("DTC");
  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(MONITOR_STATUS_SINCE_DTCS_CLEARED);
  CANmod.write(*dtc++);
  CANmod.write(*dtc++);
  CANmod.write(*dtc++);
  CANmod.write(*dtc);
  CANmod.endPacket();
}
void set4bytes(uint32_t data, uint8_t PID)
{
  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x06); //DLC 6
  CANmod.write(0x41);
  CANmod.write(PID);
  CANmod.write((uint8_t)(data >> 24));
  CANmod.write((uint8_t)(data >> 16));
  CANmod.write((uint8_t)(data >> 8));
  CANmod.write((uint8_t)(data & 0x000000ff));
  CANmod.endPacket();
}
void set2bytes(uint16_t data, uint8_t PID) {
  CANmod.beginPacket(ECU_ID);
  CANmod.write(4); //DLC 4
  CANmod.write(0x41);
  CANmod.write(PID);
  CANmod.write((uint8_t)(data >> 8));
  CANmod.write((uint8_t)(data & 0x00ff));
  CANmod.endPacket();
}

void set1byte( uint8_t data, uint8_t PID) {
  CANmod.beginPacket(ECU_ID);
  CANmod.write(0x03); //DLC 3
  CANmod.write(0x41);
  CANmod.write(PID);
  CANmod.write(data);
  CANmod.endPacket();
}

void odb2responder(void * parameter) {


  while (1) {

    int packetSize = CANmod.parsePacket();
    if (packetSize) {
      
      if (!CANmod.packetRtr()) {
        if (!CANmod.packetExtended()) {
          uint8_t len = CANmod.read();
          uint8_t service = CANmod.read();
          uint8_t pid = CANmod.read();

//          Serial.printf("DLC %02x Service %02x PID %02x\r\n", len, service, pid);
          if (len != 2) break;

          switch (service) {
            case 0x01:
              switch (pid) {
                case PIDS_SUPPORT_01_20:
                  Serial.println("PID list requested");
                  setPidList1_20(pid);
                  break;
                case PIDS_SUPPORT_21_40:
                  setPidList21_40(pid);
                  break;
                case PIDS_SUPPORT_41_60:
                  setPidList41_60(pid);
                  break;
                case 0x60:
                  setPidList61_80(pid);
                  break;
                case 0x80:
                  setPidList81_a0(pid);
                  break;
                case 0xa0:
                  setPidLista1_c0(pid);
                  break;
                case 0xc0:
                  setPidListc1_e0(pid);
                  break;

                case MONITOR_STATUS_SINCE_DTCS_CLEARED:
                  setDTC(DTC);
                  break;
                case FREEZE_DTC:
                  set2bytes(freezeDTC, pid);
                  break;
                case FUEL_SYSTEM_STATUS:
                  set2bytes(fuelSystemStatus, pid);
                  break;
                case CALCULATED_ENGINE_LOAD:  // engine efficiency
                  if (engineEfficiency > 100.0 ) engineEfficiency = 100.0;
                  if (engineEfficiency < 0 ) engineEfficiency = 0;
                  set1byte( (uint8_t)((engineEfficiency * 255.0) / 100.0), pid);
                  break;
                case ENGINE_COOLANT_TEMPERATURE:  // engine coolant temperature
                  if (engineCoolantTemperature < -40) engineCoolantTemperature = -40;
                  if (engineCoolantTemperature > 215 ) engineCoolantTemperature = 215;
                  set1byte(engineCoolantTemperature + 40, pid);
                  break;
                case SHORT_TERM_FUEL_TRIM_BANK_1  :
                  set1byte((uint8_t)(((shortTermFuelTrimBank1 + 100.0) * 128.0) / 100.0), pid);
                  break;
                case LONG_TERM_FUEL_TRIM_BANK_1  :
                  set1byte((uint8_t)(((longTermFuelTrimBank1 + 100.0) * 128.0) / 100.0), pid);
                  break;
                case SHORT_TERM_FUEL_TRIM_BANK_2  :
                  set1byte((uint8_t)(((shortTermFuelTrimBank2 + 100.0) * 128.0) / 100.0), pid);
                  break;
                case LONG_TERM_FUEL_TRIM_BANK_2  :
                  set1byte((uint8_t)(((longTermFuelTrimBank2 + 100.0) * 128.0) / 100.0), pid);
                  break;
                case FUEL_PRESSURE :
                  set1byte(fuelPressure / 3, pid);
                  break;
                case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
                  set1byte(intakeManifoldAbsolutePressure, pid);
                  break;
                case ENGINE_RPM:  // rpm
                  set2bytes( engineRPM * 4, pid);
                  //digitalWrite(LEDPIN0, LOW);
                  break;
                case VEHICLE_SPEED:  // speed
                  set1byte(vehicleSpeed, pid );
                  digitalWrite(LEDPIN0, HIGH);
                  //digitalWrite(LEDPIN0, LOW); // Quick one if using the ESP32CAM LED (very hot if continous)
                  break;
                case TIMING_ADVANCE :
                  set1byte((timingAdvance + 64) * 2, pid);
                  break;
                case AIR_INTAKE_TEMPERATURE:
                  set1byte(airIntakeTemperature + 40, pid );
                  break;
                case MAF_AIR_FLOW_RATE:
                  set2bytes((uint16_t)(mafAirFlowRate * 100.0), pid );
                  break;
                case THROTTLE_POSITION  :
                case RELATIVE_THROTTLE_POSITION :
                  set1byte((uint8_t)((throttlePosition * 255.0) / 100.0), pid);
                  //digitalWrite(LEDPIN0, HIGH); // Quick one if using the ESP32CAM LED (very hot if continous)
                  digitalWrite(LEDPIN0, LOW);
                  break;
                case COMMANDED_SECONDARY_AIR_STATUS :
                  set1byte(commandedSecondaryAirStatus, pid );
                  break;
                case OXYGEN_SENSORS_PRESENT_IN_2_BANKS:
                  set1byte( oxygenSensorsPresentIn2Banks, pid);
                  break;
                case OXYGEN_SENSOR_1_SHORT_TERM_FUEL_TRIM:
                case OXYGEN_SENSOR_2_SHORT_TERM_FUEL_TRIM:
                case OXYGEN_SENSOR_3_SHORT_TERM_FUEL_TRIM:
                case OXYGEN_SENSOR_4_SHORT_TERM_FUEL_TRIM:
                case OXYGEN_SENSOR_5_SHORT_TERM_FUEL_TRIM:
                case OXYGEN_SENSOR_6_SHORT_TERM_FUEL_TRIM:
                case OXYGEN_SENSOR_7_SHORT_TERM_FUEL_TRIM:
                case OXYGEN_SENSOR_8_SHORT_TERM_FUEL_TRIM:
                  if ( oxygenSensor1 > 1.275 ) oxygenSensor1 = 1.275;
                  if ( oxygenSensor1 < 0 ) oxygenSensor1 = 0.0;
                  if (shortTermFuelTrim < -100.0 ) shortTermFuelTrim = -100.0;
                  if (shortTermFuelTrim > 99.2 ) shortTermFuelTrim = 99.2;
                  set2bytes((uint16_t)((oxygenSensor1 * 200.0) * 256.0 + (shortTermFuelTrim + 100.0) * 128.0 / 100.0), pid);
                  break;

                case OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO:
                  set1byte(obdStandardThisVehicleConformsTo, pid );
                  break;
                case OXYGEN_SENSORS_PRESENT_IN_4_BANKS :
                  set1byte(oxygenSensorsPresentIn4Banks, pid );
                  break;
                case AUXILIARY_INPUT_STATUS:
                  set1byte(auxiliaryInputStatus, pid );
                  break;
                case RUN_TIME_SINCE_ENGINE_START:
                  set2bytes( (uint16_t)(millis() / 1000) , pid);
                  break;
                case DISTANCE_TRAVELED_WITH_MIL_ON  :
                  set2bytes(distanceTraveledWithMilOn , pid);
                  break;
                case FUEL_RAIL_PRESSURE:
                  if ( fuelRailPressure > 5177.265 ) fuelRailPressure = 5177.265;
                  if ( fuelRailPressure < 0) fuelRailPressure = 0.0;
                  set2bytes((uint16_t)(fuelRailPressure / 0.079), pid);
                  break;
                case FUEL_RAIL_GAUGE_PRESSURE:
                  if ( fuelRailGaugePressure > 655350 )fuelRailGaugePressure = 655350.0;
                  if ( fuelRailGaugePressure < 0 ) fuelRailGaugePressure = 0.0;
                  set2bytes((uint16_t)(fuelRailGaugePressure / 10), pid );
                  break;

                case OXYGEN_SENSOR_1_FUEL_AIR_EQUIVALENCE_RATIO  :
                case OXYGEN_SENSOR_2_FUEL_AIR_EQUIVALENCE_RATIO  :
                case OXYGEN_SENSOR_3_FUEL_AIR_EQUIVALENCE_RATIO  :
                case OXYGEN_SENSOR_4_FUEL_AIR_EQUIVALENCE_RATIO  :
                case OXYGEN_SENSOR_5_FUEL_AIR_EQUIVALENCE_RATIO  :
                case OXYGEN_SENSOR_6_FUEL_AIR_EQUIVALENCE_RATIO  :
                case OXYGEN_SENSOR_7_FUEL_AIR_EQUIVALENCE_RATIO  :
                case OXYGEN_SENSOR_8_FUEL_AIR_EQUIVALENCE_RATIO  :
                  set4bytes((uint32_t) (oxygenSensorFuelAir * 65535.0 / 2.0) << 16 + (uint32_t)((oxygenSensorFuelAirVoltage) * 65535.0 / 8.0), pid);
                  break;

                case COMMANDED_EGR:
                  set1byte( (uint8_t)((commandedEGR * 255.0) / 100.0), pid );
                  break;
                case EGR_ERROR  :
                  set1byte( (uint8_t)((EGRError + 100) * 128.0 / 100.0), pid );
                  break;
                case COMMANDED_EVAPORATIVE_PURGE:
                  set1byte((uint8_t)((commandedEvaporativePurge * 255.0) / 100.0), pid );
                  break;
                case FUEL_TANK_LEVEL_INPUT:  // fuel level
                  set1byte( (uint8_t)((fuelLevel * 255) / 100), pid );
                  break;
                case WARM_UPS_SINCE_CODES_CLEARED :
                  set1byte((uint8_t)warmUpsSinceCodesCleared , pid );
                  break;
                case DISTANCE_TRAVELED_SINCE_CODES_CLEARED:
                  break;
                  set2bytes((uint16_t) distanceTraveledSinceCodesCleared, pid);
                  break;
                case EVAP_SYSTEM_VAPOR_PRESSURE:
                  if ( evapSystemVaporPressure > 8191.75 ) evapSystemVaporPressure = 8191.75;
                  if (evapSystemVaporPressure < -8192  )evapSystemVaporPressure = -8, 192;
                  set2bytes((int16_t)(evapSystemVaporPressure * 4.0), pid);
                  break;
                case ABSOLULTE_BAROMETRIC_PRESSURE:
                  set1byte(absoluteBarometricPressure, pid);
                  break;
                case CATALYST_TEMPERATURE_BANK_1_SENSOR_1 :
                case CATALYST_TEMPERATURE_BANK_2_SENSOR_1 :
                case CATALYST_TEMPERATURE_BANK_1_SENSOR_2 :
                case CATALYST_TEMPERATURE_BANK_2_SENSOR_2 :
                  if (catalystTemperatureBank1Sensor1 > 6513.5) catalystTemperatureBank1Sensor1 = 6513.5;
                  if (catalystTemperatureBank1Sensor1 < -40.0) catalystTemperatureBank1Sensor1 = -40.0;
                  set2bytes( (uint16_t)((catalystTemperatureBank1Sensor1 + 40.0) * 10.0), pid);
                  break;
                case CONTROL_MODULE_VOLTAGE:
                  set2bytes( (uint16_t)(controlModuleVoltage * 1000.0), pid);
                  break;
                case ABSOLUTE_LOAD_VALUE:
                  if (absoluteLoadValue > 25700.0 )  absoluteLoadValue = 25700.0;
                  set2bytes( (uint16_t)((absoluteLoadValue * 255.0) / 100.0) , pid);
                  break;
                case AMBIENT_AIR_TEMPERATURE  :
                  set1byte((uint8_t) ambientAirTemperature + 40, pid);
                  break;
                case FUEL_TYPE  :
                  set1byte((uint8_t) fuelType, pid);
                  break;
                default:
                  break;
              }
              break;
            case 0x09:
              switch (pid) {
                case 0x02:
                  // setVIN();
                  break;
                case 0x0a:
                  //  setECUname();
                  break;
                default:
                  break;
              }
              break;
            default:
              break;
          }

        }

        while (CANmod.available()) {
          CANmod.read();
          
        }
        delay(1);
      }
    }

  }

}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ECU emulator");
  pinMode(LEDPIN0, OUTPUT);
  digitalWrite(LEDPIN0, LOW);

  // start the CAN bus at 500 kbps
  CANmod.setPins(GPIO_NUM_36, GPIO_NUM_22);  
  if (!CANmod.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    delay(10000);
    ESP.restart();
    while (1);
  }

  xTaskCreatePinnedToCore( odb2responder, "odb2responder", 8096, NULL, 1, NULL, 1);

}

void loop() {
  char buf[80];
  bool refresh = false;

  fuelLevel += 1;
  vehicleSpeed += 1;
  engineRPM += random(1, 10);
  airIntakeTemperature = 42 + random(1,10);
  controlModuleVoltage = 14.24 + (float) random(1,100)/100;
  if ((engineRPM*4) > 20000) engineRPM = 0;
  shortTermFuelTrimBank1 = (float)random(1, 10) - 5.0;
  mafAirFlowRate = (float)random(1, 30) / 3.0;
  throttlePosition += 1;
  if (throttlePosition > 40) throttlePosition = 0; 
  engineCoolantTemperature += 1; engineCoolantTemperature = engineCoolantTemperature % 128;
  engineEfficiency += 1.0; engineEfficiency = (float)((int)engineEfficiency % 100);
  catalystTemperatureBank1Sensor1 = 48 + random(1,10);
  delay(50);


}
