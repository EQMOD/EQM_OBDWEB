# EQM_OBDWEB

## Introduction

A WEB Based or Wifi/BLE (ELM327 Compatible) OBDII Dongle Platform using the ESP32 WROVER (16MB) Module

An OBDII Dongle using the ESP32 Wrover  with built-in SPIFFF memory is used as a CAN BUS OBDII reader and the data is presented as web page using a built-in webserver. The webpage code communicates to the dongle via WiFi and uses websockets to retrieve OBDII data.  The platform is using an ESP32 microcontroller which is a dual core mcu  with the first core handling the CAN Bus comunications part via SN65HVD230 CAN BUS Transceiver and the 2nd core handling the websocket part. OBDII data is pushed to the UI every 60ms to 100ms for critical data such as rpm and speed, and every 1000ms for not so critical data such as temp, voltage, etc. 

The platform code is using ESP32 RTOS for the multi core multi tasking component. That's basically an OBDII with a webserver running inside that  dongle attached to the car's obd port. And literally you can read every car metric data and show it on the car infotainment / Web browser / Phone app  with customizable fancy UI stuff (gauges, graphs, etc.) using standard HTML/JS/CSS/JPG renders.

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/ff318b5f-7a61-4340-b203-8979be0f3c4c)  

## Web Interface

Mobile phone apps not required as long as the infotainment/mobilephone/etc can render the UI using a standard web browser.

Once the UI client is connected to the ESP32 via WIFI, the dongle webserver can be accessed via these urls:  

**OBD HomePage:** 

http://192.168.0.10/

**ESP32 Load Monitoring:** 

http://192.168.0.10/page?var=file&val=/ui/CPULoad/index.html

WEB ASSETS are stored in the ESP32 SPIFFS memory storage which is automatically populated one time from the data on the SD Card. A SPIFF memory update is triggred by deleting the lock.file on the SD card. Any updates on the WEBSERVER files should be done on the SD card with the "lock.file" deleted.

### Bluetooth Connections

There are two BLE modules used for this project. The first BLE/Bluetooth Module built in with the ESP32 is used as the Bluetooth connector for the OBDII dongle and can be used to to connect to Mobile OBDII apps such as TORQUE. 

The 2nd Bluetooth/BLE module a JDY-08 contains a user programmable CC2541 BLE Chipset which can be used to receive Car sensor data that are bluetooth based. DEMO CC2541 codes are provided to receive BLE Advertisments from TPMS Sensors such as the Bluetooth based VC601 Tire Pressure Sensors or a Battery Monitoring Sensor (BM2).
The same BLE module can be loaded with a UART-BLE for ELM327 AT command compatible communication.

### Connections Overview

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/8c971291-0b14-477d-abf0-11e375a93849)

### WEB-DOCROOT Filesystem

EQM_OBDWEB uses the SD Card as temporary storage. The WEB DOCROOT files are loaded to the ESP32 Wrover SPIFFs memory for faster page load time. It also utilizes the PSRAM as a temporary buffer for SPIFFs loading up to HTTP delivery. SPIFFS file storage is initially formatted and populated with contents coming from the SD-Card. This is automatically done once the code detects if the SD-Card doesnt have the 'lock.file' inside. It copies the content of the SD-Card to the SPIFFs storage and
then creates a 'lock.file' on the SD-card to prevent succeeding SPIFFS formatting and file copy.

<img width="1024" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/d078c07d-7e55-4241-9a0c-be90f82c1aad">  

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/834baed4-5b0b-4a7f-9e21-36f9c00129a6)  

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/62d68b79-26b9-4151-946a-aeec21a278c1)  

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/8ed34b38-b398-489b-8261-c8cbc8cf7dad)  

## Wiring Diagram

Here is the Connection diagram of the ESP32 Wrover with SD Card Reader and JDY08 BLE Module

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/64a7ada1-9e7a-4788-9aba-bed030aca00d)

## Bluetooth Firmware

Here is the suggested development platform (codes included) which provides the user options flash code on the ESP32 Wrover and to burn firmware on the CC2541 by simply dropping a cc2541.bin file on the SD card.  The EQM_OBDWEB dongle contain codes to flash the CC2541 JDY-08 module. Another option is by flashing the Wrover with a CCloader arduino code and connecting it to a laptop then uploadthe cc2541 firmware using the ccloader windows program.

<img width="897" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/714c41b0-87a3-4769-ac86-8fdbf50bd103">

#### Using a separate ARDUINO Board and sketch to burn firmware on the JDY-08 Module using the CCLoader Program

https://github.com/EQMOD/EQM_OBDWEB/tree/master/CC254x_TPMS_BATMON/JDY08_CCLoader

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/9f0cb754-15d4-401e-bdb6-24ad4b765d67)



## EQM_OBDWEB Development Platform Structure

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/5004d103-8bce-4605-83f4-3de82ef758c6)

Current firmware codes are stable with ESP32 Version 2.0.5 up to 2.0.12. Seems to be encountering issues on 2.0.13 and 2.0.14 particularly on the Wifi and
SD Card handlers. This using the ESP32 Wrover (16MB) as the code utilizes the SD Card, the SPIFFS Memory, and the PSRAM.

<img width="957" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/55d90e89-64c3-4814-b38f-ec89a1f38a46">



USER CUSTOMIZABLE WEB PAGES FOR CAR DASHBOARDS OR OTHER CAR METRICS DATA USING HTML/CSS/JS/IMG with live data using websockets.
Just load your designs to the SD Card:

### Sample Code

https://github.com/EQMOD/EQM_OBDWEB/tree/master/Custom_USERS_WEBUI/Sample_SPDO

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/2de52d29-4121-49ac-9799-9da895ade9de)

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/fba9a1da-94eb-4f18-9d98-998feb2f289a)


## Sample WebUI Renders

### Homepage

<img width="883" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/c73f5365-5a80-47ea-a897-e879e56ff0a9">

### OBDII Screen (Screenshot using data from an OBDII Emulator)  

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/6237eb39-76e0-43a0-86b7-357491b815a1)  

### TPMS (Tire Pressure Monitoring Screen)  

<img width="912" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/b3a094ac-f161-4a8f-b364-854bd627f710">

### Battery Monitor via BM2 BLE Broadcasts. Other Battery data is also possible  

<img width="969" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/c11f6775-d5a7-4e06-84ad-4a189e4e7c38">

### ESP32 LOAD Monitoring Screen  

<img width="961" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/0481a5cb-47ff-49e9-9991-760a7c0b5ec1">

### A sample WEBPAGE with a LIVE video camera/webcam FEED element as the background with the OBDII/ESP32  data (text, needle , bars, charts) on the foreground

https://github.com/EQMOD/EQM_OBDWEB/tree/main/Custom_USERS_WEBUI/WebCAM_and%20WebSocket_Only

<img width="958" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/b7d27b1e-5a88-4ee1-b274-ad3395398397">

### Rendering OBD II Data on top of a Mobile Camera or Tablet LIVE Video feed

It is also possible to provide OBD-II data on top of a Mobile or Tablet camera live feed using the phone's browser ONLY if the webpage with your esp32 embedded data  is hosted as localhost or as a direct filesystem html page load or a page loaded via https with a valid SSL certificate. For this demo , all html/css/js/img (files on the above link) are loaded via localhost webserver(using KSWeb app) running on an Android phone. The js part of the page is the one responsible of fetching the esp32 data via websockets and rendering it on top of the phonecam video feed

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/c21f6350-980d-491a-aa6b-155d11f7e640)


### Transparent Gauges with a Mobile Phone CAM

Using the original gauge assets and replacing the background images  with a live feed of phonecam video. The approach is the same as what was described on the previous section. Here is another demo of a mobile phone web browser with the phonecam live feed rendering on the background and the gauge assets rendering on the chrome browser foreground. Rendered using chrome browser html/js/css/img with the JavaScript communicating to the esp32 obdii using websockets. Obdii dongle is acting as a wifi hotspot while connected to the car's OBD Port. I used jQuery and charts.js for the gauge renders. Upper left corner shows a live load stats of the ESP32 dual cores.

Webpage Code and Assets:

https://github.com/EQMOD/EQM_OBDWEB/tree/main/Custom_USERS_WEBUI/OBDGauges_WEBCam_Websockets

<img width="958" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/a7cc0032-9046-48c7-8942-3bd2a7107fe9">



### Sample 3D Printed OBD Dongle (3d files/stl included in the respository)  

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/18fc7b90-b6f0-44de-b6d9-f7ac2d33e340)


How to flash the esp32 itself ;

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/219494cd-2c7f-46d5-8f14-071f5b2612c8)

Using an "ESP32 Downloader"

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/8385255c-e1f7-4747-b793-4b1737f40e6e)

### CC2541 JDY08 BLE Module Firmwares

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/e9ee0860-a884-4f26-a952-db3776b59644)


TPMS VC601 Broadcast emulator for testing the cc2541 codes uing a CCDebugger

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/e762d1d4-6cbb-4614-a0e2-ff4a878afb4c)

#### Firmware Source ( You need IAR to compile these) 

https://github.com/EQMOD/EQM_OBDWEB/tree/master/CC254x_TPMS_BATMON/Custom%20JDY08-CC2541%20Firmware%20TPMS%20BATMON/SDK/BLE-CC254x-1.4.0/Projects/ble/TPMSBroadcast_Tester

#### JDY08 Firmware source codes to receive BLE VC601 TPMS Sensor broadcasts and presented to the ESP32 via UART

https://github.com/EQMOD/EQM_OBDWEB/tree/master/CC254x_TPMS_BATMON/Custom%20JDY08-CC2541%20Firmware%20TPMS%20BATMON/SDK/BLE-CC254x-1.4.0/Projects/ble/TPMS_OBD

## Compiling and Flashing

This version of the program works best with a ESP32-D0WD-V3 (revision 3) 16MB
with the following compile settings;

- Board: "ESP Wrover Module"
- Upload Speed: "921600"
- Flash Frequency: "80MHz"
- Flash Mode: "QIO"
- Partition Scheme: "Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)"
- Core Debug Level: "None"
- Erase All Flash Before Sketch Upload: "Disabled"

<img width="960" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/d99bd190-b4f2-46fa-9959-13d611277ca3">

Formatting and flashing the SPIFF Memory is initiated by checking if a detected/attached SD Card doesn't have a "lock.file" inside;

<img width="1920" alt="image" src="https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/af0fd553-99ad-45f9-a32c-b54a19f7b46d">



## On-Site Debugging

A portable flasher and serial Console Monitoring port in one is implemented on the EQM_OBDWEB Dongle. The idea here is we use the same 6 pin port to flash new code on the ESP32 inside the obd  dongle OR use it as Serial Console port by plugging in a JDY-16 Bluetooth UART module and connect it to a mobile app BLE Serial Terminal  and monitor the ESP32 activity through a mobile phone;

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/fd193d7f-0f1a-49eb-bc8a-70d599164461)

### JDY-16 Module for Bluetooth UART Console Port Monitoring

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/828c9ec3-278d-4989-b80c-19fa451150d2)

### Connection Diagram

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/f9dcf423-b83c-4fc4-968b-96162d01d422)

### Mobile APP with the Serial Console Data

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/b5c5d805-cb0c-42b5-a1af-8e539e9dc70a)  

Flasher and JDY-16 modules both fit on the same flashing port on the EQM_OBDWEB Dongle:

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/7f59b2c6-b49e-42fd-8880-529d55bf98ad)

## Development Environment 

Multiple esp32 dongles are used with one acting as an ECU Emulator.

The development / debugging can be done using multiple instances of the OBD ESP32 Dongles - One with loaded code functioning as an OBDII-Simulator (code included in the repository), A Dummy OBD--II Node (To simulate CAN BUS Traffic) and the EQM_OBDWEB ESP32 OBDII Dongle. The Dummy OBDII CAN Bus node is just the same code as the EQM_OBDWEB with a different WIFI/BLE name and it doesn't wait for CAN BUS Activity. The EQM_OBDWEB waits for CAN BUS Activity specifically to put it in SLEEP MODE (Power Management).
This one needs more improvement as we are still trying to find options to lessen the the power consumption if in case we decide to leave the dongle attached to the OBDII port of the car....


![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/4f9f7559-49e1-4cb0-bcc9-6a6b38460935)

**...and YES!!** You can technically use any ESP32 Board such as a ESP32-CAM as an OBDII Dongle (but you need to check the memory specs)

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/6ec90e6d-9703-4562-81ae-175638fadb18)

Here is one example of an ESP32-CAM Board used as an OBD-II Emulator and its flashing LED as an indicator for CAN BUS Activity :-D

![image](https://github.com/EQMOD/EQM_OBDWEB/assets/29789200/51ed0ea7-c57d-46cc-8f41-440a0ab3e130)














 
