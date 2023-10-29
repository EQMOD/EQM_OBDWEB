
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

#include "cJSON.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

int charcount;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

int rxofst;
int jsondat;
int eschar;
int arcnt;
char  rxbuf[1024];

void setup() {

  rxofst = 0;
  jsondat = 0;
  eschar = 0;
  arcnt = 0;

  charcount = 0;
  Serial.begin(115200);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...


  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font



}

void loop() {

char c;
int i;
double graphdat[30];

cJSON *jsonout;
cJSON *jvalue;

  if (Serial.available()) { // if there is data comming
    c = Serial.read();

    switch (c)
    {
        case 0x27:
                    eschar++;
                    break;
        case '#' :
                    if (eschar == 2)
                    {
                        if (jsondat < 2)
                        {
                            // Process the JSON Array stored on the rxbuf
                          
                            rxbuf[rxofst] = 0;
                            jsonout = cJSON_Parse(rxbuf);
                            rxofst = 0;
                            for(i=0;i<cJSON_GetArraySize(jsonout);i++)
                            {
                                jvalue = cJSON_GetArrayItem(jsonout,i);
                                graphdat[i] = jvalue->valuedouble;
                            }
                            cJSON_Delete(jsonout);

                            
                            // Display tge graphdat contents here

                            if (jsondat == 0)
                               display.clearDisplay();
                                 
                            display.setCursor(0, 0);
                            
                            for(i=0;i<19;i++)
                                display.drawLine(i*6, (display.height()-1)-(graphdat[i]) , (i+1)*6, (display.height()-1)-(graphdat[i+1]), SSD1306_WHITE);

                            jsondat++;
                            
                        } else
                        if (jsondat == 2)
                        {
                           // Process the 3rd parameter on the data stream containing current core 0 value
                          
                           rxbuf[rxofst] = 0;
                           rxofst = 0;
                           display.setCursor(0, 0);
                           display.print("Core 0:");
                           display.print(rxbuf);

                           jsondat++;
                        } else
                        if (jsondat == 3)
                        {
                           // Process the 3rd parameter on the data stream containing current core 0 value
                          
                           rxbuf[rxofst] = 0;
                           rxofst = 0;
                           display.print(" 1:");
                           display.print(rxbuf);
                           display.print("   ");
                           display.display();

                           jsondat++;
                        }
                    } 
                    else
                    {
                        rxofst=0;
                        eschar=0;
                        jsondat=0;
                    }
                    break;

        default:

                   // Just store the serial stream data to the array
                   if (eschar == 2)
                   {
                      rxbuf[rxofst] = c;
                      rxofst++;
                   }
                   break;
    }

 }

}
