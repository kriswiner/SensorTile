/* SPIFlash_Ladybug.ino
Sketch by Kris Winer December 16. 2016

License: Use this sketch any way you choose; if you like it, buy me a beer sometime

Purpose: Checks function of a variety of SPI NOR flash memory chips hosted by the STM32L4
Dragonfly (STM32L476), Butterfly (STM32L433), and Ladybug (STML432) development boards or their variants.

Sketch takes advantage of the SPI.beginTransaction/SPI.EndTransaction protocol for efficiency
and maximum speed.

Sketch based on the work of Pete (El Supremo) as follows:
 * Copyright (c) 2014, Pete (El Supremo), el_supremo@shaw.ca
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 */

#include <SPI.h>
#include "SPIFlash.h"

// Highest page number is 0xFFFF = 65535 for 128 Mbit flash
// Highest page number is 0x0EFF =  4095 for   8 Mbit flash
uint16_t max_page_number = 0x0EFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t sector_number = 0;

uint32_t compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
uint16_t rawVbat;
float VDDA, VBAT;
uint16_t eCO2 = 0, TVOC = 0;
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float aRes = 2.0f/8192.0f;   // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output

#define csPin 10

SPIFlash SPIFlash(csPin);

void setup(void)
{ 
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  // check SPI Flash ID
  SPIFlash.SPIFlashinit();
  SPIFlash.getChipID();
 
  // read Sensor Tile SPI flash
  for(page_number = 0; page_number < 20; page_number++)  {

   //  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
   SPIFlash.flash_read_pages(flashPage, page_number, 1);
      
   for(sector_number = 0; sector_number < 8; sector_number++) {

    accelCount[0] = (int16_t) ((int16_t) flashPage[sector_number*32 + 1] << 8 | flashPage[sector_number*32 + 2]);
    accelCount[1] = (int16_t) ((int16_t) flashPage[sector_number*32 + 3] << 8 | flashPage[sector_number*32 + 4]);
    accelCount[2] = (int16_t) ((int16_t) flashPage[sector_number*32 + 5] << 8 | flashPage[sector_number*32 + 6]);
    
    ax = (float)accelCount[0]*aRes/4.0f;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes/4.0f;   
    az = (float)accelCount[2]*aRes/4.0f;  

    temperature = 0.5f * ((float) flashPage[sector_number*32 + 0]) + 23.0f; // Accel chip temperature in degrees Centigrade
    
    compTemp = ((uint32_t) flashPage[sector_number*32 + 11] << 24) |  ((uint32_t)flashPage[sector_number*32 + 12] << 16) |  ((uint32_t)flashPage[sector_number*32 + 13] << 8) | flashPage[sector_number*32 + 14];
    compHumidity = ((uint32_t) flashPage[sector_number*32 + 15] << 24) |  ((uint32_t)flashPage[sector_number*32 + 16] << 16) |  ((uint32_t)flashPage[sector_number*32 + 17] << 8) | flashPage[sector_number*32 + 18];
    compPress = ((uint32_t) flashPage[sector_number*32 + 19] << 24) |  ((uint32_t)flashPage[sector_number*32 + 20] << 16) |  ((uint32_t)flashPage[sector_number*32 + 21] << 8) | flashPage[sector_number*32 + 22];

    eCO2 = (uint16_t) ((uint16_t) flashPage[sector_number*32 + 7] << 8 | flashPage[sector_number*32 + 8]);
    TVOC = (uint16_t) ((uint16_t) flashPage[sector_number*32 + 9] << 8 | flashPage[sector_number*32 + 10]);

    Seconds = flashPage[sector_number*32 + 23];
    Minutes = flashPage[sector_number*32 + 24];
    Hours = flashPage[sector_number*32 + 25];
    Day = flashPage[sector_number*32 + 26];
    Month = flashPage[sector_number*32 + 27];
    Year = flashPage[sector_number*32 + 28];
    rawVbat = ((uint16_t) flashPage[sector_number*32 + 29] << 8) |  flashPage[sector_number*32 + 30];

    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    pressure = (float) compPress/25600.f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH

    VBAT = (127.0f/100.0f) * 3.30f * ((float)rawVbat)/4095.0f;

    // Output for spreadsheet analysis
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.print(Seconds);} else Serial.print(Seconds); Serial.print(",");      
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(Year); Serial.print(",");
    Serial.print(pressure, 2); Serial.print(",");Serial.print(temperature_C, 2); Serial.print(",");Serial.print(temperature_F, 2); Serial.print(",");
    Serial.print(altitude, 2); Serial.print(",");Serial.print(humidity, 1); Serial.print(","); Serial.print(VBAT, 2); Serial.print(","); 
    Serial.print((int) 1000*ax); Serial.print(",");Serial.print((int) 1000*ay); Serial.print(","); Serial.print((int) 1000*az); Serial.print(","); Serial.println(temperature, 2); 

    
/*   // reconstitute proper output
    Serial.println("BME280:");
    Serial.print("Altimeter temperature = "); 
    Serial.print( temperature_C, 2); 
    Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); 
    Serial.print(temperature_F, 2); 
    Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); 
    Serial.print(pressure, 2);  
    Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); 
    Serial.print(altitude, 2); 
    Serial.println(" feet");
    Serial.print("Altimeter humidity = "); 
    Serial.print(humidity, 1);  
    Serial.println(" %RH");// pressure in millibar
    Serial.println(" ");
    
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");

    Serial.print("VBAT = "); Serial.println(VBAT, 2); 
  */
    }
  
  }


}

void loop(void)
{
}

