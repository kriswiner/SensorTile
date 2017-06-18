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

// Highest page number is 0xFFFF = 65535 for 128 Mbit flash
// Highest page number is 0x0EFF =  4095 for   8 Mbit flash
int page_number = 0x0EFF;
unsigned char flashPage[256];

#define CSPIN  10  // A1 for IU board

#define STAT_WIP 1
#define STAT_WEL 2

#define CMD_WRITE_STATUS_REG   0x01
#define CMD_PAGE_PROGRAM       0x02
#define CMD_READ_DATA          0x03
#define CMD_WRITE_DISABLE      0x04//not tested
#define CMD_READ_STATUS_REG    0x05
#define CMD_WRITE_ENABLE       0x06
#define CMD_READ_HIGH_SPEED    0x0B//not tested
#define CMD_SECTOR_ERASE       0x20//not tested
#define CMD_BLOCK32K_ERASE     0x52//not tested
#define CMD_RESET_DEVICE       0xF0//<<-different from winbond
#define CMD_READ_ID            0x9F
#define CMD_RELEASE_POWER_DOWN 0xAB//not tested
#define CMD_POWER_DOWN         0xB9//not tested
#define CMD_CHIP_ERASE         0xC7
#define CMD_BLOCK64K_ERASE     0xD8//not tested

unsigned char flash_wait_for_write = 0;

void setup(void)
{
  pinMode(CSPIN, OUTPUT);
  digitalWrite(CSPIN, HIGH);

  SPI.begin();
  
  unsigned char id_tab[32];
  unsigned long t_start;
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
  
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  Serial.print("ID bytes: ");
  uint16_t id[3];
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(0x9F);
  id[0] = SPI.transfer(0);
  id[1] = SPI.transfer(0);
  id[2] = SPI.transfer(0);
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  Serial.print(id[0], HEX); Serial.print(" "); Serial.print(id[1], HEX);  Serial.print(" ");  Serial.println(id[2], HEX); 

  Serial.println("Winbond  W25Q80BLUX1G   Chip ID = 0xEF, 0x40, 0x14, 0x0");
  Serial.println("Macronix MX25L12835FZNI Chip ID = 0xC2, 0x20, 0x18, 0xC2");
  Serial.println("Spansion S25FL127S      Chip ID = 0x01, 0x20, 0x18, 0x4D");
  Serial.println(" ");

  // read Sensor Tile SPI flash
  for(page_number = 0; page_number < 20; page_number++)  {

   //  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
   flash_read_pages(flashPage, page_number, 1);
      
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

/*********************************************************************************************/
// Useful functions
/*********************************************************************************************/
void write_pause(void)
{
  if(flash_wait_for_write) {
    while(flash_read_status() & STAT_WIP);
    flash_wait_for_write = 0;
  }
}

//=====================================
// convert a page number to a 24-bit address
int page_to_address(int pn)
{
  return(pn << 8);
}

//=====================================
// convert a 24-bit address to a page number
int address_to_page(int addr)
{
  return(addr >> 8);
}

//=====================================
void flash_read_id(unsigned char *idt)
{
  write_pause();
  //set control register 
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_READ_ID);
  for(uint16_t i = 0; i < 20; i++) {
    *idt++ = SPI.transfer(0x00);
  }
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
}

//=====================================
unsigned char flash_read_status(void)
{
  unsigned char c;

// This can't do a write_pause
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);  
  SPI.transfer(CMD_READ_STATUS_REG);
  c = SPI.transfer(0x00);
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  return(c);
}

//=====================================

void flash_hard_reset(void)
{
  // Make sure that the device is not busy before
  // doing the hard reset sequence
  // At the moment this does NOT check the
  // SUSpend status bit in Status Register 2
  // but the library does not support suspend
  // mode yet anyway
  write_pause();
  
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_RESET_DEVICE );
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);
  // Wait for the hard reset to finish
  // Don't use flash_wait_for_write here
  while(flash_read_status() & STAT_WIP);
  // The spec says "the device will take
  // approximately tRST=30 microseconds
  // to reset"
}

//=====================================
void flash_chip_erase(boolean wait)
{
  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(CSPIN, HIGH);
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_CHIP_ERASE);
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  flash_wait_for_write = 1;
  if(wait)write_pause();
}

//=====================================
// Tse Typ=0.6sec Max=3sec
// measured 549.024ms
// Erase the sector which contains the specified
// page number.
// The smallest unit of memory which can be erased
// is the 4kB sector (which is 16 pages)
void flash_erase_pages_sector(int pn)
{
  int address;

  write_pause(); 
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(CSPIN, HIGH);
  
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_SECTOR_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xff);
  SPI.transfer((address >> 8) & 0xff);
  SPI.transfer(address & 0xff);
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();  
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
// Erase the 32kb block which contains the specified
// page number.
void flash_erase_pages_block32k(int pn)
{
  int address;

  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(CSPIN, HIGH);
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_BLOCK32K_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
// Erase the 64kb block which contains the specified
// page number.
void flash_erase_pages_block64k(int pn)
{
  int address;
  
  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(CSPIN, HIGH);
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_BLOCK64K_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
void flash_page_program(unsigned char *wp,int pn)
{
  int address;

  write_pause(); 
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_PAGE_PROGRAM);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // Now write 256 bytes to the page
  for(uint16_t i = 0; i < 256; i++) {
  SPI.transfer(*wp++);
  }
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
void flash_read_pages(unsigned char *p,int pn,const int n_pages)
{
  int address;
  unsigned char *rp = p;
  
  write_pause();
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_READ_DATA);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // Now read the page's data bytes
  for(uint16_t i = 0; i < n_pages * 256; i++) {
    *rp++ = SPI.transfer(0);
  }
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
}

//=====================================
// Read specified number of pages starting with pn
void flash_fast_read_pages(unsigned char *p,int pn,const int n_pages)
{
  int address;
  unsigned char *rp = p;
  
  write_pause();
// The chip doesn't run at the higher clock speed until
// after the command and address have been sent
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSPIN, LOW);
  SPI.transfer(CMD_READ_HIGH_SPEED);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // send dummy byte
  SPI.transfer(0);
  // Now read the number of pages required
  for(uint16_t i = 0; i < n_pages * 256; i++) {
    *rp++ = SPI.transfer(0);
  }
  digitalWrite(CSPIN, HIGH);
  SPI.endTransaction();
}
