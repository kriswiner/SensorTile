/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch is to operate the Sensor Tile (https://hackaday.io/project/19649-stm32l4-sensor-tile) 
 with BME280 pressure/temperature/humidity sensor, CCS811 eCO2 and VOC sensor, BMA280 accelerometer, 
 and ICS43434 I2S microphone all hosted by an STM32L432 MCU. There is also a BMD-350 (nRF52) module 
 for BLE connectivity as a UART bridge (slave) to the STM32L432 and a 1 MByte SPI flash for data 
 logging and storage.
 
 The sketch uses default SDA/SCL pins on the Ladybug development board.

 Library may be used freely and without limit with attribution.
 
  */
#include <RTC.h>
#include <avr/dtostrf.h>
#include "BMA280.h"
#include "BME280.h"
#include "CCS811.h"
//#include "SPIFlashClass.h"
#include  <ArduinoSound.h>
#include <I2S.h>
#include <SPI.h>

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

unsigned char w_page[256];
unsigned char r_page[256];

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

// ICS43434 I2S Digital Microphone
const int sampleRate = 8000;          // sample rate for the input
const int fftSize = 128;              // size of the FFT to compute
const int spectrumSize = fftSize / 2; // size of the spectrum output, half of FFT size
int spectrum[spectrumSize];           // array to store spectrum output
int maxSpectrum = 1;                  // maximum value of spectrum[i] for a given sample
float normSpectrum[spectrumSize];     // holds the normalized audio normal mode spectrum

FFTAnalyzer fftAnalyzer(fftSize);     // create an FFT analyzer to be used with the I2S input

// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced,, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_16, Hosr = H_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_021ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate

uint32_t rawPress, rawTemp, compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
uint16_t rawHumidity;  // variables to hold raw BME280 humidity value

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280; // instantiate BME280 class

// RTC set up
/* Change these values to set the current initial time */
const uint8_t seconds = 0;
const uint8_t minutes = 15;
const uint8_t hours = 17;

/* Change these values to set the current initial date */
const uint8_t day = 4;
const uint8_t month = 7;
const uint8_t year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year, count = 0;

bool alarmFlag = false;

// battery voltage monitor definitions
uint16_t rawVbat;
float VDDA, VBAT;
#define VbatMon A4

//BMA280 definitions
#define BMA280_intPin1 4   // interrupt1 pin definitions
#define BMA280_intPin2 1   // interrupt2 pin definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      BW_7_81Hz, BW_15_63Hz, BW_31_25Hz, BW_62_5Hz, BW_125Hz, BW_250Hz, BW_500Hz, BW_1000Hz
      normal_Mode, deepSuspend_Mode, lowPower_Mode, suspend_Mode
      sleep_0_5ms, sleep_1ms, sleep_2ms, sleep_4ms, sleep_6ms, sleep_10ms, sleep_25ms, sleep_50ms, sleep_100ms, sleep_500ms, sleep_1000ms
*/ 
uint8_t Ascale = AFS_2G, BW = BW_62_5Hz, power_Mode = lowPower_Mode, sleep_dur = sleep_50ms, tapStatus, tapType;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 

bool newBMA280Data = false;
bool newBMA280Tap  = false;

BMA280 BMA280(BMA280_intPin1, BMA280_intPin2); // instantiate BMA280 class

// CCS811 definitions
#define CCS811_intPin  A1
#define CCS811_wakePin A2

/* Specify CCS811 sensor parameters
 *  Choices are   dt_idle , dt_1sec, dt_10sec, dt_60sec
 */
uint8_t AQRate = dt_10sec;  // set the sample rate
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // array to hold the CCS811 raw data
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;

bool newCCS811Data  = false; // boolean flag for interrupt

CCS811 CCS811(CCS811_intPin, CCS811_wakePin); // instantiate CCS811 class

// 8 MBit (1 MByte) SPI Flash 4096, 256-byte pages
#define csPin 10 // SPI Flash chip select pin

uint16_t page_number = 0;     // set the page mumber for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

//SPIFlash SPIFlash(csPin);

// BMD-350 UART bridge
//Sensor Tile
#define ATMD        A3 // toggle pin for AT mode or UART pass through mode
#define BMD350Reset  0 // BMD-350 reset pin active LOW

char Packet[20], StringVBAT[20], StringP[20], StringH[20], StringTempC[20], StringeCO2[20], StringTVOC[20];
char             Audio1[20], Audio2[20], Audio3[20];

void setup() {
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  Serial2.begin(57600); // BMD-350 baud rate should be 57600
  delay(4000);
  Serial.println("Serial2 enabled!"); // for BMD-350 UART Bridge

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led on

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  // Configure BMD-350 mode pins
  pinMode(ATMD, OUTPUT);
  pinMode(BMD350Reset, OUTPUT);  

  // Configure SPI Flash chip select
  pinMode(CSPIN, OUTPUT);
  digitalWrite(CSPIN, HIGH);

  SPI.begin(); // initiate SPI  

  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  //Enable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor

  BME280.I2Cscan(); // should detect BME280 at 0x77, BMA280 at 0x18, and CCS811 at 0x5A

  //Disable the CCS811 for I2C scan
  digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor

  delay(1000);
    
  // Set the time
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);

  // Initialize BMD-350
  Serial.println("Initializing BMD-350!");
  delay(1000);
  initializeBMD350();

  // Read the BMS280 Chip ID register, this is a good test of communication
  Serial.println("BMA280 accelerometer...");
  byte c = BMA280.getChipID();  // Read CHIP_ID register for BMA280
  Serial.print("BMA280 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xFB, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte d = BME280.getChipID();  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  delay(1000);  

  // Read the WHO_AM_I register of the CCS811 this is a good test of communication
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  byte e = CCS811.getChipID();
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  Serial.print("CCS811 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x81, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(c == 0xFB && d == 0x60 && e == 0x81 ) // check if all I2C sensors have acknowledged
  {
   Serial.println("BMA280+BME280+CCS811 are online..."); Serial.println(" ");
   
  aRes = BMA280.getAres(Ascale);                                     // get sensor resolutions, only need to do this once
  BMA280.selfTestBMA280();                                           // perform sensor self test
  BMA280.resetBMA280();                                              // software reset before initialization
  delay(1000);                                                       // give some time to read the screen
  BMA280.initBMA280(Ascale, BW, normal_Mode, sleep_dur);             // initialize sensor in normal mode for calibration
  BMA280.fastCompensationBMA280();                                   // quickly estimate offset bias
  BMA280.initBMA280(Ascale, BW, power_Mode, sleep_dur);              // initialize sensor for data acquisition

  BME280.resetBME280(); // reset BME280 before initilization
  delay(100);

  BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy); // Initialize BME280 altimeter

  // initialize CCS811 and check version and status
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  CCS811.CCS811init(AQRate);
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  }
  else 
  {
  if(c != 0xFB) Serial.println(" BMA280 not functioning!");
  if(d != 0x60) Serial.println(" BME280 not functioning!");    
  if(e != 0x81) Serial.println(" CCS811 not functioning!");   
  }


  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!AudioInI2S.begin(sampleRate, 32)) {
    Serial.println("Failed to initialize I2S input!");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the FFT analyzer
  if (!fftAnalyzer.input(AudioInI2S)) {
    Serial.println("Failed to set FFT analyzer input!");
    while (1); // do nothing
  }

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

    // Check VBAT to avoid brown-out loss of data!
    rawVbat = analogRead(VbatMon);
    VBAT = (127.0f/100.0f) * 3.30f * ((float)rawVbat)/4095.0f;
    if(VBAT > 3.70) 
    {
      flash_chip_erase(true); // full erase only if the battery is still good
      Serial.println("SPI flash erased!");
    }

  
    // set alarm to update the RTC every second
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
    RTC.attachInterrupt(alarmMatch);

    attachInterrupt(BMA280_intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA280
    attachInterrupt(BMA280_intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA280
    attachInterrupt(CCS811_intPin,  myinthandler3, FALLING); // enable CCS811 interrupt

  /* end of setup */

}

void loop() {

    /* ICS43434 I2S Digital Microphone */
    // check if a new analysis is available
    if (fftAnalyzer.available()) {
    // read the new spectrum
    fftAnalyzer.read(spectrum, spectrumSize);
    }


    // BMA280 acceleration
    if(newBMA280Data == true) {  // On interrupt, read data
     newBMA280Data = false;  // reset newData flag

     BMA280.readBMA280AccelData(accelCount); // get 14-bit signed accel data
     
     // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes/4.0f;  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes/4.0f;   
     az = (float)accelCount[2]*aRes/4.0f;  
    }


    // Check for BMA280 taps
    if(newBMA280Tap == true) {  // On interrupt, identify tap
      newBMA280Tap = false;
 
      tapType = BMA280.getTapType();
      if(tapType & 0x20) Serial.println("Single tap detected!");
      if(tapType & 0x10) Serial.println("Double tap detected!"); 
      
      tapStatus = BMA280.getTapStatus();  // Read tap status register

      if(tapStatus & 0x80) {
        Serial.println("Tap is negative");
        }
      else {
        Serial.println("Tap is positive");
      }

       if(tapStatus & 0x40) {
        Serial.println("Tap is on x");
       }
       if(tapStatus & 0x20) {
        Serial.println("Tap is on y");
       }
       if(tapStatus & 0x10) {
        Serial.println("Tap is on z");
       }      
    }


    // CCS811 data
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor

    // If intPin goes LOW, all data registers have new data
    if(newCCS811Data == true) {  // On interrupt, read data
    newCCS811Data = false;  // reset newData flag
     
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.readCCS811Data(rawData);

    eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
    TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
    Current = (rawData[6] & 0xFC) >> 2;
    Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f), 3; 
    }           
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 

    
    if(alarmFlag)  { // update RTC output (serial display) whenever the RTC alarm condition is achieved
       alarmFlag = false;
       count++;

    // Normalize audio spectrum
    maxSpectrum = 1;
    for (int i = 0; i < spectrumSize; i++) {
    if(spectrum[i] > maxSpectrum) maxSpectrum = spectrum[i];
    }
 
    // print out the audio spectrum
      Serial.println("Audio Normal Modes");
      for (int i = 0; i < spectrumSize; i++) {
      normSpectrum[i] = (float) spectrum[i]/maxSpectrum;
      if(normSpectrum[i] > 0.50f)
      {
        Serial.print((i * sampleRate) / fftSize); // the starting frequency
        Serial.print("\t"); //
        Serial.println(normSpectrum[i]); // normalized spectrum value

        Serial2.print((i * sampleRate) / fftSize); // the starting frequency
        Serial2.print("\t"); //
        Serial2.println(normSpectrum[i]); // normalized spectrum value
      }
    }
    Serial.println(" ");


    if(SerialDebug) {
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    }

    tempCount = BMA280.readBMA280TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

    rawTemp =  BME280.readBME280Temperature();
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    compPress = BME280.BME280_compensate_P(rawPress);
    pressure = (float) compPress/25600.f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    rawHumidity =  BME280.readBME280Humidity();
    compHumidity = BME280.BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH
 
    if(SerialDebug){
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

    Serial.println("CCS811:");
    Serial.print("Eq CO2 in ppm = "); Serial.println(eCO2);
    Serial.print("TVOC in ppb = "); Serial.println(TVOC);
    Serial.print("Sensor current (uA) = "); Serial.println(Current);
    Serial.print("Sensor voltage (V) = "); Serial.println(Voltage, 2);  
    Serial.println(" ");
    }
    
    // Read RTC
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");

    rawVbat = analogRead(VbatMon);
    VBAT = (127.0f/100.0f) * 3.30f * ((float)rawVbat)/4095.0f;
    Serial.print("VBAT = "); Serial.println(VBAT, 2); 

    // Send some data to the BMD-350
    dtostrf(temperature_C, 4, 1, StringTempC);
    dtostrf(pressure, 5, 1, StringP);
    dtostrf(humidity, 4, 1, StringH);
    sprintf(Packet, "%s %s %s ", StringTempC, StringP, StringH);
    Serial2.write(Packet);

    dtostrf(eCO2, 4, 2, StringeCO2);
    dtostrf(TVOC, 5, 1, StringTVOC);
    dtostrf(VBAT, 4, 2, StringVBAT);
    sprintf(Packet, "%s %s %s\n", StringeCO2, StringTVOC, StringVBAT);
    Serial2.write(Packet);
    Serial2.flush();

    // store some data to the SPI flash
    if(count > 9) {
      count = 0;

      if(sector_number < 8 && page_number < 0x0FFF) {
      flashPage[sector_number*32 + 0]  = tempCount;                     // Accel chip temperature
      flashPage[sector_number*32 + 1]  = (accelCount[0] & 0xFF00) >> 8; // MSB x-axis accel
      flashPage[sector_number*32 + 2]  = accelCount[0] & 0x00FF;        // LSB x-axis accel
      flashPage[sector_number*32 + 3]  = (accelCount[1] & 0xFF00) >> 8; // MSB y-axis accel
      flashPage[sector_number*32 + 4]  = accelCount[1] & 0x00FF;        // LSB y-axis accel
      flashPage[sector_number*32 + 5]  = (accelCount[2] & 0xFF00) >> 8; // MSB z-axis accel
      flashPage[sector_number*32 + 6]  = accelCount[2] & 0x00FF;        // LSB z-axis accel
      flashPage[sector_number*32 + 7]  = rawData[0];                    // eCO2 MSB
      flashPage[sector_number*32 + 8]  = rawData[1];                    // eCO2 LSB
      flashPage[sector_number*32 + 9]  = rawData[2];                    // TVOC MSB
      flashPage[sector_number*32 + 10] = rawData[3];                    // TVOC LSB
      flashPage[sector_number*32 + 11] = (compTemp & 0xFF000000) >> 24;
      flashPage[sector_number*32 + 12] = (compTemp & 0x00FF0000) >> 16;
      flashPage[sector_number*32 + 13] = (compTemp & 0x0000FF00) >> 8;
      flashPage[sector_number*32 + 14] = (compTemp & 0x000000FF);
      flashPage[sector_number*32 + 15] = (compHumidity & 0xFF000000) >> 24;
      flashPage[sector_number*32 + 16] = (compHumidity & 0x00FF0000) >> 16;
      flashPage[sector_number*32 + 17] = (compHumidity & 0x0000FF00) >> 8;
      flashPage[sector_number*32 + 18] = (compHumidity & 0x000000FF);
      flashPage[sector_number*32 + 19] = (compPress & 0xFF000000) >> 24;
      flashPage[sector_number*32 + 20] = (compPress & 0x00FF0000) >> 16;
      flashPage[sector_number*32 + 21] = (compPress & 0x0000FF00) >> 8;
      flashPage[sector_number*32 + 22] = (compPress & 0x000000FF);
      flashPage[sector_number*32 + 23] = Seconds;
      flashPage[sector_number*32 + 24] = Minutes;
      flashPage[sector_number*32 + 25] = Hours;
      flashPage[sector_number*32 + 26] = Day;
      flashPage[sector_number*32 + 27] = Month;
      flashPage[sector_number*32 + 28] = Year;
      flashPage[sector_number*32 + 29] = (rawVbat & 0xFF00) >> 8;
      flashPage[sector_number*32 + 30] =  rawVbat & 0x00FF;
      sector_number++;
    }
    else if(sector_number == 8 && page_number < 0x0FFF)
    {
      flash_page_program(flashPage, page_number);
      Serial.print("Wrote flash page: "); Serial.println(page_number);
      sector_number = 0;
      page_number++;
    }  
    else
    {
      Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");
    }
    }
  }
    
    STM32.sleep();
   
}

/* Useful functions */

 void alarmMatch()
{
  alarmFlag = true; // Just set flag when interrupt received, don't try reading data in an interrupt handler
}

void myinthandler1()
{
  newBMA280Data = true;  
}

void myinthandler2()
{
  newBMA280Tap = true;
}

void myinthandler3()
{
  newCCS811Data = true;
}

void  initializeBMD350()
  {
  /*Put BMD-350 in AT mode*/
  digitalWrite(ATMD, LOW); // set ATMD pin for AT mode
  Serial.println("ATMD pin set LOW!");
  delay(100);
  digitalWrite(BMD350Reset, LOW); // reset BMD-350
  Serial.println("Reset pin set LOW!");
  delay(100); // wait a while
  digitalWrite(BMD350Reset, HIGH); // restart BMD-350 
  Serial.println("Reset pin set HIGH!");
  delay(3000); // hold ATMD pin LOW for at least 2.5 seconds

  /* Get some basic info about the module using AT commands */
  Serial2.write("at\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("AT working? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
  
  Serial2.write("at$ver?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("BMDware version? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
   
  Serial2.write("at$blver?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Bootloader version? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
   
  Serial2.write("at$pver?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Protocol version? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
  
  Serial2.write("at$hwinfo?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Hardware info: "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);

  Serial2.write("at$mac?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("MAC address: "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
  
   
  /* Configure UART pass through mode*/
  // Set advertising interval
  Serial2.write("at$cadint 09C4\r"); // set to maximum 2500 ms interval
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Set advertising interval? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
   
  delay(100);
  
  // Query advertising interval
  Serial2.write("at$cadint?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("New advertising interval? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);


   //Set TX Power
  Serial2.write("at$ctxpwr E2\r"); // set to minimum E2 == -30 dB, EC == -20, FC == -4 (default), etc
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Set connectable TX power? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
  //Query TX power
  Serial2.write("at$ctxpwr?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Connectable TX Power level? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);



  // Enable hot swap AT Mode
  Serial2.write("at$hotswap 1\r"); // enable hot swap
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Enable hot swap? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
   
  delay(100);
  
  // Query hot swap AT Mode
  Serial2.write("at$hotswap?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Hot swap enabled? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);

 
  //Set Baud rate
  Serial2.write("at$ubr 57600\r"); // decimal values 38400, 57600, etc
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Set new Baud rate? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
   
  delay(100);
  
  // Query Baud rate
  Serial2.write("at$ubr?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("New Baud rate? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
 
  //Configure Flow Control
  Serial2.write("at$ufc 00\r"); // 0x00 to disable, 0x01 to enable
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Configured flow control? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
 
  delay(100);
  
  // Query Flow control
  Serial2.write("at$ufc?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Flow control enabled? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
 
  //Configure Parity
  Serial2.write("at$upar 00\r"); // 0x00 to disable, 0x01 to enable
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Configured parity? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
 
  delay(100);
  
  // Query Flow control
  Serial2.write("at$upar?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Parity enabled? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);
 
  //Enable UART pass through mode
  Serial2.write("at$uen 01\r"); // 0x00 to disable, 0x01 to enable
  while(!Serial2.available() ) { delay(100); }
  Serial.print("Set UART pass through mode ? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);

  delay(100);
   
  // Query UART pass through mode
  Serial2.write("at$uen?\r");
  while(!Serial2.available() ) { delay(100); }
  Serial.print("UART pass through mode enabled? "); 
  while ( Serial2.available() ) { Serial.write(Serial2.read() ); }
  Serial.println(" ");
  digitalWrite(myLed, HIGH); delay(100); digitalWrite(myLed, LOW);

  /*Put BMD-350 in UART pass through mode*/
  digitalWrite(ATMD, HIGH); // set ATMD pin for UART pass through (normal) mode
  Serial.println("ATMD pin set HIGH!");
  delay(100);
  digitalWrite(BMD350Reset, LOW); // reset BMD-350
  Serial.println("Reset pin set LOW!");
  delay(100); // wait a while
  digitalWrite(BMD350Reset, HIGH); // restart BMD-350 
  Serial.println("Reset pin set HIGH!");
  }


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
