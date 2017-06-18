/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch is to operate the Sensor Tile (https://hackaday.io/project/19649-stm32l4-sensor-tile) 
 with BME280 pressure/temperature/humidity sensor, CCS811 eCO2 and VOC sensor, BMA280 accelerometer, 
 and ICS43434 I2S microphone all hosted by an STM32L432 MCU. There is also a BMD-350 (nRF52) module 
 for BLE connectivity as a UART bridge (slave) to the STM32L432 and a 1 MByte SPI flash fr data 
 logging and storage.
 
 The sketch uses default SDA/SCL pins on the Ladybug development board.

 Library may be used freely and without limit with attribution.
 
  */
#include "Wire.h"   
#include <RTC.h>
#include <avr/dtostrf.h>
#include "BMA280.h"
#include "BME280.h"
#include "CCS811.h"

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

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

float Temperature, Pressure, Humidity; // stores BME280 pressures sensor pressure and temperature
uint32_t rawPress, rawTemp, compHumidity, compTemp;   // pressure and temperature raw count output for BME280
uint16_t rawHumidity;  // variables to hold raw BME280 humidity value

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

uint32_t delt_t = 0, count = 0, sumCount = 0, slpcnt = 0;  // used to control display output rate

// RTC set up
/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 16;
const uint8_t hours = 9;

/* Change these values to set the current initial date */
const byte day = 10;
const byte month = 1;
const byte year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false;

// battery voltage monitor definitions
float VDDA, VBAT;
const byte VbatMon = A4;

BME280 BME280; // instantiate BME280 class

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
uint8_t Ascale = AFS_2G, BW = BW_125Hz, power_Mode = normal_Mode, sleep_dur = sleep_0_5ms, tapStatus, tapType;

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
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // array to hold the raw data
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;

bool newCCS811Data  = false; // boolean flag for interrupt

CCS811 CCS811(CCS811_intPin, CCS811_wakePin); // instantiate CCS811 class

// BMD-350 UART bridge
//Sensor Tile
#define ATMD        A3 // toggle pin for AT mode or UART pass through mode
#define BMD350Reset  0 // BMD-350 reset pin active LOW

char Packet[3], StringVBAT[4], StringP[5], StringH[4], StringT[4];


void setup()
{
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  Serial2.begin(57600);
  delay(4000);
  Serial.println("Serial2 enabled!"); // for BMD-350 UART Bridge

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led on

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  // Configure BMD-350 mode pins
  pinMode(ATMD, OUTPUT);
  pinMode(BMD350Reset, OUTPUT);  

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
  
  if(c == 0xFB && d == 0x60 && e == 0x81 ) {

   Serial.println("BMA280+BME280+CCS811 are online..."); Serial.println(" ");

   BME280.resetBME280(); // reset BME280 before initilization
   delay(100);

   BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy); // Initialize BME280 altimeter

   aRes = BMA280.getAres(Ascale);                                     // get sensor resolutions, only need to do this once
   BMA280.selfTestBMA280();                                           // perform sensor self test
   BMA280.resetBMA280();                                              // software reset before initialization
   delay(1000);                                                       // give some time to read the screen
   BMA280.initBMA280(Ascale, BW, power_Mode, sleep_dur);              // initialize sensor for data acquisition
   BMA280.fastCompensationBMA280();                                   // quickly estimate offset bias

    // initialize CCS811 and check version and status
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.CCS811init(AQRate);
    digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

    // set alarm to update the RTC every second
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
    RTC.attachInterrupt(alarmMatch);

    attachInterrupt(BMA280_intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA280
    attachInterrupt(BMA280_intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA280
    attachInterrupt(CCS811_intPin,  myinthandler3, FALLING); // enable CCS811 interrupt
 
    }
    else 
    if(c != 0xFB) Serial.println(" BMA280 not functioning!");
    if(d != 0x60) Serial.println(" BME280 not functioning!");    
    if(e != 0x81) Serial.println(" CCS811 not functioning!");    
}

void loop()
{
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
//    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
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

    
    
    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved
       alarmFlag = false;
   
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");

    tempCount = BMA280.readBMA280GyroTempData();  // Read the gyro adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

    rawTemp =  BME280.readBME280Temperature();
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    pressure = (float) BME280.BME280_compensate_P(rawPress)/25600.f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    rawHumidity =  BME280.readBME280Humidity();
    compHumidity = BME280.BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH
 
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

    VBAT = (127.0f/100.0f) * 3.30f * ((float)analogRead(VbatMon))/4095.0f;
    Serial.print("VBAT = "); Serial.println(VBAT, 2); 
      
    // Send some data to the BMD-350
    digitalWrite(myLed, HIGH);   // set the LED on
/*    dtostrf(VBAT, 4, 2, StringVBAT);
    dtostrf(pressure, 5, 1, StringP);
    dtostrf(humidity, 4, 2, StringH);
    sprintf(Packet, "%s,%s,%s", StringVBAT, StringP, StringH);
    Serial2.write(Packet);
    */
    delay(10);
    digitalWrite(myLed, LOW);   // set the LED off
    }  
      
 //     STM32.stop();    // time out in stop mode to save power, wake on alarm or sensor interrupt
 
}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler1()
{
  newBMA280Data = true;  // Just set flag when interrupt received, don't try reading data in an interrupt handler
}

void myinthandler2()
{
  newBMA280Tap = true;
}

void myinthandler3()
{
  newCCS811Data = true;
}

 void alarmMatch()
{
  alarmFlag = true;
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
 
  /* Configure UART pass through mode*/
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
