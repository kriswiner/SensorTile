/* Sensor Tile Basic Example Code using Ladybug
 by: Kris Winer
 date: March 9, 2017
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 MPU6500 accel/gyro is the main component of the MPU9250 9 DoF motion sensor and offers a superb accelerometer and gyro, 
 almost as good as the MAX21000. Here it is used for low frequency vibration detection.
 
 The CCS811 is an air quality sensor returning equivalent CO2 concentration in ppm and volatile organic compond concentration on ppb.
 The temperature and humidyt data from the BME280 are used in the air quality determination. The sensor is surprisingly
 sensitive to my breath when I am typing over it.
 
 The BMD-350 BLE module by Rigado uses the nRF52 as a simple UART bridge to allow data packets to be streamed to a console
 app on the iPhone. This is barely adequate as a dta logging method.
 
 The sensor tile (https://hackaday.io/project/19649-stm32l4-sensor-tile) uses the STM32L432 MCU which is also found on the 
 Ladybug developent board (https://www.tindie.com/products/TleraCorp/ladybug-stm32l432-development-board/?pt=ac_prod_search).
 Both are programmed using the Arduino IDE.
 
 SDA and SCL should have 4K7 pull-up resistors (to 3.3V).
 
 Hardware setup:
 SDA ----------------------- 21
 SCL ----------------------- 22
 
  */
#include "Wire.h"   
#include <RTC.h>
#include <avr/dtostrf.h>

// https://www.bosch-sensortec.com/bst/products/all_products/bme280
// BME280 registers
#define BME280_HUM_LSB    0xFE
#define BME280_HUM_MSB    0xFD
#define BME280_TEMP_XLSB  0xFC
#define BME280_TEMP_LSB   0xFB
#define BME280_TEMP_MSB   0xFA
#define BME280_PRESS_XLSB 0xF9
#define BME280_PRESS_LSB  0xF8
#define BME280_PRESS_MSB  0xF7
#define BME280_CONFIG     0xF5
#define BME280_CTRL_MEAS  0xF4
#define BME280_STATUS     0xF3
#define BME280_CTRL_HUM   0xF2
#define BME280_RESET      0xE0
#define BME280_ID         0xD0  // should be 0x60
#define BME280_CALIB00    0x88
#define BME280_CALIB26    0xE1

// http://ams.com/eng/Products/Environmental-Sensors/Gas-Sensors/CCS811
// CCS811 Registers
#define CCS811_STATUS             0x00
#define CCS811_MEAS_MODE          0x01
#define CCS811_ALG_RESULT_DATA    0x02
#define CCS811_RAW_DATA           0x03
#define CCS811_ENV_DATA           0x05
#define CCS811_NTC                0x06
#define CCS811_THRESHOLDS         0x10
#define CCS811_BASELINE           0x11
#define CCS811_HW_ID              0x20  // WHO_AM_I should be 0x81
#define CCS811_ID                 0x20  // WHO_AM_I should be 0x1X
#define CCS811_HW_VERSION         0x21  
#define CCS811_FW_BOOT_VERSION    0x23
#define CCS811_FW_APP_VERSION     0x24
#define CCS811_ERROR_ID           0xE0
#define CCS811_APP_START          0xF4
#define CCS811_SW_RESET           0xFF

// https://www.invensense.com/products/motion-tracking/6-axis/mpu-6500/
// MPU6500
#define SELF_TEST_X_GYRO  0x00                  
#define SELF_TEST_Y_GYRO  0x01                                                                          
#define SELF_TEST_Z_GYRO  0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F
#define XG_OFFSET_H       0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E   
#define WOM_THR           0x1F   
#define FIFO_EN           0x23
#define I2C_MST_CTRL      0x24   
#define I2C_SLV0_ADDR     0x25
#define I2C_SLV0_REG      0x26
#define I2C_SLV0_CTRL     0x27
#define I2C_SLV1_ADDR     0x28
#define I2C_SLV1_REG      0x29
#define I2C_SLV1_CTRL     0x2A
#define I2C_SLV2_ADDR     0x2B
#define I2C_SLV2_REG      0x2C
#define I2C_SLV2_CTRL     0x2D
#define I2C_SLV3_ADDR     0x2E
#define I2C_SLV3_REG      0x2F
#define I2C_SLV3_CTRL     0x30
#define I2C_SLV4_ADDR     0x31
#define I2C_SLV4_REG      0x32
#define I2C_SLV4_DO       0x33
#define I2C_SLV4_CTRL     0x34
#define I2C_SLV4_DI       0x35
#define I2C_MST_STATUS    0x36
#define INT_PIN_CFG       0x37
#define INT_ENABLE        0x38
#define INT_STATUS        0x3A
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48
#define EXT_SENS_DATA_00  0x49
#define EXT_SENS_DATA_01  0x4A
#define EXT_SENS_DATA_02  0x4B
#define EXT_SENS_DATA_03  0x4C
#define EXT_SENS_DATA_04  0x4D
#define EXT_SENS_DATA_05  0x4E
#define EXT_SENS_DATA_06  0x4F
#define EXT_SENS_DATA_07  0x50
#define EXT_SENS_DATA_08  0x51
#define EXT_SENS_DATA_09  0x52
#define EXT_SENS_DATA_10  0x53
#define EXT_SENS_DATA_11  0x54
#define EXT_SENS_DATA_12  0x55
#define EXT_SENS_DATA_13  0x56
#define EXT_SENS_DATA_14  0x57
#define EXT_SENS_DATA_15  0x58
#define EXT_SENS_DATA_16  0x59
#define EXT_SENS_DATA_17  0x5A
#define EXT_SENS_DATA_18  0x5B
#define EXT_SENS_DATA_19  0x5C
#define EXT_SENS_DATA_20  0x5D
#define EXT_SENS_DATA_21  0x5E
#define EXT_SENS_DATA_22  0x5F
#define EXT_SENS_DATA_23  0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO       0x63
#define I2C_SLV1_DO       0x64
#define I2C_SLV2_DO       0x65
#define I2C_SLV3_DO       0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define ACCEL_DETECT_CTRL  0x69
#define USER_CTRL         0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1        0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2        0x6C
#define FIFO_COUNTH       0x72
#define FIFO_COUNTL       0x73
#define FIFO_R_W          0x74
#define WHO_AM_I_MPU6500  0x75 // Should return 0x70
#define XA_OFFSET_H       0x77
#define XA_OFFSET_L       0x78
#define YA_OFFSET_H       0x7A
#define YA_OFFSET_L       0x7B
#define ZA_OFFSET_H       0x7D
#define ZA_OFFSET_L       0x7E

#define BME280_ADDRESS            0x77   // Address of BMP280 altimeter when ADO = 0
#define CCS811_ADDRESS            0x5A   // Address of the CCS811 Air Quality Sensor
#define MPU6500_ADDRESS           0x68   // Device address when ADO = 0

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Hosr {
  H_OSR_00 = 0,  // no op
  H_OSR_01,
  H_OSR_02,
  H_OSR_04,
  H_OSR_08,
  H_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BME280Sleep = 0,
  forced,
  forced2,
  normal
};

enum SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_10ms,
  t_20ms
};

enum AQRate {  // specify  frequency of air quality measurement
  dt_idle = 0,
  dt_1sec = 1,
  dt_10sec,
  dt_60sec
};

// Specify CCS811 rate
uint8_t AQRate = dt_10sec;

// Specify MPU6500 sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;

float aRes, gRes;        // scale resolutions per LSB for the sensors

// Specify BME280 configuration
uint8_t Posr = P_OSR_16, Hosr = H_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_021ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate
// t_fine carries fine temperature as global value for BME280
int32_t t_fine;

float Temperature, Pressure, Humidity; // stores BME280 pressures sensor pressure and temperature
uint32_t rawPress, rawTemp, compHumidity, compTemp;   // pressure and temperature raw count output for BME280
uint16_t rawHumidity;  // variables to hold raw BME280 humidity value

// BME280 compensation parameters
uint8_t dig_H1, dig_H3, dig_H6;
uint16_t dig_T1, dig_P1, dig_H4, dig_H5;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2;

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

uint32_t delt_t = 0, count = 0, sumCount = 0, slpcnt = 0;  // used to control display output rate

// MPU6500 and HMC5883L variables
int16_t MPU6500Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
float ax, ay, az, gx, gy, gz; // variables to hold latest sensor data values 
float   temperature;          // Stores the MPU6500 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 16;
const uint8_t hours = 9;

/* Change these values to set the current initial date */
const byte day = 10;
const byte month = 1;
const byte year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;
float VDDA, VBAT;
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};

char Packet[4], StringVBAT[3], StringP[7], StringH[5], StringT[4];

// Sensor Tile
const byte CCS811Interrupt = A1;
const byte CCS811Enable    = A2;
// Breath Analyzer
//const byte CCS811Interrupt = 4;
//const byte CCS811Enable = 3;

const byte VbatMon = A4;
const byte MPU6500Interrupt = 1; // Sensor Tile
//const byte MPU6500Interrupt = 10; // Ladybug v.02

//Sensor Tile
#define ATMD        A3 // toggle pin for AT mode or UART pass through mode
#define BMD350Reset  0 // BMD-350 reset pin active LOW

bool newMPU6500Data = false;
bool newCCS811Data  = false;

void setup()
{
  Serial.begin(57600);
  delay(4000);
  Serial.println("Serial enabled!");

  Serial2.begin(57600);
  delay(4000);
  Serial.println("Serial2 enabled!");

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led on

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  // Configure interrupts
  pinMode(MPU6500Interrupt, INPUT);
  pinMode(CCS811Interrupt, INPUT_PULLUP); // active LOW

  // Configure CCS811 enable
  pinMode(CCS811Enable, OUTPUT);
  digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor

  // Configure BMD-350 mode pins
  pinMode(ATMD, OUTPUT);
  pinMode(BMD350Reset, OUTPUT);  
 
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  I2Cscan(); // should detect BME280 at 0x76 and CCS811 at 0x5B

  digitalWrite(CCS811Enable, HIGH); // set LOW to enable the CCS811 air quality sensor

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

   // Read the WHO_AM_I register of the MPU6500, this is a good test of communication
  byte c = readByte(MPU6500_ADDRESS, WHO_AM_I_MPU6500);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU6500 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x70, HEX);
  delay(1000); 
  
  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte f = readByte(BME280_ADDRESS, BME280_ID);  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" I should be ");Serial.println(0x60, HEX);
  delay(1000); 

  // Read the WHO_AM_I register of the CCS811 this is a good test of communication
  digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor
  byte g = readByte(CCS811_ADDRESS, CCS811_ID);  // Read WHO_AM_I register for CCS8110
  digitalWrite(CCS811Enable, HIGH); // set HIGH to disable the CCS811 air quality sensor
  Serial.print("CCS811 "); Serial.print("I AM "); Serial.print(g, HEX); Serial.print(" I should be "); Serial.println(0x81, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(c == 0x70 && f == 0x60 && g == 0x81) {

  Serial.println("MPU6500+CCS811+BME280 are online..."); Serial.println(" ");

  MPU6500SelfTest(SelfTest); // Start by performing self test and reporting values
  Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
  delay(1000);

    // get MPU6500 resolutions, only need to do this once
    getAres();
    getGres();

    Serial.println("Calibrate gyro and accel"); Serial.println(" ");
    accelgyrocalMPU6500(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
    Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

    initMPU6500(); 
    Serial.println("MPU6500 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println(" ");
   
    writeByte(BME280_ADDRESS, BME280_RESET, 0xB6); // reset BME280 before initialization
    delay(100);

    BME280Init(); // Initialize BME280 altimeter

    // initialize CCS811 and check version and status
    digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811init();
    digitalWrite(CCS811Enable, HIGH); // set HIGH to disable the CCS811 air quality sensor

    attachInterrupt(MPU6500Interrupt, myinthandler1, RISING); // enable MPU6500 interrupt
    attachInterrupt(CCS811Interrupt,  myinthandler2, FALLING); // enable CCS811 interrupt
    }
    else 
    if(c != 0x70) Serial.println(" MPU6500 not functioning!");
    if(f != 0x60) Serial.println(" BME280 not functioning!");    
    if(g != 0x81) Serial.println(" CCS811 not functioning!");
}

void loop()
{
    // MPU6500 data
    // If intPin goes HIGH, all data registers have new data
    if(newMPU6500Data == true) {  // On interrupt, read data
     newMPU6500Data = false;  // reset newData flag
     
     readMPU6500Data(MPU6500Data); // INT cleared on any read
   
    // Now we'll calculate the accleration value into actual g's
     ax = (float)MPU6500Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)MPU6500Data[1]*aRes - accelBias[1];   
     az = (float)MPU6500Data[2]*aRes - accelBias[2];  

    // Calculate the gyro value into actual degrees per second
     gx = (float)MPU6500Data[4]*gRes;  // get actual gyro value, this depends on scale being set
     gy = (float)MPU6500Data[5]*gRes;  
     gz = (float)MPU6500Data[6]*gRes; 

     temperature = ((float) MPU6500Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
     } 
     
    Serial.println("MPU6500:");
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    Serial.println(" ");
    
    // BME280 Data
    rawTemp =   readBME280Temperature();
    compTemp = BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f; // temperature in Centigrade
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
    rawPress =  readBME280Pressure();
    pressure = (float) BME280_compensate_P(rawPress)/25600.0f; // Pressure in millibar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));
    rawHumidity =  readBME280Humidity();
    compHumidity = BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH

      Serial.println("BME280:");
      Serial.print("Altimeter temperature = "); 
      Serial.print( temperature_C, 2); 
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = "); 
      Serial.print(9.0f*temperature_C/5.0f + 32.0f, 2); 
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); 
      Serial.print(pressure, 2);  
      Serial.println(" mbar");// pressure in millibar
      altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));
      Serial.print("Altitude = "); 
      Serial.print(altitude, 2); 
      Serial.println(" feet");
      Serial.print("Altimeter humidity = "); 
      Serial.print(humidity, 1);  
      Serial.println(" %RH");// pressure in millibar
      Serial.println(" ");

      // CCS811 data 
      Serial.println("CCS811:");

      // Update CCS811 humidity and temperature compensation
      uint8_t temp[5] = {0, 0, 0, 0, 0};
      temp[0] = CCS811_ENV_DATA;
      temp[1] = ((compHumidity % 1024) / 100) > 7 ? (compHumidity/1024 + 1)<<1 : (compHumidity/1024)<<1;
      temp[2] = 0;
      if(((compHumidity % 1024) / 100) > 2 && (((compHumidity % 1024) / 100) < 8))
      {
       temp[1] |= 1;
      }

      compTemp += 2500;
      temp[3] = ((compTemp % 100) / 100) > 7 ? (compTemp/100 + 1)<<1 : (compTemp/100)<<1;
      temp[4] = 0;
      if(((compTemp % 100) / 100) > 2 && (((compTemp % 100) / 100) < 8))
      {
       temp[3] |= 1;
      }

      digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor    
      Wire.transfer(CCS811_ADDRESS, &temp[0], 5, NULL, 0);
      digitalWrite(CCS811Enable, HIGH); // set HIGH to disable the CCS811 air quality sensor
            
      // If intPin goes LOW, all data registers have new data
      if(newCCS811Data == true) {  // On interrupt, read data
      newCCS811Data = false;  // reset newData flag
     
      digitalWrite(CCS811Enable, LOW); // set LOW to enable the CCS811 air quality sensor
      uint8_t status = readByte(CCS811_ADDRESS, CCS811_STATUS);
      
      if(status & 0x01) { // check for errors
        uint8_t error = readByte(CCS811_ADDRESS, CCS811_ERROR_ID);
        if(error & 0x01) Serial.println("CCS811 received invalid I2C write request!");
        if(error & 0x02) Serial.println("CCS811 received invalid I2C read request!");
        if(error & 0x04) Serial.println("CCS811 received unsupported mode request!");
        if(error & 0x08) Serial.println("Sensor resistance measurement at maximum range!");
        if(error & 0x10) Serial.println("Heater current is not in range!");
        if(error & 0x20) Serial.println("Heater voltage is not being applied correctly!");
      }

      readBytes(CCS811_ADDRESS, CCS811_ALG_RESULT_DATA, 8, &rawData[0]);
          
      eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
      TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
      Current = (rawData[6] & 0xFC) >> 2;
      Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f), 3;  
     } 
     
      digitalWrite(CCS811Enable, HIGH); // set LOW to enable the CCS811 air quality sensor
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

      // Send some data to the BMD-350
      digitalWrite(myLed, HIGH);   // set the LED on
      dtostrf(VBAT, 4, 2, StringVBAT);
      dtostrf(pressure, 4, 2, StringP);
      dtostrf(humidity, 4, 2, StringH);
      sprintf(Packet, "%s,%s,%s", StringVBAT, StringP, StringH);
      Serial2.write(Packet);
      delay(100);
      digitalWrite(myLed, LOW);   // set the LED off

      delay(900);
      
 //     STM32.sleep();    // time out in stop mode to save power
}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler1()
{
  newMPU6500Data = true;
}

void myinthandler2()
{
  newCCS811Data = true;
}

  void getGres() {
  switch (Gscale)
  {
   // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void readMPU6500Data(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(MPU6500_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU6500_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU6500_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU6500_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initMPU6500()
{  
 // wake up device
  writeByte(MPU6500_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU6500_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU6500, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU6500_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU6500_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU6500_ADDRESS, GYRO_CONFIG);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6500_ADDRESS, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0] 
  writeByte(MPU6500_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6500_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
 // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU6500_ADDRESS, ACCEL_CONFIG);
//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6500_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6500_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU6500_ADDRESS, ACCEL_CONFIG2);
  writeByte(MPU6500_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  writeByte(MPU6500_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU6500_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
   writeByte(MPU6500_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU6500(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  writeByte(MPU6500_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU6500_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU6500_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeByte(MPU6500_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6500_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6500_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6500_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6500_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6500_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU6500_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU6500_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU6500_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6500_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU6500_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU6500_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6500_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU6500_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU6500_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  writeByte(MPU6500_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU6500_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU6500_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU6500_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU6500_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU6500_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6500_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU6500_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU6500_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU6500_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU6500_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU6500_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU6500_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU6500_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU6500_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6500SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0, 0, 0}, aAvg[3]  = {0, 0, 0}, aSTAvg[3]  = {0, 0, 0}, gSTAvg[3]  = {0, 0, 0};
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPU6500_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU6500_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU6500_ADDRESS, GYRO_CONFIG, 0x00);   // Set full scale range for the gyro to 250 dps
  writeByte(MPU6500_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU6500_ADDRESS, ACCEL_CONFIG, 0x00);  // Set full scale range for the accelerometer to 2 g

  for( uint8_t ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(MPU6500_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += ((int16_t)rawData[2] << 8) | rawData[3] ;  
  aAvg[2] += ((int16_t)rawData[4] << 8) | rawData[5] ; 
  
    readBytes(MPU6500_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += ((int16_t)rawData[2] << 8) | rawData[3] ;  
  gAvg[2] += ((int16_t)rawData[4] << 8) | rawData[5] ; 
  }
  
  for (uint8_t ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(MPU6500_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU6500_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( uint8_t ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(MPU6500_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU6500_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (uint8_t ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU6500_ADDRESS, ACCEL_CONFIG, 0x00);  
   writeByte(MPU6500_ADDRESS, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU6500_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU6500_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU6500_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU6500_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPU6500_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU6500_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[0] - 1.0f) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[1] - 1.0f) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[2] - 1.0f) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[3] - 1.0f) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[4] - 1.0f) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[5] - 1.0f) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // LSB only reported as FT
 // To get percent, must multiply by 100
   for (uint8_t i = 0; i < 3; i++) {
     destination[i]   = 100.0f*((float)(((aSTAvg[i]) & 0x000000FF) - (aAvg[i] & 0x000000FF)))/factoryTrim[i];   // Report percent differences
     destination[i+3] = 100.0f*((float)((gSTAvg[i] & 0x000000FF) - (gAvg[i] & 0x000000FF)))/factoryTrim[i+3]; // Report percent differences
   }
   
}

  uint32_t readBME280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME280_ADDRESS, BME280_TEMP_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

uint32_t readBME280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME280_ADDRESS, BME280_PRESS_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

uint16_t readBME280Humidity()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME280_ADDRESS, BME280_HUM_MSB, 2, &rawData[0]);  
  return (uint16_t) (((uint16_t) rawData[0] << 8 | rawData[1]) );
}


void BME280Init()
{
  // Configure the BME280
  // Set H oversampling rate
  writeByte(BME280_ADDRESS, BME280_CTRL_HUM, 0x07 & Hosr);
  // Set T and P oversampling rates and sensor mode
  writeByte(BME280_ADDRESS, BME280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BME280_ADDRESS, BME280_CONFIG, SBy << 5 | IIRFilter << 2);
  // Read and store calibration data
  uint8_t calib[26];
  readBytes(BME280_ADDRESS, BME280_CALIB00, 26, &calib[0]);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
  dig_H1 = calib[25];
  readBytes(BME280_ADDRESS, BME280_CALIB26, 7, &calib[0]);
  dig_H2 = ( int16_t)((( int16_t) calib[1] << 8) | calib[0]);
  dig_H3 = calib[2];
  dig_H4 = ( int16_t)(((( int16_t) calib[3] << 8) | (0x0F & calib[4]) << 4) >> 4);
  dig_H5 = ( int16_t)(((( int16_t) calib[5] << 8) | (0xF0 & calib[4]) ) >> 4 );
  dig_H6 = calib[6];

  Serial.println("Calibration coefficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
  Serial.print("dig_H1 ="); 
  Serial.println(dig_H1);
  Serial.print("dig_H2 ="); 
  Serial.println(dig_H2);
  Serial.print("dig_H3 ="); 
  Serial.println(dig_H3);
  Serial.print("dig_H4 ="); 
  Serial.println(dig_H4);
  Serial.print("dig_H5 ="); 
  Serial.println(dig_H5);
  Serial.print("dig_H6 ="); 
  Serial.println(dig_H6);
  Serial.println(" ");
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t BME280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22integer and 10fractional bits).
// Output value of “47445”represents 47445/1024= 46.333%RH
uint32_t BME280_compensate_H(int32_t adc_H)
{
int32_t var;

var = (t_fine - ((int32_t)76800));
var = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var)) +
((int32_t)16384)) >> 15) * (((((((var * ((int32_t)dig_H6)) >> 10) * (((var *
((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
var = (var < 0 ? 0 : var); 
var = (var > 419430400 ? 419430400 : var);
return(uint32_t)(var >> 12);
}

void checkCCS811Status() 
{
    // Check CCS811 status
  uint8_t status = readByte(CCS811_ADDRESS, CCS811_STATUS);
  Serial.print("status = 0X"); Serial.println(status, HEX);
  if(status & 0x80) {Serial.println("Firmware is in application mode. CCS811 is ready!");}
  else { Serial.println("Firmware is in boot mode!");}
  
  if(status & 0x10) {Serial.println("Valid application firmware loaded!");}
  else { Serial.println("No application firmware is loaded!");}

  if(status & 0x08) {Serial.println("New data available!");}
  else { Serial.println("No new data available!");}

  if(status & 0x01) {Serial.println("Error detected!");
        uint8_t error = readByte(CCS811_ADDRESS, CCS811_ERROR_ID);
        if(error & 0x01) Serial.println("CCS811 received invalid I2C write request!");
        if(error & 0x02) Serial.println("CCS811 received invalid I2C read request!");
        if(error & 0x04) Serial.println("CCS811 received unsupported mode request!");
        if(error & 0x08) Serial.println("Sensor resistance measurement at maximum range!");
        if(error & 0x10) Serial.println("Heater current is not in range!");
        if(error & 0x20) Serial.println("Heater voltage is not being applied correctly!");
  }
  else { Serial.println("No error detected!");}
  
  Serial.println(" ");
  }

  void CCS811init()
  {
    // initialize CCS811 and check version and status
  byte HWVersion = readByte(CCS811_ADDRESS, CCS811_HW_VERSION);
  Serial.print("CCS811 Hardware Version = 0x"); Serial.println(HWVersion, HEX); 

  uint8_t FWBootVersion[2] = {0, 0}, FWAppVersion[2] = {0,0};
  readBytes(CCS811_ADDRESS, CCS811_FW_BOOT_VERSION, 2, &FWBootVersion[0]);
  Serial.println("CCS811 Firmware Boot Version: "); 
  Serial.print("Major = "); Serial.println((FWBootVersion[1] & 0xF0) >> 4); 
  Serial.print("Minor = "); Serial.println(FWBootVersion[1] & 0x04); 
  Serial.print("Trivial = "); Serial.println(FWBootVersion[0]); 
  
  readBytes(CCS811_ADDRESS, CCS811_FW_APP_VERSION, 2, &FWAppVersion[0]);
  Serial.println("CCS811 Firmware App Version: "); 
  Serial.print("Major = "); Serial.println((FWAppVersion[1] & 0xF0) >> 4); 
  Serial.print("Minor = "); Serial.println(FWAppVersion[1] & 0x04); 
  Serial.print("Trivial = "); Serial.println(FWAppVersion[0]); 

  // Check CCS811 status
  checkCCS811Status();
        uint8_t temp[1] = {0};
        temp[0] = CCS811_APP_START;
        Wire.transfer(CCS811_ADDRESS, &temp[0], 1, NULL, 0); 
        delay(100);
  checkCCS811Status();

  // set CCS811 measurement mode
  writeByte(CCS811_ADDRESS, CCS811_MEAS_MODE, AQRate << 4 | 0x08); // pulsed heating mode, enable interrupt
  uint8_t measmode = readByte(CCS811_ADDRESS, CCS811_MEAS_MODE);
  Serial.print("Confirm measurement mode = 0x"); Serial.println(measmode, HEX);
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
 
// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    error = Wire.transfer(address, NULL, 0, NULL, 0);

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the BMP280 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }

