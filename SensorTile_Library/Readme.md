Library for the new and improved STM32L432-based Sensor Tile, with BME280 pressure/temperature/humidity sensor, CCS811 equivalent CO2 and Volatile Organics sensor, and BMA280 accelerometer for low frequency vibrations and tap/orientation detection.

Also on the board are the ICS43434 I2S digital microphone, a 1 MByte SPI NOR flash memory, and the BMD-350 (nRF52) BLE module for wireless connectivity.

All of the I2C sensor RTC (time and date) data can be represented by 30 bytes of data, which at one second intervals (fastest CCS811 sample rate) fills up one 256-byte page of SPI flash memory every eight seconds. With 4096 pages, all of this data can be logged at 1 Hz for more than 9 hours. At ten second intervals, corresponding to one of the low power CCS811 sample rates, data can be logged for 4 days, and at the lowest rate of 60 seconds, the flash fills up after three weeks of logging.

The 20 mm x 20 mm sensor tile also has a variety of indicator leds, a user/boot button, and a MAX1555 LiPo battery charger.

![SensorTile.v01a](https://user-images.githubusercontent.com/6698410/27265138-e64e11ba-5444-11e7-8ddc-caf0cc17e75c.jpg)
