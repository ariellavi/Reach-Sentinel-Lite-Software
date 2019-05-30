# Reach-Sentinel-Lite-Software #
Written for Project Reach (sponsored by Bruin Spacecraft Group at UCLA) by the Software Team (Edward Chu, Hirday Gupta) in 2017-2018.

The goal of Reach was to create an on-board telemetry system for an amateur rocket. The payload system was called SENTINEL-Lite, named after its parent iteration, SENTINEL-I. SENTINEL-Lite is composed of an Arduino UNO (used as a flight computer), five digital sensors (Altimeter, Accelerometer, Gyroscope, Magnetometer, Temp Sensor), a GPS, and a digital radio (LoRa). Data is stored on-board on an SD card, and also transmitted to a ground station consisting of a receiving Arduino and LoRa.

The sensors communicate with the UNO on a single I2C bus, and the LoRa utilize the SPI bus to connect to the Arduino. The GPS uses the serial (Rx, Tx) line one the UNO.

This repository contains the code for the Arduino that manages data from the sensors.

## Required Libraries: ##
* Adafruit_Sensor
* Adafruit_ADXL345_U (Accelerometer)
* Adafruit_HMC5883_U (Magnetometer)
* Adafruit_MPL3115A2 (Barometer)
* Adafruit_L3GD20_U (Gyroscope)
* Adafruit_MCP9808 (Temperature Sensor)
* RadioHead Packet Radio Library (Radio)

Libraries are included in the Required Libraries folder for convenience. All thanks to Adafruit.com and the RadioHead Packet Radio Library for providing open-source code.

## Data Packet ##
This is the struct used to store each packet of data received from the sensors.

```C++
struct datapacket {
  unsigned long timestamp;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float mag_heading; 
  float temp_tempC;
  float baro_pressure; // in mmHg
  float baro_altitude; // in meters
  float baro_tempC;
};
```

## Currently known bugs ##
- MAG_FLAG is set when magnetometer is disconnected
