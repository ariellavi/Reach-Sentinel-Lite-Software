# Reach-Sentinel-Lite-Software #
Written for Bruin Spacecraft Group Reach Project 2017-2018.
Contains the code for the on-board Arduino that will manage the data from the sensors.

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
  float temp_tempF; 
  float temp_tempC;
  float baro_pressure; // in mmHg
  float baro_altitude; // in meters
  float baro_tempC;
};
```

## Currently known bugs ##
- IMPORTANT: Threading is not implemented, program have to wait until radio finishes sending data to continue reading from sensors
- MAG_FLAG is set when magnetometer is disconnected
