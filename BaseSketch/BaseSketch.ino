
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_L3GD20_U.h>
#include "Adafruit_MCP9808.h"

/* Note: Since the sensors are declared globally, the respective sensor functions assume their successful declaration here */
/* Must assign a unique ID to each sensor: */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // Magnetometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12346); // Accelerometer
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(12347); // Gyroscope
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808(); // Temperature Sensor

/*  Barometer Instructions:
 *  Power by connecting Vin to 3-5V, GND to GND
 *  Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
 *  See the Wire tutorial for pinouts for each Arduino
 *  http://arduino.cc/en/reference/wire 
 *  TODO: No Unique ID in example. Why?
*/
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

bool MAG_FLAG = false, ACC_FLAG = false, GYRO_FLAG = false, BARO_FLAG = false, TEMP_FLAG = false;

struct datapacket {
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
  float temp_tempF; // TODO: remove or keep depending on if the Adafruit_MCP9808 sensor is used
  float temp_tempC; // TODO: remove or keep depending on if the Adafruit_MCP9808 sensor is used
  float baro_pressure; // in mmHg
  float baro_altitude; // in meters
  float baro_tempC;
};

void setup() {
  Serial.begin(9600);

  // Initialising Magnetometer
  if(!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 (Magnetometer) detected ... Check your wiring!");
  } else {
    Serial.println("The following sensor has been initialised:");
    displaySensorDetails(&mag);
    MAG_FLAG = true;
  }

  // Initialising Accelerometer
  if(!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 (Accelerometer) detected ... Check your wiring!");
  } else {
    Serial.println("The following sensor has been initialised:");
    displaySensorDetails(&accel);
    ACC_FLAG = true;
  }
  // TODO: Setting range for accelerometer. Choose from 16,8,4,2 Gs.
  accel.setRange(ADXL345_RANGE_16_G);

  // Initialising Barometer
  if(!baro.begin()) {
    /* There was a problem detecting the MPL3115A2 ... check your connections */
    Serial.println("Ooops, no MPL3115A2 (Barometer) detected ... Check your wiring!");
  } else {
    Serial.println("The following sensor has been initialised: BAROMETER");
    // ~displaySensorDetails(&baro);~ Display details functionality not enabled by Adafruit.
    BARO_FLAG = true;
  }

  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
  } else {
    Serial.println("The following sensor has been initialised: MPCP9808");
    // ~displaySensorDetails(&tempsensor);~ Display details functionality not enabled by Adafruit.
    TEMP_FLAG = true;
  }

  // Initialising Gyroscope
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 (Gyroscope) detected ... Check your wiring!");
  } else {
    Serial.println("The following sensor has been initialised:");
    displaySensorDetails(&gyro);
    GYRO_FLAG = true;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  datapacket* currentPacket = new datapacket;
  populateDataPacket(currentPacket);
  printDataPacket(currentPacket);
  delay(500);
}

void populateDataPacket(struct datapacket* packet) {
  getAccelerometerData(packet);
  getTemperatureData(packet);
  getBarometerData(packet);
  getGyroscopeData(packet);
  getMagnetometerReading(packet);
}

// Takes a pointer to the sensor object as a parameter and displays the details.
void displaySensorDetails(Adafruit_Sensor* currentSensor) {
  sensor_t sensor;
  currentSensor->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// TODO: Currently prints X, Y, Z acceleration. Return format TBD.
void getAccelerometerData(struct datapacket* packet) {
  sensors_event_t event; 
  accel.getEvent(&event);
  /* Populate the datapacket with the results (acceleration is measured in m/s^2) */
  packet->accel_x = event.acceleration.x;
  packet->accel_y = event.acceleration.y;
  packet->accel_z = event.acceleration.z;
}

void getTemperatureData(struct datapacket* packet) {
  // Read and populate datapacket with the temperature, then convert to *F
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  packet->temp_tempC = c;
  packet->temp_tempF = f;
}

void getBarometerData(struct datapacket* packet) {
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  packet->baro_pressure = baro.getPressure()/3377; // NOTE: in mmHG!!
  packet->baro_altitude = baro.getAltitude();
  packet->baro_tempC = baro.getTemperature();
}

void getGyroscopeData(struct datapacket* packet) {
  sensors_event_t event; 
  gyro.getEvent(&event);

  /* Populate datapacket with the results (speed is measured in rad/s) */
  packet->gyro_x = event.gyro.x;
  packet->gyro_x = event.gyro.x;
  packet->gyro_x = event.gyro.x;
}


/* TODO: Figure out what mag.getEvent() (and, by extension, a call to this function) means for synchronicity.
 *  Will it wait until it gets data from the Magnetometer?
 *  Will it instruct the sensor to take a reading at the instant the function is called?
 *  Will it read from a buffer of existing data?
*/
// TODO: Figure out what we want returned from a Magnetometer read - {x, y, z} OR {heading} OR both?
// TODO: For now, functions just print to serial, will see what we want to do with data once we have a better understanding of things.
void getMagnetometerReading(struct datapacket* packet) {
  sensors_event_t event; 
  mag.getEvent(&event);

  /* Populate datapacket with the results (magnetic vector values are in micro-Tesla (uT)) */
  packet->mag_x = event.magnetic.x;
  packet->mag_y = event.magnetic.y;
  packet->mag_z = event.magnetic.z;

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // TODO: Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  packet->mag_heading = headingDegrees;
}

void printDataPacket(struct datapacket* packet) {
  
  Serial.println("------------------------------------");
  Serial.println("---------!!START OF PACKET!!--------");
  Serial.println("------------------------------------\n");

  // Print Accelerometer Data
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER:");
  Serial.println("------------------------------------");
  if (ACC_FLAG) {
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(packet->accel_x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(packet->accel_y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(packet->accel_z); Serial.print("  ");Serial.println("m/s^2 ");
  } else {
    Serial.print("NO ACCELEROMETER CONNECTED!");
  }
  
  // Print Gyroscope Data
  Serial.println("------------------------------------");
  Serial.println("GYROSCOPE:");
  Serial.println("------------------------------------");
  if (GYRO_FLAG) {
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(packet->gyro_x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(packet->gyro_y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(packet->gyro_z); Serial.print("  ");Serial.println("rad/s ");
  } else {
    Serial.print("NO GYROSCOPE CONNECTED!");
  }

  // Print Magnetometer Data
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER:");
  Serial.println("------------------------------------");
  if (MAG_FLAG) {
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(packet->mag_x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(packet->mag_y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(packet->mag_z); Serial.print("  ");Serial.println("m/s^2 ");
    Serial.print("Heading (degrees): "); Serial.println(packet->mag_heading);
  } else {
    Serial.print("NO MAGNETOMETER CONNECTED!");
  }

  // Print Temperature data
  Serial.println("------------------------------------");
  Serial.println("TEMPERATURE:");
  Serial.println("------------------------------------");
  if (TEMP_FLAG) {
    Serial.print("Temp: "); Serial.print(packet->temp_tempC); Serial.print("*C\t"); 
    Serial.print(packet->temp_tempF); Serial.println("*F");
  } else {
    Serial.print("NO TEMPERATURE SENSOR CONNECTED!");
  }

  // Print Barometer Data
  Serial.println("------------------------------------");
  Serial.println("BAROMETER:");
  Serial.println("------------------------------------");
  if (BARO_FLAG) {
    Serial.print(packet->baro_pressure); Serial.println(" Inches (Hg)");
    Serial.print(packet->baro_altitude); Serial.println(" meters");
    Serial.print(packet->baro_tempC); Serial.println("*C");
  } else {
    Serial.print("NO BAROMETER CONNECTED!");
  }


  Serial.println("\n------------------------------------");
  Serial.println("----------!!END OF PACKET!!---------");
  Serial.println("------------------------------------");
}

