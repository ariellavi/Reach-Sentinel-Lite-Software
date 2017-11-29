
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

struct datapacket {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float max_y;
  float mag_z;
  float temp_tempF; // TODO: remove or keep depending on if the Adafruit_MCP9808 sensor is used
  float baro_pascals;
  float baro_altitude;
  float baro_tempC;
};

void setup() {
  Serial.begin(9600);

  // Initialising Magnetometer
  if(!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 (Magnetometer) detected ... Check your wiring!");
    while(1);
  } else {
    Serial.println("The following sensor has been initialised:");
    displaySensorDetails(&mag);
  }

  // Initialising Accelerometer
  if(!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 (Accelerometer) detected ... Check your wiring!");
    while(1);
  } else {
    Serial.println("The following sensor has been initialised:");
    displaySensorDetails(&accel);
  }
  // TODO: Setting range for accelerometer. Choose from 16,8,4,2 Gs.
  accel.setRange(ADXL345_RANGE_16_G);

  // Initialising Barometer
  if(!baro.begin()) {
    /* There was a problem detecting the MPL3115A2 ... check your connections */
    Serial.println("Ooops, no MPL3115A2 (Barometer) detected ... Check your wiring!");
    while(1);
  } else {
    Serial.println("The following sensor has been initialised: BAROMETER");
    // ~displaySensorDetails(&baro);~ Display details functionality not enabled by Adafruit.
  }

  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  } else {
    Serial.println("The following sensor has been initialised: MPCP9808");
    // ~displaySensorDetails(&tempsensor);~ Display details functionality not enabled by Adafruit.
  }

  // Initialising Gyroscope
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 (Gyroscope) detected ... Check your wiring!");
    while(1);
  } else {
    Serial.println("The following sensor has been initialised:");
    displaySensorDetails(&gyro);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

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
void getAccelerometerData() {
  sensors_event_t event; 
  accel.getEvent(&event);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER:");
  Serial.println("------------------------------------");
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  Serial.println("------------------------------------");
  Serial.println("------------------------------------");
}

void getTemperatureData() {
  // Read and print out the temperature, then convert to *F
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  Serial.println("------------------------------------");
  Serial.println("TEMPERATURE:");
  Serial.println("------------------------------------");
  Serial.print("Temp: "); Serial.print(c); Serial.print("*C\t"); 
  Serial.print(f); Serial.println("*F");
  Serial.println("------------------------------------");
  Serial.println("------------------------------------");
}

void getBarometerData() {
  Serial.println("------------------------------------");
  Serial.println("BAROMETER:");
  Serial.println("------------------------------------");
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");
  Serial.println("------------------------------------");
  Serial.println("------------------------------------");
}

void getGyroscopeData() {
  sensors_event_t event; 
  gyro.getEvent(&event);

  Serial.println("------------------------------------");
  Serial.println("GYROSCOPE:");
  Serial.println("------------------------------------");
  /* Display the results (speed is measured in rad/s) */
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");
  Serial.println("------------------------------------");
  Serial.println("------------------------------------");
  delay(500);
}


/* TODO: Figure out what mag.getEvent() (and, by extension, a call to this function) means for synchronicity.
 *  Will it wait until it gets data from the Magnetometer?
 *  Will it instruct the sensor to take a reading at the instant the function is called?
 *  Will it read from a buffer of existing data?
*/
// TODO: Figure out what we want returned from a Magnetometer read - {x, y, z} OR {heading} OR both?
// TODO: For now, functions just print to serial, will see what we want to do with data once we have a better understanding of things.
void getMagnetometerReading() {
  sensors_event_t event; 
  mag.getEvent(&event);

  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER:");
  Serial.println("------------------------------------");
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

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
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  // TODO: Delay at the end of the reading? Any significance?
  Serial.println("------------------------------------");
  Serial.println("------------------------------------");
}

