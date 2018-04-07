
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_L3GD20_U.h>
#include "Adafruit_MCP9808.h"

#include <SPI.h>
#include <RH_RF95.h>

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

bool MAG_FLAG = false, ACC_FLAG = false, GYRO_FLAG = false, BARO_FLAG = false, TEMP_FLAG = false, RADIO_FLAG = false;

float myNAN = sqrt(-1);
unsigned long previousMillis = 0;

//////////////////////
/*Radio definitions */
//////////////////////

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 431.3

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//////////////////////
/*DATA PACKET FORMAT */
//////////////////////

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
  float temp_tempC; // TODO: remove or keep depending on if the Adafruit_MCP9808 sensor is used
  float baro_pressure; // in mmHg
  float baro_altitude; // in meters
  float baro_tempC;
} currentPacket;

//////////////////////
/*SETUP */
//////////////////////

void setup() {
  Serial.begin(9600);

  // Initialising Magnetometer
  //if(!mag.begin()) {
  //  /* There was a problem detecting the HMC5883 ... check your connections */
  //  Serial.println(F("Ooops, no HMC5883 (Magnetometer) detected ... Check your wiring!"));
  //} else {
  //  Serial.println(F("The following sensor has been initialised:"));
  //  displaySensorDetails(&mag);
    //MAG_FLAG = true;
  //}

  // Initialising Accelerometer
  if(!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no ADXL345 (Accelerometer) detected ... Check your wiring!"));
  } else {
    Serial.println(F("The following sensor has been initialised:"));
    displaySensorDetails(&accel);
    ACC_FLAG = true;
  }
  // TODO: Setting range for accelerometer. Choose from 16,8,4,2 Gs.
  //accel.setRange(ADXL345_RANGE_16_G);

  // Initialising Barometer
  if(!baro.begin()) {
    /* There was a problem detecting the MPL3115A2 ... check your connections */
    Serial.println(F("Ooops, no MPL3115A2 (Barometer) detected ... Check your wiring!"));
  } else {
    Serial.println(F("The following sensor has been initialised: BAROMETER"));
    // ~displaySensorDetails(&baro);~ Display details functionality not enabled by Adafruit.
    BARO_FLAG = true;
  }

  if (!tempsensor.begin()) {
    Serial.println(F("Couldn't find MCP9808!"));
  } else {
    Serial.println(F("The following sensor has been initialised: MPCP9808"));
    // ~displaySensorDetails(&tempsensor);~ Display details functionality not enabled by Adafruit.
    TEMP_FLAG = true;
  }

  // Initialising Gyroscope
  Serial.println(F("Init gyro"));

  Wire.beginTransmission(0x6B); //Gyro address
  if(Wire.endTransmission() == 0) {
    GYRO_FLAG = true;
  }
  if(GYRO_FLAG && !gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println(F("Ooops, no L3GD20 (Gyroscope) detected ... Check your wiring!"));
  } else {
    Serial.println(F("The following sensor has been initialised:"));
    displaySensorDetails(&gyro);
    gyro.enableAutoRange(true);   
 }
  

  ///////
  // TODO: CLEAN THIS UP LATER
  ///////

  ///////
  // RADIO TRANSMIT CODE
  ///////

  Serial.println(F("Initializing radio"));

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
  
  } else {

    Serial.println(F("LoRa radio init OK!"));

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println(F("setFrequency failed"));
      while (1);
    }
    Serial.print(F("Set Freq to: ")); Serial.println(RF95_FREQ);
    
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  
    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
    //index values for setModemConfig, all are predefined
    //0 = Bw125Cr45Sf128
    //1 = Bw500Cr45Sf128
    //2 = Bw31_25Cr48Sf512
    //3 = Bw125Cr48Sf4096
    //optional to set own modem configuration values, but must use setModemRegisters(const ModemConfig * config)
    rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512); // can also use  rh_rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); 

    RADIO_FLAG = true;
  }
  
}


/////////////////////////
/* populateDataPacket */
/////////////////////////

/*
 * Function for populating datapacket with relevant data
 * 
 * Arguments:
 *  packet: datapacket * - datapacket to be modified
 * 
 * Return:
 *  None
 * 
 */

void populateDataPacket(struct datapacket* packet) {

  packet->timestamp = millis(); //Should not overflow until over 50 days
  
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
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

/////////////////////////
/* getxxxData */
/////////////////////////

/*
 * Helper Functions for getting data from sensors
 * 
 * Arguments:
 *  packet: datapacket * - datapacket to be modified
 * 
 * Return:
 *  True if sensor is working, false otherwise
 * 
 */
 
bool getAccelerometerData(struct datapacket* packet) {
  
  if(!ACC_FLAG) {
    packet->accel_x = myNAN;
    packet->accel_y = myNAN;
    packet->accel_z = myNAN;
    return false;
  }
  sensors_event_t event; 
  accel.getEvent(&event);
  /* Populate the datapacket with the results (acceleration is measured in m/s^2) */
  packet->accel_x = event.acceleration.x;
  packet->accel_y = event.acceleration.y;
  packet->accel_z = event.acceleration.z;

  return true;
}

bool getTemperatureData(struct datapacket* packet) {
  // Read and populate datapacket with the temperature, then convert to *F
  if(!TEMP_FLAG) {
    packet->temp_tempC = myNAN;
    
    return false;
  }
  
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  packet->temp_tempC = c;

  return true;
}

bool getBarometerData(struct datapacket* packet) {
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  if(!BARO_FLAG) {
    packet->baro_pressure = myNAN;
    packet->baro_altitude = myNAN;
    packet->baro_tempC = myNAN;
    return false;
  }
  
  packet->baro_pressure = baro.getPressure()/3377; // NOTE: in mmHG!!
  packet->baro_altitude = baro.getAltitude();
  packet->baro_tempC = baro.getTemperature();

  return true;
}

bool getGyroscopeData(struct datapacket* packet) {
  
  if(!GYRO_FLAG) {
    packet->gyro_x = myNAN;
    packet->gyro_y = myNAN;
    packet->gyro_z = myNAN;
    return false;
  }
  
  sensors_event_t event; 
  gyro.getEvent(&event);

  /* Populate datapacket with the results (speed is measured in rad/s) */
  packet->gyro_x = event.gyro.x;
  packet->gyro_y = event.gyro.y;
  packet->gyro_z = event.gyro.z;

  return true;
}


/* TODO: Figure out what mag.getEvent() (and, by extension, a call to this function) means for synchronicity.
 *  Will it wait until it gets data from the Magnetometer?
 *  Will it instruct the sensor to take a reading at the instant the function is called?
 *  Will it read from a buffer of existing data?
*/
// TODO: Figure out what we want returned from a Magnetometer read - {x, y, z} OR {heading} OR both?
// TODO: For now, functions just print to serial, will see what we want to do with data once we have a better understanding of things.
bool getMagnetometerReading(struct datapacket* packet) {
  if(!MAG_FLAG) {
    packet->mag_x = myNAN;
    packet->mag_y = myNAN;
    packet->mag_z = myNAN;
    packet->mag_heading = myNAN;
    return false;
  }
  
  
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

  return true;
}

/////////////////////////
/* printDataPacket */
/////////////////////////

/*
 * Function for printingDataPacket
 * 
 * Arguments:
 *  packet: datapacket * - datapacket to be printed
 *  
 * Return:
 *  none
 * 
 */


void printDataPacket(struct datapacket* packet) {
  
  //Serial.println(F("------------------------------------"));
  //Serial.println(F("---------!!START OF PACKET!!--------"));
  //Serial.println(F("------------------------------------\n"));

  // Print timestamp
  Serial.print(F("Timestamp: ")); Serial.println(packet->timestamp);

  // Print Accelerometer Data
  //Serial.println(F("------------------------------------"));
  Serial.println(F("ACCELEROMETER:"));
  //Serial.println(F("------------------------------------"));
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print(F("X: ")); Serial.print(packet->accel_x); Serial.print(F("  "));
  Serial.print(F("Y: ")); Serial.print(packet->accel_y); Serial.print(F("  "));
  Serial.print(F("Z: ")); Serial.print(packet->accel_z); Serial.print(F("  "));Serial.println(F("m/s^2 "));
  
  
  // Print Gyroscope Data
  Serial.println(F("------------------------------------"));
  Serial.println(F("GYROSCOPE:"));
  Serial.println(F("------------------------------------"));
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print(F("X: ")); Serial.print(packet->gyro_x); Serial.print(F("  "));
  Serial.print(F("Y: ")); Serial.print(packet->gyro_y); Serial.print(F("  "));
  Serial.print(F("Z: ")); Serial.print(packet->gyro_z); Serial.print(F("  "));Serial.println(F("rad/s "));
  

  // Print Magnetometer Data
  Serial.println(F("------------------------------------"));
  Serial.println(F("MAGNETOMETER:"));
  Serial.println(F("------------------------------------"));
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print(F("X: ")); Serial.print(packet->mag_x); Serial.print(F("  "));
  Serial.print(F("Y: ")); Serial.print(packet->mag_y); Serial.print(F("  "));
  Serial.print(F("Z: ")); Serial.print(packet->mag_z); Serial.print(F("  "));Serial.println(F("m/s^2 "));
  Serial.print(F("Heading (degrees): ")); Serial.println(packet->mag_heading);
 

  // Print Temperature data
  Serial.println(F("------------------------------------"));
  Serial.println(F("TEMPERATURE:"));
  Serial.println(F("------------------------------------"));
  
  Serial.print(F("Temp: ")); Serial.print(packet->temp_tempC); Serial.println(F("*C\t")); 
  

  // Print Barometer Data
  Serial.println(F("------------------------------------"));
  Serial.println(F("BAROMETER:"));
  Serial.println(F("------------------------------------"));
  
  Serial.print(packet->baro_pressure); Serial.println(F(" Inches (Hg)"));
  Serial.print(packet->baro_altitude); Serial.println(F(" meters"));
  Serial.print(packet->baro_tempC); Serial.println(F("*C"));
  

  Serial.println(F("\n------------------------------------"));
  Serial.println(F("----------!!END OF PACKET!!---------"));
  Serial.println(F("------------------------------------"));
}

/////////////////////////
/* radio_send */
/////////////////////////

/*
 * Function for sending message through radio
 * 
 * Arguments:
 *  msg: char * - Message to be sent
 *  len: int - Length of the message
 * 
 * Return:
 *  True if message is sent, false otherwise
 * 
 */

bool radio_send(uint8_t * msg, int len) {
  Serial.println(F("Sending to rf95_server"));
  // Send a message to rf95_server
  
  Serial.println(F("Sending...")); 
  bool ret = rf95.send(msg, len);


  Serial.println(F("Waiting for packet to complete...")); 
  //rf95.waitPacketSent();

  return ret;
}


/////////////////////////
/* send_packet */
/////////////////////////

/*
 * Function for sending packet through radio
 * 
 * Arguments:
 *  packet: struct datapacket * - packet to be sent
 * 
 * Return:
 *  True if message is sent, false otherwise
 * 
 */

bool send_packet(struct datapacket *packet) {
  byte tx_buf[sizeof(datapacket)] = {0};

  int zize = sizeof(*packet);
  memcpy(tx_buf, packet, zize);
  
  return radio_send((uint8_t *)tx_buf, zize);
}


 

/////////////////////////
/////// MAIN LOOP ///////
/////////////////////////

void loop() {

  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= 50) {
    
    Serial.println(F("Populating packet"));
    populateDataPacket(&currentPacket);
  
    Serial.println(F("Printing packet"));
    //printDataPacket(&currentPacket);
    Serial.println(currentPacket.timestamp);
    Serial.println(currentPacket.accel_x);
  
    previousMillis = currentMillis;
    
  }

  //Send data packet to radio, if radio is available
  if(RADIO_FLAG && rf95.waitPacketSent(1)) {
    bool ret = send_packet(&currentPacket);
    if(!ret) {
      Serial.println(F("Packet sending failed"));
    }
  }

  
}

