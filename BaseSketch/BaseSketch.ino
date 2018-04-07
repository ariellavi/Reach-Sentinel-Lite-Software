
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_L3GD20_U.h>
#include "Adafruit_MCP9808.h"

/////////////////////////////
////// GPS Globals///////////
/////////////////////////////
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false; // (init to false, but set to true in set up code)
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

#include <SPI.h>
#include <RH_RF95.h>

/* Note: Since the sensors are declared globally, the respective sensor functions assume their successful declaration here */
/* Must assign a unique ID to each sensor: */
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

// IMP_TODO: Figure out a way to handle disconnected GPS!!!

bool MAG_FLAG = false, ACC_FLAG = false, GYRO_FLAG = false, BARO_FLAG = false, TEMP_FLAG = false, RADIO_FLAG = false, GPS_FLAG = true;

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
  float GPS_latitude;
  float GPS_longitude;
  float GPS_altitude;
  uint8_t GPS_hour;
  uint8_t GPS_minute;
  uint8_t GPS_seconds;
  float temp_tempC; // TODO: remove or keep depending on if the Adafruit_MCP9808 sensor is used
  float baro_pressure; // in mmHg
  float baro_altitude; // in meters
  float baro_tempC;
} currentPacket;

//////////////////////
/*SETUP */
//////////////////////

void setup() {
  Serial.begin(115200);

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

  // Initialising GPS
  Serial.println(F("Init GPS"));
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  

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
  delay(3000);
}

////////////////////////////////////////////////
//////// GPS Interrupt Functions ///////////////
////////////////////////////////////////////////

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
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
  getGPSData(packet);
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

bool getGPSData(struct datapacket* packet) {
  if (!GPS_FLAG) {
    packet->GPS_latitude = myNAN;
    packet->GPS_longitude = myNAN;
    packet->GPS_altitude = myNAN;
    packet->GPS_hour = 0;
    packet->GPS_minute = 0;
    packet->GPS_seconds = 0;
    return false;
  }

  if (GPS.newNMEAreceived()) {
  // a tricky thing here is if we print the NMEA sentence, or data
  // we end up not listening and catching other sentences! 
  // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
  //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA())) {   // this also sets the newNMEAreceived() flag to false
      packet->GPS_latitude = myNAN;
      packet->GPS_longitude = myNAN;
      packet->GPS_altitude = myNAN;
      packet->GPS_hour = 0;
      packet->GPS_minute = 0;
      packet->GPS_seconds = 0;
      return false;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }

  if (GPS.fix) {
    packet->GPS_latitude = GPS.latitudeDegrees;
    packet->GPS_longitude = GPS.longitudeDegrees;
    packet->GPS_altitude = GPS.altitude;
    packet->GPS_hour = GPS.hour;
    packet->GPS_minute = GPS.minute;
    packet->GPS_seconds = GPS.seconds;
  } else {
    packet->GPS_latitude = myNAN;
    packet->GPS_longitude = myNAN;
    packet->GPS_altitude = myNAN;
    packet->GPS_hour = 0;
    packet->GPS_minute = 0;
    packet->GPS_seconds = 0;
  }
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

  // Print GPS Data
  Serial.println(F("------------------------------------"));
  Serial.println(F("GPS:"));
  Serial.println(F("------------------------------------"));
  
  Serial.print(packet->GPS_latitude); Serial.println(F(" latitude"));
  Serial.print(packet->GPS_longitude); Serial.println(F(" longitude"));
  Serial.print(packet->GPS_altitude); Serial.println(F(" altitude"));
  

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
    Serial.print(F("Location (in degrees, works with Google Maps): "));
    Serial.print(currentPacket.GPS_latitude, 4);
    Serial.print(F(", ")); 
    Serial.println(currentPacket.GPS_longitude, 4);
  
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

