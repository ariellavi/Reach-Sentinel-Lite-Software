// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 431.30

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

//code for receiving on the other end
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
String unpack_packet(uint8_t* recvbuf) {
     memcpy(&currentPacket, recvbuf, sizeof(currentPacket));
     String ret;
     ret += String(currentPacket.timestamp) + ",";
     ret += String(currentPacket.accel_x) + "," + String(currentPacket.accel_y) + "," + String(currentPacket.accel_z) + ",";
     ret += String(currentPacket.gyro_x) + "," + String(currentPacket.gyro_y) + "," + String(currentPacket.gyro_z) + ",";
     ret += String(currentPacket.GPS_latitude) + "," + String(currentPacket.GPS_longitude) + "," + String(currentPacket.GPS_altitude) + ",";
     ret += String(currentPacket.GPS_hour) + "," + String(currentPacket.GPS_minute) + "," + String(currentPacket.GPS_seconds) + ",";
     ret += String(currentPacket.temp_tempC) + ",";
     ret += String(currentPacket.baro_altitude);
     return ret;
}

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600); //baud rate
  delay(100);

  Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
    
 //Setting Custom ModemConfig values
 // look at RH_RF95.h for specific regsiter values

  /*
  RH_RF95::ModemConfig custom_config = {
    [1] Sets BW: 
    [2] Sets SF
    [3] Sets AGC and Low Data Rate Optimization 
  };  
  

  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512); // can also use  rh_rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); 
*/
 
  RH_RF95::ModemConfig custom_config = {
    0x92, // BW = 500, CR = 4/5
    0xc4, //SF = 4096 chips/symbol, CRC = enable
    0x08  // Low date rate = on, AGC = off
  };
    

  rf95.setModemRegisters(&custom_config);
  
}

void loop()
{
  //delay(1000);
  //Serial.println("1,2,3,4,5,6,7,8,9,10,11,12,13");
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
      
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      //RH_RF95::printBuffer("", buf, len);
      //RH_RF95::printBuffer("Received: ", buf, len);
      Serial.println(unpack_packet(buf));
      //Serial.print("Got: ");
      //Serial.println((char*)buf);
      // Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);

      // No reply necessary
      /* Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
      */
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
