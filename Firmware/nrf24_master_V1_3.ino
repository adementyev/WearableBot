// nrf24_reliable_datagram_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
// It is designed to work with the other example nrf24_reliable_datagram_server
// Tested on Uno with Sparkfun WRL-00691 NRF24L01 module
// Tested on Teensy with Sparkfun WRL-00691 NRF24L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module

#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define  LED_BLUE  9
#define  LED_RED   8 
#define  CE        11  //Toggle between RX/TX modes for the radio 
#define  CSN       7 //SS for SPI communications
#define TESTPIN    A0

// Singleton instance of the radio driver
RH_NRF24 nrf24(CE, CSN);// CE, CSN pins

// RH_NRF24 driver(8, 7);   // For RFM73 on Anarduino Mini

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(nrf24, SERVER_ADDRESS);

uint8_t data[] = "Hello World!";

// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() 
{
  Serial.begin(115200);
  pinMode(TESTPIN, OUTPUT);
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  
  
  //  if (!nrf24.setRF(RH_NRF24::DataRate1Mbps, RH_NRF24::TransmitPower0dBm))
  //  digitalWrite(LED_RED,HIGH);
  data[0] = 0x02;
  data[1] = 0x00;
  data[2] = 0x00;
  
  manager.setTimeout(100);

}



void loop()
{
  

  
 if (manager.available())
  {
    digitalWrite(TESTPIN, HIGH);
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("S");
      Serial.print(",");
      Serial.print(from, DEC);
      Serial.print(",");
      Serial.print(buf[1], DEC);
      Serial.print(",");
      Serial.print(buf[2], DEC);
      Serial.print(",");
      Serial.print(buf[3], DEC);
      Serial.print(",");
      Serial.print(buf[4], DEC); 
      Serial.print(",");
      Serial.print((buf[6]<<8) + buf[5]);
      Serial.print(",");
      Serial.print((buf[8]<<8) + buf[7]); 
      Serial.print(",");
      Serial.print((buf[10]<<8) + buf[9]); 
      Serial.print(",");
      Serial.println((buf[12]<<8) + buf[11]); 
     // Serial.print(",");
      //Serial.println((char*)buf);
      digitalWrite(LED_BLUE,!digitalRead(LED_BLUE));

      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from)) {
         digitalWrite(LED_RED,!digitalRead(LED_RED));
          Serial.println("sendtoWait failed");
      }
    }
    digitalWrite(TESTPIN, LOW);
    //  data[1] = data[1]++;
    //   data[2] = data[2]++; 
  }
  
  
  if(Serial.available()==12) {
    digitalWrite(LED_RED,!digitalRead(LED_RED));
    for (int i=0; i<13; i++) { 
      data[i] = Serial.read();
    }
  }
  
}//end loop

/*
void serialEvent() {
  byte counter = 0;
       digitalWrite(LED_RED,!digitalRead(LED_RED));

  while (Serial.available()) {
    // get the new byte:
    uint8_t inChar = Serial.read();
    // add it to the inputString:
    data[counter] += inChar;
    counter++;

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
*/

