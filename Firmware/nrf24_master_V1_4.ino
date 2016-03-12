//Created by Artem Dementyev on Feb 23, 2016
//Added sending commands to a particular slave
//TODO fix serial sometimes crashes when computer sends data to the master


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

uint8_t whereToSend = 1;

void setup() 
{
  Serial.begin(115200);
  pinMode(TESTPIN, OUTPUT);
  pinMode(LED_RED, OUTPUT); 
  pinMode(LED_BLUE, OUTPUT);
  if (!manager.init()) { 
    Serial.println("init failed");
    digitalWrite(LED_RED, HIGH); 
  }
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

      if (from == whereToSend) { 
        data[0] =0x02;
      }
      else data [0]=0x01;
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
  
 // else {digitalWrite(LED_RED, !digitalRead(LED_RED));}
  
  if(Serial.available()>=13) {
   // delay(1);
    digitalWrite(LED_RED,!digitalRead(LED_RED));
    whereToSend = Serial.read();
    for (int i=0; i<12; i++) {
      
      data[i] = Serial.read();
    }
   // delay(1);
    //Serial.flush();
  }    
}//end loop


