//This version is trying to use nrf24_reliale_datagram_server
//REMEMBER to set I2C implementation to 2CDEV_ARDUINO_WIRE

 
#include <SPI.h>
#include <RH_NRF24.h>
#include <RHReliableDatagram.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <avr/io.h>
#include <avr/interrupt.h>   
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

//Define Pins
#define  LED_BLUE A2
#define  LED_RED  A3 
#define  CE       9  
#define  CSN      10 
#define  M1_DIR   4 
#define  M1_PWM   5 
#define  M2_PWM   6 
#define  M2_DIR   7 
#define  INT_IMU  A0
//#define  INT0_PIN     2
//#define  INT1_PIN     3  

#define CLIENT_ADDRESS 6
#define SERVER_ADDRESS 2

#define DAC_LEFT_ADDRESS  0x60
#define DAC_RIGHT_ADDRESS  0x61

RH_NRF24 nrf24(CE, CSN);// CE, CSN pins
RHReliableDatagram manager(nrf24, CLIENT_ADDRESS);
Adafruit_MCP4725 dacLeft;
Adafruit_MCP4725 dacRight;
MPU6050 mpu;


uint8_t data[] = "And hello back to you1223";
// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    
byte counter  = 0; 

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int encoder1Counter = 0; 
int encoder2Counter = 0;

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize; 
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t fifoCount;     // count of all bytes currently in FIFO


boolean timer1int = false;

Quaternion q;           // [w, x, y, z]         quaternion container

void setup()
{
  pinMode(LED_RED, OUTPUT);     
  pinMode(LED_BLUE,OUTPUT);
  pinMode(M1_DIR, OUTPUT); 
  pinMode(M1_PWM, OUTPUT); 
  pinMode(M2_PWM, OUTPUT); 
  pinMode(M2_DIR, OUTPUT); 
  pinMode(INT_IMU, INPUT);
  
  
  dacLeft.begin(DAC_LEFT_ADDRESS);
  dacRight.begin(DAC_RIGHT_ADDRESS);
  dacLeft.setVoltage(250,false); //950 // 30
  dacRight.setVoltage(250,false); //4094 is max
  
  
 // Wire.begin();
//  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz).
  TWBR = 2;  //This sets the clock to 400kHz for I2C ((8*10^6) / (16+2*(2)*1))/1000
  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();
  
    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  if (devStatus ==0 ) { 
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else { 
    digitalWrite(LED_RED, HIGH);
  }



/*
  if (!nrf24.init())
    digitalWrite(LED_RED,HIGH);
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    digitalWrite(LED_RED,HIGH);
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    digitalWrite(LED_RED,HIGH);
    */
    
  if (!manager.init())
      digitalWrite(LED_RED,HIGH);
      
//  if (!nrf24.setRF(RH_NRF24::DataRate1Mbps, RH_NRF24::TransmitPower0dBm))
//    digitalWrite(LED_RED,HIGH);
    //  Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
 
  //  analogWrite(M1_PWM, 100); 
  //  analogWrite(M2_PWM, 100); 
   manager.setTimeout(random(5,100));
   setupExternalInterrupts();
   setupTimer1();
   setupTimer2();
//   OCR0A = 0xAF;
//  TIMSK0 |= _BV(OCIE0A);
   
   sei(); //enable interrupts
}

//-----------------INTERRUPT SERVICE ROUTINES----------------
ISR (INT0_vect)
{
  encoder1Counter++;   
}

ISR (INT1_vect)
{
 encoder2Counter++;  
}

ISR (TIMER1_COMPA_vect)  // timer1 overflow interrupt
{
    //event to be exicuted every 4ms here
    data[3] = encoder1Counter; 
    data[4] = encoder2Counter;
    encoder1Counter=0; 
    encoder2Counter=0;
    timer1int = true;
   // PINC = (1<<2);
  //  PORTC^= (1<<2) ;
}

ISR (TIMER2_COMPA_vect) // timer2 overflow interrupt
{ 
  //PINC = (1<<2);
 // mpuIntStatus = mpu.getIntStatus();
}

ISR (TIMER0_COMPA_vect)  // timer0 overflow interrupt
{
    //event to be exicuted every 4ms here
}


void loop()
{

   mpuIntStatus = mpu.getIntStatus();
   fifoCount = mpu.getFIFOCount();
  
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }

 
 //mpu.getFIFOBytes(fifoBuffer, packetSize);

 

  
  if (timer1int == true) { 
    timer1int=false;
    counter++;
   if (counter>243)  {counter=0;}
     data[1] = counter; //counter
    data[2] = manager.retransmissions();
 // data[3] = encoder1Counter; 
 // data[4] = encoder2Counter;
    data[5] = fifoBuffer[1];
    data[6] = fifoBuffer[0]; 
    data[7] = fifoBuffer[5];
    data[8] = fifoBuffer[4]; 
    data[9] = fifoBuffer[9]; 
    data[10] =fifoBuffer[8];
    data[11] =fifoBuffer[13]; 
    data[12] =fifoBuffer[12];
    
       
    if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
    {
      
     // digitalWrite(INT0, HIGH);
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      uint8_t from;   
      if (manager.recvfromAckTimeout(buf, &len, 100, &from))
      {     
          switch (buf[0]) {
            //do nothing
            case 0x01:
            break;
            
            //Control the motor
            case 0x02: 
              analogWrite(M1_PWM, buf[1]);
              analogWrite(M2_PWM, buf[2]);
              digitalWrite(M1_DIR, buf[3]); 
              digitalWrite(M2_DIR, buf[4]);
            break;
            
            //Blink the blue LED
            case 0x03:
                digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
            break;
            
            //Set DACs to a specific voltage 
            case 0x04: 
                  dacLeft.setVoltage(buf[1]*16,false); //950 // 30
                  dacRight.setVoltage(buf[2]*16,false); //4094 is max
            break; 
            
               
          }//end switch
      //  Serial.print("got reply from : 0x");
      //  Serial.print(from, HEX);
      //  Serial.print(": ");
      //  Serial.println((char*)buf);      
      }
      else
      {
      //  Serial.println("No reply, is nrf24_reliable_datagram_server running?");
      }
     // digitalWrite(INT0, LOW);
      
    }
    else {
  //        digitalWrite(LED_RED, !digitalRead(LED_RED));
    }
  }//end timer1Int
  
    //Serial.println("sendtoWait failed");
  //  digitalWrite(LED_RED, !digitalRead(LED_RED));
 // delay(100);
  
}

void setupExternalInterrupts() { 
    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
    DDRD &= ~(1 << DDD3);     // Clear the PD3 pin
    // PD2 (PCINT0 pin) is now an input
    
   // PORTD |= (1 << PORTD2);    // turn On the Pull-up
   // PORTD |= (1 << PORTD3);    // turn On the Pull-up
    // PD2 is now an input with pull-up enabled
    
    // set INT0 to trigger on RISING EDGE
    EICRA |= (1 << ISC00);    
    EICRA |= (1 << ISC01);    // set INT0 to trigger on ANY logic change
    EIMSK |= (1 << INT0);     // Turns on INT0
    
    // set INT1 to trigger on RISING EDGE
    EICRA |= (1 << ISC10);    // set INT1 to trigger on ANY logic change
    EICRA |= (1 << ISC11);    // set INT1 to trigger on ANY logic change
    EIMSK |= (1 << INT1);     // Turns on INT1
  //  sei();                    // turn on interrupts
}

void setupTimer1() { 
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
//  OCR1A = 7812;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  OCR1A = 780;// = (8*10^6) / (10*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}


void setupTimer2() { 
      OCR2A = 200;

    TCCR2A |= (1 << WGM21);
    // Set to CTC Mode

    TIMSK2 |= (1 << OCIE2A);
    //Set interrupt on compare match

    TCCR2B |= (1 << CS20);
    TCCR2B |= (1 << CS21);
    TCCR2B |= (1 << CS22);
    // set prescaler to 64 and starts PWM
}


