//Created by Artem Dementyev on Feb 23, 2016
//TODO fix serial crashes

//Added the PID controller to this version

//REMEMBER to set I2C implementation to 2CDEV_ARDUINO_WIRE

//FIFO data rate is determined by MPU6050_6Axis_MotionApps20.h line 305
//last line 0x13 seems to be working ok, this drops data rate to 10 Hz
//now I want to increase it more so lets say 20 Hz

#include <SPI.h>
#include <RH_NRF24.h>
#include <RHReliableDatagram.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <avr/io.h>
#include <avr/interrupt.h>   
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include <PID_v1.h>
#include <RovablesARMMotor.h>

//Define PID
//Kd is at least 20 times bigger than Kp in my case
#define Kp .7 //0.1 before 
#define Kd 0.1
#define Ki 0 
//Tune Kp first? 

//.1/.5/.01

//Define MotorSpeeds
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 100 
#define leftBaseSpeed 100  

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
#define  PC1_PIN  A1
//#define  INT0_PIN     2
//#define  INT1_PIN     3  

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define DAC_LEFT_ADDRESS  0x60
#define DAC_RIGHT_ADDRESS  0x61

RH_NRF24 nrf24(CE, CSN);// CE, CSN pins
RHReliableDatagram manager(nrf24, CLIENT_ADDRESS);
Adafruit_MCP4725 dacLeft;
Adafruit_MCP4725 dacRight;
MPU6050 mpu;
RovablesMotor motor(0xE0);    //Wire library handle 0xE0 as 0xC0 (true value from datasheet)

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

uint8_t distanceToTravel = 0;
float yaw =0.0; 

boolean remoteNavigation = false; 
boolean timer1int = false;

Quaternion q;           // [w, x, y, z]         quaternion container
//double PID_input, PID_output, PID_setPoint; 
//PID myPID(&PID_input, &PID_output, &PID_setPoint,2,5,1,DIRECT);
//PID myPID(&PID_input, &PID_output, &PID_setPoint,.3,0.001,0.001,DIRECT);
uint8_t MOTOR_BASE_VALUE = 60;
uint8_t sumEncoder1 = 0; 

int lastError = 0; 
int PID_setPoint = 0;
int integral = 0;

void setup()
{
  pinMode(LED_RED, OUTPUT);     
  pinMode(LED_BLUE,OUTPUT);
  pinMode(M1_DIR, OUTPUT); 
  pinMode(M1_PWM, OUTPUT); 
  pinMode(M2_PWM, OUTPUT); 
  pinMode(M2_DIR, OUTPUT); 
  pinMode(INT_IMU, INPUT);
  pinMode(PC1_PIN, INPUT);
  
  
  dacLeft.begin(DAC_LEFT_ADDRESS);
  dacRight.begin(DAC_RIGHT_ADDRESS);
  dacLeft.setVoltage(650,false); //950 // 30
  dacRight.setVoltage(650,false); //4094 is max
  
  
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
  // setupTimer2();
//   OCR0A = 0xAF;
//  TIMSK0 |= _BV(OCIE0A);
   
   sei(); //enable interrupts
}

//-----------------INTERRUPT SERVICE ROUTINES----------------
ISR (INT0_vect)
{
  encoder1Counter++;   
  sumEncoder1++;
  //enable timer 2 
  //disable ISR0
  
  
   //----ENABLE TIMER 2--------
    OCR2A = 249; //2 ms delay

    TCCR2A |= (1 << WGM21);
    // Set to CTC Mode

    TIMSK2 |= (1 << OCIE2A);
    //Set interrupt on compare match

    TCCR2B |= (1 << CS22);
    // set prescaler to 64 and starts PWM
    
  //----DISABLE INT0 ---------
  EIMSK |= (0 << INT0);     
  
  
}

ISR (INT1_vect)
{
 encoder2Counter++;  
 //enable timer 2
 //disable ISR 1
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
 //disable timer 2
 //enable ISR0
 
     //---DISABLE TIMER 2-------
    TCCR2B |= (0 << CS22);
    
    //----ENABLE INT 0--------
    EIMSK |= (1 << INT0);     // Turns on INT0
}

ISR (TIMER0_COMPA_vect)  // timer0 overflow interrupt
{
    //Used to PWM motors already :(, So can't be used. 
}


void loop()
{
  
  //-----------GET MPU DATA--------------
   mpuIntStatus = mpu.getIntStatus();
   fifoCount = mpu.getFIFOCount();
  
 if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
    //  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
      
      if(remoteNavigation==true) { 
        //digitalWrite(LED_RED, HIGH);
        delay(1);
        doPIDstuff();
        if (sumEncoder1 > distanceToTravel) { 
         // remoteNavigation = false;
        //  sumEncoder1 = 0;
        }//end if
      }//end if 

    }//end else if
  
    


//-----------CHECK TIMER 1--------------
  if (timer1int == true && remoteNavigation==false) { 
    timer1int=false;
    counter++;
   if (counter>243)  {counter=0;}
    data[1] = counter; //counter
    data[2] = manager.retransmissions();
    data[3] = encoder1Counter; 
    data[4] = encoder2Counter;
    data[5] = fifoBuffer[1];
    data[6] = fifoBuffer[0]; 
    data[7] = fifoBuffer[5];
    data[8] = fifoBuffer[4]; 
    data[9] = fifoBuffer[9]; 
    data[10] =fifoBuffer[8];
    data[11] =fifoBuffer[13]; 
    data[12] =fifoBuffer[12];
    
 //-----------SEND THE DATA TO COMPUTER--------------      
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
            //Go forward with PID control
            case 0x05: 
              digitalWrite(LED_RED, !digitalRead(LED_RED));
              digitalWrite(M1_DIR, 1);
              digitalWrite(M2_DIR, 0);
              updateYaw();
              //PID_setPoint = (double)(yaw*(180/3.14));          
              distanceToTravel = buf[1];
              sumEncoder1=0;
              remoteNavigation = true;
              break;
            case 0x06: 
              break;           
            case 0xAA: 
              digitalWrite(LED_RED,HIGH);
              motor.driveForward(buf[1]);
              break;
            case 0xAB: 
              digitalWrite(LED_RED,LOW);
              motor.brake(); 
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

void computeYawPitchRoll() { 
     //Takes about 73us to compute on 16MHz clock, so about 140 us on 8MHz clock 
    float q[4];
    float w,x,y,z;
    float roll,pitch,yaw;
    
    q[0] = ((fifoBuffer[1]<<8) + fifoBuffer[0]) / 16384.0f;
    q[1] = ((fifoBuffer[5]<<8) + fifoBuffer[4]) / 16384.0f;
    q[2] = ((fifoBuffer[8]<<8) + fifoBuffer[9]) / 16384.0f;
    q[3] = ((fifoBuffer[12]<<8) + fifoBuffer[13]) / 16384.0f;
    for (int i = 0; i < 4; i++) {
        if (q[i] >= 2) q[i] = -4 + q[i];
    }
    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];
    
    roll = atan2((float) (2.0*(y*z + w*x)), w*w - x*x - y*y + z*z);
    pitch = (float) (-1.0 * asin((float) (-2.0*(x*z - w*y))));
    yaw = atan2((float) (2.0*(x*y + w*z)), w*w + x*x - y*y - z*z);
}

void updateYaw() { 
    float q[4];
    float w,x,y,z;
    
    q[0] = ((fifoBuffer[0]<<8) + fifoBuffer[1]) / 16384.0f;
    q[1] = ((fifoBuffer[4]<<8) + fifoBuffer[5]) / 16384.0f;
    q[2] = ((fifoBuffer[8]<<8) + fifoBuffer[9]) / 16384.0f;
    q[3] = ((fifoBuffer[12]<<8) + fifoBuffer[13]) / 16384.0f;
    for (int i = 0; i < 4; i++) {
        if (q[i] >= 2) q[i] = -4 + q[i];
    }
    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];
    
   // roll = atan2((float) (2.0*(y*z + w*x)), w*w - x*x - y*y + z*z);
   // pitch = (float) (-1.0 * asin((float) (-2.0*(x*z - w*y))));
    yaw = atan2((float) (2.0*(x*y + w*z)), w*w + x*x - y*y - z*z);
} 

void doPIDstuff() { 
  updateYaw();
  int currentYaw = (int)(yaw*(180/3.14));
  int error = currentYaw - 50;
  int motorSpeed = Kp * error + Kd * (error-lastError) + integral * error * Ki; 
  lastError = error; 
  integral = integral + error;
 // myPID.Compute(); 
 
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
  analogWrite(M1_PWM, rightMotorSpeed);
  analogWrite(M2_PWM, leftMotorSpeed);
  
  if (error < 5) { 
  digitalWrite(LED_RED, HIGH); 
  }
  else {digitalWrite(LED_RED, LOW);}
  //SPI.transfer((byte)motorSpeed);
  //SPI.transfer((byte)rightMotorSpeed);
  //SPI.transfer((byte)leftMotorSpeed); 
  
}

void initializePID() {
  updateYaw(); 
  PID_setPoint = (int)(yaw*(180/3.14));
  //myPID.SetMode(AUTOMATIC);
  //myPID.SetSampleTime(2);
  sumEncoder1=0;
  integral = 0; 
}





