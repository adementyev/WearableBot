#Rovables Project 


#Motor Driver V2 (Main robot controller)
The robot controller board contains the necessary components to make functional robot. 
It has a brain of ATmega328p microcontroller, 6-Axis IMU and 2.4GHz wireless board
Also, it can take input from two encoders. 
A extension board can be placed on the top. 

![MediaLab Logo](/images/circuit_diagram.png)

##Programming the controller board 
The board needs AVRISPmkII programmer, and a special programming adapter. The programming connector can be just pressed fited into the holes. The power is not supplied by the programmer, so a seperate power supply will be needed. 
When programming choose Arduino Pro Mini (3.3V, 8MHz) w/ ATmega328 

![MediaLab Logo](/images/programming.jpg)

##Optical Encoders
The board can take two optical encoders. The onboard comparators will convert the analog signals into digital interrupts. Two digital to analog converters (DACs) can be used to set the threshold for comparator. It can be tricky to tune it. It is best done with two oscillascope probes. One on the analog output of optical sensor, and second on the comparator digital output. 

##Problems 
The packet loss is quite large (1 out of 20 ). I suspect it is because of the antenna. 


#Base station board 
Base station allows the robot to wirelessly talk to the computer. 
It is essentially Arduino Leonardo with special connector for a RF dongle. 
The ATMega32u4 processor allow fast USB communications to the computer

##Programming the controller board
The board has a Ardiono USB bootloader so it can be programmed as regular Arduino Leonardo. 


#Software
The current version of the software runs in OpenFrameworks. Tested on Xcode/MacBook Pro only C++ allows for faster 3D graphics, and faster response. The robot can be controlled remotely from the computer. 