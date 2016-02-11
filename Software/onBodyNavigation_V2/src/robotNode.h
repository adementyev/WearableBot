//
//  robotNode.h
//  onBodyNavigation_V2
//
//  Created by Artem Dementyev  artemd@mit.edu on 2/10/16.
//
//

#ifndef __onBodyNavigation_V2__robotNode__
#define __onBodyNavigation_V2__robotNode__

#include <stdio.h>
#include <iostream>
#include "ofApp.h"

#endif /* defined(__onBodyNavigation_V2__robotNode__) */

class robotNode : public ofBaseApp{
    
public: // place public functions or variables declarations here
    
    // methods, equivalent to specific functions of your class objects
    void addData(int []);
    int getCurrentXpos();
    int getCurrentYpos();
    int * getLatest();
    void calculatePosition();
    void displayLatest();
    void setID(int);
    void visualizePosition();
    void showDebugData();
    int  getID();
    void sendMotorCommand(unsigned char, bool, unsigned char, bool);
    
    
    // variables
    int sensorType;
    int numberOfSensors = 3;
    int inputData [3][10];//Number of Sensors=3, Buffer size=10
    int sensorID;
    int sum [3];
    int latestInput [9];
    int currentXpos = 500;
    int currentYpos = 500;
    int visualizationType = 1;
    int bufferSize = 10;
    float Xdis = 0;
    float Ydis = 0;
    
    
    robotNode(int, int, int ); // constructor - used to initialize an object, if no properties are passed
    //               the program sets them to the default value
    robotNode();
    
private: // place private functions or variables declarations here
    int arraySize = 10;
    
}; // dont't forget the semicolon!!
