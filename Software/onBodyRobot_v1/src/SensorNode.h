//
//  SensorNode.h
//  onBodyRobot_v1
//
//  Created by Artem Dementyev  artemd@mit.edu on 1/15/16.
//
//

#ifndef __SerialTest__SensorNode__
#define __SerialTest__SensorNode__

#include <stdio.h>

#endif /* defined(__SerialTest__SensorNode__) */

class SensorNode {
    
public: // place public functions or variables declarations here
    
    // methods, equivalent to specific functions of your class objects
    void addData(int []);
    int getXPosition();
    int * getLatest();
    // variables
    
    int sensorType;
    int numberOfSensors = 3;
    int inputData [3][10];//Number of Sensors=3, Buffer size=10
    int sensorID;
    int sum [3];
    int latestInput [8];
    int turn = 0;
    int xpos = 0;
    int ypos = 0;
    int visualizationType = 1;
    int bufferSize = 10;
    bool isSelected = false;
    
    SensorNode(int, int, int ); // constructor - used to initialize an object, if no properties are passed
    //               the program sets them to the default value
    SensorNode();
    
private: // place private functions or variables declarations here
    int arraySize = 10;
    
}; // dont't forget the semicolon!!
