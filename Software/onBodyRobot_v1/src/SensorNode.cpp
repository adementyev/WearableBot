//
//  SensorNode.cpp
//  onBodyRobot_v1
//
//  Created by Artem Dementyev  artemd@mit.edu on 1/15/16.
//
//

#include "SensorNode.h"
#include <iostream>

SensorNode::SensorNode(int ID, int numSensors, int senType)
{
    sensorType = senType;
    sensorID = ID;
    numberOfSensors = numSensors;
}



int SensorNode::getXPosition() {
    return xpos;
}


void SensorNode::addData(int inputArray []) {
    
    int *p;
    p = inputArray;
    // latestInput = *p;
    std::cout << sizeof(p) << std::endl;
    
    for (int i=0; i<sizeof(p); i++) {
        latestInput[i] = inputArray[i];
    }
    
    for ( int i = 0; i < 8; i++ )
    {
        std::cout << "*(p + " << i << ") : ";
        std::cout << *(p + i) << std::endl;
    }
    
    
    // int latestInput [numberOfSensors] = inputArray;
    int tempArray [arraySize];
    // tempArray = new int[arraySize];
    // sum = new int[arraySize];
    
    /*
     for (int g =0; g<numberOfSensors; g++) {
     for(int i =1; i<arraySize; i++) {
     tempArray[i] = inputData[g][i-1];
     }
     tempArray[0] = inputArray[g];
     
     for (int h = 0; h<arraySize ; h++) {
     inputData[g][h] = tempArray[h];
     }
     }
     */
    
}

int * SensorNode::getLatest(){
    return latestInput;
}
