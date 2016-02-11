//
//  robotNode.cpp
//  onBodyNavigation_V2
//
//  Created by Artem Dementyev  artemd@mit.edu on 2/10/16.
//
//

#include "robotNode.h"
#include <stdio.h>
#include <iostream>
#include <math.h>       /* atan2 */

robotNode::robotNode(int ID, int numSensors, int senType)
{
    sensorType = senType;
    sensorID = ID;
    numberOfSensors = numSensors;
}

robotNode::robotNode() {
}



void robotNode::addData(int inputArray []) {
    
    int *p;
    p = inputArray;
    // latestInput = *p;
   // std::cout << sizeof(p) << std::endl;
    
    for (int i=0; i<sizeof(p)+1; i++) {
        latestInput[i] = inputArray[i];
    }
    
    /*
    for ( int i = 0; i < 9; i++ )
    {
        std::cout << "*(p + " << i << ") : ";
        std::cout << *(p + i) << std::endl;
    }
    */
    
    // int latestInput [numberOfSensors] = inputArray;
    int tempArray [arraySize];
    // tempArray = new int[arraySize];
    // sum = new int[arraySize];
    
    
     for (int g =0; g<numberOfSensors; g++) {
         for(int i =1; i<arraySize; i++) {
             tempArray[i] = inputData[g][i-1];
         }
         tempArray[0] = inputArray[g];
     
         for (int h = 0; h<arraySize ; h++) {
             inputData[g][h] = tempArray[h];
         }
     }
  //  displayLatest();
    calculatePosition();
}

int * robotNode::getLatest(){
    return latestInput;
}

void robotNode::calculatePosition() {
    float q[4];
    float w,x,y,z;
    float roll,pitch,yaw;
    Xdis = 0;
    Ydis = 0;
    
    q[0] = (latestInput[5]) / 16384.0f;
    q[1] = (latestInput[6]) / 16384.0f;
    q[2] = (latestInput[7]) / 16384.0f;
    q[3] = (latestInput[8]) / 16384.0f;
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
    Xdis =  (latestInput[3] * sin(yaw));
    Ydis =  (latestInput[3] * cos(yaw));
    currentXpos = currentXpos + Xdis;
    currentYpos = currentYpos + Ydis;

    
}

int robotNode::getCurrentXpos(){
    return currentXpos;
}

int robotNode::getCurrentYpos(){
    return currentYpos;
}

void robotNode::displayLatest(){
    for ( int i = 0; i < 9; i++ )
    {
        std::cout << "*(p + " << i << ") : ";
        std::cout << latestInput[i]<< std::endl;
    }
    
    std::cout << sizeof(latestInput) << std::endl;
    
}

void robotNode::setID(int ID){
     sensorID = ID;
}

int robotNode::getID() {
    return sensorID;
}

void robotNode::visualizePosition() {
    if (sensorID == 1) {ofSetColor(ofColor::blue);}
    if (sensorID == 2) {ofSetColor(ofColor::yellow);}
    if (sensorID == 3) {ofSetColor(ofColor::green);}
    ofSetLineWidth(2);
    ofDrawCircle(ofVec3f(currentXpos, currentYpos, 0),10);
    
    
    
    ofDrawLine(ofVec3f(currentXpos, currentYpos, 0),
               ofVec3f(currentXpos+50*Xdis, currentYpos+50*Ydis, 0));
    
    ofDrawBitmapStringHighlight("  ID: " + ofToString(sensorID), currentXpos, currentYpos);
}

void robotNode::sendMotorCommand(unsigned char M1speed, bool M1direction,
                                 unsigned char M2speed, bool M2direction) {
    
}



void robotNode::showDebugData() {
    
        // ofDrawBitmapStringHighlight("Everything works!", 20, 20);
        ofDrawBitmapStringHighlight("ID: " + ofToString(latestInput[0]), 40, 40);
        ofDrawBitmapStringHighlight("Counter: " + ofToString(latestInput[1]), 40, 60);
        ofDrawBitmapStringHighlight("Retries: " + ofToString(latestInput[2]), 40, 80);
        ofDrawBitmapStringHighlight("INT0: " + ofToString(latestInput[3]), 40, 100); //*60*(10/8)
        ofDrawBitmapStringHighlight("INT1: " + ofToString(latestInput[4]), 40, 120);
        ofDrawBitmapStringHighlight("Quat W: " + ofToString(latestInput[5]), 40, 140);
        ofDrawBitmapStringHighlight("Quat X: " + ofToString(latestInput[6]), 40, 160);
        ofDrawBitmapStringHighlight("Quat Y: " + ofToString(latestInput[7]), 40, 180);
        ofDrawBitmapStringHighlight("Quat Z: " + ofToString(latestInput[8]), 40, 200);
        
        // ofSetLineWidth(4);
        /*
         line.clear();
         
         ofPoint pt;
         pt.set(400,400,400);
         pt.
         line.addVertex(pt);
         pt.set(Ax,Ay,Az);
         //line.addVertex(400+ 10*Ax,400+ 10* Ay, 400+ 10*Az);
         line.addVertex(400+20*Ay,400+20*Ax,400+20*Az);
         
         line.draw();
         
         */
        
        //  easyCam.begin();
        
        /*
         ofQuaternion someRot(x,y,z,w);
         ofVec3f center = ofVec3f(300,300,0);
         ofVec3f worldPoint = someRot * center;
         ofDrawLine(ofVec3f(300,300,0), worldPoint);
         //ofDrawLine(ofVec3f(100,200,300), ofVec3f(300,200,300));
         
         ofVec3f eulerAngles = someRot.getEuler();
         ofDrawLine(ofVec3f(400,400,0), ofVec3f(400+50*Ax, 400+50*Ay, 400+10*Az));
         
         
         ofDrawBitmapStringHighlight("Roll: " + ofToString(roll), 100, 200);
         ofDrawBitmapStringHighlight("Pitch: " + ofToString(pitch), 100, 220);
         ofDrawBitmapStringHighlight("Yaw: " + ofToString(yaw), 100, 240);
         
         
         ofDrawBitmapStringHighlight("Ax:" + ofToString(eulerAngles.x) + "Ay:" +
         ofToString(eulerAngles.y) + "Az:" +
         ofToString(eulerAngles.z), 400+20*Ay,400+20*Ax,400+20*Az);
         
         */
        /*
         ofPushMatrix();
         //   ofTranslate(ofGetWidth()/2, ofGetHeight()/2, 40);
         //Extract the rotation from the current rotation
         ofVec3f axis;
         float angle;
         //curRot.getRotate(angle, axis);
         curRot.set(x, y, z, w);
         //apply the quaternion's rotation to the viewport and draw the sphere
         
         static ofVec3f fromAxis,toAxis;
         static float fromAngle=0,toAngle;
         
         curRot.getRotate(toAngle,toAxis);
         
         //  ofRotate(toAngle, toAxis.x, toAxis.y, toAxis.z);
         //cone.setPosition(ofGetWidth()*.1, ofGetHeight()*.1, 0);
         ofTranslate(mouseX, mouseY, 0);
         
         ofRotate(toAngle, -(toAxis.x), toAxis.z, toAxis.y);
         //ofRotate(angle, axis.x, axis.y, axis.z);
         // ofDrawSphere(0, 0, 0, 200);
         //ofDrawCone(ofGetWidth() * .1, ofGetHeight()* .1, 0, 50, 200);
         
         //  cone.rotate(toAngle, toAxis.x, toAxis.y, toAxis.z);
         //cone.rotate(spinY, 0, 1.0, 0.0);
         //cone.setPosition(ofGetWidth()*.1, ofGetHeight()*.1, 0);
         //ofTranslate(-ofGetWidth()/2, -ofGetHeight()/2, 0);
         cone.drawAxes(cone.getHeight()+100);
         
         
         //cone.ofDrawArrow();
         //cone.drawWireframe();
         //ofTranslate(ofGetWidth()/2, ofGetHeight()/2, 0);
         model.drawFaces();
         
         
         
         //model.draw(OF_MESH_FILL); //same as model.drawFaces();
         ofPopMatrix();
         
         ofTranslate(500, 500, 0);
         ofSetColor(ofColor::gray);
         mesh.drawWireframe();
         
         glPointSize(2);
         ofSetColor(ofColor::white);
         mesh.drawVertices();
         ofVec3f cur = mesh.getVertex(2);
         
         */
        ofDrawBitmapStringHighlight("FPS: " + ofToString(ofGetFrameRate()), 300, 240);
    
}



