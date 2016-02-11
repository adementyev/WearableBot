//
//  ofApp.h
//  onBodyNavigation_V2
//
//  Created by Artem Dementyev  artemd@mit.edu on 2/10/16.
//
//
#pragma once

#include "ofMain.h"
#include "ofxAssimpModelLoader.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    ofSerial	serial;
    
    //current state of the rotation
    ofQuaternion curRot;
    
    ofConePrimitive cone;
    ofLight pointLight;
    ofLight pointLight2;
    ofLight pointLight3;
    
    ofEasyCam easyCam;
    ofPolyline line;
    ofxAssimpModelLoader model;
    
    ofMesh mesh;
    
    ofVec3f positionTrackingData [1000];
    
    unsigned char MOTOR_CONTROL = 0x02;
    
    
};
