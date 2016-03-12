//
//  ofApp.cpp
//  onBodyNavigation_V2
//
//  Created by Artem Dementyev  artemd@mit.edu on 2/10/16.
//
//  Added support for multiple robots
//  Added tracking of the robots

//  TODO:
//  1) Sending motor commands to different robots
//  2) Motor control commands in robotNode


#include "ofApp.h"
#include "robotNode.h"

const int NUM_ROBOTS = 10;


int inputData [9]= {0,0,0,0,0,0,0,0,0};
int leftEncoder = 4000;
int rightEncoder = 4000;

int counterPos = 0;

unsigned char left_motor = 100;
unsigned char right_motor = 100;
unsigned char left_motor_dir = 1;
unsigned char right_motor_dir = 0;

int currentXPos =500;
int currentYPos =500;


robotNode AllRobots[NUM_ROBOTS];

string ofxGetSerialString(ofSerial &serial, char until) {
    static string str;
    stringstream ss;
    char ch;
    int ttl=1000;
    while ((ch=serial.readByte())>0 && ttl-->0 && ch!=until) {
        ss << ch;
    }
    str+=ss.str();
    if (ch==until) {
        string tmp=str;
        str="";
        return tmp;
    } else {
        return "";
    }
}

//--------------------------------------------------------------
void ofApp::setup(){
    serial.listDevices();
    vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
    int baud = 115200;
    serial.setup("/dev/tty.usbmodem1d1131", baud); // mac osx example
    
    
    float width     = ofGetWidth() * .12;
    float height    = ofGetHeight() * .12;
    cone.set( width*.75, height*2.2 );
    ofSetSmoothLighting(true);
    pointLight.setDiffuseColor( ofFloatColor(.85, .85, .55) );
    pointLight.setSpecularColor( ofFloatColor(1.f, 1.f, 1.f));
    
    pointLight2.setDiffuseColor( ofFloatColor( 238.f/255.f, 57.f/255.f, 135.f/255.f ));
    pointLight2.setSpecularColor(ofFloatColor(.8f, .8f, .9f));
    
    pointLight3.setDiffuseColor( ofFloatColor(19.f/255.f,94.f/255.f,77.f/255.f) );
    pointLight3.setSpecularColor( ofFloatColor(18.f/255.f,150.f/255.f,135.f/255.f) );
    
    
    //model.loadModel("jeep1.dae",20);
    model.setRotation(0, 90, 0, 0, 0);
    //model.setRotation(1, 270, 0, 0, 1);
    model.setScale(0.5, 0.5, 0.5);
    model.setPosition(0, 0, 0);
    ofEnableDepthTest();
    glShadeModel (GL_SMOOTH);
    glColorMaterial (GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable (GL_COLOR_MATERIAL);
    
    mesh.load("lofi-bunny.ply");
    //to increase number of triangles in meshlab: remeshing->subdivision surfaces -> midpoint.
    
    
    // ofNoFill();
    ofSetFrameRate(60);
    ofBackground(20);
    
    
    for (int i=0; i<NUM_ROBOTS; i++ ) {
        AllRobots[i].setID(i);
    }
    
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
    string str;
    
    do {
        str = ofxGetSerialString(serial,'\n'); //read until end of line
        if (str=="") continue;
        for(int i = 0; i < str.length(); i++) {
            printf("%c",str[i]);
        }
        // printf("\n");
        vector<string> splitItems = ofSplitString(str, ",");
        vector <int> intVector;
        //cout << splitItems.size() << endl;
        
        
        for(int t=1;t<splitItems.size();++t){
            //intVector.at(t)= ofToInt(splitItems.at(t));
            //cout<<intVector.at(t);
            inputData[t-1] = ofToInt(splitItems.at(t));
            //cout<<inputData[t-1];
            //printf(",");
        }
        printf("\n");
        
        if(splitItems.at(0)=="S" && splitItems.size()==10) {
            int robotID = inputData[0];
            AllRobots[robotID].addData(inputData);
        }//end if
        
    } while (str!="");
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    drawGrid();
    AllRobots[1].visualizePosition();
    AllRobots[2].visualizePosition();
    
    AllRobots[1].showDebugData();
    
    
}

void ofApp::drawGrid() {
    int X_CELLS = 18;
    int Y_CELLS = 26;
    int CELL_SIZE = 25;
    int X_START = 250;
    int Y_START = 50;
    for (int i=0; i<X_CELLS+1; i++) {
        ofDrawLine(ofVec2f(X_START+CELL_SIZE*i, Y_START),
                   ofVec2f(X_START+CELL_SIZE*i, Y_START+CELL_SIZE*Y_CELLS));
        
    ofDrawBitmapStringHighlight(ofToString(i+1), X_START+CELL_SIZE*i, Y_START);
    }
    
    for (int i=0; i<Y_CELLS+1; i++) {
        ofDrawLine(ofVec2f(X_START, Y_START+CELL_SIZE*i),
                   ofVec2f(X_START+CELL_SIZE*X_CELLS, Y_START+CELL_SIZE*i));
    
        ofDrawBitmapStringHighlight(ofToString(i+1), X_START-CELL_SIZE, CELL_SIZE/2+Y_START+CELL_SIZE*i);
    }
    


}

void ofApp::sendMotorCommand(unsigned char robotID, unsigned char M1speed, bool M1direction,unsigned char M2speed, bool M2direction) {
    
    unsigned char buf[13] = {robotID, MOTOR_OPCODE, M1speed, M2speed, M1direction,M2direction, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00};
    serial.writeBytes(&buf[0], 13);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    //FORWARD
    if (key == '1'){
    }
    
    //STOP
    if (key == '2'){

    }
    
    //LEFT
    if (key == '3'){

    }
    
    //RIGHT
    if (key == '4'){
        unsigned char buf[12] = {0x02, 130, 30, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    if (key == '5'){
        sendMotorCommand(2,200,1,100,1);
    }
    
    if (key == '6'){
        sendMotorCommand(2,0,0,0,0);
    }
    
    if (key == '7'){
        sendMotorCommand(1,100,1,100,1);
    }
    
    if (key == '8'){
        sendMotorCommand(1,0,0,0,0);
    }
    
    
    if (key == '<'){
        leftEncoder = leftEncoder - 100;
        rightEncoder = rightEncoder - 100;
        unsigned char enc1 = leftEncoder/16;
        unsigned char enc2 = rightEncoder/16;
        unsigned char buf[12] = {0x04, enc1, enc2, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
        
        
        
    }
    
    if (key == '>'){
        leftEncoder = leftEncoder + 100;
        rightEncoder = rightEncoder + 100;
        unsigned char enc1 = leftEncoder/16;
        unsigned char enc2 = rightEncoder/16;
        unsigned char buf[12] = {0x04, enc1, enc2, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
        
    }
    
    //FORWARD THROTTLE
    if (key == 'w'){
        left_motor = left_motor + 10;
        right_motor = right_motor + 10;
        unsigned char buf[12] = {MOTOR_OPCODE, left_motor, right_motor, 0x00,0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    //FORWARD BREAK
    if (key == 's'){
        left_motor = left_motor - 10;
        right_motor = right_motor - 10;
        
        unsigned char buf[12] = {MOTOR_OPCODE, left_motor, right_motor, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    //LEFT TURN
    if (key == 'a'){
        left_motor = left_motor - 10;
        right_motor = right_motor + 10;
        unsigned char buf[12] = {MOTOR_OPCODE, left_motor, right_motor, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    //RIGHT TURN
    if (key == 'd'){
        left_motor = left_motor + 10;
        right_motor = right_motor - 10;
        unsigned char buf[12] = {MOTOR_OPCODE, left_motor, right_motor, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    
    
}



//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    ofPoint pt;
    pt.set(x,y);
    line.addVertex(pt);
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
