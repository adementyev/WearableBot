#include "ofApp.h"


int inputData [9]= {0,0,0,0,0,0,0,0,0};
int leftEncoder = 4000;
int rightEncoder = 4000;

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
    serial.setup("/dev/tty.usbmodem1d1111", baud); // mac osx example
   
    ofNoFill();
    ofSetFrameRate(30);
    ofBackground(0);
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
    string str;
    // int  inputData [9]= {0,0,0,0,0,0,0,0,0};
    
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
            
            //Sometimes crashes here...
            /*
            SensorNode tempNode = listSensors.at(inputData[0]-1);
            
            tempNode.addData(inputData);
            listSensors[inputData[0]-1] = tempNode;
            */
        }
        
    } while (str!="");

}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
   // ofDrawBitmapStringHighlight("Everything works!", 20, 20);
    ofDrawBitmapStringHighlight("ID: " + ofToString(inputData[0]), 40, 40);
    ofDrawBitmapStringHighlight("Counter: " + ofToString(inputData[1]), 40, 60);
    ofDrawBitmapStringHighlight("Retries: " + ofToString(inputData[2]), 40, 80);
    ofDrawBitmapStringHighlight("Encoder 1 RPM: " + ofToString(inputData[3]*60*(10/8)), 40, 100);
    ofDrawBitmapStringHighlight("Encoder 2 RPM: " + ofToString(inputData[4]*60*(10/8)), 40, 120);
    ofDrawBitmapStringHighlight("Quat W: " + ofToString(inputData[5]), 40, 140);
    ofDrawBitmapStringHighlight("Quat X: " + ofToString(inputData[6]), 40, 160);
    ofDrawBitmapStringHighlight("Quat Y: " + ofToString(inputData[7]), 40, 180);
    ofDrawBitmapStringHighlight("Quat Z: " + ofToString(inputData[8]), 40, 200);
       ofDrawBitmapStringHighlight("Encoder 1: " + ofToString(leftEncoder), 40, 220);
       ofDrawBitmapStringHighlight("Encoder 2: " + ofToString(rightEncoder), 40, 240);
    
    
    
    
    ofPushMatrix();
    ofTranslate(ofGetWidth()/2, ofGetHeight()/2, 40);
    //Extract the rotation from the current rotation
    ofVec3f axis;
    float angle;
    //curRot.getRotate(angle, axis);
    curRot.set(inputData[6], inputData[7], inputData[8], inputData[5]);
    //apply the quaternion's rotation to the viewport and draw the sphere
    
    static ofVec3f fromAxis,toAxis;
    static float fromAngle=0,toAngle;
    
     curRot.getRotate(toAngle,toAxis);
    
  //  ofRotate(toAngle, toAxis.x, toAxis.y, toAxis.z);
    ofRotate(toAngle, -(toAxis.x), toAxis.z, toAxis.y);
    //ofRotate(angle, axis.x, axis.y, axis.z);
    ofDrawSphere(0, 0, 0, 200);
    
    ofPopMatrix();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    //FORWARD
    if (key == '1'){
        unsigned char buf[12] = {0x02, 250, 250, 0x10, //130,30
                                0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    //STOP
    if (key == '2'){
        unsigned char buf[12] = {0x02, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    //LEFT
    if (key == '3'){
        unsigned char buf[12] = {0x02, 250, 250, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
    }
    
    //RIGHT
    if (key == '4'){
        unsigned char buf[12] = {0x02, 130, 30, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00};
        serial.writeBytes(&buf[0], 12);
        printf("Key sent");
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

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

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
