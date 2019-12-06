#include "frontSensors.h"

/***** class FrontSensors ******/

//constructor
FrontSensors::FrontSensors(uint8_t leftTrigPin, uint8_t leftEchoPin, uint8_t rightTrigPin, int8_t rightEchoPin){
    _flSensors = UltrasonicSensor(leftEchoPin, leftTrigPin);
    _frSensors = UltrasonicSensor(rightEchoPin, rightTrigPin);
    left_right = false;
}

int getDist(){
    if(left_right){
        left_right = false;
        return _flSensors.getDist();
    }
    else{
        left_right = true;
        return _frSensors.getDist();
    }
}