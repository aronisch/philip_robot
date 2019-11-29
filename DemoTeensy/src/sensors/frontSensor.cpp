#include "frontSensors.h"

/***** class FrontSensors ******/

//constructor
FrontSensors::FrontSensors(uint8_t leftTrigPin, uint8_t leftEchoPin, uint8_t rightTrigPin, int8_t rightEchoPin){
    _flSensors.attach(leftTrigPin, leftEchoPin);
    _frSensors.attach(rightTrigPin, rightEchoPin);

}
void FrontSensors::newMeasurement(){
    static bool leftRight = true;
    _flSensors.update();
    _frSensors.update();
    Serial.println(_frSensors.getReady());
    if (_flSensors.getReady()){
        _flSensors.trig();
    }
    if(_frSensors.getReady()){
      _frSensors.trig();
      leftRight = true;
    }
}

uint16_t FrontSensors::getLeftDist(){
    return _flSensors.getMesure();
}

uint16_t FrontSensors::getRightDist(){
    return _frSensors.getMesure();
}

void FrontSensors::start(){
    
    _flSensors.trig();
    _frSensors.trig();
}