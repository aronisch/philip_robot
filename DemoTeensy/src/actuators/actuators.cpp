#include "actuators.h"

/***** class ServoAct ******/

//constructor
ServoAct::ServoAct(uint8_t min, uint8_t max, uint8_t def) :     
_maxPosition(max), _minPosition(min), _defaultPosition(def){
    writeDefault();
}

//Protected write for servo working as actuators
void ServoAct::write(uint8_t pos){

    if(pos <= _minPosition){
        pos = _minPosition;
    }
    else if(pos > _maxPosition){
        pos = _maxPosition;
    }
    Serial.println(pos);
    Servo::write(pos);    
}

/***** class SimpleArm ******/

//constructor
SimpleArm::SimpleArm(uint8_t gripperPin, uint8_t armPin){

    // create two servo devices (ServoAct(min, max, default))
    _rotat = ServoAct(PLATE, FRONT, MIDDLE);
    _grip = ServoAct(CLOSED, OPEN, DEFAULT);

    _rotat.attach(armPin);
    _grip.attach(gripperPin);

    toDefault();
}

//set arm to default position 
void SimpleArm::toDefault(){
    _rotat.writeDefault();
    _grip.writeDefault();
}

/***** class PlateHolder ******/

PlateHolder::PlateHolder(uint8_t servoPin){
    _servo = ServoAct(CLOSED, OPEN, CLOSED);
    _servo.attach(servoPin);

    toDefault();
}