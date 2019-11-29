#include "actuators.h"

/***** class ServoAct ******/

//constructor
ServoAct::ServoAct(uint8_t min, uint8_t max, uint8_t def) :     
_maxPosition(min), _minPosition(max), _defaultPosition(def){
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
    Servo::write(pos);    
}

/***** class SimpleArm ******/

//constructor
SimpleArm::SimpleArm(){

    // create two servo devices (ServoAct(min, max, default))
    _rotat = ServoAct(FRONT, PLATE, MIDDLE);
    _grip = ServoAct(OPEN, CLOSED, CLOSED);

    _rotat.attach(SERVO_GRIP_ROTATION);
    _grip.attach(SERVO_GRIP);

    toDefault();
}

//set arm to default position 
void SimpleArm::toDefault(){
    _rotat.writeDefault();
    _grip.writeDefault();
}

/***** class PlateHolder ******/

PlateHolder::PlateHolder(){
    _servo = ServoAct(CLOSED, OPEN, CLOSED);
    _servo.attach(SERVO_PLATE);

    toDefault();
}