#ifndef FRONT_SENSORS_H
#define FRONT_SENSORS_H

#include "ultrasonicSensor.h"
#include <Arduino.h>

class FrontSensors
{
    private: 
        UltrasonicSensor _flSensors;
        UltrasonicSensor _frSensors;
        bool left_right;
    public: 
        FrontSensors(uint8_t leftTrigPin, uint8_t leftEchoPin, uint8_t rightTrigPin, int8_t rightEchoPin);
        int getDist();
};

#endif