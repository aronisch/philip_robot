#ifndef SERIALRPI_H
#define SERIALRPI_H

#include <Arduino.h>
#include "../MotorControl/DifferentialDrive.h"
#include "../actuators/actuators.h"
#include "../sensors/frontSensors.h"

class SerialRPi {
private:
    HardwareSerial *_serialPort;
    DifferentialDrive *_diffDrive;
    SimpleArm *_gripperArm;
    PlateHolder *_plateHolder;
    FrontSensors *_frontSensors;
public:
    SerialRpi(HardwareSerial *serPort, DifferentialDrive *diffD, SimpleArm *arm, PlateHolder *plateH, FrontSensors *sensors):_serialPort(serPort), _diffDrive(diffD), _gripperArm(arm), _plateHolder(plateH), _frontSensors(sensors){};
    void update();
};

#endif