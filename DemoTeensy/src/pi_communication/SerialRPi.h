#ifndef SERIALRPI_H
#define SERIALRPI_H

#include <Arduino.h>
#include "../MotorControl/DifferentialDrive.h"
#include "../actuators/actuators.h"
#include "../sensors/frontSensors.h"
#include "../MotorControl/Odometry.h"

class SerialRPi {
private:
    HardwareSerial *_serialPort;
    DifferentialDrive *_diffDrive;
    SimpleArm *_gripperArm;
    PlateHolder *_plateHolder;
    FrontSensors *_frontSensors;
    Odometry *_odo;
public:
    SerialRPi(HardwareSerial *serPort, DifferentialDrive *diffD, SimpleArm *arm, PlateHolder *plateH, FrontSensors *sensors, Odometry *o):_serialPort(serPort), _diffDrive(diffD), _gripperArm(arm), _plateHolder(plateH), _frontSensors(sensors), _odo(o){};
    void update();
};

#endif