#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "Motor.h"

class DifferentialDrive {
private:
    MotorPID *_motorController;
    double _axleDistance;

public:
    DifferentialDrive(MotorPID *motC, double axleD):_motorController(motC), _axleDistance(axleD){};
    void setVelocities(double linearVel, double angularVel);
};

#endif