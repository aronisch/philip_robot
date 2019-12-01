#include "DifferentialDrive.h"

void DifferentialDrive::setVelocities(double linearVel, double angularVel){
    _motorController->setSpeedRight(linearVel + angularVel * _axleDistance / 2);
    _motorController->setSpeedLeft(linearVel - angularVel * _axleDistance / 2);
}