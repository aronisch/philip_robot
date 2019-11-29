#include "Odometry.h"
#include <math.h>

void Odometry::resetPosition(){
    _positionX = 0;
    _positionY = 0;
    _positionTheta = 0;
}

void Odometry::updateSpeedsAndPositions(){
    static uint32_t _previousTicksRight = 0;
    static uint32_t _previousTicksLeft = 0;

    double displacementRight = (double)(_decoderRight.getDCount() - _previousTicksRight) * 2 * M_PI * _wheelRadius / _ticksPerRev;
    double displacementLeft = (double)(_decoderLeft.getDCount() - _previousTicksLeft) * 2 * M_PI * _wheelRadius / _ticksPerRev;

    _speedRight = (displacementRight) * 1000000 / (_updateTimeIntervalUS);
    _speedLeft = (displacementLeft) * 1000000 / (_updateTimeIntervalUS);

    double deltaS = (displacementRight + displacementLeft) / _axleDistance;
    double deltaTheta = (displacementRight - displacementLeft)/2;
    
    _positionX += deltaS * cosl(_positionTheta + deltaTheta/2);
    _positionY += deltaS * sinl(_positionTheta + deltaTheta/2);
    _positionTheta += deltaTheta;
}